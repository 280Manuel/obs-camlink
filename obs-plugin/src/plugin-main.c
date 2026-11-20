/*
CamLink - usa tu teléfono Android como cámara en OBS (por USB / adb)
Copyright (C) 2026

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
*/

/* Los headers de sockets de Windows deben ir ANTES de cualquier include que cargue windows.h */
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define SOCK_BAD INVALID_SOCKET
#define sock_close closesocket
typedef HANDLE thread_t;
#else
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_BAD (-1)
#define sock_close close
typedef pthread_t thread_t;
#endif

#include <obs-module.h>
#include <util/platform.h>
#include <util/threading.h>
#include <util/pipe.h>
#include <plugin-support.h>

#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE(PLUGIN_NAME, "en-US")

/* ------------------------------------------------------------------------- */
/* Protocolo (big-endian). Debe coincidir con CamLinkStreamer.kt              */
/*   Handshake 12 bytes: "CMLK" | ver=1 | codec=1 | u16 w | u16 h | u8 fps | 0 */
/*   Paquete: u32 size | u8 flags (1=config, 2=key) | u64 pts_us | datos       */
/* ------------------------------------------------------------------------- */
#define DEFAULT_PORT 27183
#define HANDSHAKE_SIZE 12
#define PACKET_HDR_SIZE 13
#define MAX_PACKET_SIZE (16 * 1024 * 1024)
#define RECV_TIMEOUT_MS 500

struct camlink {
	obs_source_t *source;

	/* Ajustes (se copian al crear/actualizar; el hilo los lee) */
	char *adb_path;
	int port;
	bool low_latency;

	volatile bool stop;
	thread_t thread;
	bool thread_valid;
};

/* ------------------------------ utilidades de red ------------------------- */

static uint16_t rd16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

static uint32_t rd32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static bool sock_timed_out(void)
{
#ifdef _WIN32
	return WSAGetLastError() == WSAETIMEDOUT;
#else
	return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
}

/*
 * Lee exactamente len bytes. Cada timeout de 500 ms comprueba la bandera de parada.
 * max_idle: cuántos timeouts seguidos se toleran (-1 = infinito).
 * Devuelve 0 si ok, -1 si error / cierre / parada / demasiado tiempo sin datos.
 */
static int recv_all(struct camlink *s, sock_t sock, uint8_t *buf, size_t len, int max_idle)
{
	size_t got = 0;
	int idle = 0;

	while (got < len) {
		if (os_atomic_load_bool(&s->stop))
			return -1;

		int r = recv(sock, (char *)buf + got, (int)(len - got), 0);
		if (r > 0) {
			got += (size_t)r;
			idle = 0;
			continue;
		}
		if (r == 0)
			return -1;
		if (sock_timed_out()) {
			if (max_idle >= 0 && ++idle > max_idle)
				return -1;
			continue;
		}
		return -1;
	}
	return 0;
}

static sock_t connect_local(int port)
{
	sock_t sk = socket(AF_INET, SOCK_STREAM, 0);
	if (sk == SOCK_BAD)
		return SOCK_BAD;

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)port);
	inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

	if (connect(sk, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		sock_close(sk);
		return SOCK_BAD;
	}

#ifdef _WIN32
	DWORD tmo = RECV_TIMEOUT_MS;
	setsockopt(sk, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof(tmo));
#else
	struct timeval tv = {0, RECV_TIMEOUT_MS * 1000};
	setsockopt(sk, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
	int one = 1;
	setsockopt(sk, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));
	int rcvbuf = 4 * 1024 * 1024;
	setsockopt(sk, SOL_SOCKET, SO_RCVBUF, (const char *)&rcvbuf, sizeof(rcvbuf));
	return sk;
}

/* ------------------------------ adb ------------------------------------- */

/* Ejecuta `adb forward tcp:P tcp:P` y espera a que termine. */
static void adb_forward(struct camlink *s)
{
	char cmd[1024];
	snprintf(cmd, sizeof(cmd), "\"%s\" forward tcp:%d tcp:%d", s->adb_path, s->port, s->port);

	os_process_pipe_t *pp = os_process_pipe_create(cmd, "r");
	if (!pp) {
		obs_log(LOG_WARNING, "no pude ejecutar adb (%s). Revisa la ruta de adb en las propiedades de la fuente.",
			s->adb_path);
		return;
	}

	char out[512];
	uint8_t tmp[128];
	size_t tot = 0, n;
	while ((n = os_process_pipe_read(pp, tmp, sizeof(tmp))) > 0) {
		if (tot + n < sizeof(out) - 1) {
			memcpy(out + tot, tmp, n);
			tot += n;
		}
	}
	out[tot] = 0;
	os_process_pipe_destroy(pp);

	if (tot > 0)
		obs_log(LOG_INFO, "adb: %s", out);
}

/* ------------------------------ decodificador ----------------------------- */

static AVCodecContext *open_decoder(bool low_latency)
{
	const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (!codec)
		return NULL;

	AVCodecContext *c = avcodec_alloc_context3(codec);
	if (!c)
		return NULL;

	c->thread_count = 0; /* automático */
	if (low_latency) {
		/* menos latencia, pero a 4K en una CPU modesta puede no alcanzar los 30 fps */
		c->thread_type = FF_THREAD_SLICE;
		c->flags |= AV_CODEC_FLAG_LOW_DELAY;
		c->flags2 |= AV_CODEC_FLAG2_FAST;
	} else {
		c->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
	}

	if (avcodec_open2(c, codec, NULL) < 0) {
		avcodec_free_context(&c);
		return NULL;
	}
	return c;
}

static void output_frame(struct camlink *s, const AVFrame *f, bool *warned)
{
	if (f->format != AV_PIX_FMT_YUV420P && f->format != AV_PIX_FMT_YUVJ420P) {
		if (!*warned) {
			obs_log(LOG_WARNING, "formato de píxel no soportado: %d", f->format);
			*warned = true;
		}
		return;
	}

	bool full = (f->format == AV_PIX_FMT_YUVJ420P) || (f->color_range == AVCOL_RANGE_JPEG);
	enum video_colorspace cs = (f->colorspace == AVCOL_SPC_BT470BG || f->colorspace == AVCOL_SPC_SMPTE170M)
					   ? VIDEO_CS_601
					   : VIDEO_CS_709;

	struct obs_source_frame of;
	memset(&of, 0, sizeof(of));
	of.format = VIDEO_FORMAT_I420;
	of.width = (uint32_t)f->width;
	of.height = (uint32_t)f->height;
	of.timestamp = os_gettime_ns();
	of.full_range = full;
	for (int i = 0; i < 3; i++) {
		of.data[i] = f->data[i];
		of.linesize[i] = (uint32_t)f->linesize[i];
	}
	video_format_get_parameters_for_format(cs, full ? VIDEO_RANGE_FULL : VIDEO_RANGE_PARTIAL, VIDEO_FORMAT_I420,
					       of.color_matrix, of.color_range_min, of.color_range_max);

	/* obs_source_output_video copia los datos, así que el AVFrame se puede reutilizar. */
	obs_source_output_video(s->source, &of);
}

/* Recibe paquetes de un socket ya conectado hasta que se cierre o se pida parar. */
static void stream_loop(struct camlink *s, sock_t sock)
{
	uint8_t hs[HANDSHAKE_SIZE];
	if (recv_all(s, sock, hs, sizeof(hs), 10) != 0)
		return; /* adb acepta la conexión aunque la app no esté: se reintenta */

	if (memcmp(hs, "CMLK", 4) != 0 || hs[4] != 1 || hs[5] != 1) {
		obs_log(LOG_WARNING, "handshake inválido (¿versión distinta de la app?)");
		return;
	}
	obs_log(LOG_INFO, "teléfono conectado: %ux%u @ %u fps", rd16(hs + 6), rd16(hs + 8), hs[10]);

	AVCodecContext *dec = open_decoder(s->low_latency);
	AVPacket *pkt = av_packet_alloc();
	AVFrame *frame = av_frame_alloc();
	if (!dec || !pkt || !frame) {
		obs_log(LOG_ERROR, "no pude iniciar el decodificador H.264");
		goto done;
	}

	bool warned = false;
	uint8_t hdr[PACKET_HDR_SIZE];

	while (!os_atomic_load_bool(&s->stop)) {
		if (recv_all(s, sock, hdr, sizeof(hdr), -1) != 0)
			break;

		uint32_t size = rd32(hdr);
		if (size == 0 || size > MAX_PACKET_SIZE) {
			obs_log(LOG_WARNING, "tamaño de paquete inválido: %u", size);
			break;
		}

		if (av_new_packet(pkt, (int)size) < 0)
			break;
		if (recv_all(s, sock, pkt->data, size, 20) != 0) {
			av_packet_unref(pkt);
			break;
		}

		int ret = avcodec_send_packet(dec, pkt);
		av_packet_unref(pkt);
		if (ret < 0 && ret != AVERROR(EAGAIN))
			continue; /* paquete roto: seguir y esperar el próximo keyframe */

		while (avcodec_receive_frame(dec, frame) == 0) {
			output_frame(s, frame, &warned);
			av_frame_unref(frame);
		}
	}

done:
	av_frame_free(&frame);
	av_packet_free(&pkt);
	avcodec_free_context(&dec);
}

/* ------------------------------ hilo de trabajo --------------------------- */

static void worker(struct camlink *s)
{
	while (!os_atomic_load_bool(&s->stop)) {
		adb_forward(s);

		sock_t sock = connect_local(s->port);
		if (sock != SOCK_BAD) {
			stream_loop(s, sock);
			sock_close(sock);
			obs_source_output_video(s->source, NULL); /* limpiar imagen congelada */
		}

		/* Espera 2 s antes de reintentar, pero reacciona rápido a parar. */
		for (int i = 0; i < 20 && !os_atomic_load_bool(&s->stop); i++)
			os_sleep_ms(100);
	}
}

#ifdef _WIN32
static DWORD WINAPI worker_entry(LPVOID p)
{
	worker((struct camlink *)p);
	return 0;
}
#else
static void *worker_entry(void *p)
{
	worker((struct camlink *)p);
	return NULL;
}
#endif

static void start_worker(struct camlink *s)
{
	os_atomic_set_bool(&s->stop, false);
#ifdef _WIN32
	s->thread = CreateThread(NULL, 0, worker_entry, s, 0, NULL);
	s->thread_valid = s->thread != NULL;
#else
	s->thread_valid = pthread_create(&s->thread, NULL, worker_entry, s) == 0;
#endif
}

static void stop_worker(struct camlink *s)
{
	if (!s->thread_valid)
		return;
	os_atomic_set_bool(&s->stop, true);
#ifdef _WIN32
	WaitForSingleObject(s->thread, INFINITE);
	CloseHandle(s->thread);
#else
	pthread_join(s->thread, NULL);
#endif
	s->thread_valid = false;
}

/* ------------------------------ fuente de OBS ----------------------------- */

static const char *camlink_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("CamLink.Name");
}

static void camlink_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "port", DEFAULT_PORT);
	obs_data_set_default_string(settings, "adb_path", "adb");
	obs_data_set_default_bool(settings, "low_latency", false);
}

static obs_properties_t *camlink_properties(void *unused)
{
	UNUSED_PARAMETER(unused);
	obs_properties_t *props = obs_properties_create();
	obs_properties_add_int(props, "port", obs_module_text("CamLink.Port"), 1024, 65535, 1);
	obs_properties_add_text(props, "adb_path", obs_module_text("CamLink.AdbPath"), OBS_TEXT_DEFAULT);
	obs_properties_add_bool(props, "low_latency", obs_module_text("CamLink.LowLatency"));
	return props;
}

static void camlink_update(void *data, obs_data_t *settings)
{
	struct camlink *s = data;

	stop_worker(s);

	bfree(s->adb_path);
	s->adb_path = bstrdup(obs_data_get_string(settings, "adb_path"));
	s->port = (int)obs_data_get_int(settings, "port");
	s->low_latency = obs_data_get_bool(settings, "low_latency");

	start_worker(s);
}

static void *camlink_create(obs_data_t *settings, obs_source_t *source)
{
	struct camlink *s = bzalloc(sizeof(*s));
	s->source = source;

	/* Sin búfer: mostrar el cuadro en cuanto llega (menor latencia). */
	obs_source_set_async_unbuffered(source, true);

	camlink_update(s, settings);
	return s;
}

static void camlink_destroy(void *data)
{
	struct camlink *s = data;
	stop_worker(s);
	bfree(s->adb_path);
	bfree(s);
}

static struct obs_source_info camlink_info = {
	.id = "camlink_source",
	.type = OBS_SOURCE_TYPE_INPUT,
	.output_flags = OBS_SOURCE_ASYNC_VIDEO | OBS_SOURCE_DO_NOT_DUPLICATE,
	.icon_type = OBS_ICON_TYPE_CAMERA,
	.get_name = camlink_name,
	.create = camlink_create,
	.destroy = camlink_destroy,
	.update = camlink_update,
	.get_defaults = camlink_defaults,
	.get_properties = camlink_properties,
};

bool obs_module_load(void)
{
#ifdef _WIN32
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		obs_log(LOG_ERROR, "WSAStartup falló");
		return false;
	}
#endif
	obs_register_source(&camlink_info);
	obs_log(LOG_INFO, "plugin loaded successfully (version %s)", PLUGIN_VERSION);
	return true;
}

void obs_module_unload(void)
{
#ifdef _WIN32
	WSACleanup();
#endif
	obs_log(LOG_INFO, "plugin unloaded");
}
