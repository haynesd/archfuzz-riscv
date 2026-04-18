#include "rigol.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif

typedef SOCKET rigol_socket_t;

typedef struct {
    int format;
    int type;
    long points;
    long count;
    double xincr;
    double xorig;
    long xref;
    double yincr;
    double yorig;
    long yref;
} rigol_preamble_t;

#define RIGOL_CLOSESOCK closesocket
#define RIGOL_SOCK_ERR INVALID_SOCKET
#define RIGOL_MAX_LINE 4096

static void rigol_die_wsa(const char *msg) {
    rl_log_message(RL_LOG_ERROR, "%s (WSA=%d)", msg, WSAGetLastError());
    rl_log_close();
    exit(1);
}

void rigol_net_init(void) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        die("WSAStartup failed");
    }
    rl_log_message(RL_LOG_INFO, "Rigol network stack initialized");
}

void rigol_net_cleanup(void) {
    WSACleanup();
    rl_log_message(RL_LOG_INFO, "Rigol network stack cleaned up");
}

static rigol_socket_t rigol_tcp_connect(const char *host, const char *port) {
    struct addrinfo hints, *results = NULL, *rp = NULL;
    rigol_socket_t sock = RIGOL_SOCK_ERR;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int rc = getaddrinfo(host, port, &hints, &results);
    if (rc != 0) {
        rl_log_message(RL_LOG_ERROR, "getaddrinfo failed: %d", rc);
        exit(1);
    }

    for (rp = results; rp != NULL; rp = rp->ai_next) {
        sock = (rigol_socket_t)socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock == RIGOL_SOCK_ERR) {
            continue;
        }
        if (connect(sock, rp->ai_addr, (int)rp->ai_addrlen) == 0) {
            break;
        }
        RIGOL_CLOSESOCK(sock);
        sock = RIGOL_SOCK_ERR;
    }

    freeaddrinfo(results);

    if (sock == RIGOL_SOCK_ERR) {
        rigol_die_wsa("scope connect failed");
    }

    return sock;
}

static void rigol_send_all(rigol_socket_t sock, const void *buffer, size_t length) {
    const char *p = (const char *)buffer;
    while (length > 0) {
        int sent = send(sock, p, (int)length, 0);
        if (sent <= 0) {
            rigol_die_wsa("send failed");
        }
        p += sent;
        length -= (size_t)sent;
    }
}

static void rigol_scpi_write(rigol_socket_t sock, const char *cmd) {
    rigol_send_all(sock, cmd, strlen(cmd));
    rigol_send_all(sock, "\n", 1);
    rl_log_message(RL_LOG_DEBUG, "[SCPI TX] %s", cmd);
}

static int rigol_recv_one(rigol_socket_t sock, char *ch) {
    int rc = recv(sock, ch, 1, 0);
    if (rc < 0) {
        rigol_die_wsa("recv failed");
    }
    return rc;
}

static int rigol_scpi_readline(rigol_socket_t sock, char *out, size_t max_len) {
    size_t count = 0;
    while (count + 1 < max_len) {
        char ch;
        int rc = rigol_recv_one(sock, &ch);
        if (rc == 0) {
            break;
        }
        out[count++] = ch;
        if (ch == '\n') {
            break;
        }
    }
    out[count] = '\0';
    return (int)count;
}

static void rigol_trim_eol(char *text) {
    size_t n = strlen(text);
    while (n > 0 && (text[n - 1] == '\n' || text[n - 1] == '\r')) {
        text[--n] = '\0';
    }
}

static void rigol_scpi_query(rigol_socket_t sock, const char *cmd, char *out, size_t max_len) {
    rigol_scpi_write(sock, cmd);
    if (rigol_scpi_readline(sock, out, max_len) <= 0) {
        die("empty SCPI response");
    }
    rigol_trim_eol(out);
    rl_log_message(RL_LOG_DEBUG, "[SCPI RX] %s", out);
}

static int rigol_read_exact(rigol_socket_t sock, void *buffer, size_t length) {
    char *p = (char *)buffer;
    size_t received = 0;
    while (received < length) {
        int rc = recv(sock, p + received, (int)(length - received), 0);
        if (rc < 0) {
            rigol_die_wsa("recv failed");
        }
        if (rc == 0) {
            return 0;
        }
        received += (size_t)rc;
    }
    return 1;
}

static uint8_t *rigol_read_binblock(rigol_socket_t sock, size_t *out_length) {
    char hash = 0;
    if (!rigol_read_exact(sock, &hash, 1) || hash != '#') {
        die("expected binary block header '#'");
    }

    char digits_char = 0;
    if (!rigol_read_exact(sock, &digits_char, 1)) {
        die("failed to read binary block digit count");
    }

    if (digits_char < '0' || digits_char > '9') {
        die("invalid binary block digit count");
    }

    const int digits = digits_char - '0';
    if (digits <= 0 || digits > 9) {
        die("unsupported binary block digit count");
    }

    char length_buffer[16];
    memset(length_buffer, 0, sizeof(length_buffer));
    if (!rigol_read_exact(sock, length_buffer, (size_t)digits)) {
        die("failed to read binary block payload length");
    }

    size_t payload_length = (size_t)strtoull(length_buffer, NULL, 10);
    uint8_t *data = (uint8_t *)malloc(payload_length);
    if (!data) {
        die("malloc failed for waveform payload");
    }

    if (!rigol_read_exact(sock, data, payload_length)) {
        free(data);
        die("failed to read binary block payload");
    }

    char maybe_newline = 0;
    int peeked = recv(sock, &maybe_newline, 1, MSG_PEEK);
    if (peeked == 1 && (maybe_newline == '\n' || maybe_newline == '\r')) {
        (void)rigol_recv_one(sock, &maybe_newline);
    }

    *out_length = payload_length;
    return data;
}

static rigol_preamble_t rigol_parse_preamble(const char *text) {
    rigol_preamble_t preamble;
    memset(&preamble, 0, sizeof(preamble));

    int matched = sscanf(text,
                         "%d,%d,%ld,%ld,%lf,%lf,%ld,%lf,%lf,%ld",
                         &preamble.format,
                         &preamble.type,
                         &preamble.points,
                         &preamble.count,
                         &preamble.xincr,
                         &preamble.xorig,
                         &preamble.xref,
                         &preamble.yincr,
                         &preamble.yorig,
                         &preamble.yref);
    if (matched != 10) {
        die("failed to parse waveform preamble");
    }

    return preamble;
}

static double rigol_sample_to_volts(uint8_t raw, const rigol_preamble_t *preamble) {
    return (((double)raw) - (double)preamble->yref - preamble->yorig) * preamble->yincr;
}

static waveform_t rigol_capture_channel_on_socket(rigol_socket_t sock, const char *channel) {
    char line[RIGOL_MAX_LINE];
    char cmd[128];

    waveform_t wave;
    memset(&wave, 0, sizeof(wave));

    snprintf(cmd, sizeof(cmd), ":WAV:SOUR %s", channel);
    rigol_scpi_write(sock, cmd);
    rigol_scpi_write(sock, ":WAV:MODE RAW");
    rigol_scpi_write(sock, ":WAV:FORM BYTE");
    rigol_scpi_query(sock, ":WAV:PRE?", line, sizeof(line));

    rigol_preamble_t preamble = rigol_parse_preamble(line);

    rigol_scpi_write(sock, ":WAV:DATA?");
    size_t raw_length = 0;
    uint8_t *raw = rigol_read_binblock(sock, &raw_length);

    wave.time_s = (double *)malloc(raw_length * sizeof(double));
    wave.volts = (double *)malloc(raw_length * sizeof(double));
    if (!wave.time_s || !wave.volts) {
        free(raw);
        waveform_free(&wave);
        die("malloc failed for waveform buffers");
    }

    for (size_t i = 0; i < raw_length; ++i) {
        wave.time_s[i] = preamble.xorig + ((double)i - (double)preamble.xref) * preamble.xincr;
        wave.volts[i] = rigol_sample_to_volts(raw[i], &preamble);
    }

    wave.sample_count = raw_length;
    wave.dt_s = preamble.xincr;
    wave.metrics = waveform_compute_metrics(wave.volts, wave.sample_count, wave.dt_s);

    free(raw);
    rl_log_message(RL_LOG_INFO,
                   "Captured %s: samples=%zu dt=%.12e energy=%.12e",
                   channel,
                   wave.sample_count,
                   wave.dt_s,
                   wave.metrics.energy_proxy);
    return wave;
}

void rigol_arm_single_capture(const rigol_config_t *config) {
    if (!config || !config->enabled) {
        return;
    }

    rigol_socket_t sock = rigol_tcp_connect(config->scope_ip, config->scope_port);
    char idn[RIGOL_MAX_LINE];

    rigol_scpi_query(sock, "*IDN?", idn, sizeof(idn));
    rl_log_message(RL_LOG_INFO, "Rigol IDN: %s", idn);

    rigol_scpi_write(sock, ":RUN");
    rigol_scpi_write(sock, ":SING");
    RIGOL_CLOSESOCK(sock);

    rl_log_message(RL_LOG_INFO, "Rigol scope armed for single-shot acquisition");
}

waveform_capture_set_t rigol_capture_scope_set(const rigol_config_t *config) {
    waveform_capture_set_t capture;
    memset(&capture, 0, sizeof(capture));

    if (!config || !config->enabled) {
        return capture;
    }

    rigol_socket_t sock = rigol_tcp_connect(config->scope_ip, config->scope_port);
    char idn[RIGOL_MAX_LINE];

    rigol_scpi_query(sock, "*IDN?", idn, sizeof(idn));
    rl_log_message(RL_LOG_INFO, "Rigol IDN: %s", idn);

    rigol_scpi_write(sock, ":STOP");

    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        capture.power[i] = rigol_capture_channel_on_socket(sock, config->power_channels[i]);
    }
    capture.trigger = rigol_capture_channel_on_socket(sock, config->trigger_channel);
    capture.valid = capture.trigger.metrics.valid;

    RIGOL_CLOSESOCK(sock);
    return capture;
}
