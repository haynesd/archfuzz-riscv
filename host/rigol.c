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

/*
-------------------------------------------------------------------------------
rigol_die_wsa
-------------------------------------------------------------------------------
Logs a Winsock-specific fatal error and terminates the process.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rigol_tcp_connect
-------------------------------------------------------------------------------
Creates a TCP connection to the configured scope endpoint.
-------------------------------------------------------------------------------
*/
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

    rl_log_message(RL_LOG_INFO, "Connected to Rigol scope %s:%s", host, port);
    return sock;
}

/*
-------------------------------------------------------------------------------
rigol_send_all
-------------------------------------------------------------------------------
Sends the entire buffer over the socket.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rigol_scpi_write
-------------------------------------------------------------------------------
Writes one SCPI command terminated by a newline.
-------------------------------------------------------------------------------
*/
static void rigol_scpi_write(rigol_socket_t sock, const char *cmd) {
    rigol_send_all(sock, cmd, strlen(cmd));
    rigol_send_all(sock, "\n", 1);
    rl_log_message(RL_LOG_DEBUG, "[SCPI TX] %s", cmd);
}

/*
-------------------------------------------------------------------------------
rigol_recv_one
-------------------------------------------------------------------------------
Reads one byte from the socket.
-------------------------------------------------------------------------------
*/
static int rigol_recv_one(rigol_socket_t sock, char *ch) {
    int rc = recv(sock, ch, 1, 0);
    if (rc < 0) {
        rigol_die_wsa("recv failed");
    }
    return rc;
}

/*
-------------------------------------------------------------------------------
rigol_scpi_readline
-------------------------------------------------------------------------------
Reads a newline-terminated SCPI response line.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rigol_trim_eol
-------------------------------------------------------------------------------
Removes trailing CR/LF characters from a response string.
-------------------------------------------------------------------------------
*/
static void rigol_trim_eol(char *text) {
    size_t n = strlen(text);
    while (n > 0 && (text[n - 1] == '\n' || text[n - 1] == '\r')) {
        text[--n] = '\0';
    }
}

/*
-------------------------------------------------------------------------------
rigol_scpi_query
-------------------------------------------------------------------------------
Sends one SCPI query and receives its single-line response.
-------------------------------------------------------------------------------
*/
static void rigol_scpi_query(rigol_socket_t sock, const char *cmd, char *out, size_t max_len) {
    rigol_scpi_write(sock, cmd);
    if (rigol_scpi_readline(sock, out, max_len) <= 0) {
        die("empty SCPI response");
    }
    rigol_trim_eol(out);
    rl_log_message(RL_LOG_DEBUG, "[SCPI RX] %s", out);
}

/*
-------------------------------------------------------------------------------
rigol_read_exact
-------------------------------------------------------------------------------
Reads an exact byte count from the scope transport.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rigol_read_binblock
-------------------------------------------------------------------------------
Reads an IEEE 488.2 definite-length binary block containing waveform samples.
-------------------------------------------------------------------------------
*/
static uint8_t *rigol_read_binblock(rigol_socket_t sock, size_t *payload_len_out) {
    char hash;
    if (!rigol_read_exact(sock, &hash, 1) || hash != '#') {
        die("expected binary block header '#'");
    }

    char ndig_ch;
    if (!rigol_read_exact(sock, &ndig_ch, 1)) {
        die("failed to read binary block digit count");
    }
    if (ndig_ch < '0' || ndig_ch > '9') {
        die("invalid binary block digit count");
    }

    int ndig = ndig_ch - '0';
    if (ndig <= 0 || ndig > 9) {
        die("unsupported binary block digit count");
    }

    char lenbuf[16];
    memset(lenbuf, 0, sizeof(lenbuf));
    if (!rigol_read_exact(sock, lenbuf, (size_t)ndig)) {
        die("failed to read binary block length");
    }

    size_t payload_len = (size_t)strtoull(lenbuf, NULL, 10);
    uint8_t *payload = (uint8_t *)malloc(payload_len);
    if (!payload) {
        die("malloc failed for waveform buffer");
    }

    if (!rigol_read_exact(sock, payload, payload_len)) {
        free(payload);
        die("failed to read binary block payload");
    }

    char maybe_nl;
    int peeked = recv(sock, &maybe_nl, 1, MSG_PEEK);
    if (peeked == 1 && (maybe_nl == '\n' || maybe_nl == '\r')) {
        rigol_recv_one(sock, &maybe_nl);
    }

    *payload_len_out = payload_len;
    return payload;
}

/*
-------------------------------------------------------------------------------
rigol_parse_preamble
-------------------------------------------------------------------------------
Parses the waveform preamble returned by the scope.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rigol_sample_to_volts
-------------------------------------------------------------------------------
Converts one raw scope sample into volts using the preamble scaling.
-------------------------------------------------------------------------------
*/
static double rigol_sample_to_volts(uint8_t raw, const rigol_preamble_t *preamble) {
    return (((double)raw) - (double)preamble->yref - preamble->yorig) * preamble->yincr;
}

waveform_t rigol_capture_waveform(const rigol_config_t *config) {
    waveform_t waveform;
    memset(&waveform, 0, sizeof(waveform));

    if (!config || !config->enabled) {
        return waveform;
    }

    rigol_socket_t sock = rigol_tcp_connect(config->scope_ip, config->scope_port);
    char line[RIGOL_MAX_LINE];
    char cmd[128];

    rigol_scpi_query(sock, "*IDN?", line, sizeof(line));
    rl_log_message(RL_LOG_INFO, "[SCOPE] IDN: %s", line);

    rigol_scpi_write(sock, ":STOP");
    snprintf(cmd, sizeof(cmd), ":WAV:SOUR %s", config->channel);
    rigol_scpi_write(sock, cmd);
    rigol_scpi_write(sock, ":WAV:MODE RAW");
    rigol_scpi_write(sock, ":WAV:FORM BYTE");
    rigol_scpi_write(sock, ":TRIG:MODE EDGE");
    rigol_scpi_write(sock, ":TRIG:EDGE:SOUR CHAN2");
    rigol_scpi_write(sock, ":TRIG:EDGE:SLOP POS");

    rigol_scpi_query(sock, ":WAV:PRE?", line, sizeof(line));
    rigol_preamble_t preamble = rigol_parse_preamble(line);

    rigol_scpi_write(sock, ":WAV:DATA?");
    size_t raw_len = 0;
    uint8_t *raw = rigol_read_binblock(sock, &raw_len);

    waveform.time_s = (double *)malloc(raw_len * sizeof(double));
    waveform.volts = (double *)malloc(raw_len * sizeof(double));
    if (!waveform.time_s || !waveform.volts) {
        free(raw);
        die("malloc failed for converted waveform");
    }

    waveform.sample_count = raw_len;
    waveform.dt_s = preamble.xincr;

    for (size_t i = 0; i < raw_len; ++i) {
        waveform.time_s[i] = preamble.xorig + ((double)i - (double)preamble.xref) * preamble.xincr;
        waveform.volts[i] = rigol_sample_to_volts(raw[i], &preamble);
    }

    waveform.metrics = waveform_compute_metrics(waveform.volts, waveform.sample_count, waveform.dt_s);
    rl_log_message(RL_LOG_INFO,
                   "Captured waveform: samples=%zu dt=%.12e energy=%.12e area=%.12e rms=%.12e peak=%.12e",
                   waveform.sample_count,
                   waveform.dt_s,
                   waveform.metrics.energy_proxy,
                   waveform.metrics.abs_area,
                   waveform.metrics.rms,
                   waveform.metrics.peak_abs);

    free(raw);
    RIGOL_CLOSESOCK(sock);
    return waveform;
}
