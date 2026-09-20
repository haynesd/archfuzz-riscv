#include "rigol.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
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
 * Socket-level recv/send timeout for the scope link. Without this, a dropped
 * network connection or a scope that stops responding mid-capture blocks the
 * host process forever with no diagnostic, which is fatal for an unattended
 * multi-hour/day fuzzing campaign.
 */
#define RIGOL_SOCK_TIMEOUT_MS 15000

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

    DWORD timeout_ms = RIGOL_SOCK_TIMEOUT_MS;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) != 0 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) != 0) {
        rl_log_message(RL_LOG_WARN, "Failed to set scope socket timeout (WSA=%d)", WSAGetLastError());
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

/*
 * Determines which 1-indexed sample number in the scope's internal
 * acquisition memory corresponds to the trigger point (t=0), by taking a
 * throwaway preamble reading with :WAV:STAR pinned to 1 so xorigin reports
 * the timestamp of the very first memory sample. Without this, callers have
 * no way to aim :WAV:STAR/:WAV:STOP at the trigger - the scope's default
 * window (samples 1-1200) reads from the start of memory, which is nowhere
 * near the trigger once memory depth exceeds a few thousand points.
 */
static long rigol_compute_trigger_index(rigol_socket_t sock, const char *reference_channel, long *out_points) {
    char cmd[128];
    char line[RIGOL_MAX_LINE];

    if (out_points) {
        *out_points = 1200;
    }

    snprintf(cmd, sizeof(cmd), ":WAV:SOUR %s", reference_channel);
    rigol_scpi_write(sock, cmd);
    rigol_scpi_write(sock, ":WAV:MODE RAW");
    rigol_scpi_write(sock, ":WAV:FORM BYTE");
    rigol_scpi_write(sock, ":WAV:STAR 1");
    rigol_scpi_write(sock, ":WAV:STOP 1200");
    rigol_scpi_query(sock, ":WAV:PRE?", line, sizeof(line));

    rigol_preamble_t preamble = rigol_parse_preamble(line);
    if (out_points) {
        *out_points = preamble.points;
    }
    if (preamble.xincr <= 0.0) {
        return 1;
    }

    /* preamble.xorig is the timestamp of memory sample #1 (STAR was pinned to
       1 above); the trigger sits at t=0, so solve for the 1-indexed sample
       number whose time is closest to zero. */
    double idx0 = 1.0 - preamble.xorig / preamble.xincr;
    long trig_index = (long)(idx0 + 0.5);
    if (trig_index < 1) {
        trig_index = 1;
    }
    if (preamble.points > 0 && trig_index > preamble.points) {
        trig_index = preamble.points;
    }
    return trig_index;
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
                   "Captured %s: samples=%zu dt=%.12e energy=%.12e peak=%.6f V",
                   channel,
                   wave.sample_count,
                   wave.dt_s,
                   wave.metrics.energy_proxy,
                   wave.metrics.peak_abs);
    return wave;
}

struct rigol_session {
    rigol_socket_t sock;
    const rigol_config_t *config;
};

rigol_session_t *rigol_arm_single_capture(const rigol_config_t *config) {
    if (!config || !config->enabled) {
        return NULL;
    }

    rigol_socket_t sock = rigol_tcp_connect(config->scope_ip, config->scope_port);
    char idn[RIGOL_MAX_LINE];

    rigol_scpi_query(sock, "*IDN?", idn, sizeof(idn));
    rl_log_message(RL_LOG_INFO, "Rigol IDN: %s", idn);

    if (config->timebase_scale_s > 0.0) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), ":TIM:SCAL %.9e", config->timebase_scale_s);
        rigol_scpi_write(sock, cmd);
    }
    if (config->timebase_offset_s != 0.0) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), ":TIM:OFFS %.9e", config->timebase_offset_s);
        rigol_scpi_write(sock, cmd);
    }

    /*
     * A single-shot acquisition leaves the scope in STOP once it completes.
     * Sending :SING again while it is still sitting in STOP was observed to
     * be silently ignored - every subsequent iteration just kept re-serving
     * the one acquisition captured way back before this state was ever
     * reached, no matter how many :SING commands followed. :RUN is what
     * actually gets it out of STOP, but firing :RUN immediately followed by
     * :SING with no gap doesn't reliably work either (that transition isn't
     * instantaneous) - so :RUN is sent, then :TRIG:STATUS? is polled until
     * it actually reports something other than STOP, and only then is
     * :SING sent to arm the next capture.
     */
    rigol_scpi_write(sock, ":RUN");
    {
        char status[RIGOL_MAX_LINE];
        int attempt;
        for (attempt = 0; attempt < 50; ++attempt) {
            rigol_scpi_query(sock, ":TRIG:STATUS?", status, sizeof(status));
            if (strncmp(status, "STOP", 4) != 0) {
                break;
            }
            Sleep(20);
        }
        if (attempt == 50) {
            rl_log_message(RL_LOG_WARN, "Scope still reports STOP after :RUN; arming single-shot anyway");
        }
    }
    rigol_scpi_write(sock, ":SING");

    rl_log_message(RL_LOG_INFO, "Rigol scope armed for single-shot acquisition");

    /*
     * Deliberately keep this connection open and hand it back to the caller
     * rather than closing it here: closing the TCP session right after
     * arming :SING and reopening a fresh one later to read the result was
     * observed to leave the scope permanently stuck re-serving one stale
     * acquisition, capture after capture, regardless of trigger source/level/
     * wiring - i.e. the arm silently never survives a connection close. A
     * session held open through the FPGA/board sequence and into the
     * eventual :STOP + read does not have this problem.
     */
    rigol_session_t *session = (rigol_session_t *)malloc(sizeof(rigol_session_t));
    if (!session) {
        RIGOL_CLOSESOCK(sock);
        die("malloc failed for rigol session");
    }
    session->sock = sock;
    session->config = config;
    return session;
}

void rigol_session_abandon(rigol_session_t *session) {
    if (!session) {
        return;
    }
    RIGOL_CLOSESOCK(session->sock);
    free(session);
}

waveform_capture_set_t rigol_capture_scope_set(rigol_session_t *session) {
    waveform_capture_set_t capture;
    memset(&capture, 0, sizeof(capture));

    if (!session) {
        return capture;
    }

    rigol_socket_t sock = session->sock;
    const rigol_config_t *config = session->config;

    rigol_scpi_write(sock, ":STOP");

    long total_points = 1200;
    long trig_index = rigol_compute_trigger_index(sock, config->trigger_channel, &total_points);

    /* Bracket the trigger sample with the caller's requested pre/post margin
       plus a little slack for real-world trigger jitter/propagation delay,
       then pin :WAV:STAR/:WAV:STOP to that range so every channel's
       :WAV:DATA? below actually reads memory around the trigger instead of
       the scope's default (start-of-memory) window. */
    long slack = 200;
    long pre = (long)config->pre_trigger_samples + slack;
    long post = (long)config->window_samples + slack;

    long star = trig_index - pre;
    if (star < 1) {
        star = 1;
    }
    long stop = trig_index + post;
    if (total_points > 0 && stop > total_points) {
        stop = total_points;
    }
    if (stop < star) {
        stop = star;
    }

    char star_cmd[64];
    char stop_cmd[64];
    snprintf(star_cmd, sizeof(star_cmd), ":WAV:STAR %ld", star);
    snprintf(stop_cmd, sizeof(stop_cmd), ":WAV:STOP %ld", stop);
    rigol_scpi_write(sock, star_cmd);
    rigol_scpi_write(sock, stop_cmd);

    rl_log_message(RL_LOG_INFO,
                   "Waveform capture window: trigger_index=%ld star=%ld stop=%ld (of %ld points)",
                   trig_index, star, stop, total_points);

    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        capture.power[i] = rigol_capture_channel_on_socket(sock, config->power_channels[i]);
    }
    capture.trigger = rigol_capture_channel_on_socket(sock, config->trigger_channel);
    capture.valid = capture.trigger.metrics.valid;

    RIGOL_CLOSESOCK(sock);
    free(session);
    return capture;
}
