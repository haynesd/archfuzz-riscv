/*
===============================================================================
File        : rigol_client.c
Language    : C
Target      : Linux / Windows
Purpose     : Minimal SCPI waveform client for Rigol DS1000Z-class scopes

What it does
-------------------------------------------------------------------------------
- Connects to a scope over TCP/IP
- Sends SCPI commands
- Configures waveform transfer
- Reads waveform preamble and binary block data
- Converts samples to volts
- Computes a simple energy-like metric

Build
-------------------------------------------------------------------------------
Linux:
  gcc -O2 -Wall -Wextra -o rigol_client rigol_client.c -lm

Windows (MinGW-w64):
  gcc -O2 -Wall -Wextra -o rigol_client.exe rigol_client.c -lws2_32 -lm

Example
-------------------------------------------------------------------------------
  ./rigol_client 192.168.1.55 5555 CHAN1

Notes
-------------------------------------------------------------------------------
- This uses a raw TCP socket transport.
- Some installations use different SCPI transports/ports.
- Verify the correct port and exact command set for your instrument/firmware.
===============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef SOCKET socket_t;
  #define CLOSESOCK closesocket
  #define SOCK_ERR INVALID_SOCKET
  #define LAST_SOCK_ERR WSAGetLastError()
#else
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netdb.h>
  typedef int socket_t;
  #define CLOSESOCK close
  #define SOCK_ERR (-1)
  #define LAST_SOCK_ERR errno
#endif

#define RECV_BUF_SZ 8192
#define MAX_PREAMBLE 512
#define MAX_LINE 4096

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
} wav_preamble_t;

static void die(const char *msg) {
    fprintf(stderr, "ERROR: %s\n", msg);
    exit(1);
}

static void die_sock(const char *msg) {
#ifdef _WIN32
    fprintf(stderr, "ERROR: %s (WSA=%d)\n", msg, LAST_SOCK_ERR);
#else
    fprintf(stderr, "ERROR: %s (%s)\n", msg, strerror(LAST_SOCK_ERR));
#endif
    exit(1);
}

//=============================================================================
//  TCP/SCPI Helpers
static void net_init(void) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        die("WSAStartup failed");
    }
#endif
}

//  Cleanup network resources (Windows WSACleanup)
static void net_cleanup(void) {
#ifdef _WIN32
    WSACleanup();
#endif
}

// Connect to a TCP server at the given host and port, returning a socket
static socket_t tcp_connect(const char *host, const char *port) {
    struct addrinfo hints, *res = NULL, *rp = NULL;
    socket_t sock = SOCK_ERR;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int rc = getaddrinfo(host, port, &hints, &res);
    if (rc != 0) {
#ifdef _WIN32
        fprintf(stderr, "getaddrinfo failed: %d\n", rc);
#else
        fprintf(stderr, "getaddrinfo failed: %s\n", gai_strerror(rc));
#endif
        exit(1);
    }

    for (rp = res; rp != NULL; rp = rp->ai_next) {
        sock = (socket_t)socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock == SOCK_ERR) {
            continue;
        }
        if (connect(sock, rp->ai_addr, (int)rp->ai_addrlen) == 0) {
            break;
        }
        CLOSESOCK(sock);
        sock = SOCK_ERR;
    }

    freeaddrinfo(res);

    if (sock == SOCK_ERR) {
        die_sock("connect failed");
    }

    return sock;
}

// Send the full buffer to the socket, handling partial sends
static void send_all(socket_t sock, const void *buf, size_t len) {
    const char *p = (const char *)buf;
    while (len > 0) {
#ifdef _WIN32
        int n = send(sock, p, (int)len, 0);
#else
        ssize_t n = send(sock, p, len, 0);
#endif
        if (n <= 0) {
            die_sock("send failed");
        }
        p += n;
        len -= (size_t)n;
    }
}

// Send a SCPI command (string with newline) to the instrument
static void scpi_write(socket_t sock, const char *cmd) {
    send_all(sock, cmd, strlen(cmd));
    send_all(sock, "\n", 1);
}

// Read a single byte from the socket, handling errors
static int recv_one(socket_t sock, char *c) {
#ifdef _WIN32
    int n = recv(sock, c, 1, 0);
#else
    ssize_t n = recv(sock, c, 1, 0);
#endif
    if (n < 0) die_sock("recv failed");
    return (int)n;
}

// Read a line of text from the socket, up to max-1 chars, null-terminated
static int scpi_readline(socket_t sock, char *out, size_t max) {
    size_t n = 0;
    while (n + 1 < max) {
        char c;
        int r = recv_one(sock, &c);
        if (r == 0) break;
        out[n++] = c;
        if (c == '\n') break;
    }
    out[n] = '\0';
    return (int)n;
}

// Read a SCPI response line and trim trailing newlines
static void trim_eol(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == '\n' || s[n-1] == '\r')) {
        s[--n] = '\0';
    }
}

// Send a SCPI query (command + newline, then read response line)
static void scpi_query(socket_t sock, const char *cmd, char *out, size_t max) {
    scpi_write(sock, cmd);
    if (scpi_readline(sock, out, max) <= 0) {
        die("empty SCPI response");
    }
    trim_eol(out);
}

// Read an exact number of bytes from the socket into the buffer
static int read_exact(socket_t sock, void *buf, size_t len) {
    char *p = (char *)buf;
    size_t got = 0;
    while (got < len) {
#ifdef _WIN32
        int n = recv(sock, p + got, (int)(len - got), 0);
#else
        ssize_t n = recv(sock, p + got, len - got, 0);
#endif
        if (n < 0) die_sock("recv failed");
        if (n == 0) return 0;
        got += (size_t)n;
    }
    return 1;
}

//=============================================================================
// SCPI Binary Block Reader
// IEEE 488.2 definite-length binary block:
// #<digits><len><payload>
//
// Example:
//   #900001200<1200 bytes of data>
//==============================================================================
static uint8_t *scpi_read_binblock(socket_t sock, size_t *out_len) {
    char hash;
    if (!read_exact(sock, &hash, 1) || hash != '#') {
        die("expected binary block header '#'");
    }

    char ndig_ch;
    if (!read_exact(sock, &ndig_ch, 1)) {
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
    if (!read_exact(sock, lenbuf, (size_t)ndig)) {
        die("failed to read binary block length");
    }

    size_t payload_len = (size_t)strtoull(lenbuf, NULL, 10);
    uint8_t *data = (uint8_t *)malloc(payload_len);
    if (!data) die("malloc failed for waveform buffer");

    if (!read_exact(sock, data, payload_len)) {
        free(data);
        die("failed to read binary block payload");
    }

    /* Many instruments end with '\n'. Drain one byte if present. */
    char maybe_nl;
#ifdef _WIN32
    int n = recv(sock, &maybe_nl, 1, MSG_PEEK);
#else
    ssize_t n = recv(sock, &maybe_nl, 1, MSG_PEEK);
#endif
    if (n == 1 && (maybe_nl == '\n' || maybe_nl == '\r')) {
        recv_one(sock, &maybe_nl);
    }

    *out_len = payload_len;
    return data;
}

//=============================================================================
static wav_preamble_t parse_preamble(const char *s) {
    wav_preamble_t p;
    memset(&p, 0, sizeof(p));

    /* Typical format:
       format,type,points,count,xincr,xorig,xref,yincr,yorig,yref
    */
    int matched = sscanf(s,
        "%d,%d,%ld,%ld,%lf,%lf,%ld,%lf,%lf,%ld",
        &p.format,
        &p.type,
        &p.points,
        &p.count,
        &p.xincr,
        &p.xorig,
        &p.xref,
        &p.yincr,
        &p.yorig,
        &p.yref
    );

    if (matched != 10) {
        die("failed to parse waveform preamble");
    }

    return p;
}

// Convert a raw sample byte to volts using the preamble scaling factors
static double sample_to_volts(uint8_t raw, const wav_preamble_t *p) {
    /* Common waveform conversion form used by many DS-class scopes:
       volts = (raw - yref - yorig) * yincr
    */
    return (((double)raw) - (double)p->yref - p->yorig) * p->yincr;
}

//=============================================================================
// Simple energy-like metric.
// If V is shunt voltage and R is constant, this is proportional to current^2 energy.
// For quick comparisons, the constant scale factor often cancels out.
//==============================================================================
static double compute_energy_proxy(const double *v, size_t n, double dt) {

    double e = 0.0;
    for (size_t i = 0; i < n; i++) {
        e += v[i] * v[i] * dt;
    }
    return e;
}

// Compute the absolute area under the voltage curve, as a simple proxy for total signal magnitude
static double compute_abs_area(const double *v, size_t n, double dt) {
    double a = 0.0;
    for (size_t i = 0; i < n; i++) {
        a += fabs(v[i]) * dt;
    }
    return a;
}

// Save time-voltage data to a CSV file for external analysis/plotting
static void save_csv(const char *path, const double *t, const double *v, size_t n) {
    FILE *f = fopen(path, "w");
    if (!f) die("failed to open CSV output");
    fprintf(f, "time_s,volts\n");
    for (size_t i = 0; i < n; i++) {
        fprintf(f, "%.12e,%.12e\n", t[i], v[i]);
    }
    fclose(f);
}


int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr,
            "Usage: %s <scope_ip> <port> <channel>\n"
            "Example: %s 192.168.1.55 5555 CHAN1\n",
            argv[0], argv[0]);
        return 2;
    }

    const char *scope_ip = argv[1];
    const char *scope_port = argv[2];
    const char *channel = argv[3];

    net_init();
    socket_t sock = tcp_connect(scope_ip, scope_port);

    char line[MAX_LINE];

    /* Identify instrument */
    scpi_query(sock, "*IDN?", line, sizeof(line));
    printf("IDN: %s\n", line);

    /* Stop acquisition so transfer is stable */
    scpi_write(sock, ":STOP");

    /* Set waveform source and transfer mode */
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), ":WAV:SOUR %s", channel);
        scpi_write(sock, cmd);
    }
    scpi_write(sock, ":WAV:MODE RAW");
    scpi_write(sock, ":WAV:FORM BYTE");

    /* Optional acquisition setup - adjust to your experiment */
    scpi_write(sock, ":TRIG:MODE EDGE");
    scpi_write(sock, ":TRIG:EDGE:SOUR CHAN2");
    scpi_write(sock, ":TRIG:EDGE:SLOP POS");

    /* Read waveform preamble */
    scpi_query(sock, ":WAV:PRE?", line, sizeof(line));
    printf("PREAMBLE: %s\n", line);
    wav_preamble_t pre = parse_preamble(line);

    /* Request waveform data */
    scpi_write(sock, ":WAV:DATA?");
    size_t raw_len = 0;
    uint8_t *raw = scpi_read_binblock(sock, &raw_len);

    printf("Received %zu waveform bytes\n", raw_len);

    double *t = (double *)malloc(raw_len * sizeof(double));
    double *v = (double *)malloc(raw_len * sizeof(double));
    if (!t || !v) die("malloc failed for converted waveform");

    for (size_t i = 0; i < raw_len; i++) {
        t[i] = pre.xorig + ((double)i - (double)pre.xref) * pre.xincr;
        v[i] = sample_to_volts(raw[i], &pre);
    }

    double dt = pre.xincr;
    double e_proxy = compute_energy_proxy(v, raw_len, dt);
    double abs_area = compute_abs_area(v, raw_len, dt);

    printf("dt (s/sample): %.12e\n", dt);
    printf("Energy proxy  : %.12e\n", e_proxy);
    printf("Abs area      : %.12e\n", abs_area);

    save_csv("waveform.csv", t, v, raw_len);
    printf("Saved waveform.csv\n");

    free(raw);
    free(t);
    free(v);

    CLOSESOCK(sock);
    net_cleanup();
    return 0;
}