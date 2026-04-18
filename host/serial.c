#define _CRT_SECURE_NO_WARNINGS
#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
-------------------------------------------------------------------------------
serial_dump_bytes_hex
-------------------------------------------------------------------------------
Helper used for debug logging of framed serial traffic.
-------------------------------------------------------------------------------
*/
static void serial_dump_bytes_hex(const uint8_t *buffer, int length, char *out, size_t out_size) {
    size_t used = 0;
    if (!out || out_size == 0) {
        return;
    }
    out[0] = '\0';
    for (int i = 0; i < length; ++i) {
        int rc = snprintf(out + used, out_size - used, "%02X ", buffer[i]);
        if (rc < 0 || (size_t)rc >= out_size - used) {
            break;
        }
        used += (size_t)rc;
    }
}

serial_t serial_open(const char *port_name) {
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", port_name);

    HANDLE handle = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE) {
        die("COM open failed");
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb)) {
        die("GetCommState failed");
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(handle, &dcb)) {
        die("SetCommState failed");
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 200;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(handle, &timeouts)) {
        die("SetCommTimeouts failed");
    }

    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    rl_log_message(RL_LOG_INFO, "Opened serial port %s", port_name);

    serial_t serial = { handle };
    return serial;
}

void serial_close(serial_t *serial) {
    if (serial && serial->handle && serial->handle != INVALID_HANDLE_VALUE) {
        CloseHandle(serial->handle);
        serial->handle = INVALID_HANDLE_VALUE;
        rl_log_message(RL_LOG_INFO, "Closed serial port");
    }
}

/*
-------------------------------------------------------------------------------
serial_write_frame
-------------------------------------------------------------------------------
Builds the FPGA frame and transmits it to the serial link.
-------------------------------------------------------------------------------
*/
static void serial_write_frame(serial_t *serial, int board_index, const char *payload) {
    uint8_t frame[256];
    int payload_length = (int)strlen(payload);

    if (board_index < 0 || board_index >= RL_BOARD_COUNT) {
        die("Board must be 0, 1, or 2");
    }
    if (payload_length + 2 > (int)sizeof(frame)) {
        die("Payload too large");
    }

    frame[0] = 0x01;
    frame[1] = (uint8_t)board_index;
    memcpy(&frame[2], payload, (size_t)payload_length);

    DWORD written = 0;
    if (!WriteFile(serial->handle, frame, (DWORD)(payload_length + 2), &written, NULL) ||
        written != (DWORD)(payload_length + 2)) {
        die("WriteFile failed");
    }

    char hexbuf[1024];
    serial_dump_bytes_hex(frame, payload_length + 2, hexbuf, sizeof(hexbuf));
    rl_log_message(RL_LOG_DEBUG, "[HOST TX][board %d] %s", board_index, hexbuf);
    rl_log_message(RL_LOG_INFO, "[HOST TX ASCII][board %d] %s", board_index, payload);
}

void serial_send_ping(serial_t *serial, int board_index) {
    serial_write_frame(serial, board_index, "PING\n");
}

void serial_send_run(serial_t *serial, int board_index, uint32_t seed, int steps) {
    char payload[128];
    snprintf(payload, sizeof(payload), "RUN %u %d\n", seed, steps);
    serial_write_frame(serial, board_index, payload);
}

/*
-------------------------------------------------------------------------------
serial_readline_internal
-------------------------------------------------------------------------------
Internal blocking line reader built on Win32 ReadFile.
-------------------------------------------------------------------------------
*/
static int serial_readline_internal(serial_t *serial, char *buffer, int max_length) {
    int i = 0;
    while (i < max_length - 1) {
        char ch = 0;
        DWORD bytes_read = 0;

        if (!ReadFile(serial->handle, &ch, 1, &bytes_read, NULL)) {
            return -1;
        }
        if (bytes_read == 0) {
            continue;
        }

        buffer[i++] = ch;
        if (ch == '\n') {
            break;
        }
    }

    buffer[i] = '\0';
    return i;
}

bool serial_read_line(serial_t *serial, char *line, int line_size) {
    int count = serial_readline_internal(serial, line, line_size);
    if (count < 0) {
        die("ReadFile failed");
    }
    if (count > 0) {
        rl_log_message(RL_LOG_INFO, "[HOST RX] %s", line);
    }
    return count != 0;
}

int parse_board_index(const char *text) {
    int board = atoi(text);
    if (board < 0 || board >= RL_BOARD_COUNT) {
        die("Board must be 0, 1, or 2");
    }
    return board;
}

uint32_t parse_u32_decimal(const char *text, const char *what) {
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 10);
    if (end == text || *end != '\0') {
        rl_log_message(RL_LOG_ERROR, "Invalid %s: %s", what, text);
        exit(1);
    }
    return (uint32_t)value;
}

int parse_int_decimal(const char *text, const char *what) {
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        rl_log_message(RL_LOG_ERROR, "Invalid %s: %s", what, text);
        exit(1);
    }
    return (int)value;
}
