#define _CRT_SECURE_NO_WARNINGS
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static rl_log_level_t g_log_level = RL_LOG_INFO;
static FILE *g_log_stream = NULL;
static bool g_log_owns_stream = false;

static const char *rl_log_level_name(rl_log_level_t level) {
    switch (level) {
        case RL_LOG_DEBUG: return "DEBUG";
        case RL_LOG_INFO:  return "INFO";
        case RL_LOG_WARN:  return "WARN";
        case RL_LOG_ERROR: return "ERROR";
        default:           return "LOG";
    }
}

void rl_log_set_level(rl_log_level_t level) {
    g_log_level = level;
}

void rl_log_set_file(const char *path) {
    if (!path || !*path) {
        return;
    }

    FILE *f = fopen(path, "a");
    if (!f) {
        fprintf(stderr, "[WARN] Failed to open log file: %s\n", path);
        return;
    }

    if (g_log_owns_stream && g_log_stream) {
        fclose(g_log_stream);
    }

    g_log_stream = f;
    g_log_owns_stream = true;
}

void rl_log_close(void) {
    if (g_log_owns_stream && g_log_stream) {
        fclose(g_log_stream);
    }
    g_log_stream = NULL;
    g_log_owns_stream = false;
}

void rl_log_vmessage(rl_log_level_t level, const char *fmt, va_list args) {
    if (level < g_log_level) {
        return;
    }

    FILE *out = g_log_stream ? g_log_stream : stderr;
    time_t now = time(NULL);
    struct tm tm_buf;
    struct tm *tm_info = NULL;
#ifdef _WIN32
    localtime_s(&tm_buf, &now);
    tm_info = &tm_buf;
#else
    tm_info = localtime(&now);
#endif

    if (tm_info) {
        char ts[32];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm_info);
        fprintf(out, "[%s][%s] ", ts, rl_log_level_name(level));
    } else {
        fprintf(out, "[%s] ", rl_log_level_name(level));
    }

    vfprintf(out, fmt, args);
    fputc('\n', out);
    fflush(out);
}

void rl_log_message(rl_log_level_t level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    rl_log_vmessage(level, fmt, args);
    va_end(args);
}

void die(const char *msg) {
    rl_log_message(RL_LOG_ERROR, "%s", msg);
    rl_log_close();
    exit(1);
}
