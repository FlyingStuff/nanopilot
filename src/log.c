#include <stdarg.h>
#include <inttypes.h>
#include <stdio.h>
#include "log.h"

// internal
size_t log_format_message(char *buf,
    size_t buf_sz,
    uint64_t time_us,
    log_level_t lvl,
    const char *module,
    const char *msg,
    ...);
size_t log_vformat_message(char *buf,
    size_t buf_sz,
    uint64_t time_us,
    log_level_t lvl,
    const char *module,
    const char *msg,
    va_list args);



#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

static void log_v(log_level_t lvl, const char *fmt, va_list args)
{
    static char log_buffer[200];
    uint64_t timestamp = 0;
    const char *module = "unknown";
    size_t len = log_vformat_message(log_buffer, sizeof(log_buffer), timestamp, lvl, module, fmt, args);
    log_call_handlers(lvl, log_buffer, len);
}

#else

#include <hal.h>
#include "timestamp/timestamp.h"

MUTEX_DECL(log_lock);

static const char *thread_name_get(void)
{
    const char *thread_name = chRegGetThreadNameX(chThdGetSelfX());
    if (thread_name == NULL) {
        thread_name = "";
    }
    return thread_name;
}

static void log_v(log_level_t lvl, const char *fmt, va_list args)
{
    static char log_buffer[200];
    chMtxLock(&log_lock);
    size_t len = log_vformat_message(log_buffer, sizeof(log_buffer), timestamp_get(), lvl, thread_name_get(), fmt, args);
    log_call_handlers(lvl, log_buffer, len);
    chMtxUnlock(&log_lock);
}

#endif // ChibiOS port


void log_debug(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_v(LOG_LVL_DEBUG, fmt, args);
    va_end(args);
}

void log_info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_v(LOG_LVL_INFO, fmt, args);
    va_end(args);
}

void log_warning(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_v(LOG_LVL_WARNING, fmt, args);
    va_end(args);
}

void log_error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_v(LOG_LVL_ERROR, fmt, args);
    va_end(args);
}


#define CL_BLUE "\033[1;34m"
#define CL_ORANGE "\033[1;33m"
#define CL_RED "\033[1;31m"
#define CL_CLEAR "\033[0m"

static const char *lvls[] = {
    "DEBUG",
    "INFO",
    CL_ORANGE"WARNING"CL_CLEAR,
    CL_RED"ERROR"CL_CLEAR,
};

static const char *log_fmt_str = CL_BLUE"[%5"PRIu64".%06"PRIu64"]"CL_CLEAR" [%s] %s: ";

size_t log_format_message(char *buf,
    size_t buf_sz,
    uint64_t time_us,
    log_level_t lvl,
    const char *module,
    const char *msg,
    ...)
{
    va_list args;
    va_start(args, msg);
    size_t ret = log_vformat_message(buf, buf_sz, time_us, lvl, module, msg, args);
    va_end(args);
    return ret;
}

size_t log_vformat_message(char *buf,
    size_t buf_sz,
    uint64_t time_us,
    log_level_t lvl,
    const char *module,
    const char *msg,
    va_list args)
{
    size_t bytes_written = 0;
    uint64_t s = time_us / 1000000;
    uint64_t us = time_us - s * 1000000;
    int ret = snprintf(buf, buf_sz, log_fmt_str, s, us, lvls[lvl], module);
    if (ret < 0 || (size_t)ret >= buf_sz) {
        buf[0] = '\0';
        return 0;
    }
    bytes_written += ret;

    ret = vsnprintf(&buf[bytes_written], buf_sz - bytes_written, msg, args);
    if (ret > 0) {
        if ((size_t)ret < buf_sz - bytes_written) {
            bytes_written += ret;
        } else { // not enough space
            bytes_written = buf_sz -2; // just clamp, but leave space for \n\0
        }
    }

    if (buf_sz - bytes_written >= 2) {
        buf[bytes_written++] = '\n';
        buf[bytes_written] = '\0';
    }
    return bytes_written;
}


static log_handler_t *log_handler_list = NULL;

void log_init(void)
{
    log_handler_list = NULL;
}

void log_handler_register(log_handler_t *handler,
    log_level_t min_log_lvl,
    void (*callback)(log_level_t lvl, const char *msg, size_t len))
{
    handler->min_log_lvl = min_log_lvl;
    handler->callback = callback;
    handler->next = log_handler_list;
    log_handler_list = handler;

}

void log_call_handlers(log_level_t lvl, const char *msg, size_t len)
{
    log_handler_t *handler = log_handler_list;
    while (handler != NULL) {
        if (lvl >= handler->min_log_lvl) {
            handler->callback(lvl, msg, len);
        }
        handler = handler->next;
    }
}
