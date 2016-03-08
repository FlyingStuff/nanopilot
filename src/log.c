#include <hal.h>
#include <chprintf.h>
#include "timestamp/timestamp.h"
#include "main.h"

#include "log.h"


MUTEX_DECL(log_lock);

static void write_entry_format(const char *loglevel)
{
    const char *thread_name = "";
    if (ch.rlist.r_current != NULL && ch.rlist.r_current->p_name != NULL) {
        thread_name = ch.rlist.r_current->p_name;
    }

    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;
    chprintf(stdout, LOG_COLOR_BLUE "[%4d.%06d] %s: %s" LOG_COLOR_CLEAR,
             s, us, thread_name, loglevel);
}


void log_message(const char *lvl, const char *fmt, ...)
{
    chMtxLock(&log_lock);

    va_list args;
    write_entry_format(lvl);

    va_start(args, fmt);
    chvprintf(stdout, fmt, args);
    va_end(args);

    chprintf(stdout, "\n");

    chMtxUnlock(&log_lock);
}
