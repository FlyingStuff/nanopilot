#include <hal.h>
#include <chprintf.h>
#include <memstreams.h>
#include "timestamp/timestamp.h"
#include "main.h"

#include "log.h"

MUTEX_DECL(log_lock);

static void write_entry_format(BaseSequentialStream *stream, const char *loglevel)
{
    const char *thread_name = "";
    if (ch.rlist.r_current != NULL && ch.rlist.r_current->p_name != NULL) {
        thread_name = ch.rlist.r_current->p_name;
    }

    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;
    chprintf(stream, LOG_COLOR_BLUE "[%4d.%06d] %s: %s" LOG_COLOR_CLEAR,
             s, us, thread_name, loglevel);
}


void log_message(const char *lvl, const char *fmt, ...)
{
    chMtxLock(&log_lock);
    static uint8_t writebuf[200];
    static MemoryStream writebuf_stream;
    msObjectInit(&writebuf_stream, writebuf, sizeof(writebuf), 0);

    va_list args;
    write_entry_format((BaseSequentialStream *)&writebuf_stream, lvl);

    va_start(args, fmt);
    chvprintf((BaseSequentialStream *)&writebuf_stream, fmt, args);
    va_end(args);

    chprintf((BaseSequentialStream *)&writebuf_stream, "\n");

    chSequentialStreamWrite(stdout, writebuf, writebuf_stream.eos);

    chMtxUnlock(&log_lock);
}
