#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>


void log_debug(const char *fmt, ...);
void log_info(const char *fmt, ...);
void log_warning(const char *fmt, ...);
void log_error(const char *fmt, ...);


typedef enum {
    LOG_LVL_DEBUG = 0,
    LOG_LVL_INFO,
    LOG_LVL_WARNING,
    LOG_LVL_ERROR,
} log_level_t;


typedef struct log_handler_s log_handler_t;
struct log_handler_s {
    log_handler_t *next;
    void (*callback)(log_level_t lvl, const char *msg, size_t len);
    log_level_t min_log_lvl;
};

void log_init(void);

void log_handler_register(log_handler_t *handler,
    log_level_t min_log_lvl,
    void (*callback)(log_level_t lvl, const char *msg, size_t len));

void log_handler_change_log_level(log_handler_t *handler, log_level_t min_log_lvl);

void log_call_handlers(log_level_t lvl, const char *msg, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
