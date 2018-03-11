#ifndef SDLOG_H
#define SDLOG_H

#ifdef __cplusplus
extern "C" {
#endif

void sdlog_start(void);

#ifdef __cplusplus
}
#endif



/* Internal, for integration tests */

#include <ff.h>
#include "msgbus/msgbus.h"
#include "msgbus_scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

struct logfile_s {
    const char *topic;
    const char *logfile;
    FIL _fd;
    bool _fd_valid;
    msgbus_subscriber_t _sub;
};

void sdlog_initialize_and_add_to_scheduler(msgbus_scheduler_t *sched,
        struct logfile_s *log_files,
        unsigned nb_log_files);
void sdlog_sync(struct logfile_s *log_files, unsigned nb_log_files);

#ifdef __cplusplus
}
#endif

#endif /* SDLOG_H */