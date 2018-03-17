#ifndef SDLOG_H
#define SDLOG_H

#include "msgbus/msgbus.h"

#ifdef __cplusplus
extern "C" {
#endif

void sdlog_start(msgbus_t *bus);

#ifdef __cplusplus
}
#endif



/* Internal, for integration tests */

#include <ff.h>
#include "msgbus_scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

struct logfile_s {
    const char *topic;
    FIL _fd;
    bool _fd_valid;
    msgbus_subscriber_t _sub;
};

void sdlog_initialize_and_add_to_scheduler(msgbus_scheduler_t *sched,
    struct logfile_s *log_files,
    unsigned nb_log_files,
    const char *log_dir);
void sdlog_sync(struct logfile_s *log_files, unsigned nb_log_files);
void sdlog_filename_from_logdir_and_topic(char *buffer,
    size_t size,
    const char *dir,
    const char *topic);

int sdlog_find_logfile_dir(const char *path,
    const char *prefix,
    char *logdir,
    size_t logdir_size);

#ifdef __cplusplus
}
#endif

#endif /* SDLOG_H */