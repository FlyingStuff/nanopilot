#include <ch.h>
#include "log.h"
#include <string.h>
#include <stdio.h>
#include <ff.h>
#include <assert.h>
#include "msgbus/msgbus.h"
#include "ts/serialization_csv.h"
#include "msgbus_scheduler.h"

#include "sdlog.h"


static void file_write(struct logfile_s *l, char *buf, size_t size)
{
    if (!l->_fd_valid) {
        return;
    }
    UINT _bytes_written;
    int ret = f_write(&l->_fd, buf, size, &_bytes_written);
    if (ret != FR_OK) {
        log_error("log for %s write failed: %d", l->topic, ret);
    }
}

void sdlog_filename_from_logdir_and_topic(char *buffer,
    size_t size,
    const char *dir,
    const char *topic)
{
    if (topic[0] == '/') {
        topic++; // ignore leading '/' from topic
    }
    const char *ext = ".csv";
    size_t dir_len = strlen(dir);
    size_t topic_len = strlen(topic);
    size_t ext_len = strlen(ext);
    if (dir_len + 1 + topic_len + ext_len + 1 > size) {
        buffer[0] = '\0';
        return;
    }
    memcpy(buffer, dir, dir_len);
    buffer[dir_len] = '/';
    memcpy(&buffer[dir_len+1], topic, topic_len+1); // copy topic with \0 terminator
    char *topic_c = &buffer[dir_len+1];
    while (*topic_c != '\0') {
        if (*topic_c == '/') {
            *topic_c = '_';
        }
        topic_c++;
    }
    memcpy(&buffer[dir_len+1+topic_len], ext, ext_len+1); // copy ext with \0 terminator
}

static int topic_serialize_csv_init(struct logfile_s *l, msgbus_t *bus, const char *log_dir)
{
    char linebuffer[200];
    char *filename = &linebuffer[0];
    if (!msgbus_topic_subscribe(&l->_sub, bus, l->topic, 1000000)) {
        log_warning("topic %s not found", l->topic);
        return -1;
    }
    sdlog_filename_from_logdir_and_topic(filename, sizeof(linebuffer), log_dir, l->topic);
    FRESULT res = f_open(&l->_fd, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res) {
        log_warning("error %d opening %s", res, filename);
        l->_fd_valid = false;
        return -2;
    }
    l->_fd_valid = true;
    msgbus_topic_t *topic = msgbus_subscriber_get_topic(&l->_sub);
    const ts_type_definition_t *type = msgbus_topic_get_type(topic);
    int bytes_written = ts_serialize_csv_header(type, linebuffer, sizeof(linebuffer));
    if (bytes_written > 0) {
        file_write(l, linebuffer, bytes_written);
    }
    return 0;
}

static void topic_serialize_csv(struct logfile_s *l) {
    char linebuffer[200];
    if (msgbus_subscriber_topic_is_valid(&l->_sub)) {
        msgbus_topic_t *topic = msgbus_subscriber_get_topic(&l->_sub);
        const ts_type_definition_t *type = msgbus_topic_get_type(topic);

        void *buf = malloc(type->struct_size);
        if (buf == NULL) {
            log_warning("sdlog, malloc failed");
            return;
        }
        msgbus_subscriber_read(&l->_sub, buf);
        int bytes_written;
        bytes_written = ts_serialize_csv(buf, type, linebuffer, sizeof(linebuffer));
        free(buf);
        if (bytes_written > 0) {
            file_write(l, linebuffer, bytes_written);
        }
    }
}


void sdlog_initialize_and_add_to_scheduler(msgbus_scheduler_t *sched,
        struct logfile_s *log_files,
        unsigned nb_log_files,
        const char *log_dir)
{
    FRESULT res = f_mkdir(log_dir);
    if (!(res == FR_OK || res == FR_EXIST)) {
        log_warning("could not create directory %s", log_dir);
    }
    unsigned i;
    for (i = 0; i < nb_log_files; i++) {
        if (topic_serialize_csv_init(&log_files[i], msgbus_scheduler_get_bus(sched), log_dir) != 0) {
            log_warning("log for %s could not be initialized", log_files[i].topic);
            continue;
        }
        if (!msgbus_scheduler_add_task(sched, &log_files[i]._sub, (void (*)(void *))topic_serialize_csv, &log_files[i])) {
            log_warning("log for %s could not be allocated", log_files[i].topic);
        }
        log_debug("log for %s added", log_files[i].topic);
    }
}

void sdlog_sync(struct logfile_s *log_files, unsigned nb_log_files)
{
    unsigned i;
    for (i = 0; i < nb_log_files; i++) {
        if (log_files[i]._fd_valid) {
            f_sync(&log_files[i]._fd);
        }
    }
}

int sdlog_find_logfile_nbr(const char *path, const char *prefix)
{
    DIR dir;
    FILINFO finfo;
    FRESULT res = f_opendir(&dir, path);

    size_t prefix_len = strlen(prefix);
    int max_log_nbr = 0;
    if (res == FR_OK) {
        while (true) {
            res = f_readdir(&dir, &finfo);
            if (res != FR_OK || finfo.fname[0] == 0) {
                break;
            }
            if (strncmp(prefix, finfo.fname, prefix_len) == 0) { // match prefix
                int log_nbr = atoi(&finfo.fname[prefix_len]);
                if (log_nbr > max_log_nbr) {
                    max_log_nbr = log_nbr;
                }
            }
        }
        f_closedir(&dir);
        return max_log_nbr + 1;
    }
    return -1;
}

int sdlog_find_logfile_dir(const char *path,
    const char *prefix,
    char *logdir,
    size_t logdir_size)
{
    int log_nbr = sdlog_find_logfile_nbr(path, prefix);
    if (log_nbr < 0) {
        return -1;
    }
    int ret = snprintf(logdir, logdir_size, "%s/%s%d", path, prefix, log_nbr);
    if (ret <= 0 || (unsigned)ret >= logdir_size) {
        return -1;
    }
    return log_nbr;
}



static THD_WORKING_AREA(sdlog_wa, 2000);
static THD_FUNCTION(sdlog, arg)
{
    msgbus_t *bus = (msgbus_t*)arg;
    chRegSetThreadName("sdlog");

    static struct logfile_s log_files[] = {
        {.topic="/sensors/mpu6000"},
        {.topic="/sensors/ms5611"},
        {.topic="/sensors/ms4525do"},
    };
#define NB_LOGFILES (sizeof(log_files)/sizeof(struct logfile_s))

    static msgbus_scheduler_t sched;
    static msgbus_scheduler_task_buffer_space_t buf[NB_LOGFILES];
    msgbus_scheduler_init(&sched, bus, buf, NB_LOGFILES);

    static char logdir[100];
    if (sdlog_find_logfile_dir("/", "log_", logdir, sizeof(logdir)) < 0) {
        log_error("could not determine log file directory");
        return;
    }

    sdlog_initialize_and_add_to_scheduler(&sched, log_files, NB_LOGFILES, logdir);

    if (msgbus_scheduler_get_nb_tasks(&sched) == 0) {
        log_warning("no logfiles to log, exiting");
        return;
    }
    int periodic_sync_countdown = 0;
    while (1) {
        msgbus_scheduler_spin(&sched, 10);
        if (periodic_sync_countdown == 0) {
            sdlog_sync(log_files, NB_LOGFILES);
            periodic_sync_countdown = 100;
        }
        periodic_sync_countdown--;
    }

}

void sdlog_start(msgbus_t *bus)
{
    chThdCreateStatic(sdlog_wa, sizeof(sdlog_wa), LOWPRIO, sdlog, bus);
}
