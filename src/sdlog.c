#include <ch.h>
#include <log.h>
#include <string.h>
#include <ff.h>
#include <assert.h>
#include "main.h"
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
    if (ret != 0) {
        log_error("%s write failed: %d", l->logfile, ret);
    }
    if (ret == 9) {
        log_info("reopening file %s", l->logfile, ret);
        f_open(&l->_fd, l->logfile, FA_WRITE);
        f_lseek(&l->_fd, f_size(&l->_fd));
    }
    if (l->_sync_countdown == 0) {
        f_sync(&l->_fd);
        l->_sync_countdown = 100;
    }
    l->_sync_countdown--;
}

static int topic_serialize_csv_init(struct logfile_s *l, msgbus_t *bus)
{
    l->_sync_countdown = 0;
    char linebuffer[200];
    if (!msgbus_topic_subscribe(&l->_sub, bus, l->topic, MSGBUS_TIMEOUT_NEVER)) {
        return -1;
    }
    FRESULT res = f_open(&l->_fd, l->logfile, FA_WRITE | FA_CREATE_ALWAYS);
    if (res) {
        log_warning("error %d opening %s", res, l->logfile);
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
        unsigned nb_log_files)
{
    unsigned i;
    for (i = 0; i < nb_log_files; i++) {
        if (topic_serialize_csv_init(&log_files[i], msgbus_scheduler_get_bus(sched)) != 0) {
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


static THD_WORKING_AREA(sdlog_wa, 2000);
static THD_FUNCTION(sdlog, arg)
{
    (void)arg;
    chRegSetThreadName("sdlog");

    static struct logfile_s log_files[] = {
        {.topic="/sensors/mpu6000", .logfile="/log/mpu6000.csv"},
        {.topic="/sensors/ms5611", .logfile="/log/ms5611.csv"},
        {.topic="/sensors/ms4525do", .logfile="/log/ms4525do.csv"},
    };

    static msgbus_scheduler_t sched;
    static msgbus_scheduler_task_buffer_space_t buf[sizeof(log_files)/sizeof(struct logfile_s)];
    msgbus_scheduler_init(&sched, &bus, buf, sizeof(buf)/sizeof(buf[0]));

    sdlog_initialize_and_add_to_scheduler(&sched, log_files, sizeof(log_files)/sizeof(struct logfile_s));

    while (1) {
        msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_NEVER);
    }

}

void sdlog_start(void)
{
    chThdCreateStatic(sdlog_wa, sizeof(sdlog_wa), LOWPRIO, sdlog, NULL);
}
