#include "../sdlog.h"
#include "../log.h"
#include <types/test.h>
#include <iostream>
#include <CppUTest/TestHarness.h>



TEST_GROUP(SDLog)
{

};


TEST(SDLog, log_topic)
{
    // init logging
    log_init();
    log_handler_t stdout_log;
    log_handler_register(&stdout_log, LOG_LVL_WARNING, [](log_level_t lvl, const char *msg, size_t len)
        { std::cout << msg << std::endl;
        });

    // initialize messagebus
    msgbus_t bus;
    msgbus_init(&bus);
    msgbus_topic_t topic;
    test_t topic_buf;
    msgbus_topic_create(&topic, &bus, &test_type, &topic_buf, "/test");

    // create and mount filesystem
    FATFS fs;
    FRESULT mkfs_res = f_mkfs("", FM_ANY, 0, NULL, 0);
    CHECK_EQUAL(FR_OK, mkfs_res);
    FRESULT mount_res = f_mount(&fs, "", 1);
    CHECK_EQUAL(FR_OK, mount_res);

    // initialize logger & scheduler
    msgbus_scheduler_t sched;
    msgbus_scheduler_task_buffer_space_t buf[10];
    msgbus_scheduler_init(&sched, &bus, buf, 10);
    static struct logfile_s log_files[] = {
        {.topic="/test", .logfile="/test.csv"},
    };
    sdlog_initialize_and_add_to_scheduler(&sched, log_files, 1);

    // publish a value and spin the scheduler
    test_t value = {.x = 42};
    msgbus_topic_publish(&topic, &value);
    msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_IMMEDIATE);

    // make sure all files are written
    sdlog_sync(log_files, 1);

    // read the log file
    FIL file;
    FRESULT open_res = f_open(&file, "/test.csv", FA_READ);
    CHECK_EQUAL(FR_OK, open_res);
    char read_buf[1000];
    UINT bytes_read;
    FRESULT read_res = f_read(&file, read_buf, 1000, &bytes_read);
    CHECK_EQUAL(FR_OK, read_res);
    read_buf[bytes_read] = '\0'; // terminate string
    STRCMP_EQUAL("x\n42\n", read_buf);

    FRESULT close_res = f_close(&file);
    CHECK_EQUAL(FR_OK, close_res);
}
