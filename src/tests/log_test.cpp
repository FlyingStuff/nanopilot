#include "../log.h"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

TEST_GROUP(log)
{
    void teardown() {
        mock().clear();
    }
};


#define CL_BLUE "\033[1;34m"
#define CL_ORANGE "\033[1;33m"
#define CL_RED "\033[1;31m"
#define CL_CLEAR "\033[0m"


extern "C" {
size_t log_format_message(char *buf,
    size_t buf_sz,
    uint64_t time_us,
    log_level_t lvl,
    const char *module,
    const char *msg,
    ...);
}

TEST(log, format)
{
    char log_buffer[100];
    uint64_t time = 42123456;
    log_level_t lvl = LOG_LVL_WARNING;
    const char *module = "test";
    const char *msg = "critical value %d";
    int arg1 = 23;
    size_t len = log_format_message(log_buffer,
        sizeof(log_buffer),
        time,
        lvl,
        module,
        msg,
        arg1);
    CHECK_EQUAL(len, strlen(log_buffer));
    STRCMP_EQUAL(CL_BLUE "[   42.123456]" CL_CLEAR " [" CL_ORANGE "WARNING" CL_CLEAR "] test: critical value 23\n", log_buffer);
}

TEST(log, format_string_too_long_clamps)
{
    char log_buffer[62];
    uint64_t time = 42123456;
    log_level_t lvl = LOG_LVL_WARNING;
    const char *module = "test";
    const char *msg = "critical value %d";
    int arg1 = 23;
    size_t len = log_format_message(log_buffer,
        sizeof(log_buffer),
        time,
        lvl,
        module,
        msg,
        arg1);
    CHECK_EQUAL(sizeof(log_buffer) -1, strlen(log_buffer));
    CHECK_EQUAL(sizeof(log_buffer) -1, len);
    STRCMP_EQUAL(CL_BLUE "[   42.123456]" CL_CLEAR " [" CL_ORANGE "WARNING" CL_CLEAR "] test: critica\n", log_buffer);
}


void log_cb(log_level_t lvl, const char *msg, size_t len)
{
    mock().actualCall("log_cb")
        .withParameter("lvl", lvl)
        .withParameter("msg", msg)
        .withParameter("len", len);
}

TEST(log, handler_called)
{
    const char *msg = "log test";
    mock().expectOneCall("log_cb")
        .withParameter("lvl", LOG_LVL_ERROR)
        .withParameter("msg", msg)
        .withParameter("len", strlen(msg));
    log_init();
    log_handler_t handler;
    log_handler_register(&handler, LOG_LVL_WARNING, log_cb);
    log_call_handlers(LOG_LVL_ERROR, msg, strlen(msg));

    mock().checkExpectations();
}

TEST(log, handler_not_called_below_log_level)
{
    const char *msg = "log test";

    log_init();
    log_handler_t handler;
    log_handler_register(&handler, LOG_LVL_WARNING, log_cb);
    log_call_handlers(LOG_LVL_INFO, msg, strlen(msg));

    mock().checkExpectations();
}

