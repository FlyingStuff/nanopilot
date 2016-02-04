#include "rate_limiter.h"
#include "timestamp_mock.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(RateLimiterTestGroup)
{
    parameter_namespace_t param;
    rate_limiter_t rl;

    void setup()
    {
        timestamp_mock_set_time_s(100);
        parameter_namespace_declare(&param, NULL, NULL);
        rate_limiter_init(&rl, "test", &param);
    }
};

TEST(RateLimiterTestGroup, InitRegistersParameter)
{
    parameter_t *p = parameter_find(&param, "test");
    CHECK(p != NULL);
}

TEST(RateLimiterTestGroup, RateParameterHasDefault)
{
    parameter_t *p = parameter_find(&param, "test");
    CHECK(parameter_defined(p));
    CHECK_EQUAL(0, parameter_scalar_get(p));
}

TEST(RateLimiterTestGroup, InitZeroTime)
{
    CHECK_EQUAL(0, rl.last_update);
}

TEST(RateLimiterTestGroup, ZeroRateNeverRuns)
{
    parameter_scalar_set(parameter_find(&param, "test"), 0);
    CHECK_EQUAL(false, rate_limiter_should_run(&rl));
    timestamp_mock_set_time_s(100); // let some time pass
    CHECK_EQUAL(false, rate_limiter_should_run(&rl));
}

TEST(RateLimiterTestGroup, Rate1Hz)
{
    parameter_scalar_set(parameter_find(&param, "test"), 1.0f);
    CHECK_EQUAL(true, rate_limiter_should_run(&rl)); // first run
    timestamp_mock_inc_time_s(0.5);
    CHECK_EQUAL(false, rate_limiter_should_run(&rl)); // don't run after 0.5s
    timestamp_mock_inc_time_s(0.5);
    CHECK_EQUAL(true, rate_limiter_should_run(&rl)); // run again after 1s
}

TEST(RateLimiterTestGroup, Rate100Hz)
{
    parameter_scalar_set(parameter_find(&param, "test"), 100.0f);
    CHECK_EQUAL(true, rate_limiter_should_run(&rl)); // first run
    timestamp_mock_inc_time_s(0.005);
    CHECK_EQUAL(false, rate_limiter_should_run(&rl)); // don't run after 5ms
    timestamp_mock_inc_time_s(0.005);
    CHECK_EQUAL(true, rate_limiter_should_run(&rl)); // run again after 10ms
}
