#include "rate_limiter.h"
#include "CppUTest/TestHarness.h"

TEST_GROUP(RateLimiterTestGroup)
{
    parameter_namespace_t param;
    rate_limiter_t rl;

    void setup()
    {
        parameter_namespace_declare(&param, NULL, NULL);
    }
};

TEST(RateLimiterTestGroup, InitRegistersParameter)
{
    rate_limiter_init(&rl, "test", &param);
    parameter_t *p = parameter_find(&param, "test");
    CHECK(p != NULL);
}

TEST(RateLimiterTestGroup, RateParameterHasDefault)
{
    rate_limiter_init(&rl, "test", &param);
    parameter_t *p = parameter_find(&param, "test");
    CHECK(parameter_defined(p));
    CHECK_EQUAL(0, parameter_scalar_get(p));
}



