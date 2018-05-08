#include "CppUTest/TestHarness.h"
#include "timestamp.h"

TEST_GROUP(Timestamp)
{

};

TEST(Timestamp, DurationPositive)
{
    timestamp_t t1 = 10;
    timestamp_t t2 = 22;
    CHECK_EQUAL(12, timestamp_duration_ns(t1, t2));
}

TEST(Timestamp, DurationNegative)
{
    timestamp_t t1 = 22;
    timestamp_t t2 = 10;
    CHECK_EQUAL(-12, timestamp_duration_ns(t1, t2));
}

TEST(Timestamp, DurationFloatSeconds)
{
    timestamp_t t1 = 100000000;
    timestamp_t t2 = 200000000;
    DOUBLES_EQUAL(0.1f, timestamp_duration(t1, t2), 1e-7);
}

TEST(Timestamp, DurationNegativeFloatSeconds)
{
    timestamp_t t1 = 200000000;
    timestamp_t t2 = 100000000;
    DOUBLES_EQUAL(-0.1f, timestamp_duration(t1, t2), 1e-7);
}

