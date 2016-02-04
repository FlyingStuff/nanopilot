#include <timestamp/timestamp.h>
#include "timestamp_mock.h"
#include "CppUTest/TestHarness.h"

#define TIMESTAMP_RES 1000000 // timestamp is in us

TEST_GROUP(TimestampMockTest)
{

};

TEST(TimestampMockTest, SetRaw)
{
    timestamp_mock_set_time_raw(42);
    CHECK_EQUAL(42, timestamp_get());
}

TEST(TimestampMockTest, SetSeconds)
{
    timestamp_mock_set_time_s(42);
    CHECK_EQUAL(42*TIMESTAMP_RES, timestamp_get());
}

TEST(TimestampMockTest, IncRaw)
{
    timestamp_mock_set_time_raw(100);
    timestamp_mock_inc_time_raw(100);
    CHECK_EQUAL(200, timestamp_get());
}

TEST(TimestampMockTest, IncSeconds)
{
    timestamp_mock_set_time_s(1);
    timestamp_mock_inc_time_s(1);
    CHECK_EQUAL(2*TIMESTAMP_RES, timestamp_get());
}

TEST(TimestampMockTest, DurationWorks)
{
    timestamp_mock_set_time_s(100);
    timestamp_t start = timestamp_get();
    timestamp_mock_inc_time_s(23);
    timestamp_t end = timestamp_get();
    CHECK_EQUAL(23, timestamp_duration_s(start, end));
}
