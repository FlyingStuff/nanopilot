#include <stdint.h>
#include "types/test.h"
#include "../serialization_csv.h"
#include <CppUTest/TestHarness.h>


TEST_GROUP(CSVSerializationTests)
{
    char buffer[100];
    void setup()
    {
        memset(buffer, '*', sizeof(buffer));
    }
};

TEST(CSVSerializationTests, SerializeSimpleHeader)
{
    const char *exp = "x,y\n";
    int ret = ts_serialize_csv_header(&simple_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeHeaderBufferOverflowFails)
{
    buffer[3] = '*';
    int ret = ts_serialize_csv_header(&simple_type, buffer, 3);
    CHECK_EQUAL(-1, ret);
    CHECK_EQUAL('*', buffer[3]); // not overwritten
}


TEST(CSVSerializationTests, SerializeNestedHeader)
{
    const char *exp = "n.s.x,n.s.y\n";
    int ret = ts_serialize_csv_header(&doublenested_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeHeaderBufferOverflowFailsAllLenghts)
{
    int required_len = strlen("n.s.x,n.s.y\n");
    int n;
    for (n = 0; n < required_len; n++) {
        buffer[n] = '*';
        int ret = ts_serialize_csv_header(&doublenested_type, buffer, n);
        CHECK_EQUAL(-1, ret);
        CHECK_EQUAL('*', buffer[n]); // not overwritten
    }
}

TEST(CSVSerializationTests, SerializeHeaderWithArray)
{
    const char *exp = "x[0],x[1],x[2],y[0],y[1]\n";
    int ret = ts_serialize_csv_header(&static_array_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}


TEST(CSVSerializationTests, SerializeHeaderWithDynamicArrayFails)
{
    int ret = ts_serialize_csv_header(&dynamic_array_type, buffer, sizeof(buffer));
    CHECK_EQUAL(-1, ret);
}


TEST(CSVSerializationTests, SerializeValueSimple)
{
    simple_t var = {.x = 3.5, .y = 42};
    const char *exp = "3.500000,42\n";
    int ret = ts_serialize_csv(&var, &simple_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeValueNested)
{
    doublenested_t var = {.n = {.s = {.x = 3.5, .y = 42}}};
    const char *exp = "3.500000,42\n";
    int ret = ts_serialize_csv(&var, &doublenested_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeArray)
{
    static_array_t var = {.x = {1,2,3}, .y = {10,20}};
    const char *exp = "1,2,3,10,20\n";
    int ret = ts_serialize_csv(&var, &static_array_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}


TEST(CSVSerializationTests, SerializeDynamicArrayFails)
{
    dynamic_array_t var;
    int ret = ts_serialize_csv(&var, &dynamic_array_type, buffer, sizeof(buffer));
    CHECK_EQUAL(-1, ret);
}

TEST(CSVSerializationTests, SerializeBufferOverflowFailsAllLenghts)
{
    doublenested_t var = {.n = {.s = {.x = 3.5, .y = 42}}};
    int required_len = strlen("3.500000,42\n");
    int n;
    for (n = 0; n < required_len; n++) {
        buffer[n] = '*';
        int ret = ts_serialize_csv(&var, &doublenested_type, buffer, n);
        CHECK_EQUAL(-1, ret);
        CHECK_EQUAL('*', buffer[n]); // not overwritten
    }
}
