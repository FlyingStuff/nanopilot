#include <stdint.h>
#include "types/test.h"
#include "../serialization_csv.h"
#include <CppUTest/TestHarness.h>


TEST_GROUP(CSVSerializationTests)
{
    char buffer[100];
};

TEST(CSVSerializationTests, SerializeSimpleHeader)
{
    const char *exp = "x,y\n";
    int ret = msgbus_serialize_csv_header(&simple_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeHeaderBufferOverflowFails)
{
    buffer[3] = '*';
    int ret = msgbus_serialize_csv_header(&simple_type, buffer, 3);
    CHECK_EQUAL(-1, ret);
    CHECK_EQUAL('*', buffer[3]); // not overwritten
}


TEST(CSVSerializationTests, SerializeNestedHeader)
{
    const char *exp = "n.s.x,n.s.y\n";
    int ret = msgbus_serialize_csv_header(&doublenested_type, buffer, sizeof(buffer));
    CHECK_EQUAL((int)strlen(exp), ret);
    STRCMP_EQUAL(exp, buffer);
}

TEST(CSVSerializationTests, SerializeHeaderBufferOverflowFailsAllLenghts)
{
    int required_len = strlen("n.s.x,n.s.y\n");
    int n;
    for (n = 0; n < required_len; n++) {
        buffer[n] = '*';
        int ret = msgbus_serialize_csv_header(&doublenested_type, buffer, n);
        CHECK_EQUAL(-1, ret);
        CHECK_EQUAL('*', buffer[n]); // not overwritten
    }
}

