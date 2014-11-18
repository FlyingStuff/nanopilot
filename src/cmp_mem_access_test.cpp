#include "CppUTest/TestHarness.h"
#include "cmp_mem_access.h"
#include "cmp/cmp.h"

TEST_GROUP(CMPMemAccess)
{
    cmp_mem_access_t ma;
    cmp_ctx_t cmp;
};

TEST(CMPMemAccess, Read)
{
    static char testbuf[] = {0x93, 0x01, 0x02, 0x03}; // [1, 2, 3]
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    bool err = false;
    uint32_t arr_size;
    err = !cmp_read_array(&cmp, &arr_size);
    CHECK(!err);
    CHECK_EQUAL(3, arr_size);
    int32_t i;
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(1, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(2, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(3, i);
}

TEST(CMPMemAccess, ReadOverBufferEnd)
{
    static char testbuf[] = {0x93, 0x01, 0x02, 0x03}; // [1, 2, 3]
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf)-1);
    bool err = false;
    uint32_t arr_size;
    err = !cmp_read_array(&cmp, &arr_size);
    CHECK(!err);
    CHECK_EQUAL(3, arr_size);
    int32_t i;
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(1, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(2, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(err); // error must be set (end of buffer)
}
