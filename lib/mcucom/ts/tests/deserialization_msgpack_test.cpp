#include "../deserialization_msgpack.h"
#include "../serialization_msgpack.h"
#include "types/test.h"
#include "cmp_mem_access/cmp_mem_access.h"

#include <CppUTest/TestHarness.h>


TEST_GROUP(MessagePackEntryDeserializationTests)
{
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    char buf[1000];
    ts_type_entry_t base_type_entry;
    void setup(void)
    {
        memset(buf, 0, sizeof(buf));
        cmp_mem_access_init(&ctx, &mem, buf, sizeof(buf));
        base_type_entry = {
            .name = "var",
            .base_type = TS_TYPE_INT32,
            .is_base_type = 1,
            .is_array = 0,
            .is_dynamic_array = 0,
            .array_len = 0,
            .dynamic_array_len_struct_offset = 0,
            .struct_offset = 0,
            .size = 0
        };
    }
};



TEST(MessagePackEntryDeserializationTests, DeserializeFloat16Value)
{
    base_type_entry.base_type = TS_TYPE_FLOAT16;
    const float var = 3.14;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    float var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeFloat32Value)
{
    base_type_entry.base_type = TS_TYPE_FLOAT32;
    const float var = 3.14;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    float var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeFloat64Value)
{
    base_type_entry.base_type = TS_TYPE_FLOAT64;
    const double var = 3.14;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    double var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeInt8Value)
{
    base_type_entry.base_type = TS_TYPE_INT8;
    int8_t var = INT8_MIN;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    int8_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeInt16Value)
{
    base_type_entry.base_type = TS_TYPE_INT16;
    int16_t var = INT16_MIN;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    int16_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeInt32Value)
{
    base_type_entry.base_type = TS_TYPE_INT32;
    int32_t var = INT32_MIN;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    int32_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeInt64Value)
{
    base_type_entry.base_type = TS_TYPE_INT64;
    int64_t var = INT64_MIN;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    int64_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeUInt8Value)
{
    base_type_entry.base_type = TS_TYPE_UINT8;
    uint8_t var = UINT8_MAX;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    uint8_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeUInt16Value)
{
    base_type_entry.base_type = TS_TYPE_UINT16;
    uint16_t var = UINT16_MAX;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    uint16_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeUInt32Value)
{
    base_type_entry.base_type = TS_TYPE_UINT32;
    uint32_t var = UINT32_MAX;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    uint32_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeUInt64Value)
{
    base_type_entry.base_type = TS_TYPE_UINT64;
    uint64_t var = UINT64_MAX;
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    uint64_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeStringValue)
{
    base_type_entry.base_type = TS_TYPE_STRING;
    base_type_entry.size = 10;
    char str[10] = "test";
    ts_cmp_ser_value(&str, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);
    char str_read[10];
    CHECK_TRUE(ts_cmp_deser_value(&str_read, &base_type_entry, &ctx));
    STRCMP_EQUAL(str, str_read);
}

TEST(MessagePackEntryDeserializationTests, DeserializeStructTypeValue)
{
    ts_type_entry_t entry = {
        .is_base_type = 0,
        .is_array = 0,
        .is_dynamic_array = 0,
        .type = &simple_type,
        .struct_offset = 0,
    };
    simple_t var = {.x = 3.14, .y = 42};
    ts_cmp_ser_value(&var, &entry, &ctx, true);
    cmp_mem_access_set_pos(&mem, 0);
    simple_t var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &entry, &ctx));
    CHECK_EQUAL(var.x, var_read.x);
    CHECK_EQUAL(var.y, var_read.y);
}

TEST(MessagePackEntryDeserializationTests, DeserializeStaticArray)
{
    base_type_entry.base_type = TS_TYPE_INT32;
    base_type_entry.is_array = 1;
    base_type_entry.array_len = 3;
    base_type_entry.size = sizeof(int32_t);
    int32_t var[3] = {1, 2, 3};
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);

    int32_t var_read[3];
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
    CHECK_EQUAL(var[0], var_read[0]);
    CHECK_EQUAL(var[1], var_read[1]);
    CHECK_EQUAL(var[2], var_read[2]);
}

TEST(MessagePackEntryDeserializationTests, DeserializeStaticArrayWrongLength)
{
    base_type_entry.base_type = TS_TYPE_INT32;
    base_type_entry.is_array = 1;
    base_type_entry.array_len = 3;
    base_type_entry.size = sizeof(int32_t);
    int32_t var[3] = {1, 2, 3};
    ts_cmp_ser_value(&var, &base_type_entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);

    int32_t var_read[3];
    base_type_entry.array_len = 2;
    CHECK_FALSE(ts_cmp_deser_value(&var_read, &base_type_entry, &ctx));
}

TEST(MessagePackEntryDeserializationTests, DeserializeDynamicArray)
{
    struct dynamic_array_test_s {
        int32_t list[10];
        uint16_t list_len;
    };
    ts_type_entry_t entry = {
        .is_base_type = 1,
        .is_dynamic_array = 1,
        .base_type = TS_TYPE_INT32,
        .struct_offset = offsetof(dynamic_array_test_s, list),
        .dynamic_array_len_struct_offset = offsetof(dynamic_array_test_s, list_len),
        .array_len = 10,
        .size = sizeof(int32_t),
    };
    struct dynamic_array_test_s var = {.list = {1, 2, 3}, .list_len = 3};
    ts_cmp_ser_value(&var, &entry, &ctx, false);
    cmp_mem_access_set_pos(&mem, 0);

    struct dynamic_array_test_s var_read;
    CHECK_TRUE(ts_cmp_deser_value(&var_read, &entry, &ctx));
    CHECK_EQUAL(3, var_read.list_len);
    CHECK_EQUAL(1, var_read.list[0]);
    CHECK_EQUAL(2, var_read.list[1]);
    CHECK_EQUAL(3, var_read.list[2]);
}

TEST(MessagePackEntryDeserializationTests, DeserializeDynamicArrayMaxSizeCheck)
{
    struct dynamic_array_test_s {
        int32_t list[10];
        uint16_t list_len;
    };
    ts_type_entry_t entry = {
        .is_base_type = 1,
        .is_dynamic_array = 1,
        .base_type = TS_TYPE_INT32,
        .struct_offset = offsetof(dynamic_array_test_s, list),
        .dynamic_array_len_struct_offset = offsetof(dynamic_array_test_s, list_len),
        .array_len = 10,
        .size = sizeof(int32_t),
    };
    struct dynamic_array_test_s var = {.list = {1, 2, 3}, .list_len = 3};
    ts_cmp_ser_value(&var, &entry, &ctx, false);

    entry.array_len = 2;
    struct dynamic_array_test_s var_read;
    CHECK_FALSE(ts_cmp_deser_value(&var_read, &entry, &ctx));
}


TEST_GROUP(MessagePackDeserializationTests)
{
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    char buf[1000];
    void setup(void)
    {
        memset(buf, 0, sizeof(buf));
        cmp_mem_access_init(&ctx, &mem, buf, sizeof(buf));
    }
};

TEST(MessagePackDeserializationTests, DeserializeStructCompact)
{
    simple_t val = {.x = 3.14, .y = 42};
    ts_cmp_ser_type(&val, &simple_type, &ctx, true);
    cmp_mem_access_set_pos(&mem, 0);

    simple_t val_read;
    CHECK_TRUE(ts_cmp_deser_type(&val_read, &simple_type, &ctx));
    CHECK_EQUAL(val.x, val_read.x);
    CHECK_EQUAL(val.y, val_read.y);
}

TEST(MessagePackDeserializationTests, DeserializeNestedStructCompact)
{
    nested_t val = {.s = {.x = 3.14, .y = 42}};
    ts_cmp_ser_type(&val, &nested_type, &ctx, true);
    cmp_mem_access_set_pos(&mem, 0);

    nested_t val_read;
    CHECK_TRUE(ts_cmp_deser_type(&val_read, &nested_type, &ctx));
    CHECK_EQUAL(val.s.x, val_read.s.x);
    CHECK_EQUAL(val.s.y, val_read.s.y);
}
