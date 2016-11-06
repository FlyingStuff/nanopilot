#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <CppUTest/TestHarness.h>
#include "../serialization_msgpack.h"

extern "C" {
#include "cmp/cmp.h"
}
#include "cmp_mem_access/cmp_mem_access.h"

extern "C" {
bool messagebus_cmp_ser_type(const void *var,
                             const messagebus_type_definition_t *type,
                             cmp_ctx_t *ctx,
                             bool compact);
bool messagebus_cmp_ser_struct_entry(const void *var,
                                     const messagebus_type_entry_t *entry,
                                     cmp_ctx_t *ctx,
                                     bool compact);
bool messagebus_cmp_ser_value(const void *var,
                              const messagebus_type_entry_t *entry,
                              cmp_ctx_t *ctx,
                              bool compact);
bool messagebus_cmp_ser_value_once(const void *var,
                                   const messagebus_type_entry_t *entry,
                                   cmp_ctx_t *ctx,
                                   bool compact);
}


struct simple_s {
    float x;
    int32_t y;
};

messagebus_type_entry_t simple_entries[] = {
    {
        .name = "x",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(struct simple_s, x),
        .base_type = MESSAGEBUS_TYPE_FLOAT32,
    },
    {
        .name = "y",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(struct simple_s, y),
        .base_type = MESSAGEBUS_TYPE_INT32,
    }
};

messagebus_type_definition_t simple_type = {
    .nb_elements = 2,
    .elements = simple_entries,
};


struct nested_s {
    struct simple_s s;
};

messagebus_type_entry_t nested_entries[] = {
    {
        .name = "s",
        .is_base_type = 0,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(struct nested_s, s),
        .type = &simple_type,
    }
};

messagebus_type_definition_t nested_type = {
    .nb_elements = 1,
    .elements = nested_entries,
};


TEST_GROUP(MessagePackSerializationTests)
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


TEST(MessagePackSerializationTests, SerializeFloatValue)
{
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_FLOAT32,
        .struct_offset = 0,
    };
    float var = 3.14;
    CHECK_TRUE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    float var_read;
    CHECK_TRUE(cmp_read_float(&ctx, &var_read));
    CHECK_EQUAL(var, var_read);
}


TEST(MessagePackSerializationTests, SerializeInt32Value)
{
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = 0,
    };
    int32_t var = 42;
    CHECK_TRUE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    int32_t var_read;
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var, var_read);
}

TEST(MessagePackSerializationTests, SerializeStringValue)
{
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_STRING,
        .struct_offset = 0,
        .size = 10
    };
    char str[10] = "test";
    CHECK_TRUE(messagebus_cmp_ser_value(&str, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    memset(str, 0, sizeof(str));
    uint32_t len = sizeof(str);
    CHECK_TRUE(cmp_read_str(&ctx, &str[0], &len));
    STRCMP_EQUAL("test", str);
    CHECK_EQUAL(4, len);
}


TEST(MessagePackSerializationTests, SerializeCustomTypeValue)
{
    messagebus_type_entry_t entry = {
        .is_base_type = 0,
        .is_array = 0,
        .is_dynamic_array = 0,
        .type = &simple_type,
        .struct_offset = 0,
    };
    struct simple_s var = {.x = 3.14, .y = 42};
    CHECK_TRUE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    uint32_t nb_elements;
    CHECK_TRUE(cmp_read_map(&ctx, &nb_elements));
    CHECK_EQUAL(2, nb_elements);
}


TEST(MessagePackSerializationTests, SerializeStaticArray)
{
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 1,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = 0,
        .array_len = 3,
        .size = sizeof(int32_t),
    };
    int32_t var[3] = {1, 2, 3};
    CHECK_TRUE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    int32_t var_read;
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var[0], var_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var[1], var_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var[2], var_read);
}


TEST(MessagePackSerializationTests, SerializeDynamicArray)
{
    struct dynamic_array_test_s {
        int32_t list[10];
        int list_len;
    };
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 1,
        .is_dynamic_array = 1,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = offsetof(dynamic_array_test_s, list),
        .dynamic_array_len_struct_offset = offsetof(dynamic_array_test_s, list_len),
        .array_len = 10,
        .size = sizeof(int32_t),
    };
    struct dynamic_array_test_s var = {.list = {1, 2, 3}, .list_len = 3};
    CHECK_TRUE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    int32_t var_read;
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var.list[0], var_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var.list[1], var_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    CHECK_EQUAL(var.list[2], var_read);
}


TEST(MessagePackSerializationTests, SerializeDynamicArrayMaxSizeCheck)
{
    struct dynamic_array_test_s {
        int32_t list[10];
        int list_len;
    };
    messagebus_type_entry_t entry = {
        .is_base_type = 1,
        .is_array = 1,
        .is_dynamic_array = 1,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = offsetof(dynamic_array_test_s, list),
        .dynamic_array_len_struct_offset = offsetof(dynamic_array_test_s, list_len),
        .array_len = 10,
        .size = sizeof(int32_t),
    };
    struct dynamic_array_test_s var = {.list_len = 11};
    CHECK_FALSE(messagebus_cmp_ser_value(&var, &entry, &ctx, false));
}


TEST(MessagePackSerializationTests, SerializeStructEntry)
{
    messagebus_type_entry_t entry = {
        .name = "var",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = 0,
    };
    int32_t var = 42;
    CHECK_TRUE(messagebus_cmp_ser_struct_entry(&var, &entry, &ctx, false));
    cmp_mem_access_set_pos(&mem, 0);
    char name_read_buf[10];
    uint32_t name_read_buf_sz = sizeof(name_read_buf);
    int32_t var_read;
    CHECK_TRUE(cmp_read_str(&ctx, name_read_buf, &name_read_buf_sz));
    CHECK_TRUE(cmp_read_int(&ctx, &var_read));
    STRCMP_EQUAL(entry.name, name_read_buf);
    CHECK_EQUAL(var, var_read);
}

// todo buffer too short tests


TEST(MessagePackSerializationTests, SerializeStruct)
{
    struct simple_s val = {.x = 3.14, .y = 42};
    CHECK_TRUE(messagebus_cmp_ser_type(&val, &simple_type, &ctx, false));

    cmp_mem_access_set_pos(&mem, 0);
    uint32_t nb_elements;
    char name_read_buf[10];
    uint32_t name_read_buf_sz = sizeof(name_read_buf);
    float var_f_read;
    int32_t var_i_read;
    CHECK_TRUE(cmp_read_map(&ctx, &nb_elements));

    CHECK_EQUAL(2, nb_elements);
    CHECK_TRUE(cmp_read_str(&ctx, name_read_buf, &name_read_buf_sz));
    CHECK_TRUE(cmp_read_float(&ctx, &var_f_read));
    STRCMP_EQUAL("x", name_read_buf);
    CHECK_EQUAL(val.x, var_f_read);

    name_read_buf_sz = sizeof(name_read_buf);
    CHECK_TRUE(cmp_read_str(&ctx, name_read_buf, &name_read_buf_sz));
    CHECK_TRUE(cmp_read_int(&ctx, &var_i_read));
    STRCMP_EQUAL("y", name_read_buf);
    CHECK_EQUAL(val.y, var_i_read);
}


TEST(MessagePackSerializationTests, SerializeStructCompact)
{
    struct simple_s val = {.x = 3.14, .y = 42};
    CHECK_TRUE(messagebus_cmp_ser_type(&val, &simple_type, &ctx, true));

    cmp_mem_access_set_pos(&mem, 0);
    uint32_t nb_elements;
    float var_f_read;
    int32_t var_i_read;
    CHECK_TRUE(cmp_read_array(&ctx, &nb_elements));
    CHECK_EQUAL(2, nb_elements);
    CHECK_TRUE(cmp_read_float(&ctx, &var_f_read));
    CHECK_EQUAL(val.x, var_f_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_i_read));
    CHECK_EQUAL(val.y, var_i_read);
}

TEST(MessagePackSerializationTests, SerializeNestedStructCompact)
{
    struct nested_s val = {.s = {.x = 3.14, .y = 42}};
    CHECK_TRUE(messagebus_cmp_ser_type(&val, &nested_type, &ctx, true));

    cmp_mem_access_set_pos(&mem, 0);
    uint32_t nb_elements;
    float var_f_read;
    int32_t var_i_read;
    CHECK_TRUE(cmp_read_array(&ctx, &nb_elements));
    CHECK_EQUAL(1, nb_elements);
    CHECK_TRUE(cmp_read_array(&ctx, &nb_elements));
    CHECK_EQUAL(2, nb_elements);
    CHECK_TRUE(cmp_read_float(&ctx, &var_f_read));
    CHECK_EQUAL(val.s.x, var_f_read);
    CHECK_TRUE(cmp_read_int(&ctx, &var_i_read));
    CHECK_EQUAL(val.s.y, var_i_read);
}

