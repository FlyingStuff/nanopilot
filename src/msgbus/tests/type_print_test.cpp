#include <CppUTest/TestHarness.h>
#include "../type_definition.h"
#include "../type_print.h"

typedef struct simple_s {
    float x;
    int32_t y;
    char str[20+1];
} simple_t;

const static messagebus_type_entry_t simple_entries[] = {
    {
        .name = "x",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(simple_t, x),
        .base_type = MESSAGEBUS_TYPE_FLOAT32,
    },
    {
        .name = "y",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(simple_t, y),
        .base_type = MESSAGEBUS_TYPE_INT32,
    },
    {
        .name = "str",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(simple_t, str),
        .base_type = MESSAGEBUS_TYPE_STRING,
        .size=20+1
    }
};

const static messagebus_type_definition_t simple_type = {
    .nb_elements = 3,
    .elements = simple_entries,
    .struct_size = sizeof(simple_t),
};

typedef struct nested_s {
    int32_t x;
    simple_t simple;
} nested_t;

const static messagebus_type_entry_t nested_entries[] = {
    {
        .name = "x",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(simple_t, x),
        .base_type = MESSAGEBUS_TYPE_INT32,
    },
    {
        .name = "simple",
        .is_base_type = 0,
        .is_array = 0,
        .is_dynamic_array = 0,
        .struct_offset = offsetof(nested_t, simple),
        .type = &simple_type
    }
};

const static messagebus_type_definition_t nested_type = {
    .nb_elements = 2,
    .elements = nested_entries,
    .struct_size = sizeof(nested_t),
};

extern "C"
void print_fn(void *p, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char **buf = (char **)p;
    int n = vsprintf(*buf, fmt, ap);
    if (n > 0) {
        *buf += n;
    }
    va_end(ap);
}

TEST_GROUP(TypePrintTestGroup)
{
    char buffer[1000];
    void *arg;

    void setup(void)
    {
        memset(buffer, 0, sizeof(buffer));
        arg = &buffer;
    }
};

TEST(TypePrintTestGroup, CanPrintInt32)
{
    bool ret;
    messagebus_type_entry_t entry = {
        .name = "int",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_INT32,
        .struct_offset = 0,
    };
    int32_t object = 123;
    ret = messagebus_print_entry(print_fn, &arg, &entry, &object, 0);
    CHECK_TRUE(ret)
    STRCMP_EQUAL("int: 123\n", buffer);
}

TEST(TypePrintTestGroup, CanPrintFloat32)
{
    bool ret;
    messagebus_type_entry_t entry = {
        .name = "float",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_FLOAT32,
        .struct_offset = 0,
    };
    float object = 2.5;
    ret = messagebus_print_entry(print_fn, &arg, &entry, &object, 0);
    CHECK_TRUE(ret)
    STRCMP_EQUAL("float: 2.500000\n", buffer);
}

TEST(TypePrintTestGroup, CanPrintString)
{
    bool ret;
    messagebus_type_entry_t entry = {
        .name = "string",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 0,
        .base_type = MESSAGEBUS_TYPE_STRING,
        .struct_offset = 0,
        .size = 5,
    };
    const char object[] = "CVRA";
    ret = messagebus_print_entry(print_fn, &arg, &entry, &object, 0);
    CHECK_TRUE(ret)
    STRCMP_EQUAL("string: \"CVRA\"\n", buffer);
}

TEST(TypePrintTestGroup, CanPrintSimpleType)
{
    simple_t object = {.x = 1.5f, .y = 42, .str = "hello world!"};
    messagebus_print_type(print_fn, &arg, &simple_type, &object);
    STRCMP_EQUAL(
        "x: 1.500000\n"
        "y: 42\n"
        "str: \"hello world!\"\n",
        buffer);
}

TEST(TypePrintTestGroup, CanPrintNestedType)
{
    nested_t object = {.x = 42, .simple = {.x = 1.0f, .y = 123, .str = "foo"}};
    messagebus_print_type(print_fn, &arg, &nested_type, &object);
    STRCMP_EQUAL(
        "x: 42\n"
        "simple:\n"
        "    x: 1.000000\n"
        "    y: 123\n"
        "    str: \"foo\"\n",
        buffer);
}

typedef struct {
    int32_t x[4];
    uint16_t x_len;
    char strings[2][10];
} arrays_t;

const static messagebus_type_entry_t arrays_entries[] = {
    {
        .name = "x",
        .is_base_type = 1,
        .is_array = 0,
        .is_dynamic_array = 1,
        .array_len = 4,
        .dynamic_array_len_struct_offset = offsetof(arrays_t, x_len),
        .struct_offset = offsetof(arrays_t, x),
        .base_type = MESSAGEBUS_TYPE_INT32,
        .size = sizeof(int32_t)
    },
    {
        .name = "strings",
        .is_base_type = 1,
        .is_array = 1,
        .is_dynamic_array = 0,
        .array_len = 2,
        .struct_offset = offsetof(arrays_t, strings),
        .base_type = MESSAGEBUS_TYPE_STRING,
        .size = 10
    }
};

const static messagebus_type_definition_t arrays_type = {
    .nb_elements = 2,
    .elements = arrays_entries,
    .struct_size = sizeof(arrays_t),
};

TEST(TypePrintTestGroup, CanPrintArrayType)
{
    arrays_t object = {.x = {1,2,0,0}, .x_len = 2, .strings = {"hello", "world"}};
    messagebus_print_type(print_fn, &arg, &arrays_type, &object);
    STRCMP_EQUAL(
        "x: [1, 2]\n"
        "strings: [\"hello\", \"world\"]\n",
        buffer);
}

typedef struct {
    simple_t arr[3];
} arrays_of_structs_t;

const static messagebus_type_entry_t arrays_of_structs_entries[] = {
    {
        .name = "arr",
        .is_base_type = 0,
        .is_array = 1,
        .is_dynamic_array = 0,
        .array_len = 3,
        .struct_offset = offsetof(arrays_of_structs_t, arr),
        .type = &simple_type,
        .size = sizeof(simple_t)
    },
};

const static messagebus_type_definition_t arrays_of_structs_type = {
    .nb_elements = 1,
    .elements = arrays_of_structs_entries,
    .struct_size = sizeof(arrays_of_structs_t),
};

TEST(TypePrintTestGroup, CanPrintArraysOfStructs)
{
    arrays_of_structs_t object = {.arr = {
        {.x = 0.0f, .y = 0, .str = "a"},
        {.x = 1.0f, .y = 1, .str = "b"},
        {.x = 2.0f, .y = 2, .str = "c"}
    }};
    messagebus_print_type(print_fn, &arg, &arrays_of_structs_type, &object);
    STRCMP_EQUAL(
        "arr: [\n"
        "    x: 0.000000\n"
        "    y: 0\n"
        "    str: \"a\"\n"
        "\n"
        "    x: 1.000000\n"
        "    y: 1\n"
        "    str: \"b\"\n"
        "\n"
        "    x: 2.000000\n"
        "    y: 2\n"
        "    str: \"c\"\n"
        "]\n",
        buffer);
}
