#ifndef TS_TYPE_DEFINITION_H
#define TS_TYPE_DEFINITION_H

#include <stdint.h>
#include <stddef.h>

#define TS_TYPE_HASH_SIZE 4

enum ts_base_type_enum {
    TS_TYPE_INT8,
    TS_TYPE_INT16,
    TS_TYPE_INT32,
    TS_TYPE_INT64,
    TS_TYPE_UINT8,
    TS_TYPE_UINT16,
    TS_TYPE_UINT32,
    TS_TYPE_UINT64,
    TS_TYPE_FLOAT16,
    TS_TYPE_FLOAT32,
    TS_TYPE_FLOAT64,
    TS_TYPE_STRING
};


typedef struct ts_type_definition_s ts_type_definition_t;
typedef struct ts_type_entry_s ts_type_entry_t;

struct ts_type_definition_s {
    uint8_t hash[TS_TYPE_HASH_SIZE];
    uint16_t nb_elements;
    const ts_type_entry_t *elements;
    size_t struct_size;
};

struct ts_type_entry_s {
    const char *name;
    union {
        enum ts_base_type_enum base_type;
        const ts_type_definition_t *type;
    };
    uint8_t is_base_type:1;
    uint8_t is_array:1;
    uint8_t is_dynamic_array:1;
    uint16_t array_len; // or max length for dynamic arrays
    size_t dynamic_array_len_struct_offset;
    size_t struct_offset;
    size_t size;
};

#endif /* TS_TYPE_DEFINITION_H */