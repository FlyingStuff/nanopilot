#ifndef MESSAGEBUS_TYPE_DEFINITION_H
#define MESSAGEBUS_TYPE_DEFINITION_H

#include <stdint.h>
#include <stddef.h>

enum messagebus_base_type_enum {
    MESSAGEBUS_TYPE_INT32,
    MESSAGEBUS_TYPE_FLOAT32,
    MESSAGEBUS_TYPE_STRING
};


typedef struct messagebus_type_definition_s messagebus_type_definition_t;
typedef struct messagebus_type_entry_s messagebus_type_entry_t;

struct messagebus_type_definition_s {
    uint16_t nb_elements;
    const messagebus_type_entry_t *elements;
    size_t struct_size;
};

struct messagebus_type_entry_s {
    const char *name;
    union {
        enum messagebus_base_type_enum base_type;
        const messagebus_type_definition_t *type;
    };
    uint8_t is_base_type:1;
    uint8_t is_array:1;
    uint8_t is_dynamic_array:1;
    uint16_t array_len; // or max length for dynamic arrays
    size_t dynamic_array_len_struct_offset;
    size_t struct_offset;
    size_t size;
};

#endif /* MESSAGEBUS_TYPE_DEFINITION_H */