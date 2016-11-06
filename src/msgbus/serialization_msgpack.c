#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmp/cmp.h"

#include "type_definition.h"


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


bool messagebus_cmp_ser_value_once(const void *var,
                                   const messagebus_type_entry_t *entry,
                                   cmp_ctx_t *ctx,
                                   bool compact)
{
    if (entry->is_base_type) {
        switch (entry->base_type) {
        case MESSAGEBUS_TYPE_FLOAT32:
            return cmp_write_float(ctx, *(float*)var);
        case MESSAGEBUS_TYPE_INT32:
            return cmp_write_int(ctx, *(int32_t*)var);
        case MESSAGEBUS_TYPE_STRING:
            return cmp_write_str(ctx, (const char*)var, strlen((const char*)var));
        default:
            return false;
        }
    } else {
        return messagebus_cmp_ser_type(var, entry->type, ctx, compact);
    }
}


bool messagebus_cmp_ser_value(const void *var,
                              const messagebus_type_entry_t *entry,
                              cmp_ctx_t *ctx,
                              bool compact)
{
    if (entry->is_array || entry->is_dynamic_array) {
        int len;
        if (entry->is_array) {
            len = entry->array_len;
        }
        if (entry->is_dynamic_array) {
            len = *(int*)(var + entry->dynamic_array_len_struct_offset);
            if (len > entry->array_len) {
                return false;
            }
        }
        int i;
        for (i = 0; i < len; i++) {
            const void *var_entry_i = var + entry->struct_offset + entry->size * i;
            if (!messagebus_cmp_ser_value_once(var_entry_i, entry, ctx, compact)) {
                return false;
            }
        }
        return true;
    } else {
        const void *var_entry = var + entry->struct_offset;
        return messagebus_cmp_ser_value_once(var_entry, entry, ctx, compact);
    }
}


bool messagebus_cmp_ser_struct_entry(const void *var,
                                     const messagebus_type_entry_t *entry,
                                     cmp_ctx_t *ctx,
                                     bool compact)
{
    if (!compact) {
        if (!cmp_write_str(ctx, entry->name, strlen(entry->name))) {
            return false;
        }
    }
    return messagebus_cmp_ser_value(var, entry, ctx, compact);
}


bool messagebus_cmp_ser_type(const void *var,
                             const messagebus_type_definition_t *type,
                             cmp_ctx_t *ctx,
                             bool compact)
{
    if (compact) {
        if (!cmp_write_array(ctx, type->nb_elements)) {
            return false;
        }
    } else {
        if (!cmp_write_map(ctx, type->nb_elements)) {
            return false;
        }
    }
    int i;
    for (i = 0; i < type->nb_elements; i++) {
        if (!messagebus_cmp_ser_struct_entry(var, &type->elements[i], ctx, compact)) {
            return false;
        }
    }
    return true;
}


bool messagebus_serialize_msgpack(const void *var,
                                  const messagebus_type_definition_t *type,
                                  char *buf,
                                  size_t buf_sz)
{
    return false; // todo
}


bool messagebus_serialize_msgpack_compact(const void *var,
                                          const messagebus_type_definition_t *type,
                                          char *buf,
                                          size_t buf_sz)
{
    return false; // todo
}

