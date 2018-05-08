#include "deserialization_msgpack.h"
#include "cmp_mem_access/cmp_mem_access.h"


bool ts_cmp_deser_value_once(void *var,
                             const ts_type_entry_t *entry,
                             cmp_ctx_t *ctx)
{
    if (entry->is_base_type) {
        uint32_t max_str_size;
        switch (entry->base_type) {
        case TS_TYPE_FLOAT16:
        case TS_TYPE_FLOAT32:
            return cmp_read_float(ctx, (float*)var);
        case TS_TYPE_FLOAT64:
            return cmp_read_double(ctx, (double*)var);
        case TS_TYPE_INT8:
            return cmp_read_char(ctx, (int8_t*)var);
        case TS_TYPE_INT16:
            return cmp_read_short(ctx, (int16_t*)var);
        case TS_TYPE_INT32:
            return cmp_read_int(ctx, (int32_t*)var);
        case TS_TYPE_INT64:
            return cmp_read_long(ctx, (int64_t*)var);
        case TS_TYPE_UINT8:
            return cmp_read_uchar(ctx, (uint8_t*)var);
        case TS_TYPE_UINT16:
            return cmp_read_ushort(ctx, (uint16_t*)var);
        case TS_TYPE_UINT32:
            return cmp_read_uint(ctx, (uint32_t*)var);
        case TS_TYPE_UINT64:
            return cmp_read_ulong(ctx, (uint64_t*)var);
        case TS_TYPE_STRING:
            max_str_size = entry->size;
            return cmp_read_str(ctx, (char*)var, &max_str_size);
        default:
            return false;
        }
    } else {
        return ts_cmp_deser_type(var, entry->type, ctx);
    }
}

bool ts_cmp_deser_value(void *var,
                        const ts_type_entry_t *entry,
                        cmp_ctx_t *ctx)
{
    if (entry->is_array || entry->is_dynamic_array) {
        uint32_t read_len;
        if (!cmp_read_array(ctx, &read_len)) {
            return false;
        }
        if (entry->is_array && read_len != entry->array_len) {
            return false;
        }
        if (entry->is_dynamic_array) {
            if (read_len > entry->array_len) {
                return false;
            }
            *(uint16_t*)(var + entry->dynamic_array_len_struct_offset) = read_len;
        }

        uint32_t i;
        for (i = 0; i < read_len; i++) {
            void *var_entry_i = var + entry->struct_offset + entry->size * i;
            if (!ts_cmp_deser_value_once(var_entry_i, entry, ctx)) {
                return false;
            }
        }
        return true;
    } else {
        void *var_entry = var + entry->struct_offset;
        return ts_cmp_deser_value_once(var_entry, entry, ctx);
    }
}

bool ts_cmp_deser_type(void *var,
                       const ts_type_definition_t *type,
                       cmp_ctx_t *ctx)
{
    uint32_t nb_elements;
    if (!cmp_read_array(ctx, &nb_elements) || nb_elements != type->nb_elements) {
        return false;
    }
    int i;
    for (i = 0; i < type->nb_elements; i++) {
        if (!ts_cmp_deser_value(var, &type->elements[i], ctx)) {
            return false;
        }
    }
    return true;
}

int ts_deserialize_msgpack(void *var,
                           const ts_type_definition_t *type,
                           const char *buf,
                           size_t buf_sz)
{
    cmp_mem_access_t mem;
    cmp_ctx_t cmp;
    cmp_mem_access_ro_init(&cmp, &mem, buf, buf_sz);
    if (!ts_cmp_deser_type(var, type, &cmp)) {
        return -1;
    }
    return cmp_mem_access_get_pos(&mem);
}
