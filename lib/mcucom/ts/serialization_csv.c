#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include "serialization_csv.h"


struct buffer_s {
    int write;
    size_t size;
    char *buf;
};


static void buffer_init(struct buffer_s *b, char *mem, size_t size)
{
    b->write = 0;
    b->buf = mem;
    b->size = size;
}

static bool buffer_write(struct buffer_s *b, const char* s, size_t len)
{
    if (b->write + len > b->size) {
        return false;
    }
    memmove(&b->buf[b->write], s, len);
    b->write += len;
    return true;
}

static bool buffer_write_s(struct buffer_s *b, const char* string)
{
    return buffer_write(b, string, strlen(string));
}

static bool buffer_printf(struct buffer_s *b, const char *fmt, ...)
{
    size_t size_left = b->size - b->write;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(&b->buf[b->write], size_left, fmt, args);
    va_end(args);
    if (n > 0 && (size_t)n < size_left) {
        b->write += n;
        return true;
    } else {
        return false;
    }

}

static char *buffer_get_pos(struct buffer_s *b)
{
    return &b->buf[b->write];
}

static bool serialize_header(const ts_type_definition_t *type,
                            struct buffer_s *buf,
                            char *prefix_start,
                            size_t prefix_len)
{
    char *next_prefix_start;
    int element, array_idx = 0;
    for (element = 0; element < type->nb_elements;) {
        if (element == 0) {
            next_prefix_start = prefix_start; // prefix is already written
        } else {
            next_prefix_start = buffer_get_pos(buf);
            // copy the prefix
            if (!buffer_write(buf, prefix_start, prefix_len)) {
                return false;
            }
        }

        // copy the name
        const char *name = type->elements[element].name;
        if (!buffer_write_s(buf, name)) {
            return false;
        }

        if (type->elements[element].is_dynamic_array) {
            return false; // dynamic arrays are not supported
        }

        if (type->elements[element].is_array) {
            if (!buffer_printf(buf, "[%d]", array_idx)) {
                return false;
            }
        }

        if (!type->elements[element].is_base_type) { // struct type
            if (!buffer_write_s(buf, ".")) {
                return false;
            }

            size_t next_prefix_len = buffer_get_pos(buf) - next_prefix_start;
            if (!serialize_header(type->elements[element].type,
                                  buf,
                                  next_prefix_start,
                                  next_prefix_len))
            {
                return false;
            }
        }

        if (!buffer_write_s(buf, ",")) {
            return false;
        }

        array_idx++;
        if (!type->elements[element].is_array
            || array_idx == type->elements[element].array_len) {
            element++;
            array_idx = 0;
        }
    }
    buf->write--; // remove the last ','
    return true;
}


int ts_serialize_csv_header(const ts_type_definition_t *type,
                                 char *buf,
                                 size_t buf_sz)
{
    struct buffer_s b;
    buffer_init(&b, buf, buf_sz);
    if (!serialize_header(type, &b, buf, 0)) {
        return -1;
    }
    if (!buffer_write(&b, "\n\0", 2)) {
        return -1;
    }
    return b.write - 1; // buffer without '\0' terminator
}


static bool serialize_type(const void *var, const ts_type_definition_t *type, struct buffer_s *buf);
static bool serialize_entry(const void *var, const ts_type_entry_t *entry, struct buffer_s *buf);

static bool serialize_entry(const void *var, const ts_type_entry_t *entry, struct buffer_s *buf)
{
    if (entry->is_base_type) {
        switch (entry->base_type) {
        case TS_TYPE_FLOAT16:
        case TS_TYPE_FLOAT32:
            return buffer_printf(buf, "%f,", *(float*)var);
        case TS_TYPE_FLOAT64:
            return buffer_printf(buf, "%lf,", *(double*)var);
        case TS_TYPE_INT8:
            return buffer_printf(buf, "%"PRId8",", *(int8_t*)var);
        case TS_TYPE_INT16:
            return buffer_printf(buf, "%"PRId16",", *(int16_t*)var);
        case TS_TYPE_INT32:
            return buffer_printf(buf, "%"PRId32",", *(int32_t*)var);
        case TS_TYPE_INT64:
            return buffer_printf(buf, "%"PRId64",", *(int64_t*)var);
        case TS_TYPE_UINT8:
            return buffer_printf(buf, "%"PRIu8",", *(uint8_t*)var);
        case TS_TYPE_UINT16:
            return buffer_printf(buf, "%"PRIu16",", *(uint16_t*)var);
        case TS_TYPE_UINT32:
            return buffer_printf(buf, "%"PRIu32",", *(uint32_t*)var);
        case TS_TYPE_UINT64:
            return buffer_printf(buf, "%"PRIu64",", *(uint64_t*)var);
        case TS_TYPE_STRING:
            return buffer_write(buf, (const char*)var, strlen((const char*)var))
                   && buffer_write_s(buf, ",");
        default:
            return false;
        }
    } else {
        return serialize_type(var, entry->type, buf)
               && buffer_write_s(buf, ",");
    }
}


static bool serialize_type(const void *var, const ts_type_definition_t *type, struct buffer_s *buf)
{
    int i;
    for (i = 0; i < type->nb_elements; i++) {
        const ts_type_entry_t *entry = &type->elements[i];
        if (entry->is_dynamic_array) {
            return false;
        }
        int array_len = 1;
        if (entry->is_array) {
            array_len = entry->array_len;
        }
        int idx;
        for (idx = 0; idx < array_len; idx++) {
            size_t offset = entry->struct_offset + entry->size * idx;
            if (!serialize_entry(var + offset, entry, buf)) {
                return false;
            }
        }
    }
    buf->write--; // remove the last ','
    return true;
}


int ts_serialize_csv(const void *var,
                          const ts_type_definition_t *type,
                          char *buf,
                          size_t buf_sz)
{
    struct buffer_s b;
    buffer_init(&b, buf, buf_sz);

    if (!serialize_type(var, type, &b)) {
        return -1;
    }

    if (!buffer_write(&b, "\n\0", 2)) {
        return -1;
    }
    return b.write - 1;
}
