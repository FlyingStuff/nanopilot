#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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

static bool serialize_header(const msgbus_type_definition_t *type,
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


int msgbus_serialize_csv_header(const msgbus_type_definition_t *type,
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

int msgbus_serialize_csv(const void *var,
                          const msgbus_type_definition_t *type,
                          char *buf,
                          size_t buf_sz)
{
    (void)var;
    (void)type;
    (void)buf;
    (void)buf_sz;
    return 0;
}
