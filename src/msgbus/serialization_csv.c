#include <string.h>
#include "serialization_csv.h"


static int serialize_header(const msgbus_type_definition_t *type,
                            char *buf,
                            size_t buf_sz,
                            unsigned int prefix_len)
{
    unsigned int write = 0;
    int i;
    for (i = 0; i < type->nb_elements; i++) {

        // copy the prefix
        if (write + prefix_len > buf_sz) {
            return -1;
        }
        int prefix_start = write;
        memmove(&buf[write], buf, prefix_len);
        write += prefix_len;

        // copy the name
        const char *name = type->elements[i].name;
        size_t name_len = strlen(name);
        if (write + name_len > buf_sz) {
            return -1;
        }
        strncpy(&buf[write], name, name_len);
        write += strlen(name);

        if (type->elements[i].is_dynamic_array) {
            return -1;
        }

        if (!type->elements[i].is_base_type) { // struct type
            if (write + 1 > buf_sz) {
                return -1;
            }
            buf[write++] = '.';

            int new_prefix_len = prefix_len + name_len + 1;
            int len = serialize_header(type->elements[i].type,
                                       &buf[prefix_start],
                                       buf_sz - prefix_start,
                                       new_prefix_len);
            if (len > 0) {
                write += len - new_prefix_len;
            } else {
                return len;
            }
        }

        // write a ',' after the entry
        if (write + 1 > buf_sz) {
            return -1;
        }
        buf[write++] = ',';
    }
    return write - 1; // without the last ','
}


int msgbus_serialize_csv_header(const msgbus_type_definition_t *type,
                                 char *buf,
                                 size_t buf_sz)
{
    int write = serialize_header(type, buf, buf_sz, 0);
    if (write > 0 && (size_t)write + 1 < buf_sz) {
        buf[write++] = '\n';
    }
    return write;
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
