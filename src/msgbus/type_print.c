#include <string.h>
#include <stdbool.h>
#include "type_definition.h"
#include "type_print.h"

static void print_indentataion(void (*print_fn)(void *, const char *, ...), void *arg, unsigned int indent)
{
    unsigned int i;
    for (i = 0; i < indent; i++) {
        print_fn(arg, "    ");
    }
}

bool messagebus_print_base_type(void (*print_fn)(void *, const char *, ...), void *arg,
                                int type, const void *p)
{
    switch (type) {
        case MESSAGEBUS_TYPE_INT32:
            print_fn(arg, "%d", *(int32_t *)p);
            break;
        case MESSAGEBUS_TYPE_FLOAT32:
            print_fn(arg, "%f", *(float *)p);
            break;
        case MESSAGEBUS_TYPE_STRING:
            print_fn(arg, "\"%s\"", p);
            break;
        default :
            return false;
            break;
    };
    return true;
}

bool messagebus_print_entry(void (*print_fn)(void *, const char *, ...), void *arg,
                            const messagebus_type_entry_t *entry, const void *object,
                            unsigned int indent)
{
    print_indentataion(print_fn, arg, indent);
    if (entry->is_array || entry->is_dynamic_array) {
        print_fn(arg, "%s: [", entry->name);
        uint16_t count;
        if (entry->is_dynamic_array) {
            count = *(uint16_t *)((char *)object + entry->dynamic_array_len_struct_offset);
            if (count > entry->array_len) {
                return false;
            }
        } else {
            count = entry->array_len;
        }
        uint16_t index;
        for (index = 0; index < count; index++) {
            void *entry_data = (char *)object + entry->struct_offset + index * entry->size;
            if (entry->is_base_type) {
                if (index != 0) {
                    print_fn(arg, ", ");
                }
                if (!messagebus_print_base_type(print_fn, arg, entry->base_type, entry_data)) {
                    return false;
                }
            } else {
                print_fn(arg, "\n");
                messagebus_print_type_indent(print_fn, arg, entry->type, entry_data, indent+1);
            }
        }
        if (!entry->is_base_type) {
            print_indentataion(print_fn, arg, indent);
        }
        print_fn(arg, "]\n");
    } else {
        object = (char *)object + entry->struct_offset;
        if (entry->is_base_type) {
            print_fn(arg, "%s: ", entry->name);
            if (!messagebus_print_base_type(print_fn, arg, entry->base_type, object)) {
                return false;
            }
            print_fn(arg, "\n");
        } else {
            print_fn(arg, "%s:\n", entry->name);
            messagebus_print_type_indent(print_fn, arg, entry->type, object, indent+1);
        }
    }
    return true;
}

bool messagebus_print_type_indent(void (*print_fn)(void *, const char *, ...), void *arg,
                                  const messagebus_type_definition_t *type, const void *object,
                                  unsigned int indent)
{
    int i;
    for (i = 0; i < type->nb_elements; i++) {
        if (!messagebus_print_entry(print_fn, arg, &type->elements[i], object, indent)) {
            return false;
        }
    }
    return true;
}

bool messagebus_print_type(void (*print_fn)(void *, const char *, ...), void *arg,
                           const messagebus_type_definition_t *type, const void *object)
{
    return messagebus_print_type_indent(print_fn, arg, type, object, 0);
}
