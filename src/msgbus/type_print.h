#ifndef TYPE_PRINT_H
#define TYPE_PRINT_H

#include "type_definition.h"

#ifdef __cplusplus
extern "C" {
#endif

bool messagebus_print_entry(void (*print_fn)(void *, const char *, ...), void *arg,
                            const messagebus_type_entry_t *entry, const void *object,
                            unsigned int indent);

bool messagebus_print_type_indent(void (*print_fn)(void *, const char *, ...), void *arg,
                                  const messagebus_type_definition_t *type, const void *object,
                                  unsigned int indent);

bool messagebus_print_type(void (*print_fn)(void *, const char *, ...), void *arg,
                           const messagebus_type_definition_t *type, const void *object);


#ifdef __cplusplus
}
#endif

#endif /* TYPE_PRINT_H */
