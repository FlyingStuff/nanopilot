#ifndef MSGBUS_SERIALIZATION_CSV_H
#define MSGBUS_SERIALIZATION_CSV_H

#include "type_definition.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



int msgbus_serialize_csv_header(const msgbus_type_definition_t *type,
                                 char *buf,
                                 size_t buf_sz);

int msgbus_serialize_csv(const void *var,
                          const msgbus_type_definition_t *type,
                          char *buf,
                          size_t buf_sz);

#ifdef __cplusplus
}
#endif

#endif /* MSGBUS_SERIALIZATION_CSV_H */