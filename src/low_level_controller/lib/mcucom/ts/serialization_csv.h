#ifndef TS_SERIALIZATION_CSV_H
#define TS_SERIALIZATION_CSV_H

#include "type_definition.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



int ts_serialize_csv_header(const ts_type_definition_t *type,
                                 char *buf,
                                 size_t buf_sz);

int ts_serialize_csv(const void *var,
                          const ts_type_definition_t *type,
                          char *buf,
                          size_t buf_sz);

#ifdef __cplusplus
}
#endif

#endif /* TS_SERIALIZATION_CSV_H */