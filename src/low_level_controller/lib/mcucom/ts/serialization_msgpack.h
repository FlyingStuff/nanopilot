#ifndef TS_SERIALIZATION_MSGPACK_H
#define TS_SERIALIZATION_MSGPACK_H


#include "type_definition.h"
#include <stdbool.h>
#include <cmp/cmp.h>

#ifdef __cplusplus
extern "C" {
#endif

int ts_serialize_msgpack_self_describing(const void *var,
                                         const ts_type_definition_t *type,
                                         char *buf,
                                         size_t buf_sz);

int ts_serialize_msgpack(const void *var,
                         const ts_type_definition_t *type,
                         char *buf,
                         size_t buf_sz);


/*  internal, used for testing only  */

bool ts_cmp_ser_type(const void *var,
                         const ts_type_definition_t *type,
                         cmp_ctx_t *ctx,
                         bool compact);
bool ts_cmp_ser_struct_entry(const void *var,
                                 const ts_type_entry_t *entry,
                                 cmp_ctx_t *ctx,
                                 bool compact);
bool ts_cmp_ser_value(const void *var,
                          const ts_type_entry_t *entry,
                          cmp_ctx_t *ctx,
                          bool compact);
bool ts_cmp_ser_value_once(const void *var,
                               const ts_type_entry_t *entry,
                               cmp_ctx_t *ctx,
                               bool compact);

#ifdef __cplusplus
}
#endif

#endif /* TS_SERIALIZATION_MSGPACK_H */
