#ifndef TS_DESERIALIZATION_MSGPACK_H
#define TS_DESERIALIZATION_MSGPACK_H

#include "type_definition.h"
#include <stdbool.h>
#include <cmp/cmp.h>


#ifdef __cplusplus
extern "C" {
#endif

int ts_deserialize_msgpack(void *var,
                           const ts_type_definition_t *type,
                           const char *buf,
                           size_t buf_sz);

/*  internal, used for testing only  */

bool ts_cmp_deser_type(void *var,
                       const ts_type_definition_t *type,
                       cmp_ctx_t *ctx);
bool ts_cmp_deser_struct_entry(void *var,
                               const ts_type_entry_t *entry,
                               cmp_ctx_t *ctx);
bool ts_cmp_deser_value(void *var,
                        const ts_type_entry_t *entry,
                        cmp_ctx_t *ctx);
bool ts_cmp_deser_value_once(void *var,
                             const ts_type_entry_t *entry,
                             cmp_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* TS_DESERIALIZATION_MSGPACK_H */
