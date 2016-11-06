#ifndef MSGBUS_SERIALIZATION_MSGPACK_H
#define MSGBUS_SERIALIZATION_MSGPACK_H


#include "type_definition.h"

#ifdef __cplusplus
extern "C" {
#endif

bool msgbus_serialize_msgpack(const void *var,
                              const msgbus_type_definition_t *type,
                              char *buf,
                              size_t buf_sz);

bool msgbus_serialize_msgpack_compact(const void *var,
                                      const msgbus_type_definition_t *type,
                                      char *buf,
                                      size_t buf_sz);

#ifdef __cplusplus
}
#endif

#endif /* MSGBUS_SERIALIZATION_MSGPACK_H */