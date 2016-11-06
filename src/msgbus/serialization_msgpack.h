#ifndef MESSAGEBUS_SERIALIZATION_MSGPACK_H
#define MESSAGEBUS_SERIALIZATION_MSGPACK_H


#include "type_definition.h"

#ifdef __cplusplus
extern "C" {
#endif

bool messagebus_serialize_msgpack(const void *var,
                                  const messagebus_type_definition_t *type,
                                  char *buf,
                                  size_t buf_sz);

bool messagebus_serialize_msgpack_compact(const void *var,
                                          const messagebus_type_definition_t *type,
                                          char *buf,
                                          size_t buf_sz);

#ifdef __cplusplus
}
#endif

#endif /* MESSAGEBUS_SERIALIZATION_MSGPACK_H */