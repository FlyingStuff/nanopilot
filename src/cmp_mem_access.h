/*
 * Memory buffer access functions for cmp (MessagePack)
 *
 */
#ifndef CMP_MEM_ACCESS_H
#define CMP_MEM_ACCESS_H

#include <stdlib.h>
#include <stdbool.h>
#include <cmp/cmp.h>

typedef struct {
    char *buf;
    size_t index;
    size_t size;
} cmp_mem_access_t;

#ifdef __cplusplus
extern "C" {
#endif

// initialize cmp
void cmp_mem_access_init(cmp_ctx_t *cmp, cmp_mem_access_t *m, void *buf, size_t size);

// initialize cmp (read only memory access)
void cmp_mem_access_ro_init(cmp_ctx_t *cmp, cmp_mem_access_t *m, const void *buf, size_t size);

// get current read/write position. this can be used to determine the length of
// the buffer written by MessagePack
size_t cmp_mem_access_get_pos(cmp_mem_access_t *m);

#ifdef __cplusplus
}
#endif

#endif /* CMP_MEM_ACCESS_H */
