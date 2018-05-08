#ifndef SYSCALLS_H
#define SYSCALLS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
size_t sbrk_stat_free(void);
size_t sbrk_stat_used(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSCALLS_H */