#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ch.h"
#if defined(STDOUT_SD) || defined(STDIN_SD)
#include "hal.h"
#endif


int _read_r(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
#if defined(STDIN_SD)
    if (!len || (file != 0)) {
        __errno_r(r) = EINVAL;
        return -1;
    }
    len = sdRead(&STDIN_SD, (uint8_t *)ptr, (size_t)len);
    return len;
#else
    (void)file;
    (void)ptr;
    (void)len;
    __errno_r(r) = EINVAL;
    return -1;
#endif
}


int _lseek_r(struct _reent *r, int file, int ptr, int dir)
{
    (void)r;
    (void)file;
    (void)ptr;
    (void)dir;

    return 0;
}


int _write_r(struct _reent *r, int file, char * ptr, int len)
{
    (void)r;
    (void)file;
    (void)ptr;
#if defined(STDOUT_SD)
    if (file != 1) {
        __errno_r(r) = EINVAL;
        return -1;
    }
    sdWrite(&STDOUT_SD, (uint8_t *)ptr, (size_t)len);
#endif
    return len;
}


int _close_r(struct _reent *r, int file)
{
    (void)r;
    (void)file;

    return 0;
}


#if CH_CFG_USE_MEMCORE

caddr_t _sbrk_r(struct _reent *r, int incr)
{
    void *p;

    chDbgCheck(incr > 0);

    p = chCoreAlloc((size_t)incr);
    if (p == NULL) {
        __errno_r(r) = ENOMEM;
        return (caddr_t)-1;
    }
    return (caddr_t)p;
}

#else

extern char __heap_base__[];
extern char __heap_end__[];
static char *heap_end = &__heap_base__[0];

size_t sbrk_stat_free(void)
{
    return &__heap_end__[0] - heap_end;
}

size_t sbrk_stat_used(void)
{
    return heap_end - &__heap_base__[0];
}

caddr_t _sbrk_r(struct _reent *r, int incr)
{
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = __heap_base__;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > __heap_end__) {
        __errno_r(r) = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    chDbgCheck(heap_end >= __heap_base__);

    return (caddr_t) prev_heap_end;
}

#endif


int _fstat_r(struct _reent *r, int file, struct stat * st)
{
    (void)r;
    (void)file;

    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}


int _isatty_r(struct _reent *r, int fd)
{
    (void)r;
    (void)fd;

    return 1;
}


MUTEX_DECL(malloc_lock);

void __malloc_lock (struct _reent *reent)
{
    (void) reent;
    chMtxLock(&malloc_lock);
}

void __malloc_unlock (struct _reent *reent)
{
    (void) reent;
    chMtxUnlock(&malloc_lock);
}

