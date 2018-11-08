#include <parameter/parameter_port.h>
#include <ch.h>
#include <osal.h>

void parameter_port_lock(void)
{
    chSysLock();
}

void parameter_port_unlock(void)
{
    chSysUnlock();
}

void parameter_port_assert(int condition)
{
    osalDbgAssert(condition, "parameter_assert");
}

/** Allocates a buffer of at least the given size.
 *
 * @note This is only used when converting MessagePack objects.
 * @note At most one buffer is allocated at any moment, which allows the use of
 * a static buffer.
 * @note The maximum allocated size is the length of the longest parameter or
 * namespace name or the largest parameter array, matrix or string, whichever
 * is the largest.
 */
static char alloc_buffer[200];
static bool buffer_used = false;
void *parameter_port_buffer_alloc(size_t size)
{
    osalDbgAssert(buffer_used == false, "parameter buffer double alloc");
    if (size <= sizeof(alloc_buffer)) {
        buffer_used = true;
        return alloc_buffer;
    } else {
        return NULL;
    }
}

/** Frees a buffer allocated by parameter_port_buffer_alloc. */
void parameter_port_buffer_free(void *buffer)
{
    if (buffer == NULL) {
        return;
    }
    osalDbgAssert(buffer_used == true, "parameter buffer double free");
    buffer_used = false;
}
