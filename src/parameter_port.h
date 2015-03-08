#ifndef PARAMETER_PORT_H
#define PARAMETER_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

// this is for the unit tests

#define PARAMETER_LOCK() {}

#define PARAMETER_UNLOCK() {}

#define PARAMETER_ASSERT(check) {}

#else

#include <ch.h>
#include <osal.h>

#define PARAMETER_LOCK() {chSysLock();}

#define PARAMETER_UNLOCK() {chSysUnlock();}

#define PARAMETER_ASSERT(check) {osalDbgAssert(check, "parameter_assert");}

#endif

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_PORT_H */

