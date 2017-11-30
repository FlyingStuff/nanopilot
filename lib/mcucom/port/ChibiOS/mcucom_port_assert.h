#ifndef MCUCOM_PORT_ASSERT_H
#define MCUCOM_PORT_ASSERT_H

#include <osal.h>

#define MCUCOM_PORT_ASSERT(x) osalDbgAssert(x, "mcucom assert")

#endif /* MCUCOM_PORT_ASSERT_H */
