#ifndef SHELL_CMDS_H
#define SHELL_CMDS_H

#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif

void shell_run(BaseSequentialStream* dev);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_CMDS_H */
