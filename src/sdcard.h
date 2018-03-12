#ifndef SDCARD_H
#define SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include "parameter/parameter.h"
#include "log.h"

bool sdcard_mount(void);
void sdcard_unmount(void);
bool sdcard_is_mounted(void);

void sdcard_read_parameter(parameter_namespace_t *ns, const char *file_path);

void sdcard_log_handler_init(const char *path, log_level_t log_lvl);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */