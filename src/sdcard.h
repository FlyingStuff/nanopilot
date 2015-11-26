#ifndef SDCARD_H
#define SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include "parameter/parameter.h"

void sdcard_mount(void);
void sdcard_unmount(void);
void sdcard_automount(void);
void file_cat(BaseSequentialStream *out, const char *file_path);
void sdcard_read_parameter(parameter_namespace_t *ns, const char *file_path);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */