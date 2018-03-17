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

int sdcard_find_next_file_nbr_with_prefix(const char *path, const char *prefix);
int sdcard_find_next_file_name_with_prefix(const char *path,
    const char *prefix,
    char *outbuf,
    size_t outbuf_size);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */