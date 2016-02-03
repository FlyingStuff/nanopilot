#ifndef ERROR_H
#define ERROR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// initialize error handling, must be called on boot
void error_init(void);

// error levels are counting (i.e. there can be multiple errors of a certain level)
#define ERROR_LEVEL_NORMAL         0
#define ERROR_LEVEL_WARNING        1
#define ERROR_LEVEL_CRITICAL       2
// set one error of level [WARNING, CRITICAL]
void error_set(int level);
// clear one error of level [WARNING, CRITICAL]
void error_clear(int level);
// returns the current error level [NORMAL, WARNING, CRITICAL]
// (WARNING overrides NORMAL, CRITICAL overrides WARNING)
int error_level_get(void);

// returns true if booted in safemode, false otherwise
bool safemode_active(void);
// reboots and activates safemode
void reboot_in_safemode(void);

void panic_handler(const char *reason);

// returns the panic message when rebooted after a panic
// or NULL if there was no panic
const char *get_panic_message(void);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */