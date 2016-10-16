#ifndef ERROR_H
#define ERROR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// initialize error handling, must be called on boot
void error_init(void);

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