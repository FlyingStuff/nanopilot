#ifndef PANIC_HANDLER_H
#define PANIC_HANDLER_H

#include <stdbool.h>
#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

// initialize panic message handling, must be called on boot
void panic_handler_init(USART_TypeDef *panic_print_usart);

void panic_handler(const char *reason);

// returns the panic message when rebooted after a panic
// or NULL if there was no panic
const char *get_panic_message(void);

#ifdef __cplusplus
}
#endif

#endif /* PANIC_HANDLER_H */