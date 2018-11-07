#include <ch.h>
#include <hal.h>
#include <crc/crc32.h>
#include <string.h>
#include <memstreams.h>
#include <chprintf.h>
#include "blocking_uart.h"
#include "panic_handler.h"


/*
 * Panic handling
 */

#define PANIC_CRC_INIT 0x1aaadc37 // random value
static __attribute__((section(".noinit"))) char panic_buffer[700];
static __attribute__((section(".noinit"))) uint32_t panic_buffer_crc;
MemoryStream panic_bss;
static USART_TypeDef *_panic_print_usart = NULL;

static bool panic_buffer_crc_matched;

void panic_handler(const char *reason)
{
    chprintf((BaseSequentialStream *)&panic_bss, "reason: %s\n", reason);
    chprintf((BaseSequentialStream *)&panic_bss, "current thread ptr: %s", ch.rlist.current->name);

    // add terminating '\0' character
    if (panic_bss.eos < sizeof(panic_buffer)) {
        panic_buffer[panic_bss.eos] = '\0';
    } else {
        panic_buffer[sizeof(panic_buffer) - 1] = '\0';
    }
    panic_buffer_crc = crc32(PANIC_CRC_INIT, panic_buffer, sizeof(panic_buffer));

#ifdef DEBUG // debug builds block to be able to connect a debugger
    if (_panic_print_usart) {
        BlockingUARTDriver uart;
        blocking_uart_init(&uart, _panic_print_usart, SERIAL_DEFAULT_BITRATE);
        while (1) {
            blocking_uart_write(&uart, (uint8_t*)panic_buffer, strlen(panic_buffer));
        }
    }
    while (1) {
        __asm__("BKPT");
    }

#else // non-debug builds reboot in safemode
    NVIC_SystemReset();
#endif
}

// used by arm-cortex-tools/fault.c
void fault_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&panic_bss, fmt, ap);
    va_end(ap);
}

const char *get_panic_message(void)
{
    if (panic_buffer_crc_matched) {
        return panic_buffer;
    } else {
        return NULL;
    }
}

void panic_handler_init(USART_TypeDef *panic_print_usart)
{
    _panic_print_usart = panic_print_usart;
    msObjectInit(&panic_bss, (uint8_t *)&panic_buffer[0], sizeof(panic_buffer), 0);

    if (crc32(PANIC_CRC_INIT, panic_buffer, sizeof(panic_buffer)) == panic_buffer_crc) {
        panic_buffer_crc_matched = true;
    } else {
        panic_buffer_crc_matched = false;
    }
    panic_buffer_crc = ~panic_buffer_crc; // clear crc
}


void NMI_Handler(void)
{
    chSysHalt("NMI_Handler");
}
