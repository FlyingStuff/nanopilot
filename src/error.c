#include <ch.h>
#include <hal.h>
#include <crc/crc32.h>
#include <string.h>
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include <memstreams.h>
#include <chprintf.h>
#include "error.h"


/*
 * Safemode
 */

#define SAFEMODE_MAGIC_VALUE 0xdb3c9869254dc8bc // random value
static __attribute__((section(".noinit"))) uint64_t safemode_magic_value;

static bool safemode;

static void safemode_read(void)
{
    if (safemode_magic_value == SAFEMODE_MAGIC_VALUE) {
        safemode = true;
    } else {
        safemode = false;
    }
    safemode_magic_value = 0;
}

bool safemode_active(void)
{
    return safemode;
}

void reboot_in_safemode(void)
{
    safemode_magic_value = SAFEMODE_MAGIC_VALUE;
    NVIC_SystemReset();
}


/*
 * Panic handling
 */

#define PANIC_CRC_INIT 0x1aaadc37 // random value
static __attribute__((section(".noinit"))) char panic_buffer[700];
static __attribute__((section(".noinit"))) uint32_t panic_buffer_crc;
MemoryStream panic_bss;

static bool panic_buffer_crc_matched;

void panic_handler(const char *reason)
{
    (void)reason;
    palSetPad(GPIOA, GPIOA_LED_ERROR);
    palClearPad(GPIOB, GPIOB_LED_STATUS);
    palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
    palClearPad(GPIOB, GPIOB_LED_SDCARD);

    static volatile uint32_t ipsr;
    static volatile const char *msg;
    msg = reason;
    ipsr = __get_IPSR();

    (void)msg;
    (void)ipsr;

    chprintf((BaseSequentialStream *)&panic_bss, "%s", reason);

    // add terminating '\0' character
    if (panic_bss.eos < sizeof(panic_buffer)) {
        panic_buffer[panic_bss.eos] = '\0';
    } else {
        panic_buffer[sizeof(panic_buffer) - 1] = '\0';
    }
    panic_buffer_crc = crc32(PANIC_CRC_INIT, panic_buffer, sizeof(panic_buffer));

#ifdef DEBUG // debug builds block to be able to connect a debugger
    while (1) {
        // todo print error over uart
    }

#else // non-debug builds reboot in safemode
    reboot_in_safemode();
#endif
}

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

static void panic_msg_init(void)
{
    msObjectInit(&panic_bss, (uint8_t *)&panic_buffer[0], sizeof(panic_buffer), 0);

    if (crc32(PANIC_CRC_INIT, panic_buffer, sizeof(panic_buffer)) == panic_buffer_crc) {
        panic_buffer_crc_matched = true;
    } else {
        panic_buffer_crc_matched = false;
    }
    panic_buffer_crc = ~panic_buffer_crc; // clear crc
}

void error_init(void)
{
    safemode_read();
    panic_msg_init();
    mpu_init();
    fault_init();
}