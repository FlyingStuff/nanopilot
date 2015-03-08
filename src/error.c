#include <ch.h>
#include <hal.h>
#include <crc/crc32.h>
#include <string.h>

#include "error.h"

/*
 * Error levels
 */

static int error_level_cnt[2] = {0,0};

void error_set(int level)
{
    if (level > ERROR_LEVEL_NORMAL && level <= ERROR_LEVEL_CRITICAL) {
        chSysLock();
        error_level_cnt[level - 1]++;
        chSysUnlock();
    }
}

void error_clear(int level)
{
    if (level > ERROR_LEVEL_NORMAL && level <= ERROR_LEVEL_CRITICAL) {
        chSysLock();
        if (error_level_cnt[level - 1] > 0) {
            error_level_cnt[level - 1]--;
        }
        chSysUnlock();
    }
}

int error_level_get(void)
{
    int lvl = ERROR_LEVEL_NORMAL;
    chSysLock();
    if (error_level_cnt[ERROR_LEVEL_CRITICAL - 1]) {
        lvl = ERROR_LEVEL_CRITICAL;
    } else if (error_level_cnt[ERROR_LEVEL_WARNING - 1]) {
        lvl = ERROR_LEVEL_WARNING;
    }
    chSysUnlock();
    return lvl;
}


/*
 * Safemode
 */

#define SAFEMODE_MAGIC_VALUE 0xdb3c9869254dc8bc // random value
// todo section noinit
static uint64_t safemode_magic_value;

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

#define PANIC_CRC_INIT 0x1aaadc37
// todo section noinit
static char panic_buffer[200];
static uint32_t panic_buffer_crc;

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

    strncpy(panic_buffer, reason, sizeof(panic_buffer));

    panic_buffer[sizeof(panic_buffer) - 1] = '\0';
    panic_buffer_crc = crc32(PANIC_CRC_INIT, panic_buffer, sizeof(panic_buffer));

#ifdef DEBUG // debug builds block to be able to connect a debugger
    while (1) {
        // todo print error over uart
    }

#else // non-debug builds reboot in safemode
    reboot_in_safemode();
#endif
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
}