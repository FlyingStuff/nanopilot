#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "parameter/parameter.h"
#include "parameter/parameter_msgpack.h"
#include "log.h"

#include "sdcard.h"

#include <ff.h>
static FATFS SDC_FS;
bool fatfs_mounted = false;

void sdcard_mount(void)
{
    if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
        log_warning("SD card not found");
        return;
    }
    if (sdcConnect(&SDCD1)) {
        log_error("SD card connection failed");
        return;
    }
    FRESULT err;
    err = f_mount(&SDC_FS, "", 0);
    if (err != FR_OK) {
        log_error("SD card filesystem mount failed");
        sdcDisconnect(&SDCD1);
        return;
    }
    fatfs_mounted = true;
    log_info("SD card filesystem mounted");
}

void sdcard_unmount(void)
{
    f_mount(NULL, "", 0); // unmount
    sdcDisconnect(&SDCD1);
    fatfs_mounted = false;
}

void sdcard_automount(void)
{
    if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
        if (fatfs_mounted) {
            sdcard_unmount();
        }
    } else {
        if (!fatfs_mounted) {
            sdcard_mount();
        }
    }
}

void file_cat(BaseSequentialStream *out, const char *file_path)
{
    static FIL f;
    FRESULT res = f_open(&f, file_path, FA_READ);
    if (res) {
        log_error("error %d opening %s\n", res, file_path);
        return;
    }
    static char line[80];
    while (f_gets(line, sizeof(line), &f)) {
        chprintf(out, line);
    }
    f_close(&f);
}

static void read_param_cb(void *arg, const char *id, const char *err)
{
    log_warning("file: %s, parameter %s: %s", (const char*)arg, id, err);
}

void sdcard_read_parameter(parameter_namespace_t *ns, const char *file_path)
{
    static FIL f;
    FRESULT res = f_open(&f, file_path, FA_READ);
    if (res) {
        log_error("error %d opening %s\n", res, file_path);
        return;
    }
    static char config_file_buffer[1000];
    if (f_gets(config_file_buffer, sizeof(config_file_buffer), &f)) {
        log_info("file %s read", file_path);
        parameter_msgpack_read(ns, config_file_buffer, sizeof(config_file_buffer), read_param_cb, (void *)file_path);
    }
    f_close(&f);
}

