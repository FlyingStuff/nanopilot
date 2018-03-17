#include <ch.h>
#include <hal.h>
#include "parameter/parameter.h"
#include "parameter/parameter_msgpack.h"
#include "log.h"
#include <string.h>
#include <stdio.h>

#include "sdcard.h"

#include <ff.h>
static FATFS SDC_FS;

static bool sdcard_mounted = false;

bool sdcard_mount(void)
{
    if (!board_sdcard_present()) {
        log_warning("SD card not found");
        return false;
    }
    if (sdcConnect(&SDCD1)) {
        log_error("SD card connection failed");
        return false;
    }
    FRESULT err = f_mount(&SDC_FS, "", 1);
    if (err != FR_OK) {
        log_error("SD card filesystem mount failed");
        sdcDisconnect(&SDCD1);
        return false;
    }
    log_info("SD card filesystem mounted");
    sdcard_mounted = true;
    return true;
}

void sdcard_unmount(void)
{
    f_mount(NULL, "", 0); // unmount
    sdcDisconnect(&SDCD1);
    sdcard_mounted = false;
}

bool sdcard_is_mounted(void)
{
    return sdcard_mounted;
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



int sdcard_find_next_file_nbr_with_prefix(const char *path, const char *prefix)
{
    DIR dir;
    FILINFO finfo;
    FRESULT res = f_opendir(&dir, path);

    size_t prefix_len = strlen(prefix);
    int max_file_nbr = 0;
    if (res == FR_OK) {
        while (true) {
            res = f_readdir(&dir, &finfo);
            if (res != FR_OK || finfo.fname[0] == 0) {
                break;
            }
            if (strncmp(prefix, finfo.fname, prefix_len) == 0) { // match prefix
                int file_nbr = atoi(&finfo.fname[prefix_len]);
                if (file_nbr > max_file_nbr) {
                    max_file_nbr = file_nbr;
                }
            }
        }
        f_closedir(&dir);
        return max_file_nbr + 1;
    }
    return -1;
}

int sdcard_find_next_file_name_with_prefix(const char *path,
    const char *prefix,
    char *outbuf,
    size_t outbuf_size)
{
    outbuf[0] = '\0';
    int file_nbr = sdcard_find_next_file_nbr_with_prefix(path, prefix);
    if (file_nbr < 0) {
        return -1;
    }
    int ret = snprintf(outbuf, outbuf_size, "%s/%s%d", path, prefix, file_nbr);
    if (ret <= 0 || (unsigned)ret >= outbuf_size) {
        return -1;
    }
    return file_nbr;
}


static FIL log_file_sdcard;
static log_handler_t log_handler_sdcard;
static void log_handler_sdcard_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    UINT _bytes_written;
    int ret = f_write(&log_file_sdcard, msg, len, &_bytes_written);
    if (ret == 0) {
        f_sync(&log_file_sdcard);
    }
}

void sdcard_log_handler_init(const char *path, log_level_t log_lvl)
{
    FRESULT res = f_open(&log_file_sdcard, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (res == 0) {
        log_handler_register(&log_handler_sdcard, log_lvl, log_handler_sdcard_cb);
        log_info("log file %s successfully opened", path);
    } else {
        log_warning("opening log file %s failed", path);
    }
}

