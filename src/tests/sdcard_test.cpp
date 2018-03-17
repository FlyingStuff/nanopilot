#include "../sdcard.h"
#include "mock/fatfs_diskio_mem_drv.h"
#include <CppUTest/TestHarness.h>

TEST_GROUP(SDCard)
{
    void setup()
    {
        f_mkfs("", FM_ANY, 0, NULL, 0);
    }
};


TEST(SDCard, build)
{

}

TEST(SDCard, find_next_file_name_with_prefix)
{
    FATFS fs;
    FRESULT mkfs_res = f_mkfs("", FM_ANY, 0, NULL, 0);
    CHECK_EQUAL(FR_OK, mkfs_res);
    FRESULT mount_res = f_mount(&fs, "", 1);
    CHECK_EQUAL(FR_OK, mount_res);

    FRESULT mkdir_res = f_mkdir("/logs");
    CHECK_EQUAL(FR_OK, mkdir_res);
    mkdir_res = f_mkdir("/logs/log_1");
    CHECK_EQUAL(FR_OK, mkdir_res);
    mkdir_res = f_mkdir("/logs/log_02");
    CHECK_EQUAL(FR_OK, mkdir_res);
    mkdir_res = f_mkdir("/logs/log_10");
    CHECK_EQUAL(FR_OK, mkdir_res);

    char buf[100];
    int ret = sdcard_find_next_file_name_with_prefix("/logs", "log_", buf, sizeof(buf));
    CHECK_EQUAL(11, ret);
    STRCMP_EQUAL(buf, "/logs/log_11");
}
