#include "mock/fatfs_diskio_mem_drv.h"
#include <CppUTest/TestHarness.h>


TEST_GROUP(FATFS)
{

};

TEST(FATFS, mkfs_write_and_read_back)
{
    FATFS fs;
    FRESULT mkfs_res = f_mkfs("", FM_ANY, 0, NULL, 0);
    CHECK_EQUAL(FR_OK, mkfs_res);
    FRESULT mount_res = f_mount(&fs, "", 1);
    CHECK_EQUAL(FR_OK, mount_res); // opt=1 to force mount now

    FIL file;
    FRESULT open_res = f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
    CHECK_EQUAL(FR_OK, open_res);

    UINT bw;
    FRESULT write_res = f_write(&file, "Hello, World!\n\0", 15, &bw);
    CHECK_EQUAL(FR_OK, write_res);
    CHECK_EQUAL(15, bw);

    FRESULT close_res = f_close(&file);
    CHECK_EQUAL(FR_OK, close_res);

    open_res = f_open(&file, "test.txt", FA_READ);
    CHECK_EQUAL(FR_OK, open_res);

    UINT br;
    char buf[15];
    FRESULT read_res = f_read(&file, buf, 15, &br);
    CHECK_EQUAL(FR_OK, read_res);
    CHECK_EQUAL(15, br);
    STRCMP_EQUAL("Hello, World!\n", buf);

    close_res = f_close(&file);
    CHECK_EQUAL(FR_OK, close_res);
}

