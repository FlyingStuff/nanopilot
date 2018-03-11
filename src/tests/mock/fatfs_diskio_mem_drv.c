#include "fatfs_diskio_mem_drv.h"
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#define SECTOR_SIZE 512 // valid options 512, 1024, 2048 and 4096
#define SECTOR_COUNT 10000

static void *disk_memory = NULL;

DSTATUS disk_initialize (BYTE pdrv)
{
    assert(pdrv == 0);
    if (disk_memory == NULL) {
        disk_memory = malloc(SECTOR_COUNT * SECTOR_SIZE);
    }
    return disk_status(pdrv);
}

DRESULT disk_read (
    BYTE pdrv,        /* Physical drive nmuber (0..) */
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,    /* Sector address (LBA) */
    UINT count        /* Number of sectors to read (1..255) */
)
{
    assert(pdrv == 0);
    memcpy(buff, &disk_memory[SECTOR_SIZE*sector], count*SECTOR_SIZE);
    return RES_OK;
}


DRESULT disk_write (
    BYTE pdrv,            /* Physical drive nmuber (0..) */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector address (LBA) */
    UINT count            /* Number of sectors to write (1..255) */
)
{
    assert(pdrv == 0);
    memcpy(&disk_memory[SECTOR_SIZE*sector], buff, count*SECTOR_SIZE);
    return RES_OK;
}

DSTATUS disk_status (BYTE pdrv)
{
    assert(pdrv == 0);
    if (disk_memory == NULL) {
        return STA_NOINIT;
    } else {
        return 0;
    }
}

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff)
{
    assert(pdrv == 0);
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        *((DWORD *)buff) = SECTOR_COUNT;
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *)buff) = SECTOR_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 1; // erasable block is 1 sector
        return RES_OK;
    case CTRL_TRIM:
        return RES_OK;
    }
    return RES_PARERR;
}


void *ff_memalloc(UINT size)
{
    return malloc(size);
}

void ff_memfree(void *mblock)
{
    free(mblock);
}

DWORD get_fattime(void) {
    return ((uint32_t)0 | (1 << 16)) | (1 << 21); /* wrong but valid time */
}

