#ifndef FATFS_DISKIO_H
#define FATFS_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

void fatfs_diskio_set_io_callback(void (*cb)(void));

#ifdef __cplusplus
}
#endif

#endif /* FATFS_DISKIO_H */
