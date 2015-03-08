# FATFS files.
# http://elm-chan.org/fsw/ff/00index_e.html

FATFSSRC = ${CHIBIOS}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           src/fatfs/src/ff.c
           #src/fatfs/src/option/unicode.c

FATFSINC = src/fatfs/src
