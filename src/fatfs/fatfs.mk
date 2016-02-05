# FATFS files.
# http://elm-chan.org/fsw/ff/00index_e.html

FATFSSRC = $(PROJROOT)/src/fatfs/fatfs_diskio.c \
           $(PROJROOT)/src/fatfs/fatfs_syscall.c \
           $(PROJROOT)/src/fatfs/src/ff.c \
           $(PROJROOT)/src/fatfs/src/option/unicode.c

FATFSINC = $(PROJROOT)/src/fatfs/src
