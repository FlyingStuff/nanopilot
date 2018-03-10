# FATFS files.
# http://elm-chan.org/fsw/ff/00index_e.html

FATFSSRC = $(PROJROOT)/lib/fatfs/fatfs_diskio.c \
           $(PROJROOT)/lib/fatfs/fatfs_syscall.c \
           $(PROJROOT)/lib/fatfs/src/ff.c \
           $(PROJROOT)/lib/fatfs/src/ffunicode.c

FATFSINC = $(PROJROOT)/lib/fatfs/src
