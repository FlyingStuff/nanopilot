##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

C_OPT_REPRODUCIBLE_BUILD = -frandom-seed=42
C_OPT_REPRODUCIBLE_BUILD += -D__DATE__="\"not set for reproducible build\""
C_OPT_REPRODUCIBLE_BUILD += -D__TIME__="\"not set for reproducible build\""
C_OPT_REPRODUCIBLE_BUILD += -Wno-builtin-macro-redefined

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16 $(C_OPT_REPRODUCIBLE_BUILD)
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
# don't use for debugging
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ins-board

PROJROOT = ../..
TARGETROOT = .

# Imported source files and paths
CHIBIOS = $(PROJROOT)/lib/mcu/ChibiOS
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
include $(CHIBIOS)/test/rt/test.mk

include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
include $(PROJROOT)/lib/fatfs/fatfs.mk
include $(PROJROOT)/src/src.mk
include $(PROJROOT)/lib/src.mk
include $(TARGETROOT)/src/src.mk

include $(PROJROOT)/types/types.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F405xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PROJCSRC) \
	   $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(OSALSRC) \
       $(PLATFORMSRC) \
       $(STARTUPSRC) \
       $(BOARDSRC) \
       $(PROJROOT)/src/chprintf_long_long.c \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(FATFSSRC) \
       $(TYPECSRC) \
       $(PROJROOT)/src/git_revision.c \

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(PROJCPPSRC) $(CHCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PROJASMSRC)
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)


INCDIR = $(CHIBIOS)/os/license $(PORTINC) $(KERNINC) $(TESTINC) $(STARTUPINC) \
         $(HALINC) $(OSALINC) $(PLATFORMINC) $(BOARDINC) \
         $(FATFSINC) $(CHIBIOS)/os/various $(CHCPPINC) \
         $(CHIBIOS)/os/hal/lib/streams \
         $(TYPEINC) \
         $(PROJINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
# LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
NM   = $(TRGT)nm
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes -Werror

# Define C++ warning options here
CPPWARN = -Wall -Wextra -std=c++11

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DDEBUG

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

GLOBAL_SRC_DEP =

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk
-include tools.mk

PRE_MAKE_ALL_RULE_HOOK: $(notdir $(TYPECSRC))
	cd $(PROJROOT)/src/flight-stack && ./code_gen.sh
	cd $(PROJROOT) && ./git_revision.sh

POST_MAKE_ALL_RULE_HOOK:
	python $(PROJROOT)/scripts/heap_size.py $(BUILDDIR)/$(PROJECT).elf

# Generates a ctags file containing the correct definition for the build
.PHONY: ctags
ctags:
	@echo "Generating ctags file..."
	@cat .dep/*.d | grep ":$$" | sed "s/://" | sort | uniq | xargs ctags --file-scope=no --extra=+q $(CSRC) $(CPPSRC)
