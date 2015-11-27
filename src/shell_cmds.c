#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>

#include "onboardsensors.h"
#include "serial-datagram/serial_datagram.h"
#include "parameter/parameter_print.h"
#include "git_revision.h"
#include "syscalls.h"
#include "error.h"
#include "main.h"

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    chprintf(chp, "used: %u bytes, free: %u bytes\n",
             sbrk_stat_used(), sbrk_stat_free());
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: threads\n");
        return;
    }
    chprintf(chp, "    addr    stack  free prio     state     time name\n");
    tp = chRegFirstThread();
    do {
        chprintf(chp, "%08lx %08lx %5u %4lu %9s %8lu %s\n",
                 (uint32_t)tp,
                 (uint32_t)tp->p_ctx.r13,
                 (char *)tp->p_ctx.r13 - (char *)tp->p_stklimit,
                 (uint32_t)tp->p_prio,
                 states[tp->p_state],
                 chThdGetTicksX(tp),
                 tp->p_name);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
    chprintf(chp, "[sytick %d @ %d Hz]\n", chVTGetSystemTime(), CH_CFG_ST_FREQUENCY);
}

static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    chprintf(chp, "git version: %s\n", build_git_version);
    chprintf(chp, "branch:      %s\n", build_git_branch);
    chprintf(chp, "full sha:    %s\n", build_git_sha);
    chprintf(chp, "build date:  %s\n", build_date);
    chprintf(chp, "compiler:    %s\n", PORT_COMPILER_NAME);
}

static void cmd_safemode(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    reboot_in_safemode();
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    NVIC_SystemReset();
}

static void cmd_bootloader(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    reboot_st_bootloader();
}

static void cmd_panic(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    chSysHalt("panic test shell command");
}

static void cmd_panic_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    chprintf(chp, "panic was: %s\n", get_panic_message());
}

static void cmd_parameter_list(BaseSequentialStream *stream, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    parameter_print(&parameters, (parameter_printfn_t)chprintf, stream);
}

static void cmd_parameter_set(BaseSequentialStream *stream, int argc, char *argv[]) {
    if (argc < 2) {
        chprintf(stream, "usage: parameter_set name value\n");
        return;
    }
    parameter_t *p = parameter_find(&parameters, argv[0]);
    if (p == NULL) {
        chprintf(stream, "parameter doesn't exist\n");
        return;
    }
    if (p->type == _PARAM_TYPE_SCALAR) {
        parameter_scalar_set(p, strtof(argv[1], NULL));
    } else {
        chprintf(stream, "unsupported type %d\n", p->type);
    }
}


const ShellCommand shell_commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"version", cmd_version},
  {"safemode", cmd_safemode},
  {"reboot", cmd_reboot},
  {"bootloader", cmd_bootloader},
  {"panic", cmd_panic},
  {"panic_get", cmd_panic_get},
  {"parameter_list", cmd_parameter_list},
  {"parameter_set", cmd_parameter_set},
  {NULL, NULL}
};

