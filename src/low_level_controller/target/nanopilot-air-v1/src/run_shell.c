#include "ch.h"
#include "hal.h"
#include <shell.h>

#include "thread_prio.h"
#include "shell_cmds.h"

static THD_WORKING_AREA(shell_thd_wa, (2500));
static THD_FUNCTION(shell_thd, arg) {
    chRegSetThreadName("shell");
    while (1) {
        shellThread(arg);
    }
}

static const ShellCommand commands[] = {
    {"mem", cmd_mem},
    {"ps", cmd_threads},
    {"cpu", cmd_cpu},
    {"reboot", cmd_reboot},
    {"panic", cmd_panic},
    {"panic_get", cmd_panic_get},
    {"parameter_list", cmd_parameter_list},
    {"parameter_set", cmd_parameter_set},
    {"log_lvl", cmd_stdout_log_lvl},
    {"parameter_load", cmd_parameter_load},
    {"parameter_save", cmd_parameter_save},
    {"parameter_erase", cmd_parameter_erase},
    {NULL, NULL}
};

static ShellConfig shell_cfg = {
    (BaseSequentialStream *)&SD1,
    commands
};



void run_shell(BaseSequentialStream *serial) {
    shell_cfg.sc_channel = serial;
    chThdCreateStatic(shell_thd_wa, sizeof(shell_thd_wa), THD_PRIO_SHELL, shell_thd, (void *)&shell_cfg);
}
