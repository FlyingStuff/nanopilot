#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>
#include <string.h>

#include "parameter/parameter_print.h"
#include "parameter_storage.h"
#include "syscalls.h"
#include "panic_handler.h"
#include "log_handlers.h"


void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    chprintf(chp, "used: %u bytes, free: %u bytes\n",
             sbrk_stat_used(), sbrk_stat_free());
}

void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
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
                 (uint32_t)tp->ctx.sp->lr,
                 (uint32_t)tp->ctx.sp,
                 (char *)tp->ctx.sp - (char *)tp->wabase,
                 (uint32_t)tp->prio,
                 states[tp->state],
                 chThdGetTicksX(tp),
                 tp->name);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
    chprintf(chp, "[sytick %d @ %d Hz]\n", chVTGetSystemTime(), CH_CFG_ST_FREQUENCY);
}

void cmd_cpu(BaseSequentialStream *chp, int argc, char *argv[]) {
    thread_t *tp;

    (void)argv;
    (void)argc;

    tp = chRegFirstThread();
    int thd_cnt = 0;
    while (tp != NULL) {
        thd_cnt++;
        tp = chRegNextThread(tp);
    }

    struct thd_timing_s {
        thread_t *tp;
        systime_t t1;
        systime_t t2;
    };

    struct thd_timing_s *thread_times = malloc(sizeof(struct thd_timing_s)*thd_cnt);
    if (thread_times != NULL) {
        int i;
        tp = chRegFirstThread();
        for (i = 0; i < thd_cnt; i++) {
            thread_times[i].tp = tp;
            tp = chRegNextThread(tp);
        }

        systime_t t1 = chVTGetSystemTime();
        for (i = 0; i < thd_cnt; i++) {
            thread_times[i].t1 = chThdGetTicksX(thread_times[i].tp);
        }

        chThdSleepMilliseconds(1000);

        systime_t t2 = chVTGetSystemTime();
        for (i = 0; i < thd_cnt; i++) {
            thread_times[i].t2 = chThdGetTicksX(thread_times[i].tp);
        }
        chprintf(chp, "thread times:\n");
        chprintf(chp, "   total /1000 name\n");
        for (i = 0; i < thd_cnt; i++) {
            uint32_t share = 1000 * (thread_times[i].t2 - thread_times[i].t1) / (t2 - t1);
            chprintf(chp, "%8lu  %4u %s\n",
                    thread_times[i].t2,
                    share,
                    thread_times[i].tp->name);
        }

        free(thread_times);
    } else {
        chprintf(chp, "malloc failed\n");
    }
}


void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    NVIC_SystemReset();
}

void cmd_panic(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    chSysHalt("panic test shell command");
}

void cmd_panic_get(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    chprintf(chp, "panic was: %s\n", get_panic_message());
}

void cmd_parameter_list(BaseSequentialStream *stream, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    parameter_print(&parameters, (parameter_printfn_t)chprintf, stream);
}

void cmd_parameter_set(BaseSequentialStream *stream, int argc, char *argv[]) {
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
    } else if (p->type == _PARAM_TYPE_INTEGER) {
        parameter_integer_set(p, atoi(argv[1]));
    } else {
        chprintf(stream, "unsupported type %d\n", p->type);
    }
}

void cmd_stdout_log_lvl(BaseSequentialStream *stream, int argc, char *argv[]) {
    if (argc < 1) {
        chprintf(stream, "missing log_lvl [debug, info, warning, error]\n");
        return;
    }
    if (strcmp(argv[0], "debug") == 0) {
        log_handler_change_log_level(&log_handler_stdout, LOG_LVL_DEBUG);
    } else if (strcmp(argv[0], "info") == 0) {
        log_handler_change_log_level(&log_handler_stdout, LOG_LVL_INFO);
    } else if (strcmp(argv[0], "warning") == 0) {
        log_handler_change_log_level(&log_handler_stdout, LOG_LVL_WARNING);
    } else if (strcmp(argv[0], "error") == 0) {
        log_handler_change_log_level(&log_handler_stdout, LOG_LVL_ERROR);
    }
}

