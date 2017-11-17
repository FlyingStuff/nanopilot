#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>

#include "onboardsensors.h"
#include "serial-datagram/serial_datagram.h"
#include "parameter/parameter_print.h"
#include "ts/type_print.h"
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
                 (uint32_t)tp->p_ctx.r13->lr,
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

static void cmd_cpu(BaseSequentialStream *chp, int argc, char *argv[]) {
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
                    thread_times[i].tp->p_name);
        }

        free(thread_times);
    } else {
        chprintf(chp, "malloc failed\n");
    }
}

static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    chprintf(chp, "git version: %s\n", build_git_sha);
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


static void cmd_topic_print(BaseSequentialStream *stream, int argc, char *argv[]) {
    if (argc != 1) {
        chprintf(stream, "usage: topic_print name\n");
        return;
    }
    msgbus_subscriber_t sub;
    if (msgbus_topic_subscribe(&sub, &bus, argv[0], MSGBUS_TIMEOUT_IMMEDIATE)) {
        if (msgbus_subscriber_topic_is_valid(&sub)) {
            msgbus_topic_t *topic = msgbus_subscriber_get_topic(&sub);
            const ts_type_definition_t *type = msgbus_topic_get_type(topic);
            void *buf = malloc(type->struct_size);
            if (buf == NULL) {
                chprintf(stream, "malloc failed\n");
                return;
            }
            msgbus_subscriber_read(&sub, buf);
            ts_print_type((void (*)(void *, const char *, ...))chprintf,
                              stream, type, buf);
            free(buf);
        } else {
            chprintf(stream, "topic not published yet\n");
            return;
        }
    } else {
        chprintf(stream, "topic doesn't exist\n");
        return;
    }
}

static void cmd_topic_list(BaseSequentialStream *stream, int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    msgbus_topic_t *topic = msgbus_iterate_topics(&bus);
    while (topic != NULL) {
        chprintf(stream, "%s\n", msgbus_topic_get_name(topic));
        topic = msgbus_iterate_topics_next(topic);
    }
}

const ShellCommand shell_commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"cpu", cmd_cpu},
  {"version", cmd_version},
  {"safemode", cmd_safemode},
  {"reboot", cmd_reboot},
  {"bootloader", cmd_bootloader},
  {"panic", cmd_panic},
  {"panic_get", cmd_panic_get},
  {"parameter_list", cmd_parameter_list},
  {"parameter_set", cmd_parameter_set},
  {"topic_print", cmd_topic_print},
  {"topic_list", cmd_topic_list},
  {NULL, NULL}
};

