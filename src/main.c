#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "usbcfg.h"


SerialUSBDriver SDU1;


/*
 *  Heartbeat
 */
static THD_WORKING_AREA(heartbeat_wa, 128);
static THD_FUNCTION(heartbeat, arg)
{
    (void)arg;
    chRegSetThreadName("heartbeat");
    while (1) {
        palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(760);
    }
    return 0;
}


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
             (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state]);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};



int main(void)
{
    halInit();
    chSysInit();

    chThdCreateStatic(heartbeat_wa, sizeof(heartbeat_wa), LOWPRIO, heartbeat, NULL);

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    shellInit();
    thread_t *shelltp = NULL;
    while (true) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
            }
        } else {
            if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);
    }
}
