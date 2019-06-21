#include <ch.h>
#include <hal.h>
#include "control_loop.hpp"
#include "thread_prio.h"

#include "arm_led.h"


static THD_WORKING_AREA(arm_led_thread_wa, 256);
static THD_FUNCTION(arm_led_thread, arg) {

    (void)arg;
    chRegSetThreadName("arm led");
    auto armed_sub = msgbus::subscribe(output_armed_topic);
    auto ap_ctrl_sub = msgbus::subscribe(ap_in_control_topic);

    while (true) {
        if (arm_switch_is_armed()) {
            if (armed_sub.has_value() && armed_sub.get_value()) {
                if (ap_ctrl_sub.has_value() && ap_ctrl_sub.get_value()) {
                    arm_led_set(true);
                    chThdSleepMilliseconds(50);
                    arm_led_set(false);
                    chThdSleepMilliseconds(50);
                } else {
                    arm_led_set(true);
                    chThdSleepMilliseconds(100);
                    arm_led_set(false);
                    chThdSleepMilliseconds(100);
                    arm_led_set(true);
                    chThdSleepMilliseconds(100);
                    arm_led_set(false);
                    chThdSleepMilliseconds(700);
                }
            } else {
                arm_led_set(true);
                chThdSleepMilliseconds(100);
            }
        } else {
            arm_led_set(false);
            chThdSleepMilliseconds(100);
        }
    }
}

void arm_led_task_start(void)
{
    chThdCreateStatic(arm_led_thread_wa, sizeof(arm_led_thread_wa), THD_PRIO_LED, arm_led_thread, NULL);
}

