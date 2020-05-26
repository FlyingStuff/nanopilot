#include <ch.h>
#include <hal.h>
#include "control_loop.hpp"
#include "thread_prio.h"

#include "arm_led.h"


static THD_WORKING_AREA(arm_led_thread_wa, 256);
static THD_FUNCTION(arm_led_thread, arg) {

    (void)arg;
    chRegSetThreadName("arm led");
    auto control_status_sub = msgbus::subscribe(control_status_topic);
    control_status_sub.wait_for_update();
    while (true) {
        if (arm_switch_is_armed()) {
            auto stat = control_status_sub.get_value();
            switch (stat.mode) {
            default:
            case CTRL_MODE_DISARMED:
                arm_led_set(true);
                chThdSleepMilliseconds(100);
                break;
            case CTRL_MODE_MANUAL:
                arm_led_set(true);
                chThdSleepMilliseconds(100);
                arm_led_set(false);
                chThdSleepMilliseconds(100);
                arm_led_set(true);
                chThdSleepMilliseconds(100);
                arm_led_set(false);
                chThdSleepMilliseconds(700);
                break;
            case CTRL_MODE_AP:
                arm_led_set(true);
                chThdSleepMilliseconds(50);
                arm_led_set(false);
                chThdSleepMilliseconds(50);
                break;
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

