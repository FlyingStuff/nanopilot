#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <string.h>
#include <usbcfg.h>

#include "log.h"
#include "git_revision.h"
#include "thread_prio.h"
#include "sdcard.h"
#include "shell_cmds.h"
#include "sumd_input.h"
#include "onboardsensors.h"
#include "serial-datagram/serial_datagram.h"
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "error.h"
#include "parameter/parameter.h"
#include "parameter/parameter_print.h"
#include "sdlog.h"
#include "stream.h"
#include "timestamp/timestamp_stm32.h"
#include "attitude_determination.h"

BaseSequentialStream* stdout;
SerialUSBDriver SDU1;

parameter_namespace_t parameters;


/*
 *  Heartbeat, Error LED thread
 *
 * The Heartbeat-LED double-flashes every second in normal mode and continuously
 *  flashes in safemode. If the error level is above NORMAL, the Heartbeat-LED
 *  is off and the Error-LED indicates the error level.
 * If the error level is WARNING, the Error-LED blinks slowly (once per second)
 * If the error level is CRITICAL, the Error-LED blinks rapidly (10 times per second)
 * If a kernel panic occurs the Error-LED is on and all LEDs are off.
 */
static THD_WORKING_AREA(led_task_wa, 128);
static THD_FUNCTION(led_task, arg)
{
    (void)arg;
    chRegSetThreadName("led_task");
    while (1) {
        int err = error_level_get();
        if (err == ERROR_LEVEL_WARNING) {
            palSetPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(500);
            palClearPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(500);
        } else if (err == ERROR_LEVEL_CRITICAL) {
            palSetPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(50);
            palClearPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(50);
        } else {
            palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
            chThdSleepMilliseconds(80);
            palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
            chThdSleepMilliseconds(80);
            palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
            chThdSleepMilliseconds(80);
            palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
            if (safemode_active()) {
                chThdSleepMilliseconds(80);
            } else {
                chThdSleepMilliseconds(760);
            }
        }
    }
}


static void boot_message(void)
{
    chprintf(stdout, "\n\n\n");

    const char *panic_msg = get_panic_message();
    if (panic_msg != NULL) {
        log_warning("reboot after panic:\n%s\n", panic_msg);
    } else {
        log_info("boot");
    }
    if (safemode_active()) {
        log_warning("safemode active");
    }
    log_info("git: %s branch: %s", build_git_version, build_git_branch);
    log_info("built: %s", build_date);
}


BaseSequentialStream *get_base_seq_stream_device_from_str(const char *name)
{
    if (strcmp(name, "CONN1") == 0) {
        return (BaseSequentialStream*)&UART_CONN1;
    }
    if (strcmp(name, "CONN2") == 0) {
        return (BaseSequentialStream*)&UART_CONN2;
    }
    if (strcmp(name, "CONN3") == 0) {
        return (BaseSequentialStream*)&UART_CONN3;
    }
    if (strcmp(name, "CONN4") == 0) {
        return (BaseSequentialStream*)&UART_CONN4;
    }
    if (strcmp(name, "USB") == 0) {
        return (BaseSequentialStream*)&SDU1;
    }
    if (strcmp(name, "CONN_I2C") == 0) {
        return (BaseSequentialStream*)&UART_CONN_I2C;
    }
    log_warning("unknown io port %s", name);
    return NULL;
}


static parameter_namespace_t service_param;
static parameter_t shell_port;
static char shell_port_buf[10];
static parameter_t sumd_in_uart;
static char sumd_in_uart_buf[10];
static parameter_t stream_out;
static char stream_out_buf[10];


static void service_parameters_declare(parameter_namespace_t *root)
{
    parameter_namespace_declare(&service_param, root, "service");

    parameter_string_declare_with_default(&shell_port,
            &service_param, "shell_port", shell_port_buf,
            sizeof(shell_port_buf), "USB");
    parameter_string_declare_with_default(&sumd_in_uart,
            &service_param, "sumd_input", sumd_in_uart_buf,
            sizeof(sumd_in_uart_buf), "CONN2");
    parameter_string_declare_with_default(&stream_out,
            &service_param, "stream_output", stream_out_buf,
            sizeof(stream_out_buf), "CONN3");
}


static parameter_namespace_t io_param;
static parameter_t uart_conn2_baud;
static parameter_t uart_conn3_baud;
static parameter_t uart_conn4_baud;
static parameter_t uart_conn_i2c_baud;

static void io_parameters_declare(parameter_namespace_t *root)
{
    parameter_namespace_declare(&io_param, root, "io");

    parameter_integer_declare_with_default(&uart_conn2_baud,
            &io_param, "conn2_baud", SERIAL_DEFAULT_BITRATE);
    parameter_integer_declare_with_default(&uart_conn3_baud,
            &io_param, "conn3_baud", SERIAL_DEFAULT_BITRATE);
    parameter_integer_declare_with_default(&uart_conn4_baud,
            &io_param, "conn4_baud", SERIAL_DEFAULT_BITRATE);
    parameter_integer_declare_with_default(&uart_conn_i2c_baud,
            &io_param, "conn_i2c_baud", SERIAL_DEFAULT_BITRATE);
}

static void io_setup(void)
{
    SerialConfig uart_config = { SERIAL_DEFAULT_BITRATE, 0,
                                 USART_CR2_STOP1_BITS, 0 };
    uart_config.speed = parameter_integer_get(&uart_conn2_baud);
    sdStart(&UART_CONN2, &uart_config);
    uart_config.speed = parameter_integer_get(&uart_conn3_baud);
    sdStart(&UART_CONN3, &uart_config);
    uart_config.speed = parameter_integer_get(&uart_conn4_baud);
    sdStart(&UART_CONN4, &uart_config);
    uart_config.speed = parameter_integer_get(&uart_conn_i2c_baud);
    sdStart(&UART_CONN_I2C, &uart_config);
}


static void services_init(void)
{
    service_parameters_declare(&parameters);
    io_parameters_declare(&parameters);
    onboardsensors_declare_parameters(&parameters);
}


static void services_start(void)
{
    char buf[10];
    onboard_sensors_start();

    parameter_string_get(&sumd_in_uart, buf, sizeof(buf));
    sumd_input_start(get_base_seq_stream_device_from_str(buf));

    sdlog_start();

    parameter_string_get(&stream_out, buf, sizeof(buf));
    stream_start(get_base_seq_stream_device_from_str(buf));

    run_attitude_determination();
}


int main(void)
{
    halInit();
    chSysInit();

    timestamp_stm32_init();

    board_io_pwr_en(true);
    board_sensor_pwr_en(true);

    error_init();

    chThdCreateStatic(led_task_wa, sizeof(led_task_wa), THD_PRIO_LED, led_task, NULL);

    // standard output
    sdStart(&UART_CONN1, NULL);
    stdout = (BaseSequentialStream*)&UART_CONN1;

    boot_message();

    // initialization
    parameter_namespace_declare(&parameters, NULL, NULL); // root namespace
    services_init();

    // mount SD card
    chThdSleepMilliseconds(100);
    board_sdcard_pwr_en(true);
    chThdSleepMilliseconds(100);
    sdcStart(&SDCD1, NULL);
    sdcard_mount();

    // load parameters from SD card
    log_info("loading parameters from sd card")
    sdcard_read_parameter(&parameters, "/config.msgpack");
    log_info("current parameters:")
    parameter_print(&parameters, (parameter_printfn_t)chprintf, stdout);

    // UART driver
    io_setup();

    // USB serial driver
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    // start all services
    services_start();

    shellInit();
    char buf[10];
    parameter_string_get(&shell_port, buf, sizeof(buf));
    BaseSequentialStream* shell_dev = get_base_seq_stream_device_from_str(buf);
    static thread_t *shelltp = NULL;
    static ShellConfig shell_cfg;
    shell_cfg.sc_channel = shell_dev;
    shell_cfg.sc_commands = shell_commands;

    while (true) {
        if (shelltp == NULL && shell_dev != NULL) {
            shelltp = shellCreate(&shell_cfg, THD_WORKING_AREA_SIZE(2048), THD_PRIO_SHELL);
        } else if (shelltp != NULL && chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);
            shelltp = NULL;
        }

        sdcard_automount();

        chThdSleepMilliseconds(500);
    }
}
