#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <string.h>
#include <usbcfg.h>

#include "log.h"
#include "git_revision.h"
#include "thread_prio.h"
#include "sdcard.h"
#include "sumd_input.h"
#include "onboardsensors.h"
#include "ms4525do_publisher.h"
#include "serial-datagram/serial_datagram.h"
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "error.h"
#include "parameter/parameter.h"
#include "parameter/parameter_print.h"
#include "sdlog.h"
#include "stream.h"
#include "datagram_message_comm.h"
#include "timestamp/timestamp_stm32.h"
#include "attitude_determination.h"
#include <ff.h>

#include "sensors/ms4525do.h"

#include "main.h"

BaseSequentialStream* stdout;

parameter_namespace_t parameters;
parameter_t board_name;
char board_name_p_buf[32];

msgbus_t bus;

void sd_card_activity(void)
{
    chSysLock();
    palTogglePad(GPIOB, GPIOB_LED_SDCARD);
    chSysUnlock();
}


static void sd_card_activity_periodic_reset(void)
{
    if (fatfs_mounted) {
        palSetPad(GPIOB, GPIOB_LED_SDCARD);
    } else {
        palClearPad(GPIOB, GPIOB_LED_SDCARD);
    }
}


/*
 *  Heartbeat LED thread
 *
 * The Heartbeat-LED double-flashes every second in normal mode and continuously
 *  flashes in safemode.
 */
static THD_WORKING_AREA(led_task_wa, 128);
static THD_FUNCTION(led_task, arg)
{
    (void)arg;
    chRegSetThreadName("led_task");
    while (1) {
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
        sd_card_activity_periodic_reset();
    }
}


static void boot_message(void)
{
    const char *panic_msg = get_panic_message();
    if (panic_msg != NULL) {
        log_warning("reboot after panic:\n%s\n", panic_msg);
    } else {
        log_info("normal boot");
    }
    if (safemode_active()) {
        log_warning("safemode active");
    }
    log_info("git hash: %s", build_git_sha);
}

#define STREAM_DEV_STR_SIZE 10
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
static char shell_port_buf[STREAM_DEV_STR_SIZE];
static parameter_t sumd_in_uart;
static char sumd_in_uart_buf[STREAM_DEV_STR_SIZE];
static parameter_t datagram_message_port;
static char datagram_message_port_buf[STREAM_DEV_STR_SIZE];


static void service_parameters_declare(parameter_namespace_t *root)
{
    parameter_namespace_declare(&service_param, root, "service");

    parameter_string_declare_with_default(&shell_port,
            &service_param, "shell_port", shell_port_buf,
            sizeof(shell_port_buf), "CONN1");
    parameter_string_declare_with_default(&sumd_in_uart,
            &service_param, "sumd_input", sumd_in_uart_buf,
            sizeof(sumd_in_uart_buf), "CONN2");
    parameter_string_declare_with_default(&datagram_message_port,
            &service_param, "datagram_message_port", datagram_message_port_buf,
            sizeof(datagram_message_port_buf), "CONN3");
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

    // uart_config.speed = parameter_integer_get(&uart_conn_i2c_baud);
    // sdStart(&UART_CONN_I2C, &uart_config);
    const I2CConfig i2c_cfg = {
        .op_mode = OPMODE_I2C,
        .clock_speed = 400000,
        .duty_cycle = FAST_DUTY_CYCLE_2
    };
    i2cStart(&I2C_CONN, &i2c_cfg);
}


static void services_init(void)
{
    service_parameters_declare(&parameters);
    io_parameters_declare(&parameters);
    onboardsensors_declare_parameters(&parameters);
    datagram_message_init();
}


static void services_start(void)
{
    char buf[STREAM_DEV_STR_SIZE];
    onboard_sensors_start();
    ms4525do_publisher_start(&I2C_CONN, "/sensors/ms4525do");

    parameter_string_get(&sumd_in_uart, buf, sizeof(buf));
    sumd_input_start(get_base_seq_stream_device_from_str(buf));

    sdlog_start();

    parameter_string_get(&datagram_message_port, buf, sizeof(buf));
    datagram_message_start(get_base_seq_stream_device_from_str(buf));
    stream_start(get_base_seq_stream_device_from_str(buf));

    run_attitude_determination();
}

#include "ts/type_print.h"

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
            chThdSleepMilliseconds(1000);
            uint32_t rate = msgbus_subscriber_read(&sub, buf);
            ts_print_type((void (*)(void *, const char *, ...))chprintf,
                              stream, type, buf);
            chprintf(stream, "rate %d Hz\n\n", rate);
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

static log_handler_t log_handler_stdout;
static void log_handler_stdout_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    streamWrite(stdout, (uint8_t*)msg, len);
}


static FIL log_file_sdcard;
static log_handler_t log_handler_sdcard;
static void log_handler_sdcard_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    UINT _bytes_written;
    int ret = f_write(&log_file_sdcard, msg, len, &_bytes_written);
    if (ret == 0) {
        f_sync(&log_file_sdcard);
    }
}

static void log_handler_sdcard_init(const char *path, log_level_t log_lvl)
{
    FRESULT res = f_open(&log_file_sdcard, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (res == 0) {
        log_handler_register(&log_handler_sdcard, log_lvl, log_handler_sdcard_cb);
        log_info("log file %s successfully opened", path);
    } else {
        log_warning("opening log file %s failed", path);
    }
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
    chprintf(stdout, "\n\n\n");

    log_init();
    log_handler_register(&log_handler_stdout, LOG_LVL_DEBUG, log_handler_stdout_cb);
    log_info("=== boot ===");

    // mount SD card
    chThdSleepMilliseconds(100);
    board_sdcard_pwr_en(true);
    chThdSleepMilliseconds(100);
    sdcStart(&SDCD1, NULL);
    sdcard_mount();

    log_handler_sdcard_init("/log/log.txt", LOG_LVL_INFO);

    boot_message();

    // initialization
    parameter_namespace_declare(&parameters, NULL, NULL); // root namespace
    parameter_string_declare_with_default(&board_name,
                                          &parameters,
                                          "name",
                                          board_name_p_buf,
                                          sizeof(board_name_p_buf),
                                          "ins-board");

    msgbus_init(&bus);

    services_init();


    // load parameters from SD card
    log_info("loading parameters from sd card");
    sdcard_read_parameter(&parameters, "/config.msgpack");
    chprintf(stdout, "current parameters:");
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

    // shellInit();
    // char buf[STREAM_DEV_STR_SIZE];
    // parameter_string_get(&shell_port, buf, sizeof(buf));
    // BaseSequentialStream* shell_dev = get_base_seq_stream_device_from_str(buf);
    // static thread_t *shelltp = NULL;
    // static ShellConfig shell_cfg;
    // shell_cfg.sc_channel = shell_dev;
    // shell_cfg.sc_commands = shell_commands;

    chThdSleepMilliseconds(500);

    while (true) {
        // if (shelltp == NULL && shell_dev != NULL) {
        //     static THD_WORKING_AREA(shell_wa, 2048);
        //     shelltp = shellCreateStatic(&shell_cfg, shell_wa, sizeof(shell_wa), THD_PRIO_SHELL);
        // } else if (shelltp != NULL && chThdTerminatedX(shelltp)) {
        //     shelltp = NULL;
        // }

        sdcard_automount();

        char *argv[] = {"/sensors/mpu6000", "/sensors/ms5611", "/attitude_estimator_inertial/attitude_body_to_world"};
        int i;
        for (i=0; i < 3; i++) {
            chprintf(stdout, "\n%s:\n", argv[i]);
            cmd_topic_print(stdout, 1, &argv[i]);
        }
        chThdSleepMilliseconds(500);
    }
}
