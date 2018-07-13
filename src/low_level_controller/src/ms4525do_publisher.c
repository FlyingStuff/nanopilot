#include "thread_prio.h"
#include "types/sensors.h"
#include "log.h"
#include "main.h"
#include "timestamp.h"
#include "msgbus/msgbus.h"
#include "sensors/ms4525do.h"
#include <assert.h>

#include "ms4525do_publisher.h"

static I2CDriver *i2c_driver;
static const char *topic_name;

static THD_WORKING_AREA(ms4525do_publisher_wa, 800);
static THD_FUNCTION(ms4525do_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("ms4525do_publisher");
    ms4525do_t ms4525do;
    static msgbus_topic_t ms4525do_topic; // must be static in case the thread exits
    static dynamic_pressure_sample_t ms4525do_topic_buf; // must be static in case the thread exits
    msgbus_topic_create(&ms4525do_topic, &bus, &dynamic_pressure_sample_type, &ms4525do_topic_buf, topic_name);

    i2cAcquireBus(i2c_driver);
    assert(ms4525do_init(&ms4525do, i2c_driver, 'A', 'I', 2, 'D') == 0); // MS4525DO-5AI2D
    i2cReleaseBus(i2c_driver);

    while (true) {
        dynamic_pressure_sample_t ms4525do_sample;
        float p, temp;
        i2cAcquireBus(i2c_driver);
        timestamp_t ts = timestamp_get();
        int status = ms4525do_read(&ms4525do, &p, &temp);
        i2cReleaseBus(i2c_driver);
        if (status == MS4525DO_READ_RES_OK) {
            ms4525do_sample.dynamic_pressure = p;
            ms4525do_sample.temperature = temp;
            ms4525do_sample.timestamp_ns = ts;
            msgbus_topic_publish(&ms4525do_topic, &ms4525do_sample);

            // log_info("ms4525do status: %d, %f Pa, %f °C", status, p, temp);
        } else if (status == MS4525DO_READ_RES_SENSOR_FAULT) {
            log_warning("ms4525do reported sensor fault");
        } else if (status == MS4525DO_READ_RES_I2C_ERR) {
            // todo recover i2c locked state
            log_error("ms4525do I2C error, exiting driver");
            break; // for now just exit
        }
    }
}

void ms4525do_publisher_start(I2CDriver *i2c, const char *topic)
{
    i2c_driver = i2c;
    topic_name = topic;
    chThdCreateStatic(ms4525do_publisher_wa, sizeof(ms4525do_publisher_wa), THD_PRIO_SENSOR_DRV_I2C_EXT, ms4525do_publisher, NULL);
}
