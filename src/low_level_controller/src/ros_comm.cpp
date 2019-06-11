#include "ch.h"
#include "hal.h"
#include <ros_interface/msg.h>
#include "thread_prio.h"
#include "timestamp.h"
#include "rc_input.hpp"
#include "sensors.hpp"
#include "parameter_storage.h"
#include <parameter/parameter_msgpack.h>
#include "control_loop.hpp"
#include "log.h"

#include <math.h>

#include "ros_comm.hpp"

comm_interface_t comm_if;


static void comm_rcv_cb(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
{
    switch (static_cast<RosInterfaceCommMsgID>(msg_id)) {
    case RosInterfaceCommMsgID::PING:
        comm_send(&comm_if, RosInterfaceCommMsgID::PONG, msg, len);
        break;
    case RosInterfaceCommMsgID::SET_PARAMETERS: {
        int ret = parameter_msgpack_read(&parameters, msg, len, [](void *arg, const char *id, const char *err){
            (void)arg;
            log_error("parameter read error %s: %s", id, err);
        }, NULL);
        parameter_save_to_persistent_store(); // todo check return
        uint8_t ok = (ret == 0) ? 1: 0;
        comm_send(&comm_if, RosInterfaceCommMsgID::SET_PARAMETERS_RES, &ok, 1);
        break;
    }
    default:
        break;
    }
}

static THD_WORKING_AREA(comm_rx_thread_wa, (2000));
static THD_FUNCTION(comm_rx_thread, arg) {
    (void)arg;
    chRegSetThreadName("comm_rx");
    while (1) {
        comm_receive(&comm_if);
    }
}

static THD_WORKING_AREA(comm_tx_thread_wa, (2000));
static THD_FUNCTION(comm_tx_thread, arg) {
    (void)arg;
    chRegSetThreadName("comm_tx");

    auto rc_in_sub = msgbus::subscribe(rc_input);
    auto output_sub = msgbus::subscribe(actuator_output_topic);
    auto rate_setpoint_rpy_sub = msgbus::subscribe(rate_setpoint_rpy_topic);
    auto rate_measured_rpy_sub = msgbus::subscribe(rate_measured_rpy_topic);
    auto rate_ctrl_output_rpy_sub = msgbus::subscribe(rate_ctrl_output_rpy_topic);
    auto rate_gyro_sub = msgbus::subscribe(rate_gyro);
    auto accelerometer_sub = msgbus::subscribe(accelerometer);
    auto magnetometer_sub = msgbus::subscribe(magnetometer);

    std::array<msgbus::SubscriberBase*, 7> sub_list = {
        &rc_in_sub,
        &rate_gyro_sub,
        // accelerometer_sub not checked for update
        &output_sub,
        &rate_setpoint_rpy_sub,
        &rate_measured_rpy_sub,
        &rate_ctrl_output_rpy_sub,
        &magnetometer_sub,
    };
    while (true) {
        comm_send(&comm_if, RosInterfaceCommMsgID::HEARTBEAT, NULL, 0);

        uint64_t timestamp = timestamp_get();
        comm_send(&comm_if, RosInterfaceCommMsgID::TIME, &timestamp, sizeof(timestamp));

        static char buf[1000];

        if (rc_in_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(rc_in_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::RC_INPUT, buf, serializer.writer().size());
        }

        if (rate_gyro_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(rate_gyro_sub.get_value());
            serializer.Write(accelerometer_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::IMU, buf, serializer.writer().size());
        }

        if (magnetometer_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(magnetometer_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::MAGNETOMETER, buf, serializer.writer().size());
        }

        if (output_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(output_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::ACTUATOR_OUTPUT, buf, serializer.writer().size());
        }

        if (rate_setpoint_rpy_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(rate_setpoint_rpy_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::RATE_CTRL_SETPOINT_RPY, buf, serializer.writer().size());
        }

        if (rate_measured_rpy_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(rate_measured_rpy_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::RATE_CTRL_MEASURED_RPY, buf, serializer.writer().size());
        }

        if (rate_ctrl_output_rpy_sub.has_update()) {
            auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
            serializer.Write(rate_ctrl_output_rpy_sub.get_value());
            comm_send(&comm_if, RosInterfaceCommMsgID::RATE_CTRL_OUTPUT_RPY, buf, serializer.writer().size());
        }

        chThdSleepMilliseconds(1); // todo this is a temporary fix to avoid using 100% CPU
        msgbus::wait_for_update_on_any(sub_list.begin(), sub_list.end());
    }
}

static log_handler_t log_handler_comm;
static void log_handler_comm_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    comm_send(&comm_if, RosInterfaceCommMsgID::LOG, msg, len);
}


void ros_comm_init(SerialDriver *sd)
{
    comm_init(&comm_if, sd, comm_rcv_cb);

    log_handler_register(&log_handler_comm, LOG_LVL_DEBUG, log_handler_comm_cb);

    chThdCreateStatic(comm_rx_thread_wa, sizeof(comm_rx_thread_wa),
                      THD_PRIO_COMM_RX, comm_rx_thread, NULL);
    chThdCreateStatic(comm_tx_thread_wa, sizeof(comm_tx_thread_wa),
                      THD_PRIO_COMM_TX, comm_tx_thread, NULL);
}