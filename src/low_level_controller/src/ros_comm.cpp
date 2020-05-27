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
#include <cstring>

#include <math.h>

#include "ros_comm.hpp"

comm_interface_t comm_if;


static void comm_rcv_cb(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
{
    switch (static_cast<RosInterfaceCommMsgID>(msg_id)) {
    case RosInterfaceCommMsgID::PING:
        comm_send(&comm_if, RosInterfaceCommMsgID::PONG, msg, len);
        break;
    case RosInterfaceCommMsgID::TIME_SYNC:
    {
        uint64_t timestamp = timestamp_get();
        // timestamp = chTimeI2US(chVTGetSystemTimeX())*1000;
        comm_send(&comm_if, RosInterfaceCommMsgID::TIME, &timestamp, sizeof(timestamp));
        break;
    }
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
    case RosInterfaceCommMsgID::CONTOLLER_ATTITUDE_SETPT: {
        auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
        attitude_controller_input_t ctrl_msg;
        if (deserializer.Read(&ctrl_msg)) {
            attitude_controller_input_topic.publish(ctrl_msg);
        }
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

static msgbus::Topic<int> log_update_trigger;
static log_handler_t log_handler_comm;
static char log_buffer[300];
static size_t log_buffer_wi;
static MUTEX_DECL(log_mutex);
static void log_handler_comm_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    chMtxLock(&log_mutex);
    // comm_send(&comm_if, RosInterfaceCommMsgID::LOG, msg, len);
    if (log_buffer_wi + len > sizeof(log_buffer)) {
        // drop message
    } else {
        memcpy(&log_buffer[log_buffer_wi], msg, len);
        log_buffer_wi += len;
    }
    chMtxUnlock(&log_mutex);
    log_update_trigger.publish(0);
}

template<typename MsgType>
void send_if_updated(msgbus::Subscriber<MsgType> &sub, comm_msg_id_t msg_id, char *buf, size_t buf_sz)
{
    auto serializer = nop::Serializer<nop::BufferWriter>(buf, buf_sz);
    if (sub.has_update()) {
        serializer.Write(sub.get_value());
        comm_send(&comm_if, msg_id, buf, serializer.writer().size());
    }
}

static THD_WORKING_AREA(comm_tx_thread_wa, (2000));
static THD_FUNCTION(comm_tx_thread, arg) {
    (void)arg;
    chRegSetThreadName("comm_tx");

    auto rc_in_sub = msgbus::subscribe(rc_input_topic);
    auto output_sub = msgbus::subscribe(actuator_output_topic);
    auto control_status_sub = msgbus::subscribe(control_status_topic);
    // auto rate_setpoint_rpy_sub = msgbus::subscribe(rate_setpoint_rpy_topic);
    // auto rate_measured_rpy_sub = msgbus::subscribe(rate_measured_rpy_topic);
    // auto rate_ctrl_output_rpy_sub = msgbus::subscribe(rate_ctrl_output_rpy_topic);
    // auto output_armed_sub = msgbus::subscribe(output_armed_topic);
    // auto ap_in_control_sub = msgbus::subscribe(ap_in_control_topic);
    auto imu_sub = msgbus::subscribe(imu);
    auto magnetometer_sub = msgbus::subscribe(magnetometer);
    auto ap_control_latency_sub = msgbus::subscribe(ap_control_latency_topic);
    auto log_update_trigger_sub = msgbus::subscribe(log_update_trigger);

    auto attitude_controller_status_sub = msgbus::subscribe(attitude_controller_status_topic);

    std::array<msgbus::SubscriberBase*, 8> sub_list = {
        &rc_in_sub,
        &output_sub,
        &control_status_sub,
        &imu_sub,
        &magnetometer_sub,
        // &rate_setpoint_rpy_sub,
        // &rate_measured_rpy_sub,
        // &rate_ctrl_output_rpy_sub,
        // &output_armed_sub,
        // &ap_in_control_sub,
        &ap_control_latency_sub,
        &log_update_trigger_sub,
        &attitude_controller_status_sub,
    };
    while (true) {
        // comm_send(&comm_if, RosInterfaceCommMsgID::HEARTBEAT, NULL, 0);

        static char buf[1000];

        send_if_updated(rc_in_sub, RosInterfaceCommMsgID::RC_INPUT, buf, sizeof(buf));
        if (imu_sub.has_update()) {
            static unsigned i = 0;
            if (i++ % 32 == 0) {
                send_if_updated(imu_sub, RosInterfaceCommMsgID::IMU, buf, sizeof(buf));
            } else {
                imu_sub.get_value();
            }
        }
        send_if_updated(magnetometer_sub, RosInterfaceCommMsgID::MAGNETOMETER, buf, sizeof(buf));
        send_if_updated(output_sub, RosInterfaceCommMsgID::ACTUATOR_OUTPUT, buf, sizeof(buf));
        send_if_updated(control_status_sub, RosInterfaceCommMsgID::CONTROL_STATUS, buf, sizeof(buf));
        // send_if_updated(rate_setpoint_rpy_sub, RosInterfaceCommMsgID::RATE_CTRL_SETPOINT_RPY, buf, sizeof(buf));
        // send_if_updated(rate_measured_rpy_sub, RosInterfaceCommMsgID::RATE_CTRL_MEASURED_RPY, buf, sizeof(buf));
        // send_if_updated(rate_ctrl_output_rpy_sub, RosInterfaceCommMsgID::RATE_CTRL_OUTPUT_RPY, buf, sizeof(buf));
        // send_if_updated(output_armed_sub, RosInterfaceCommMsgID::OUTPUT_IS_ARMED, buf, sizeof(buf));
        // send_if_updated(ap_in_control_sub, RosInterfaceCommMsgID::AP_IN_CONTROL, buf, sizeof(buf));
        send_if_updated(ap_control_latency_sub, RosInterfaceCommMsgID::AP_LATENCY, buf, sizeof(buf));
        send_if_updated(attitude_controller_status_sub, RosInterfaceCommMsgID::CONTOLLER_ATTITUDE_STATUS, buf, sizeof(buf));

        if (log_update_trigger_sub.has_update()) {
            log_update_trigger_sub.get_value();

            size_t log_buffer_sz;
            chMtxLock(&log_mutex);
            log_buffer_sz = log_buffer_wi;
            chMtxUnlock(&log_mutex);

            comm_send(&comm_if, RosInterfaceCommMsgID::LOG, log_buffer, log_buffer_sz);

            chMtxLock(&log_mutex);
            if (log_buffer_wi > log_buffer_sz) { // more data has been written during send
                size_t new_size = log_buffer_wi - log_buffer_sz;
                memmove(&log_buffer[0], &log_buffer[log_buffer_sz], new_size);
                log_buffer_wi = new_size;
            } else {
                log_buffer_wi = 0; // reset buffer
            }
            chMtxUnlock(&log_mutex);
        }

        // chThdSleepMilliseconds(1); // todo this is a temporary fix to avoid using 100% CPU
        msgbus::wait_for_update_on_any(sub_list.begin(), sub_list.end());
    }
}


void ros_comm_init(BaseSequentialStream *sd)
{
    comm_init(&comm_if, sd, comm_rcv_cb);

    log_handler_register(&log_handler_comm, LOG_LVL_DEBUG, log_handler_comm_cb);

    chThdCreateStatic(comm_rx_thread_wa, sizeof(comm_rx_thread_wa),
                      THD_PRIO_COMM_RX, comm_rx_thread, NULL);
    chThdCreateStatic(comm_tx_thread_wa, sizeof(comm_tx_thread_wa),
                      THD_PRIO_COMM_TX, comm_tx_thread, NULL);
}