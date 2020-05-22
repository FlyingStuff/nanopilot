#include <algorithm>
#include "rc_input.hpp"

msgbus::Topic<struct rc_input_raw_s> rc_input_raw_topic;
msgbus::Topic<struct rc_input_s> rc_input_topic;


static parameter_namespace_t rc_ns;
static parameter_t arm_remote_switch_channel;
static parameter_t arm_remote_switch_threshold;
static parameter_t ap_remote_switch_channel;
static parameter_t ap_remote_switch_threshold;
static parameter_t roll_input_channel;
static parameter_t pitch_input_channel;
static parameter_t yaw_input_channel;
static parameter_t roll_rate_gain_param;
static parameter_t pitch_rate_gain_param;
static parameter_t yaw_rate_gain_param;


void rc_input_init(parameter_namespace_t *param_namespace)
{
    parameter_namespace_declare(&rc_ns, param_namespace, "rc");
    parameter_integer_declare_with_default(&arm_remote_switch_channel, &rc_ns, "arm_channel", 5);
    parameter_scalar_declare_with_default(&arm_remote_switch_threshold, &rc_ns, "arm_threshold", 0.1);
    parameter_integer_declare_with_default(&ap_remote_switch_channel, &rc_ns, "ap_switch_channel", -1);
    parameter_scalar_declare_with_default(&ap_remote_switch_threshold, &rc_ns, "ap_switch_threshold", 0.1);
    parameter_integer_declare_with_default(&roll_input_channel, &rc_ns, "roll_channel", 2);
    parameter_integer_declare_with_default(&pitch_input_channel, &rc_ns, "pitch_channel", 3);
    parameter_integer_declare_with_default(&yaw_input_channel, &rc_ns, "yaw_channel", 4);
    parameter_scalar_declare_with_default(&roll_rate_gain_param, &rc_ns, "roll_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&pitch_rate_gain_param, &rc_ns, "pitch_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&yaw_rate_gain_param, &rc_ns, "yaw_rate_gain", 2*3.14f);
}

static bool arm_remote_switch_is_armed(const struct rc_input_raw_s &rc_inputs)
{
    int ch = parameter_integer_read(&arm_remote_switch_channel);
    int ch_idx = ch-1;
    if (ch_idx >= 0 && ch_idx < rc_inputs.channel_count) {
        float val = rc_inputs.channel[ch_idx];
        if (val > parameter_scalar_read(&arm_remote_switch_threshold)) {
            return true;
        } else {
            return false;
        }
    }
    return false;
}

static bool ap_control_switch_is_on(const struct rc_input_raw_s &rc_inputs)
{
    int ch = parameter_integer_read(&ap_remote_switch_channel);
    int ch_idx = ch-1;
    if (ch_idx >= 0 && ch_idx < rc_inputs.channel_count) {
        float val = rc_inputs.channel[ch_idx];
        if (val > parameter_scalar_read(&ap_remote_switch_threshold)) {
            return true;
        } else {
            return false;
        }
    }
    return false;
}


static float get_rc_channel_value(const struct rc_input_raw_s &rc_inputs, int channel)
{
    int index = channel - 1;
    if (index < 0 || index >= rc_inputs.channel_count) {
        return 0;
    }
    return rc_inputs.channel[index];
}

void rc_input_decode(const struct rc_input_raw_s &raw, struct rc_input_s &out)
{
    int roll_ch = parameter_integer_read(&roll_input_channel);
    int pitch_ch = parameter_integer_read(&pitch_input_channel);
    int yaw_ch = parameter_integer_read(&yaw_input_channel);

    float roll_gain = parameter_scalar_read(&roll_rate_gain_param);
    float pitch_gain = parameter_scalar_read(&pitch_rate_gain_param);
    float yaw_gain = parameter_scalar_read(&yaw_rate_gain_param);

    out.roll = - roll_gain * get_rc_channel_value(raw, roll_ch);
    out.pitch = - pitch_gain * get_rc_channel_value(raw, pitch_ch);
    out.yaw = - yaw_gain * get_rc_channel_value(raw, yaw_ch);
    out.throttle = 0; // TODO
    out.switch_armed = arm_remote_switch_is_armed(raw);
    out.switch_ap_control = ap_control_switch_is_on(raw);

    std::copy_n(raw.channel, raw.channel_count, out.channel_raw);
    out.channel_raw_count = raw.channel_count;
    out.signal = raw.signal;
    out.rssi = 0; // TODO extract from channel
    out.timestamp = raw.timestamp;
}

