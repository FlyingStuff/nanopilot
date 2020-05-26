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
static parameter_t throttle_input_channel;
static parameter_t rssi_input_channel;
static parameter_t roll_reverse_param;
static parameter_t pitch_reverse_param;
static parameter_t yaw_reverse_param;
static parameter_t throttle_reverse_param;
static parameter_t bidirectional_throttle_param;


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
    parameter_integer_declare_with_default(&throttle_input_channel, &rc_ns, "throttle_channel", 1);
    parameter_integer_declare_with_default(&rssi_input_channel, &rc_ns, "rssi_channel", -1);
    parameter_boolean_declare_with_default(&roll_reverse_param, &rc_ns, "roll_reverse", false);
    parameter_boolean_declare_with_default(&pitch_reverse_param, &rc_ns, "pitch_reverse", false);
    parameter_boolean_declare_with_default(&yaw_reverse_param, &rc_ns, "yaw_reverse", false);
    parameter_boolean_declare_with_default(&throttle_reverse_param, &rc_ns, "throttle_reverse", false);
    parameter_boolean_declare_with_default(&bidirectional_throttle_param, &rc_ns, "bidirectional_throttle", false);
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
    int throttle_ch = parameter_integer_read(&throttle_input_channel);
    int rssi_ch = parameter_integer_read(&rssi_input_channel);

    out.roll = - get_rc_channel_value(raw, roll_ch);
    out.pitch = - get_rc_channel_value(raw, pitch_ch);
    out.yaw = - get_rc_channel_value(raw, yaw_ch);
    float throttle = get_rc_channel_value(raw, throttle_ch);
    if (parameter_boolean_read(&roll_reverse_param)) {
        out.roll = -out.roll;
    }
    if (parameter_boolean_read(&pitch_reverse_param)) {
        out.pitch = -out.pitch;
    }
    if (parameter_boolean_read(&yaw_reverse_param)) {
        out.yaw = -out.yaw;
    }
    if (parameter_boolean_read(&throttle_reverse_param)) {
        throttle = -throttle;
    }
    if (!parameter_boolean_read(&bidirectional_throttle_param)) {
        throttle = (throttle + 1)/2;
    }
    out.throttle = throttle;
    out.switch_armed = arm_remote_switch_is_armed(raw);
    out.switch_ap_control = ap_control_switch_is_on(raw);

    std::copy_n(raw.channel, raw.channel_count, out.channel_raw);
    out.channel_raw_count = raw.channel_count;
    out.signal = raw.signal;
    out.rssi = get_rc_channel_value(raw, rssi_ch);
    out.timestamp = raw.timestamp;
}

