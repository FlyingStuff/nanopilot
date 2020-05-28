#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

#include "msgbus/msgbus.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"



enum control_mode_t {CTRL_MODE_DISARMED=0, CTRL_MODE_MANUAL=1, CTRL_MODE_AP=2, CTRL_MODE_DIRECT_ACTUATORS=3};
typedef struct {
    control_mode_t mode;
    bool ap_timeout;
} control_status_t;

extern msgbus::Topic<float> ap_control_latency_topic;
extern msgbus::Topic<control_status_t> control_status_topic;
extern msgbus::Topic<actuators_stamped_t> actuator_output_topic;
extern parameter_namespace_t control_ns;


class ControllerInterface {
public:
    virtual control_mode_t process(const rc_input_s &rc_in, actuators_t &out) = 0;
    virtual void notify_output_disabled() = 0;
    virtual void set_update_frequency(float freq) = 0;
    virtual timestamp_t ap_control_signal_timestamp() = 0;
};


void control_init();
void control_start(ControllerInterface &controller);

#endif /* CONTROL_LOOP_HPP */
