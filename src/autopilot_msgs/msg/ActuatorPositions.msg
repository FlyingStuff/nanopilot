# RC-actuators are normalized from 0 to 1 for unidirectional actuators
# and -1 to 1 for bidirectional actuators. If the actuator controls a joint
# the unit should be radians or m depending on the type of joint.
# The value of 0 must be a safe position (eg. thruster off)

uint32 MAX_NB_ACTUATORS=32
float32[<=32] actuators
