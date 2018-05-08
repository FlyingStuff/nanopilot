import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from aircraft_dynamics import *


def sim():
    rospy.init_node('Sim')
    rate_hz = 100
    rate = rospy.Rate(rate_hz)
    pub = rospy.Publisher('pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        a = Airplane(param, init)
        inputs = {'d_e': 0, 'd_a': 0, 'd_r': 0, 'wind_I': np.array([[0], [0], [0]])}

        for i in range(rate_hz*5):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = a.state['pos_I'][0]
            pose.pose.position.y = a.state['pos_I'][1]
            pose.pose.position.z = a.state['pos_I'][2]
            pose.pose.orientation.w = a.state['q_b_to_I'][0]
            pose.pose.orientation.x = a.state['q_b_to_I'][1]
            pose.pose.orientation.y = a.state['q_b_to_I'][2]
            pose.pose.orientation.z = a.state['q_b_to_I'][3]
            pub.publish(pose)

            # rospy.loginfo(f'{a.state}')
            a.time_step(1/rate_hz, inputs)
            rate.sleep()
            if rospy.is_shutdown():
                break


if __name__ == '__main__':
    try:
        sim()
    except rospy.ROSInterruptException:
        exit(0)
