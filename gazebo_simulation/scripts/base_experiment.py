#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from uav_abstraction_layer.srv import TakeOff

shoulder_topic = '/aeroarms/q1_2_position_controller/command'
TakeOff_ALT = 2 # [m]
ARM_RATE = 1 # [Hz]


def main():
    pub = rospy.Publisher(shoulder_topic, Float64, queue_size=1)
    takeoff = rospy.ServiceProxy('/ual/take_off', TakeOff)

    rospy.init_node('shoulder_move', anonymous=True)
    rate = rospy.Rate(ARM_RATE)

    # TakeOff!
    rospy.wait_for_service('/ual/take_off')
    try:
        takeoff(TakeOff_ALT, True)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # Move arms
    flag = False
    while not rospy.is_shutdown():
        pub.publish(0.0) if flag else pub.publish(-1.5)
        rate.sleep()
        flag = not flag

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
