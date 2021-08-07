#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

shoulder_topic = '/aeroarms/q1_2_position_controller/command'

def main():
    pub = rospy.Publisher(shoulder_topic, Float64, queue_size=1)
    rospy.init_node('shoulder_move', anonymous=True)
    rate = rospy.Rate(1) # 1hz
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
