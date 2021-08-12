#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse

shoulder_topic = '/aeroarms/q1_2_position_controller/command'

move_arm = False

def handle_arm_trigger(req):
    global move_arm
    move_arm = req.data

    result = SetBoolResponse()
    result.success = True
    result.message="Arms command received: " + str(move_arm)

    return result

def main():
    rospy.Service('/aeroarms/move_arm', SetBool, handle_arm_trigger)

    pub = rospy.Publisher(shoulder_topic, Float64, queue_size=1)
    rospy.init_node('shoulder_move', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    flag = False

    while not rospy.is_shutdown():
        if move_arm:
            pub.publish(0.0) if flag else pub.publish(-1.5)
            rate.sleep()
            flag = not flag

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
