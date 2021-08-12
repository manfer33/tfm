#! /usr/bin/env python

import rospy, threading
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint

shoulder_topic = '/aeroarms/q1_2_position_controller/command'
TakeOff_ALT = 2 # [m]
ARM_RATE = 1 # [Hz]

pose = PoseStamped()

def callback_pose(data):
    global pose
    pose = data

def move_arm_nonstop():
    # Move arms
    pub = rospy.Publisher(shoulder_topic, Float64, queue_size=1)
    rate = rospy.Rate(ARM_RATE)
    flag = False

    while not rospy.is_shutdown():
        pub.publish(0.0) if flag else pub.publish(-1.5)
        rate.sleep()
        flag = not flag

def main():
    rospy.Subscriber("/ual/pose", PoseStamped, callback_pose)
    
    takeoff = rospy.ServiceProxy('/ual/take_off', TakeOff)
    goToWp = rospy.ServiceProxy('/ual/go_to_waypoint', GoToWaypoint)

    rospy.init_node('shoulder_move', anonymous=True)

    # Go WP!
    init_wp = PoseStamped()
    init_wp.pose.position.z = 2
    init_wp.pose.orientation = pose.pose.orientation

    #TakeOff!
    if pose.pose.position.z < 0.5:
        rospy.wait_for_service('/ual/take_off')
        try:
            takeoff(TakeOff_ALT, True)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    d = threading.Thread(target=move_arm_nonstop)
    d.setDaemon(True)
    d.start()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
