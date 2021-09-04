#! /usr/bin/env python

import rospy, math
from geometry_msgs.msg import PoseStamped, TwistStamped

pose = PoseStamped()
def pose_callback(data):
    global pose
    pose = data

def main():
    rospy.init_node('shoulder_move', anonymous=True)
    rospy.Subscriber('/ual/pose', PoseStamped, pose_callback)
    pub = rospy.Publisher('/ual/set_velocity', TwistStamped, queue_size=1)
    rate = rospy.Rate(10) # 1hz

    vel_base = TwistStamped()
    vel_base.header.frame_id = 'map'

    while not rospy.is_shutdown():
        vel_base.header.frame_id = "map"
        uav_yaw = 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        vel_base.twist.linear.x = 0.2*math.cos(uav_yaw)
        vel_base.twist.linear.y = 0.2*math.sin(uav_yaw)
        # vel_base.twist.linear.y = 0.2

        pub.publish(vel_base)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
