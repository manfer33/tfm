#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion

quat = (0.0,0.0,0.0,1.0)

def pose_callback(data):
    global quat
    ori = data.pose.orientation
    quat = (ori.x, ori.y, ori.z, ori.w)

def main():
    rospy.Subscriber('/ual/pose', PoseStamped, pose_callback)
    pub = rospy.Publisher('/pitch', Float64, queue_size=1)

    rospy.init_node('check_pitch', anonymous=True)
    rate = rospy.Rate(30) # 1hz
    flag = False

    min_pitch = 100.0
    max_pitch = -100.0

    while not rospy.is_shutdown():

        _, pitch, _ = euler_from_quaternion(quat)
        if pitch < min_pitch:
            min_pitch = pitch
        if pitch > max_pitch:
            max_pitch = pitch
        pub.publish(pitch)
        print("min: "+str(min_pitch)+"max: "+str(max_pitch))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
