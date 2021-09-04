#!/usr/bin/env python

import rospy, rospkg, numpy, time, q_learn_2
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from gym import wrappers

if __name__ == '__main__':
    rospy.init_node('armdrone_stabilize', anonymous=True, log_level=rospy.WARN)
    agent = q_learn_2.CartPoleQAgent()
    agent.train()
    # t = agent.run()
    # print("Time", t)


