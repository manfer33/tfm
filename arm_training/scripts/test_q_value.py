#!/usr/bin/env python

import rospy, rospkg, numpy, time, qlearn
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from utils import save_json, load_json

if __name__ == '__main__':
    q_values = {}
    data = load_json("/home/manfer/tfm_ws/src/tfm/arm_training/training_results/old/14614.txt")
    print(data)

