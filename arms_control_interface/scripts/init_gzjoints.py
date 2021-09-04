#! /usr/bin/env python
import argparse
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty

class GazeboModel(object):

    def __init__(self, model):
        # TODO: Always _1?
        self.model_name = model + "_1"
        self.model_exists = False
        self.gazebo_namespace = "/gazebo"
        self.model_param_name = ""
        self.joint_names = rospy.get_param("/arms_postures/initial/joint_names")
        self.joint_positions = rospy.get_param("/arms_postures/initial/joint_values")

    def checkForModelExistence(self, model):
        for name in model.name:
            if name == self.model_name:
                self.model_exists = True

    def setJoints(self):
        rospy.init_node("init_model_joints")

        # print "Wait for model to exist"
        rospy.Subscriber(self.gazebo_namespace + "/model_states", ModelStates, self.checkForModelExistence)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.model_exists:
            rate.sleep()

        # print "Wait for service and pause physics"
        rospy.wait_for_service(self.gazebo_namespace + "/pause_physics")
        try:
            pause_physics = rospy.ServiceProxy(self.gazebo_namespace + "/pause_physics", Empty)
            pause_physics()
        except rospy.ServiceException as e:
            rospy.logerr("Pause physics service call failed: %s", e)

        # print "Wait for service and set model configuration"
        rospy.wait_for_service(self.gazebo_namespace + "/set_model_configuration")
        try:
            set_model_configuration = rospy.ServiceProxy(self.gazebo_namespace + "/set_model_configuration", SetModelConfiguration)
            rospy.loginfo("Calling service %s/set_model_configuration", self.gazebo_namespace)
            resp = set_model_configuration(self.model_name, self.model_param_name, self.joint_names, self.joint_positions)
            rospy.loginfo("Set model configuration status: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # print "Wait for service and unpause physics"
        rospy.wait_for_service('%s/unpause_physics' % (self.gazebo_namespace))
        try:
            unpause_physics = rospy.ServiceProxy(self.gazebo_namespace + "/unpause_physics", Empty)
            unpause_physics()
        except rospy.ServiceException as e:
            rospy.logerr("Unpause physics service call failed: %s", e)

        # print "Wait for service and reset world"
        # rospy.wait_for_service(self.gazebo_namespace + "/reset_world")
        # try:
        #     reset_world = rospy.ServiceProxy(self.gazebo_namespace + "/reset_world", Empty)
        #     rospy.loginfo("Calling service %s/reset_world", self.gazebo_namespace)
        #     reset_world()
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s", e)

def main():

    # Parse arguments
    parser = argparse.ArgumentParser(description='Set initial position of arms joints in gazebo simulation')
    parser.add_argument('-model', type=str, default="aeroarms",
                        help='robot model name, must match model name in gazebo without [_1]')
    args, unknown = parser.parse_known_args()
    for arg in unknown:
        if arg[0] == '-':
            raise SyntaxWarning("Unexpected argument " + arg)

    model = GazeboModel(args.model)
    print __file__ + ": Setting initial arms posture for " + model.model_name
    model.setJoints()


if __name__ == "__main__":
    main()
