//----------------------------------------------------------------------------------------------------------------------
// GRVC Aeroarms
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <arms_control_interface/aci.h>

class JointStateGenerator {
public:
  JointStateGenerator() {
    ref_joint_state_.name.push_back("q1_1");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q1_2");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q1_3");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q1_4");
    ref_joint_state_.position.push_back(0.0);

    ref_joint_state_.name.push_back("q2_1");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q2_2");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q2_3");
    ref_joint_state_.position.push_back(0.0);
    ref_joint_state_.name.push_back("q2_4");
    ref_joint_state_.position.push_back(0.0);
  }

  grvc::aeroarms::JointState jointState(const ros::Time& _time) {
    double t = _time.toSec();
    ref_joint_state_.header.stamp = _time;

    // TODO: Play with phases and frequencies
    ref_joint_state_.position[0] = 0.0  * sin(3.14*t);  // "q1_1"
    ref_joint_state_.position[1] = 0.0  * sin(3.14*t);  // "q1_2"
    ref_joint_state_.position[2] = 0.2  * sin(3.14*t);  // "q1_3"
    ref_joint_state_.position[3] = 0.0  * sin(3.14*t);  // "q1_4"

    ref_joint_state_.position[4] =  0.0  * sin(3.14*t);  // "q2_1"
    ref_joint_state_.position[5] =  0.0  * sin(3.14*t);  // "q2_2"
    ref_joint_state_.position[6] =  0.2  * sin(3.14*t);  // "q2_3"
    ref_joint_state_.position[7] =  0.0  * sin(3.14*t);  // "q2_4"
    return ref_joint_state_;
  }

protected:
  grvc::aeroarms::JointState ref_joint_state_;
};

int main(int _argc, char **_argv) {
    grvc::utils::ArgumentParser args(_argc, _argv);
    ros::init(_argc, _argv, "test_interface");
    ros::NodeHandle nh;

    JointStateGenerator tester;
    grvc::aeroarms::ACI arms(args);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(50);  // [Hz]
    while (ros::ok()) {
        // joint_state_pub.publish(tester.jointState(ros::Time::now()));  // debug!
        arms.setJointReference(tester.jointState(ros::Time::now()));
        joint_state_pub.publish(arms.jointState());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
