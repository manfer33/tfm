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
#include <arms_control_interface/backend_udp.h>

namespace grvc { namespace aeroarms {

BackendUdp::BackendUdp(grvc::utils::ArgumentParser& _args) : Backend() {
    receiver_ = new ArmsStateReceiverUdp(config_);
    sender_ = new ArmsReferenceSenderUdp(config_);
}

void BackendUdp::receiveState() {
    receiver_->receive(&arms_state_);
}

void BackendUdp::updateState(JointState& _joint_state, Pose& _left_tcp, Pose& _right_tcp) {
    _joint_state.header.stamp = ros::Time::now();
    _joint_state.position[0] = arms_state_.left_arm_joint_values[0];
    _joint_state.position[1] = arms_state_.left_arm_joint_values[1];
    _joint_state.position[2] = arms_state_.left_arm_joint_values[2];
    _joint_state.position[3] = arms_state_.left_arm_joint_values[3];
    _joint_state.position[4] = arms_state_.right_arm_joint_values[0];
    _joint_state.position[5] = arms_state_.right_arm_joint_values[1];
    _joint_state.position[6] = arms_state_.right_arm_joint_values[2];
    _joint_state.position[7] = arms_state_.right_arm_joint_values[3];
    _left_tcp.header.stamp = _joint_state.header.stamp;
    _left_tcp.pose.position.x = arms_state_.left_TCP_cartesian_position[0];
    _left_tcp.pose.position.y = arms_state_.left_TCP_cartesian_position[1];
    _left_tcp.pose.position.z = arms_state_.left_TCP_cartesian_position[2];
    _right_tcp.header.stamp = _joint_state.header.stamp;
    _right_tcp.pose.position.x = arms_state_.right_TCP_cartesian_position[0];
    _right_tcp.pose.position.y = arms_state_.right_TCP_cartesian_position[1];
    _right_tcp.pose.position.z = arms_state_.right_TCP_cartesian_position[2];
}

void BackendUdp::updateControl(std::map<std::string, double>& _joint_map, Pose& _left_tcp, Pose& _right_tcp, ControlMode _control_mode) {
    arms_control_.control_mode = _control_mode;
    if (_control_mode == JOINT) {
        arms_control_.left_arm_joint_references[0] = _joint_map["q1_1"];
        arms_control_.left_arm_joint_references[1] = _joint_map["q1_2"];
        arms_control_.left_arm_joint_references[2] = _joint_map["q1_3"];
        arms_control_.left_arm_joint_references[3] = _joint_map["q1_4"];
        arms_control_.right_arm_joint_references[0] = _joint_map["q2_1"];
        arms_control_.right_arm_joint_references[1] = _joint_map["q2_2"];
        arms_control_.right_arm_joint_references[2] = _joint_map["q2_3"];
        arms_control_.right_arm_joint_references[3] = _joint_map["q2_4"];
    }
    if (_control_mode == CARTESIAN) {
        arms_control_.left_arm_cartesian_references[0] = _left_tcp.pose.position.x;
        arms_control_.left_arm_cartesian_references[1] = _left_tcp.pose.position.y;
        arms_control_.left_arm_cartesian_references[2] = _left_tcp.pose.position.z;
        arms_control_.right_arm_cartesian_references[0] = _right_tcp.pose.position.x;
        arms_control_.right_arm_cartesian_references[1] = _right_tcp.pose.position.y;
        arms_control_.right_arm_cartesian_references[2] = _right_tcp.pose.position.z;
    }

}

void BackendUdp::sendControl() {
    sender_->send(&arms_control_);
}

}}  // namespace grvc::aeroarms
