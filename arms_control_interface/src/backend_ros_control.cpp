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
#include <arms_control_interface/backend_ros_control.h>

namespace grvc { namespace aeroarms {

BackendRosControl::BackendRosControl(grvc::utils::ArgumentParser& _args) : Backend() {
    joint_state_sub_ = node_.subscribe<sensor_msgs::JointState>("/aeroarms/joint_states", 10, &BackendRosControl::jointStateCallback, this);
}

void BackendRosControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& _joint_state) {
    joint_state_ = *_joint_state;
}

void BackendRosControl::receiveState() {
    // Do nothing, joint_state is received in callback
    // TODO: left and right tcp from tf?
}

void BackendRosControl::updateState(JointState& _joint_state, Pose& _left_tcp, Pose& _right_tcp) {
    _joint_state = joint_state_;
    // TODO: left and right tcp from tf
}

void BackendRosControl::updateControl(std::map<std::string, double>& _joint_map, Pose& _left_tcp, Pose& _right_tcp, ControlMode _control_mode) {
    if ((_control_mode != JOINT) && (_control_mode != UNINITIALIZED)) {
        ROS_ERROR("BackendRosControl::updateControl: Only JOINT control mode is implemented!");
    }
    for (auto& j: _joint_map) {
        if (!pub_map_.count(j.first)) {
            std::string topic = "/aeroarms/" + j.first + "_position_controller/command";
            pub_map_[j.first] = node_.advertise<std_msgs::Float64>(topic, 2);
        }
        reference_map_[j.first].data = j.second;
    }
}

void BackendRosControl::sendControl() {
    for (auto& p: pub_map_) {
        p.second.publish(reference_map_[p.first]);
    }
}

}}  // namespace grvc::aeroarms
