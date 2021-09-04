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
#ifndef ARMS_CONTROL_INTERFACE_BACKEND_ROS_CONTROL_H
#define ARMS_CONTROL_INTERFACE_BACKEND_ROS_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <arms_control_interface/backend.h>
#include <argument_parser/argument_parser.h>

namespace grvc { namespace aeroarms {

class BackendRosControl: public Backend {
public:
    BackendRosControl(grvc::utils::ArgumentParser& _args);

    void receiveState() override;
    void updateState(JointState& _joint_state, Pose& _left_tcp, Pose& _right_tcp) override;

    void updateControl(std::map<std::string, double>& _joint_map, Pose& _left_tcp, Pose& _right_tcp, ControlMode _control_mode) override;
    void sendControl() override;

protected:
    ros::NodeHandle node_;
    std::map<std::string, ros::Publisher> pub_map_;
    std::map<std::string, std_msgs::Float64> reference_map_;
    sensor_msgs::JointState joint_state_;
    ros::Subscriber joint_state_sub_;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& _joint_state);
};

}}  // namespace grvc::aeroarms

#endif  // ARMS_CONTROL_INTERFACE_BACKEND_ROS_CONTROL_H
