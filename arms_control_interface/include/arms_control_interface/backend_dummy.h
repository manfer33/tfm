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
#ifndef ARMS_CONTROL_INTERFACE_BACKEND_DUMMY_H
#define ARMS_CONTROL_INTERFACE_BACKEND_DUMMY_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arms_control_interface/backend.h>
#include <argument_parser/argument_parser.h>

namespace grvc { namespace aeroarms {

class BackendDummy: public Backend {
public:

    BackendDummy(grvc::utils::ArgumentParser& _args) : Backend() {
        ROS_WARN("BackendDummy is only for testing porposes");
    }

    void receiveState() override {
        // Do nothing, joint_state is received in callback
        // TODO: left and right tcp from tf?
    }

    void updateState(JointState& _joint_state, Pose& _left_tcp, Pose& _right_tcp) override {
        _joint_state = joint_state_;
        // TODO: left and right tcp from tf
    }

    void updateControl(std::map<std::string, double>& _joint_map, Pose& _left_tcp, Pose& _right_tcp, ControlMode _control_mode) override {
        if ((_control_mode != JOINT) && (_control_mode != UNINITIALIZED)) {
            ROS_ERROR("BackendDummy::updateControl: Only JOINT control mode is implemented!");
            return;
        }

        for (auto& j: _joint_map) {
            reference_map_[j.first] = j.second;
        }
    }

    void sendControl() override {
        sensor_msgs::JointState reference;
        for (auto& j: reference_map_) {
            reference.name.push_back(j.first);
            reference.position.push_back(j.second);
        }
        joint_state_ = reference;
        joint_state_.header.stamp = ros::Time::now();
    }

protected:
    std::map<std::string, double> reference_map_;
    sensor_msgs::JointState joint_state_;
    ros::NodeHandle nh;  //TODO: Fixes ros::TimeNotInitializedException
};

}}  // namespace grvc::aeroarms

#endif  // ARMS_CONTROL_INTERFACE_BACKEND_DUMMY_H
