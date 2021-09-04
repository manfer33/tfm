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
#ifndef ARMS_CONTROL_INTERFACE_BACKEND_H
#define ARMS_CONTROL_INTERFACE_BACKEND_H

#include <map>
#include <thread>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <argument_parser/argument_parser.h>

namespace grvc { namespace aeroarms {

typedef sensor_msgs::JointState JointState;
typedef geometry_msgs::PoseStamped Pose;

class Backend {
public:
    enum ControlMode {
        UNINITIALIZED = 0,
        JOINT = 1,
        CARTESIAN = 2
    };

    virtual void receiveState() = 0;
    virtual void updateState(JointState& _joint_state, Pose& _left_tcp, Pose& _right_tcp) = 0;

    virtual void updateControl(std::map<std::string, double>& _joint_map, Pose& _left_tcp, Pose& _right_tcp, ControlMode _control_mode) = 0;
    virtual void sendControl() = 0;

    Backend();
    virtual ~Backend() = default;

    static Backend* createBackend(grvc::utils::ArgumentParser& _args);

protected:
    // Ros spinning thread
    std::thread spin_thread_;
};

}}  // namespace grvc::aeroarms

#endif  // ARMS_CONTROL_INTERFACE_BACKEND_H
