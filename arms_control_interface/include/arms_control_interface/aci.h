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
#ifndef ARMS_CONTROL_INTERFACE_ACI_H
#define ARMS_CONTROL_INTERFACE_ACI_H

#include <map>
#include <mutex>
#include <thread>
#include <argument_parser/argument_parser.h>
#include <arms_control_interface/backend.h>

namespace grvc { namespace aeroarms {

class ACI {
public:

    ACI(grvc::utils::ArgumentParser& _args);

    bool setJointReference(const JointState& _reference);

    bool setCartesianReference(const Pose& _ref_left_tcp, const Pose& _ref_right_tcp);

    bool setPosture(const std::string& _posture_name);

    inline JointState jointState() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return cur_joint_state_;
    }

    inline Pose leftTCP() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return cur_left_tcp_;
    }

    inline Pose rightTCP() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return cur_right_tcp_;
    }

protected:
    Backend* backend_;
    Backend::ControlMode control_mode_ = Backend::ControlMode::UNINITIALIZED;
    std::map<std::string, double> ref_joint_map_;
    JointState cur_joint_state_;
    Pose ref_left_tcp_;
    Pose ref_right_tcp_;
    Pose cur_left_tcp_;
    Pose cur_right_tcp_;
    std::map<std::string, JointState> posture_map_;

    std::mutex state_mutex_;
    std::mutex control_mutex_;
    std::thread state_thread_;
    std::thread control_thread_;
    std::thread server_thread_;
};

}}  // namespace grvc::aeroarms

#endif  // ARMS_CONTROL_INTERFACE_ACI_H
