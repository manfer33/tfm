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
#include <arms_control_interface/SetJointReference.h>
#include <arms_control_interface/SetCartesianReference.h>
#include <arms_control_interface/SetPosture.h>
#include <arms_control_interface/aci.h>

using namespace arms_control_interface;

namespace grvc { namespace aeroarms {

ACI::ACI(grvc::utils::ArgumentParser& _args) {
    backend_ = Backend::createBackend(_args);
    double control_rate = _args.getArgument<double>("aci_control_rate", 50);  // [Hz]

    // Get initial posture from rosparam to init joint-reference map
    JointState initial_posture;
    ros::param::get("/arms_postures/initial/joint_names", initial_posture.name);
    ros::param::get("/arms_postures/initial/joint_values", initial_posture.position);
    if (initial_posture.name.size() == 0) {
        const std::string error = "ACI::ACI: Initial posture is empty!";
        ROS_ERROR("%s", error.c_str());
        throw std::runtime_error(error);
    }
    if (initial_posture.name.size() != initial_posture.position.size()) {
        const std::string error = "ACI::ACI: Initial posture names/values size mismatch!";
        ROS_ERROR("%s", error.c_str());
        throw std::runtime_error(error);
    }
    for (size_t i = 0; i < initial_posture.name.size(); i++) {
        ref_joint_map_[initial_posture.name[i]] = initial_posture.position[i];
    }

    // Init joint-state from joint-reference
    for (auto& joint_ref : ref_joint_map_) {
        cur_joint_state_.name.push_back(joint_ref.first);
        cur_joint_state_.position.push_back(joint_ref.second);
    }

    state_thread_ = std::thread([this, control_rate]() {
        ros::Rate loop_rate(control_rate);
        while (ros::ok()) {
            // Receive state and poses from arms
            backend_->receiveState();
            state_mutex_.lock();
            backend_->updateState(cur_joint_state_, cur_left_tcp_, cur_right_tcp_);
            //cur_joint_state_.name.push_back("q0");     // TODO: Pendulum patch!
            //cur_joint_state_.position.push_back(0.0);  // TODO: Pendulum patch!
            state_mutex_.unlock();

            // Bypass to test plumbing
            // cur_joint_state_.position.clear();
            // for (auto& joint_ref : ref_joint_map_) {
            //     cur_joint_state_.header.stamp = ros::Time::now();
            //     cur_joint_state_.position.push_back(joint_ref.second);
            // }

            // TODO: Frequency?
            loop_rate.sleep();
        }
    });

    control_thread_ = std::thread([this, control_rate]() {
        ros::Rate loop_rate(control_rate);
        while (ros::ok()) {
            // Build the packet
            control_mutex_.lock();
            backend_->updateControl(ref_joint_map_, ref_left_tcp_, ref_right_tcp_, control_mode_);
            control_mutex_.unlock();
            // Send mode and references to arms
            backend_->sendControl();
            // TODO: Frequency?
            loop_rate.sleep();
        }
    });

    // Start server if explicitly asked
    std::string server_mode = _args.getArgument<std::string>("aci_server", "off");
    // TODO: Consider other modes?
    if (server_mode == "on") {
        server_thread_ = std::thread([this, &_args]() {
            if (!ros::isInitialized()) {
                // Init ros node
                ros::init(_args.argc(), _args.argv(), "aci");
            }
            std::string aci_ns = "aeroarms/arms";
            std::string joint_srv = aci_ns + "/set_joint_reference";
            std::string cartesian_srv = aci_ns + "/set_cartesian_reference";
            std::string posture_srv = aci_ns + "/set_posture";
            std::string joint_topic = "joint_states";
            std::string left_tcp_topic = aci_ns + "/left_tcp";
            std::string right_tcp_topic = aci_ns + "/right_tcp";

            ros::NodeHandle nh;
            ros::ServiceServer set_joint_reference_service =
                nh.advertiseService<SetJointReference::Request, SetJointReference::Response>(
                joint_srv,
                [this](SetJointReference::Request &req, SetJointReference::Response &res) {
                return this->setJointReference(req.joint_state);
            });
            ros::ServiceServer set_cartesian_reference_service =
                nh.advertiseService<SetCartesianReference::Request, SetCartesianReference::Response>(
                cartesian_srv,
                [this](SetCartesianReference::Request &req, SetCartesianReference::Response &res) {
                return this->setCartesianReference(req.left_tcp, req.right_tcp);
            });
            ros::ServiceServer set_posture_service =
                nh.advertiseService<SetPosture::Request, SetPosture::Response>(
                posture_srv,
                [this](SetPosture::Request &req, SetPosture::Response &res) {
                return this->setPosture(req.posture_name.data);
            });
            ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>(joint_topic, 10);
            ros::Publisher left_tcp_pub = nh.advertise<geometry_msgs::PoseStamped>(left_tcp_topic, 10);
            ros::Publisher right_tcp_pub = nh.advertise<geometry_msgs::PoseStamped>(right_tcp_topic, 10);

            // Publish @ 10Hz
            ros::Rate loop_rate(10);
            while (ros::ok()) {
                joint_pub.publish(this->jointState());
                left_tcp_pub.publish(this->leftTCP());
                right_tcp_pub.publish(this->rightTCP());
                // ros::spinOnce();  // TODO: Backend?!
                loop_rate.sleep();
            }
        });
    }

    std::vector<std::string> posture_names;
    ros::param::get("/arms_postures/names", posture_names);
    for (auto name: posture_names) {
        JointState posture;
        ros::param::get("/arms_postures/" + name + "/joint_names", posture.name);
        ros::param::get("/arms_postures/" + name + "/joint_values", posture.position);
        if (posture.name.size() == posture.position.size()) {
            posture_map_[name] = posture;
        } else {
            ROS_ERROR("Posture %s names/values size mismatch", name.c_str());
        }
    }
}

bool ACI::setPosture(const std::string& _posture_name) {
    if (!posture_map_.count(_posture_name)) {
        ROS_ERROR("Posture %s not found!", _posture_name.c_str());
        return false;
    }
    return setJointReference(posture_map_[_posture_name]);
}

bool ACI::setJointReference(const JointState& _reference) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    // Set mode
    if (control_mode_ != Backend::ControlMode::JOINT) {
        ROS_INFO("Switch to JOINT control mode");
        control_mode_ = Backend::ControlMode::JOINT;
    }
    // Check lengths
    if (_reference.name.size() != _reference.position.size()) {
        ROS_ERROR("Joint reference size mismatch: name[%d], position[%d]",
        static_cast<int>(_reference.name.size()),
        static_cast<int>(_reference.position.size()));
        return false;
    }
    // Set map, checking names
    for (size_t i = 0; i < _reference.name.size(); i++) {
        if (ref_joint_map_.count(_reference.name[i]) > 0) {
            ref_joint_map_[_reference.name[i]] = _reference.position[i];
        } else {
            ROS_WARN("Joint [%s] not found, make sure it's not misspelled!",
            _reference.name[i].c_str());
        }
    }
    return true;
}

bool ACI::setCartesianReference(const Pose& _ref_left_tcp, const Pose& _ref_right_tcp) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    // Set mode
    if (control_mode_ != Backend::ControlMode::CARTESIAN) {
        ROS_INFO("Switch to CARTESIAN control mode");
        control_mode_ = Backend::ControlMode::CARTESIAN;
    }
    // Set poses
    ref_left_tcp_ = _ref_left_tcp;
    ref_right_tcp_ = _ref_right_tcp;
    return true;
}

}}  // namespace grvc::aeroarms
