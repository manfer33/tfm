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
#include <arms_control_interface/backend.h>
#include <arms_control_interface/backend_gazebo_animator.h>
#include <arms_control_interface/backend_ros_control.h>
#include <arms_control_interface/backend_udp.h>
#include <arms_control_interface/backend_dummy.h>

namespace grvc { namespace aeroarms {

Backend::Backend() {
    // Make communications spin!
    spin_thread_ = std::thread([this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });
}

Backend* Backend::createBackend(grvc::utils::ArgumentParser& _args) {
    Backend* be = nullptr;
    // Decide backend from arguments:
    std::string selected_backend = _args.getArgument<std::string>("aci_backend", "gazebo_animator");
    if (selected_backend == "gazebo_animator") {
        be = new BackendGazeboAnimator(_args);
    }
    else if (selected_backend == "ros_control") {
        be = new BackendRosControl(_args);
    }
    else if (selected_backend == "udp") {
        be = new BackendUdp(_args);        
    }
    else if (selected_backend == "dummy") {
        be = new BackendDummy(_args);        
    }

    return be;
}

}}  // namespace grvc::aeroarms
