#ifndef ARMS_CONTROL_INTERFACE_ARMS_BRIDGE_UDP_H
#define ARMS_CONTROL_INTERFACE_ARMS_BRIDGE_UDP_H

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/ros.h>

namespace grvc { namespace aeroarms {

// Constants definition
#define NUM_ARM_JOINTS 4

// Config definition
struct ArmsBridgeUdpConfig {
	unsigned int state_publisher_udp_port = 26001;
	unsigned int control_references_udp_port = 27001;
	std::string arms_controller_ip_address = "127.0.0.1";
	// double arms_references_publisher_period = 0.02;
};

// Package definition  // TODO: mutex protection
struct ArmsStatePublisherDataPacket {
	double left_arm_joint_values[NUM_ARM_JOINTS] = {0,0,0,0};		// Joint values in [rad]
	double right_arm_joint_values[NUM_ARM_JOINTS] = {0,0,0,0};		// Joint values in [rad]
	double left_TCP_cartesian_position[3];				// Tool Center Point Cartesian position in [m]
	double right_TCP_cartesian_position[3];			    // Tool Center Point Cartesian position in [m]
};

struct ArmsControlReferencesDataPacket {
	uint8_t control_mode;							    // 1: joint control, 2: Cartesian control
	double left_arm_joint_references[NUM_ARM_JOINTS] = {0,0,0,0};	// Joint position references in [rad]
	double left_arm_cartesian_references[3];	        // Cartesioan position references in [m]
	double left_arm_torque_references[NUM_ARM_JOINTS] = {0,0,0,0};	// Torque references in [N·m]
	double left_arm_force_references[3];	            // Force references in [N]
    double right_arm_joint_references[NUM_ARM_JOINTS] = {0,0,0,0};	// Joint position references in [rad]
	double right_arm_cartesian_references[3];	        // Cartesioan position references in [m]
	double right_arm_torque_references[NUM_ARM_JOINTS] = {0,0,0,0};	// Torque references in [N·m]
	double right_arm_force_references[3];	            // Force references in [N]
    double timestamp = 0;
};

class ArmsStateReceiverUdp {
public:
    ArmsStateReceiverUdp(const ArmsBridgeUdpConfig& _config) {
        ROS_INFO("ArmsStateReceiver constructor");

        // Open the socket in stream mode
        socket_receiver_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_receiver_ < 0) {
            ROS_ERROR("ArmsStateReceiver: could not open socket!");
            return;
        }
        // Set listenning address and port for server
	    struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(_config.state_publisher_udp_port);

        // Associates the address to the socket
        if (bind(socket_receiver_, (struct sockaddr*)&addr, sizeof(struct sockaddr)) < 0) {
            ROS_ERROR("ArmsStateReceiver: could not associate address to socket!");
            return;
        }
        // Arrived here, we're ready to receive
        ready_ = true;
    }

    void receive(ArmsStatePublisherDataPacket* _arms_state) {
        if (!ready_) {
            ROS_ERROR("ArmsStateReceiver::receive: initialization failed!");
            return;
        }
        struct sockaddr_in addr_host;
        socklen_t addr_length;
		int data_received = recvfrom(socket_receiver_,
            buffer_,
            1024,
            0,
            (struct sockaddr*)&addr_host,
            &addr_length);
		if (data_received == sizeof(ArmsStatePublisherDataPacket)) {
			*_arms_state = *(ArmsStatePublisherDataPacket*)buffer_;
		} else if (data_received == -1) {
			ROS_WARN("ArmsStateReceiver::receive: [%d]: %s", errno, strerror(errno));  // TODO: Test this
		} else {
            ROS_WARN("ArmsStateReceiver::receive: Packet size is not correct, received [%d], expected [%d]", 
            data_received, (int)sizeof(ArmsStatePublisherDataPacket));
        }
    }
protected:
	char buffer_[1024];
	int socket_receiver_;
    bool ready_ = false;
};

class ArmsReferenceSenderUdp {
public:
    ArmsReferenceSenderUdp(const ArmsBridgeUdpConfig& _config) {
        ROS_INFO("ArmsReferencesSender constructor");

        // Open the socket in datagram mode
        socket_publisher_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_publisher_ < 0) {
            ROS_ERROR("ArmsReferencesSender: could not open socket!");
            close(socket_publisher_);
            return;
        }

        struct hostent* host;
        host = gethostbyname(_config.arms_controller_ip_address.c_str());
        if (host == NULL) {
            ROS_ERROR("ArmsReferencesSender: could not get host by name!");
            close(socket_publisher_);
            return;
        }

        // Set the address of the host
        bzero((char*)&addr_host_, sizeof(struct sockaddr_in));
        addr_host_.sin_family = AF_INET;
        bcopy((char*)host->h_addr, (char*)&addr_host_.sin_addr.s_addr, host->h_length);
        addr_host_.sin_port = htons(_config.control_references_udp_port);
        // Arrived here, we're ready to send
        ready_ = true;
    }

    ~ArmsReferenceSenderUdp() {
        // Close the socket
	    close(socket_publisher_);
    }

    void send(ArmsControlReferencesDataPacket* _arms_control) {
        if (!ready_) {
            ROS_ERROR("ArmsReferenceSender::send: initialization failed!");
            return;
        }
        // Debug!
        // printf("send: left  [%f, %f, %f, %f]\n", _arms_control->left_arm_joint_references[0],
        //     _arms_control->left_arm_joint_references[1],
        //     _arms_control->left_arm_joint_references[2],
        //     _arms_control->left_arm_joint_references[3]);
        // printf("send: right [%f, %f, %f, %f]\n", _arms_control->right_arm_joint_references[0],
        //     _arms_control->right_arm_joint_references[1],
        //     _arms_control->right_arm_joint_references[2],
        //     _arms_control->right_arm_joint_references[3]);
		// Send the packet
		if (sendto(socket_publisher_,
            (char*)_arms_control,
			sizeof(ArmsControlReferencesDataPacket),
			0,
			(struct sockaddr*)&addr_host_,
			sizeof(struct sockaddr)) < 0) {
			ROS_ERROR("ArmsReferenceSender::send: could not send data!");
		}
    }

protected:
	struct sockaddr_in addr_host_;
	int socket_publisher_ = -1;
    bool ready_ = false;
};

}}  // namespace grvc::aeroarms

#endif  // ARMS_CONTROL_INTERFACE_ARMS_BRIDGE_UDP_H
