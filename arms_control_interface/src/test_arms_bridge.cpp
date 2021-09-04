#include <thread>
#include <ros/ros.h>
#include <arms_control_interface/arms_bridge_udp.h>

using namespace grvc::aeroarms;

int main(int argc, char **argv) {
	std::cout << std::endl;
	std::cout << "----------------------------------------------------" << std::endl;
	std::cout << "Dual Arm ROS Bridge Program" << std::endl << std::endl;
	std::cout << "Author: Alejandro Suarez Fernandez-Miranda" << std::endl;
	std::cout << "Date: 15 May 2017" << std::endl;
	std::cout << "----------------------------------------------------" << std::endl << std::endl;

	ros::init(argc, argv, "arms_bride_node");
	ros::NodeHandle n;

	ArmsBridgeUdpConfig config;
	ArmsStatePublisherDataPacket arms_state;
	ArmsControlReferencesDataPacket arms_control;

	// This thread waits the reception of control references sent through an UDP socket
	std::thread state_receiver_thread = std::thread([&](){
		ArmsStateReceiverUdp receiver(config);
		ros::Rate r(50);  // [Hz]
		while (ros::ok()) {
			receiver.receive(&arms_state);
			printf("Arms state data packet:\n");
			double q1L = arms_state.left_arm_joint_values[0];
			double q2L = arms_state.left_arm_joint_values[1];
			double q3L = arms_state.left_arm_joint_values[2];
			double q4L = arms_state.left_arm_joint_values[3];
			double q1R = arms_state.right_arm_joint_values[0];
			double q2R = arms_state.right_arm_joint_values[1];
			double q3R = arms_state.right_arm_joint_values[2];
			double q4R = arms_state.right_arm_joint_values[3];
			double xL = arms_state.left_TCP_cartesian_position[0];
			double yL = arms_state.left_TCP_cartesian_position[1];
			double zL = arms_state.left_TCP_cartesian_position[2];
			double xR = arms_state.right_TCP_cartesian_position[0];
			double yR = arms_state.right_TCP_cartesian_position[1];
			double zR = arms_state.right_TCP_cartesian_position[2];
			printf(" - Left arm Joint/Cartesian state: <%.0lf, %.0lf, %.0lf, %.0lf> [deg], \
				{%.0lf, %.0lf, %.0lf} [m]", 57.296*q1L, 57.296*q2L, 57.296*q3L, 57.296*q4L, xL, yL, zL);
			printf(" - Right arm Joint/Cartesian state: <%.0lf, %.0lf, %.0lf, %.0lf> [deg], \
				{%.0lf, %.0lf, %.0lf} [m]", 57.296*q1R, 57.296*q2R, 57.296*q3R, 57.296*q4R, xR, yR, zR);
			std::cout << "______________" << std::endl << std::endl;
			r.sleep();
		}
	});

	// This thread waits the reception of control references sent through an UDP socket
	std::thread reference_sender_thread = std::thread([&](){
		ArmsReferenceSenderUdp sender(config);
		ros::Rate r(50);  // [Hz]
		while (ros::ok()) {
			// Build the packet
			arms_control.control_mode = 0;
			arms_control.left_arm_joint_references[0] = -0.5236;
			arms_control.left_arm_joint_references[1] = 0.2618;
			arms_control.left_arm_joint_references[2] = -0.7854;
			arms_control.left_arm_joint_references[3] = -1.5708;
			arms_control.right_arm_joint_references[0] = -0.5236;
			arms_control.right_arm_joint_references[1] = -0.2618;
			arms_control.right_arm_joint_references[2] = 0.7854;
			arms_control.right_arm_joint_references[3] = -1.5708;
			arms_control.left_arm_cartesian_references[0] = 0.25;
			arms_control.left_arm_cartesian_references[1] = 0.2;
			arms_control.left_arm_cartesian_references[2] = -0.25;
			arms_control.right_arm_cartesian_references[0] = 0.25;
			arms_control.right_arm_cartesian_references[1] = -0.2;
			arms_control.right_arm_cartesian_references[2] = -025;
			sender.send(&arms_control);
			r.sleep();
		}
	});

	ros::spin();

	return 0;
}
