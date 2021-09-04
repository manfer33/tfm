// Standard library
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>


// ROS library
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


// Constants definitio


// Structure definition
typedef struct
{
	double x_UAV;
	double y_UAV;
	double z_UAV;
	double yaw_UAV;
	double x_tool;
	double y_tool;
	double z_tool;
	double yaw_tool;
} VICON_PACKET;

typedef struct
{
	int * socketGCS2Host;
	struct sockaddr_in * addrHost;
	bool * endFlag;
} THREAD_ARGS;


// Global variables
float x_UAV = 0;
float y_UAV = 0;
float z_UAV = 0;
float yaw_UAV = 0;
float x_tool = 0;
float y_tool = 0;
float z_tool = 0;
float yaw_tool = 0;

// Function declaration
static void * viconDataSenderThreadFunction(void * args);


// Namespaces
using namespace std;



void uavCallback(const geometry_msgs::PoseStamped & msg)
{
	x_UAV = msg.pose.position.x;
	y_UAV = msg.pose.position.y;
	z_UAV = msg.pose.position.z;
	yaw_UAV = 2*atan2(msg.pose.orientation.z, msg.pose.orientation.w);
	// printf("UAV_Pose: {%.1lf, %.1lf, %.1lf - %.1lf}\n", x_UAV, y_UAV, z_UAV, yaw_UAV);
}


void toolCallback(const geometry_msgs::PoseStamped & msg)
{
	x_tool = msg.pose.position.x;
	y_tool = msg.pose.position.y;
	z_tool = msg.pose.position.z;
	yaw_tool = 2*atan2(msg.pose.orientation.z, msg.pose.orientation.w);
	// printf("Tool_Pose: {%.1lf, %.1lf, %.1lf - %.1lf}\n", x_tool, y_tool, z_tool, yaw_tool);
}


int main(int argc, char **argv)
{
	THREAD_ARGS threadArgs;
	struct sockaddr_in addrHost;
    struct hostent * host;
	pthread_t viconDataSenderThread;
	int socketGCS2Host = -1;
	int error = 0;
	int code = 0;
	bool endFlag = false;
	
	
	cout << endl;
	cout << "----------------------------------------------------" << endl;
	cout << "Vicon Interface Program" << endl << endl;
	cout << "Author: Alejandro Suarez Fernandez-Miranda" << endl;
	cout << "Date: Date: 24 May 2018" << endl;
	cout << "----------------------------------------------------" << endl << endl;
	
	if(argc < 3)
		cout << "ERROR: specify arms contoller IP address and UDP port" << endl;
	else
	{
		// Open the socket in datagram mode
		socketGCS2Host = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    	if(socketGCS2Host < 0)
    	{
    		cout << endl << "ERROR: [in main] could not open socket." << endl;
    		error = 1;
   		}
		
		// Solve tha name of the remote Odroid
   		host = gethostbyname(argv[1]);
   		// host = gethostbyname("127.0.0.1");
    	if(host == NULL)
		{
		    cout << "ERROR: [in main] could not get host by name." << endl;
		    error = 1;
		}
	
		// Set the address of the host
		if(error == 0)
		{
		    bzero((char*)&addrHost, sizeof(struct sockaddr_in));
		    addrHost.sin_family = AF_INET;
		    bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
		    addrHost.sin_port = htons(atoi(argv[2]));
		    // addrHost.sin_port = htons(22001);
		}
		else
		    close(socketGCS2Host);
		    
		    
 		if(error == 0)
		{
		    // Create thread for sending a periodic keep alive message
		    threadArgs.socketGCS2Host = &socketGCS2Host;
		    threadArgs.addrHost = &addrHost;
		    threadArgs.endFlag = &endFlag;
		    
			if(pthread_create(&viconDataSenderThread, NULL, &viconDataSenderThreadFunction, (void*)&threadArgs))
			{
				cout << "ERROR: [in main] could not create vicon data thread." << endl;
				error = 1;
			}
			else
			{
				// Wait 100 ms
				usleep(100000);
				
		
				// Init ROS and subscribe to the 3DConnexion mouse topic    
				ros::init(argc, argv, "listener");
				ros::NodeHandle n;
				ros::Subscriber subUAV = n.subscribe("vicon_client/MBZIRC_4/pose", 10, uavCallback);
				ros::Subscriber subTool = n.subscribe("vicon_client/AEROARMS_TOOL/pose", 10, toolCallback);
				ros::spin();
				
			}
    	}
    	
    	// Wait for thread termination
    	endFlag = true;
    	usleep(500000);
    	cout << "Waiting for thread termination..." << endl;
	    
		// Close socket
		close(socketGCS2Host);
		socketGCS2Host = -1;
	}

  return 0;
}




/*
 * Thread for sensing periodic keep alive messages
 **/
static void * viconDataSenderThreadFunction(void * args)
{
	THREAD_ARGS * threadArgs = (THREAD_ARGS*)args;
	VICON_PACKET viconPacket;
	struct timeval t0;
	struct timeval t1;
	double t = 0;
	const double updatePeriod = 0.02;
	int error = 0;
	
	
	cout << "Vicon data sender thread function started" << endl;
	
	// Keep Alive message loop
	while(*(threadArgs->endFlag) == false && error == 0)
	{
		// Get initial time stamp
		gettimeofday(&t0, NULL);
		
		// Build the packet
		viconPacket.x_UAV = x_UAV;
		viconPacket.y_UAV = y_UAV;
		viconPacket.z_UAV = z_UAV;
		viconPacket.yaw_UAV = yaw_UAV;
		viconPacket.x_tool = x_tool;
		viconPacket.y_tool = y_tool;
		viconPacket.z_tool = z_tool;
		viconPacket.yaw_tool = yaw_tool;
			
		// Send the packet
		if(sendto(*(threadArgs->socketGCS2Host), (char*)&viconPacket, sizeof(VICON_PACKET), 0, (struct sockaddr*)(threadArgs->addrHost), sizeof(struct sockaddr)) < 0)
		{
			error = 1;
			cout << "ERROR: [in viconDataSenderThreadFunction] could not send data." << endl;
		}
		
		// Compute the elapsed time
		gettimeofday(&t1, NULL);
		t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec);
		
		if(t > updatePeriod)
			cout << "WARNING: update period exceeded" << endl;
		else
			usleep((useconds_t)(1e6*(updatePeriod - t)));
		
	}
	cout << "Vicon data sender thread function terminated" << endl;
	
	
	return 0;
}


