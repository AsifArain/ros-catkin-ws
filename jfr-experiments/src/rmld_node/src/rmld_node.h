//****************************************************************************************
//* 	rmld_node.c
//*	version 1.0
//*	date	jan 21, 2012 (no changes made on july, 2013)
//*	Description: ROS node for the RMLD
//****************************************************************************************

#include "CerealPort.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include "std_msgs/Float32.h"

#include "rmld_msg.h"



#include <vector>

#define		DEBUG_MODE			false
//Serial connection defaults
#define		DEFAULT_BAUD_RATE		57600
#define		DEFAULT_PORT_NAME		"/dev/rmld"
#define		DEFAULT_SAMPLING_FREQUENCY_HZ	10
#define		FIXED_BAUD_RATE			true


//Serial retry strategy
#define		PACK_SIZE			78
#define		START_TIMEOUT			1000
#define		RUNNING_TIMEOUT			1000
#define		MAX_RETRIES_CONNECT		10
#define		MAX_RETRIES_ID			4
#define		INITIAL_SLEEP			3


//Node info
#define		DEVICE_NAME			"rmld_node"
#define		SENSOR_NAME			"RMLD"
#define		TOPIC_NAME			"rmld/data"


class chemNode
{
	public:
		chemNode(const char *);
		~chemNode();
		void advertise(ros::NodeHandle n);
		void publishTopics(void);
		void disconnect(void);
		bool	isConnected;


	private:

		//Variables
		cereal::CerealPort	* serial_port;	
		const char	*sPort;
		int	baudRate;
		


		//Topics
		ros::Publisher		topic1;

		//Functions
		void connect(void);

};



