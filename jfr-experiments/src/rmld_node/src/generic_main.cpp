//****************************************************************************************
//* 	generic_main.cpp
//*	Description: Generic main file
//*	version 2.0
//*	date	july 22, 2013
//*		Added support for sampling frequency through roslaunch
//*	version 1.0
//*	date	jan 21, 2012 
//*		Initial release
//****************************************************************************************

#include "generic_main.h"






//============================================================
//	Main function
//============================================================

int main(int argc, char **argv)
{
	ROS_INFO("*****************************************************");
	ROS_INFO("     %s Node starting...",DEVICE_NAME);
	ROS_INFO("*****************************************************");



	ros::init(argc, argv, DEVICE_NAME);
  	ros::NodeHandle n;

	int sampling_freq;
	double	get_double;
	int	get_int;
	


	char param_name[80];
	const char *port_id;
	std::string 	param_value;

	//***** Gets parameters from roslaunch file **************//

	//	Port number
	sprintf(param_name,"%s/port",DEVICE_NAME);
	if(n.getParam(param_name,param_value)){
		port_id=param_value.c_str();
	}
	else {
		port_id=DEFAULT_PORT_NAME;
	}
	
	//	sampling frequency
	sprintf(param_name,"%s/sampling_frequency",DEVICE_NAME);
	if(n.getParam(param_name,get_int)){
		sampling_freq=get_int;
	}
	else {
		sampling_freq=DEFAULT_SAMPLING_FREQUENCY_HZ;
	}


	ros::Rate loop_rate(sampling_freq);
	chemNode chem_node(port_id);

	if (!chem_node.isConnected){
		ROS_FATAL("%s not found",SENSOR_NAME);
	}
		
				
	else{
		chem_node.advertise(n);	
		while (ros::ok()){
			chem_node.publishTopics();
			ros::spinOnce();
    			loop_rate.sleep();
		}

		chem_node.disconnect();
	}

	chem_node.disconnect();

	
	
	std::cout<<std::endl<<"*****************************************************";
	std::cout<<std::endl<<"   "<<DEVICE_NAME<<" disconnected";
	std::cout<<std::endl<<"*****************************************************"<<std::endl;
}













