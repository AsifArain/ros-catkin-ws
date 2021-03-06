#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
//#include <stdio.h>
//#include <boost/thread.hpp>
#include "ptu_control/commandSweep.h"
#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"

#include <stdint.h>

void gasDetection();
int  statusPTU;

// =================== Sweeping Parameters ========================
double	min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle, sample_delay;
int 	num_pan_sweeps, num_tilt_sweeps;
//typedef int16_t _num_pan_sweeps, _num_tilt_sweeps;
//int16_t num_pan_sweeps, num_tilt_sweeps;

// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//                                       MAIN
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

int main(int argc, char** argv){
  ros::init(argc, argv, "plan_execution");
  ros::NodeHandle n;
  
	// ####################### PARAMETERS ########################
	ros::NodeHandle paramHandle ("~");

	// ========= Gas Detection Parameters ========================
	paramHandle.param("min_pan_angle",  min_pan_angle,  -90.0);
	paramHandle.param("max_pan_angle",  max_pan_angle,   90.0);
	paramHandle.param("min_tilt_angle", min_tilt_angle, -10.0);
	paramHandle.param("max_tilt_angle", max_tilt_angle, -10.0);
	paramHandle.param("num_pan_sweeps", num_pan_sweeps,     1);
	paramHandle.param("num_tilt_sweeps",num_tilt_sweeps,    1);
	paramHandle.param("sample_delay",   sample_delay,     0.1);
	//paramHandle.getParam("min_pan_angle",min_pan_angle);
	//std::string global_name, relative_name, default_param;
	//paramHandle.param<std::string>("default_param", default_param, "default_value");
		
	//gasDetection();
	//rosservice call /ptu_control/sweep "{min_pan: -45.0, max_pan: 45.0, min_tilt: -10.0, max_tilt: -10.0, n_pan: 1, n_tilt: 1, samp_delay: 0.1}"
	ros::ServiceClient client1 = n.serviceClient<ptu_control::commandSweep>("/ptu_control/sweep");
	ptu_control::commandSweep srvSweep;
	srvSweep.request.min_pan  	= min_pan_angle;   //-10;
	srvSweep.request.max_pan  	= max_pan_angle;   // 10;
	srvSweep.request.min_tilt 	= min_tilt_angle;  //-10;
	srvSweep.request.max_tilt 	= max_tilt_angle;  //-10;
	srvSweep.request.n_pan    	= num_pan_sweeps;  //  1;
	srvSweep.request.n_tilt   	= num_tilt_sweeps; //  1;
	srvSweep.request.samp_delay 	= sample_delay;    //0.1;
	
	if (client1.call(srvSweep)){
		ROS_INFO("Gas detection in progress ... ");}
	else{
		ROS_ERROR("Failed to initialize gas scanning.");}

  //ros::Subscriber sub1 = n.subscribe("/ptu_control/state",10,stateCallback);
  //boost::thread mythread(followPOSES);
  ros::spin();
  
return 0;
}



