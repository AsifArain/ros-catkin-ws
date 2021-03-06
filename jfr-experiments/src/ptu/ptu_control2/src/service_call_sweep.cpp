#include "ros/ros.h"
#include "ptu_control2/commandSweep.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sweep_request_client");
  
 /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  } */

  ros::NodeHandle n;  
  ros::ServiceClient client = n.serviceClient<ptu_control2::commandSweep>("/ptu_control/sweep");
  ptu_control2::commandSweep sweep_test;
  sweep_test.request.min_pan  	= -20;
  sweep_test.request.max_pan  	= 20;
  sweep_test.request.min_tilt 	= -10;
  sweep_test.request.max_tilt 	= -10;
  sweep_test.request.n_pan    	= 1;
  sweep_test.request.n_tilt   	= 1;
  sweep_test.request.samp_delay = 0.1;
  
  
  client.call(sweep_test);

  //ros::service::call("call_sweep", srv);

  if (client.call(sweep_test))
  {
    ROS_INFO("Gas scan in progress");
  }
  else
  {
    ROS_ERROR("Failed to call service sweep");
    return 1;
  }

  return 0;
}
