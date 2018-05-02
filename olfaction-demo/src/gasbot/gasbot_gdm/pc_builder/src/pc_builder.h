#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <string.h>
#include "pcl/ros/conversions.h"
#include <laser_assembler/AssembleScans.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
//#include "pcl/common/transform.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <boost/thread/mutex.hpp>


#include <tf/transform_listener.h>
#include "tf/LinearMath/Transform.h"

#include<Eigen/StdVector>


//---------------------Definitions and constants ------------------------//

#define		NODE_NAME				"pc_builder"
#define		DEFAULT_LOOP_RATE_HZ			10.0
#define		DEFAULT_FIXED_FRAME			"/map"
#define		DEFAULT_TOPIC				"lidar_front_scan"
#define		DEFAULT_PTU_TOPIC			"ptu_state_topic"
#define		SCAN_FINISHED_S				30

//---------------------Calibrations ------------------------------------//
#define		IDLE				0
#define		MOVING_JOYSTICK			1
#define		SCANNING_MODEL			2
#define		SCANNING_GAS			3
#define		RESET				4

//---------------------Global variables --------------------------------//
double		refresh_rate_hz;
std::string	fixed_frame;
std::string	scan_lidar_name;
std::string	ptu_state_topic;

int		ptu_current_state;
int		ptu_last_state;
bool		new_state_arrived;

//---------------------Function prototypes ------------------------//
void loadNodeParameters(ros::NodeHandle private_nh);
void publishPointCloud(laser_assembler::AssembleScans, ros::Publisher,ros::ServiceClient);

