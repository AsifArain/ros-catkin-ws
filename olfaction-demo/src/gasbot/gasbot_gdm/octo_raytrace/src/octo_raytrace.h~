
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <string.h>
#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>

#include <laser_assembler/AssembleScans.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include "pcl/common/transforms.h"
#include "pcl/common/eigen.h"
#include "OctomapROS.h"
#include <octomap_ros/conversions.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <boost/thread/mutex.hpp>

#include <octomap/octomap.h> 
//#include <octomap_ros/OctomapROS.h> //Asif

#include <octomap/OcTree.h>
//#include <octomap/OcTreeFileIO.h>

#include <tf/transform_listener.h>
//#include "tf/LinearMath/Transform.h"
#include <tf/LinearMath/Transform.h>
#include "rmld_msg.h"
#include <Eigen/StdVector>
#include <visualization_msgs/Marker.h>
#include "raytrace_rmld.h"


//---------------------Definitions and constants ------------------------//

#define		NODE_NAME				"octo_raytrace"
#define		DEFAULT_LOOP_RATE_HZ			10.0
#define		DEFAULT_FIXED_FRAME			"map"
#define		DEFAULT_RMLD_FRAME			"rmld_frame"
#define		DEFAULT_RMLD_TOPIC			"rmld_node/rmld_data"
#define		DEFAULT_PC_TOPIC			"pc_buider/pc"
#define		DEFAULT_MODEL_PUB			false
#define		DEFAULT_DISPLAY_RAYS			false
#define		DEFAULT_VOXEL				0.1
#define		DEFAULT_FILE_NAME			"none"

#define		FRAME_WAIT_TIME				0.5
//---------------------Calibrations ------------------------------------//
#define		IDLE				0
#define		MOVING_JOYSTICK			1
#define		SCANNING_MODEL			2
#define		SCANNING_GAS			3
#define		RESET				4

//---------------------Global variables ------------------------//
double		refresh_rate_hz;
std::string	rmld_frame;
std::string	fixed_frame;
std::string	pc_topic;
std::string	rmld_topic;
bool		publish_pc;
bool		publish_model;
bool		display_rays;
double		voxel_size;

double		last_RMLD_PPMM;
int		ptu_last_state;
int		ptu_current_state;
bool		new_state_arrived;
bool		cloud_to_process;
bool		rmld_to_process;
bool		model_available;
pcl::PointCloud<pcl::PointXYZ> cloudpcl;
octomap::OcTree 			*model_tree;

bool			data_recording_enabled;
std::string		log_file_name;

//---------------------Function prototypes ------------------------//
void loadNodeParameters(ros::NodeHandle private_nh);
void publishRayTrace(ros::Publisher,octomap::point3d,octomap::point3d,std::string,double);


