/*

    This file contains common prelude for exploration planning.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    27-Oct-2017
    Version: 0.0
    -------------------------------------------------------------------------


*/


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <fstream>
#include <boost/thread.hpp>
#include "ptu_control2/commandSweep.h"
#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <math.h>       /* atan */

//#include "gasbot_planning/ExecutedConfNum.h"
//#include "gasbot_planning/AddTwoInts.h"
#include "roscpp_tutorials/TwoInts.h"
#include <sys/stat.h> // mkdir
#include "sensor_msgs/JointState.h" // ptu joint call back
#include <nav_msgs/Odometry.h> // localization
#include <rmld_node/rmld_msg.h> // ppm data
//#include <string> // num to string

#include <std_msgs/String.h>
//#include <geometry_msgs/PoseStamped.h>



using namespace std;

//--- Log File Variables
//==========================
ofstream historyFile;
//string package_path = ros::package::getPath("gasbot_historian");
string log_FileName;
string log_FilePath;
string experimentTitle;
string explorationStrategy;

//--- Plan File Variables
//==========================
std::string plan_FileName;
std::string plan_FilePath;
//std::string plan_FilePath = ros::package::getPath("gasbot_planning");
std::vector<double> vecPoses; //vector contains goals
//double vecGoals; //vector contains goals

//--- RMLD Variables
//==========================
double measure;
string rmld_msg;
double GasMeasureThreshold;
int    OperationStatus;


//--- PTU/Sweeping Variables
//==========================
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle, sample_delay, tilt_angle;
int    statusPTU, num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
vector<double> vecJointState;
double anglePan,angleTilt;


//--- Pose/localization Variables
//==========================
double posX,posY,posW;
double roll,pitch,yaw; // human-only
double angYAW;         // human-only

//--- ROS-topic Variables
//==========================
std::string topicLocalization, topicPTUSweepStatus, topicPTUJointStatus, topicRMLDReadings, topicCurrentGoal, topicRVIZGoal;

//--- ROS-Service Variables
//==========================
std::string srvPTUSweepCommand, srvPTUJointStatus;


//--- Current Goal Variables
//==========================
double golX,golY,golW;
int    golS;

//--- Navigation Variables
//==========================
std::string plan_frame_name;


//--- Conf. Variables
//==========================
int num_of_conf;
//std_msgs::Int16 conf_msg;
//int conf_num = 0;
int conf_num;

//--- Functions
//==========================
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void create_____LogFile();
void create_____LogDirectory();
void close______LogFile();
void callback___PTUSweepingStatus();
void callback___PTUJointAngles();
void callback___Localization();
bool service____ExecutedConfNumber();
void write______LogFile();
void callback___RMLD();
void get________PlannedPosesFromFile();
void perform____GasSampling();
void execute____PlannedConfigurations();






//============================================
//          DEFAULT PARAMETERS
//============================================


//----- Gas Sampling Parameters
//============================================
	//#define	DEFAULT_MIN_PAN_ANGLE       -90.0
	//#define	DEFAULT_MAX_PAN_ANGLE       90.0
    #define	DEFAULT_SENSING_RANGE       15.0
    #define	DEFAULT_OFFSETY_BASE_RMLD   0.904
    #define	DEFAULT_FOV                 270.0
    #define	DEFAULT_NUM_PAN_SWEEPS      1
    #define	DEFAULT_NUM_TILT_SWEEPS     1
    #define	DEFAULT_SAMPLE_DELAY        0.1
	
//----- Navignation Parameters
//============================================
	#define	DEFAULT_PLAN_FRAME_NAME     "/world"
	

//----- Historian Parameters
//============================================
	// Gas Measurements
	#define	DEFAULT_GAS_MEASURE_THRESHOLD 00.00
	#define	DEFAULT_OPERATION_STATUS      3
	
//----- ROS Topic Names
//============================================
    #define	DEFAULT_TOPIC_LOCALIZATION      "/ndt_mcl"
    #define	DEFAULT_TOPIC_PTU_SWEEP_STATUS  "/ptu_control/state"
    #define	DEFAULT_TOPIC_PTU_JOINT_STATUS  "/amtec/joint_states"
    #define	DEFAULT_TOPIC_RMLD_READINGS     "/rmld/data"
    #define	DEFAULT_TOPIC_CURRENT_GOAL      "/move_base/current_goal"
    #define	DEFAULT_TOPIC_RVIZ_GOAL         "/move_base_simple/goal"

//----- ROS Service Names
//============================================
    #define	DEFAULT_SERVICE_PTU_SWEEP_COMMAND  "/ptu_control/sweep"
    #define	DEFAULT_SERVICE_PTU_JOINT_STATUS   "/amtec/get_status"
    
    
//----- Conf num
//============================================
    #define	DEFAULT_CONF_NUM  1    

