#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <rmld_node/rmld_msg.h>
#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include <ros/package.h>

using namespace std;

// File:
ofstream historyFile;
//string package_path = ros::package::getPath("gasbot_historian");
string file_path;
string file_name;

// RMLD:
int    measure;
string rmld_msg;
double GAS_MEASURE_THRESHOLD;
int    OPERATION_STATUS;

// PTU:
int statePTU;
vector<double> vecJointState;
double anglePan,angleTilt;

// Pose/localization:
double posX,posY,posW;
string localization_topic;


// ######################### Creating log file and directory ##############################
void createFile(string title, string file_path, string file_name){

	// ===== Date and Time ====
	time_t date_temp;       	// Stores seconds elapsed since 01-01-1970
	struct tm *date_format; 	// Saves in Date format
	char date_out[25];     		// Output string

	time(&date_temp);
	date_format = localtime(&date_temp);
	strftime(date_out, 25, "%Y-%m-%d-%H:%M:%S", date_format);

	char date_date[11];
	strftime(date_date, 11, "%Y-%m-%d%H:%M:%S", date_format);
	string date_str(date_date);
	string date_time_(date_out);
		
	// ===== DEFAULT PARAMETERS ======
	/*
	ros::NodeHandle paramHandle ("~");
	paramHandle.param<std::string>("title", title,	  "Sample");
	paramHandle.param<std::string>("path",  file_path, ros::package::getPath("gasbot_historian")+"/History/");
	paramHandle.param<std::string>("file",  file_name, title);
	*/

	string file_path_title = file_path+title+"/";
	string file_name_date  = file_name+"_"+date_time_+".dat";

	// Create Directory
	if(mkdir(file_path_title.c_str(),0777)==-1){ }

	// Create File
	historyFile.open((file_path_title+file_name_date).c_str());
	
	// Print info
	ROS_INFO("A file '%s' has been created in the following directory \n%s",file_name_date.c_str(),file_path_title.c_str());
}

// ######################### Writing log file ##############################
void writeFile(double posX, double posY, double posW, int statePTU, double anglePan, double angleTilt, int measure){
	if(historyFile){

		//historyFile<<ros::Time::now()<<" "<<posX<<" "<<posY<<" "<<posW<<" "<<statePTU<<" "<<anglePan<<" "<<angleTilt<<" "<<measure<<endl;
		historyFile<<posX<<" "<<posY<<" "<<posW<<" "<<statePTU<<" "<<anglePan<<" "<<angleTilt<<" "<<measure<<endl;
		// Print info
		ROS_INFO("Pose <%.2f,%.2f,%.2f>; PTU Status %d; Pan-Tilt Status <%.2f,%.2f>; Measurement %d;",\
			  posX,posY,posW,statePTU,anglePan,angleTilt,measure);
	}
}

// ######################### Closing log file ##############################
void closeFile(){
	if(historyFile){
		historyFile.close();
	}
}

// ######################### Callback: PTU Status ##############################
void callbackPTUState(const std_msgs::Int16::ConstPtr& sta){
	statePTU = sta->data;
}

// ######################### Callback: PTU Joint Status ##############################
void callbackPTUJointState(const sensor_msgs::JointState::ConstPtr& jsta){
	/*
	vecJointState  = jsta->position;
	angleTilt = vecJointState[0];
	anglePan  = vecJointState[1];
	*/
	angleTilt = jsta->position[0];
	anglePan  = jsta->position[1];

	// Print info
	//ROS_INFO("Pan Angle %.2f, Tilt Angle %.2f",anglePan,angleTilt);

}

// ######################### Callback: Husky Pose ##############################
void callbackHuskyPose(const nav_msgs::Odometry::ConstPtr& enco){
	
	//Position
	posX = enco->pose.pose.position.x;
	posY = enco->pose.pose.position.y;
	
	// Orientation
	tf::Quaternion q(enco->pose.pose.orientation.x, enco->pose.pose.orientation.y, \
	enco->pose.pose.orientation.z, enco->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	posW = (yaw)*180/M_PI;

	// Print info
	//ROS_INFO("position (X,Y,W) is... %.2f,%.2f,%.2f",posX,posY,posW);
	
}

// ######################### Callback: RMLD Measurements ##############################
void callbackRMLD(rmld_node::rmld_msg mea){
	measure  = mea.concentration_ppmm;
	rmld_msg = mea.rmld_data_string;
	

	// Writing to the file
	// Note: It is obvious that the file will be updated only 
	//       if a gas measurement is available.
	//measure >= 50.0 &&
	if (measure >= GAS_MEASURE_THRESHOLD && (statePTU==OPERATION_STATUS || OPERATION_STATUS==5)){	
		writeFile(posX,posY,posW,statePTU,anglePan,angleTilt,measure);
	}
}	

int main(int argc, char **argv)
{
	//string name = argv[1];
	string title;

	ros::init(argc, argv, "Historian");
	
	// ===== DEFAULT PARAMETERS ====== //
	// Gas Measurements	
	ros::NodeHandle paramHandle ("~");
	paramHandle.param("threshold",GAS_MEASURE_THRESHOLD,15.00);
	paramHandle.param("operation",OPERATION_STATUS,3);
	// Log File
	paramHandle.param<std::string>("title",title,"Sample");
	paramHandle.param<std::string>("path",file_path,ros::package::getPath("gasbot_historian")+"/History/");
	paramHandle.param<std::string>("file",file_name,title);
	// Localization Topic
	paramHandle.param<std::string>("localization_topic",localization_topic,"/ndt_mcl");	

	// =========================== TOPICS TO SUBSCRIBE =============================== //
	ros::NodeHandle n;
	createFile(title,file_path,file_name);
	ros::Subscriber sub1 = n.subscribe("/rmld/data",1000,callbackRMLD);
	ros::Subscriber sub2 = n.subscribe("/ptu_control/state",1000,callbackPTUState);
	ros::Subscriber sub3 = n.subscribe("/amtec/joint_states",1000,callbackPTUJointState);
	//ros::Subscriber sub4 = n.subscribe("/ndt_mcl",1000,callbackHuskyPose);
	ros::Subscriber sub4 = n.subscribe(localization_topic,1000,callbackHuskyPose);

	ros::spin();
	closeFile();
	return 0;
}

