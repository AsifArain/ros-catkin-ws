/* OPEN ISSUES:
-- Date is incorrect in output file.
*/ 


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
//#include "/usr/include/bullet/LinearMath/btMatrix3x3.h"
//#include <LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles
#include "geometry_msgs/Vector3.h"
#include <ros/package.h>

using namespace std;

// File
ofstream historyFile;
string package_path = ros::package::getPath("gasbot_historian");
string file_path;
string file_name;
//string date_time_;

// Variables for RMLD
int measure;
string rmld_msg;

// Variables for PTU
int statePTU;
vector<double> vecJointState;
double statePan,stateTilt;

// Variables for localization
double posX,posY,posW;
//ros::Publisher rpy_publisher;

void openFile(string title, string file_path, string file_name){

	// ===== Date and Time
	time_t date_temp;       	// Stores seconds elapsed since 01-01-1970
	struct tm *date_format; 	// Saves in Date format
	char date_out[25];     		// Output string

	time(&date_temp);
	date_format = localtime(&date_temp);
	strftime(date_out, 25, "%Y-%m-%d-%H:%M:%S", date_format);

	char date_date[11];
	strftime(date_date, 11, "%Y-%m-%d%H:%M:%S", date_format);
	//sprintf(date_date, "%02d/%02d/%04d", date_format->tm_mday, date_format->tm_mon + 1, date_format->year + 1900);
	string date_str(date_date);
	string date_time_(date_out);
		
	/*		// create directories
	if(mkdir(("/home/marco/Desktop/ROS_Workspace/DataLogging/History/Result_"+date_time_+name).c_str(),0777)==-1)//creating a directory
	    {    }
	// Create file
	historyFile.open(("/home/marco/Desktop/ROS_Workspace/DataLogging/History/Result_"+date_time_+name+"/"+name+date_time_+"data.dat").c_str());
	*/
	

	// ####################### DEFAULT PARAMETERS ########################
	ros::NodeHandle paramHandle ("~");
	paramHandle.param<std::string>("title", title,	  "Sample");
	paramHandle.param<std::string>("path",  file_path, package_path+"/History/"+title+"/");
	paramHandle.param<std::string>("file",  file_name, title+"_"+date_time_+".dat");
	// ###################################################################

	// Create Directory
	if(mkdir(file_path.c_str(),0777)==-1){ }

	// Create File
	historyFile.open((file_path+file_name).c_str());
	// Print info
	ROS_INFO("A file '%s' has been created in the following directory \n%s",file_name.c_str(),file_path.c_str());
}

void closeFile(){
	if(historyFile){
		historyFile.close();
	}
}

/*
void writeFile(double concentration_ppmm, double origin_x, double origin_y, double origin_z, double end_x, double end_y, double end_z, string rmld_data_string)
	{
		if(historyFile)
		{
		historyFile << ros::Time::now() <<" "<< concentration_ppmm <<" "<<origin_x<<" "<<origin_y<<" "<<origin_z<<" "<<end_x<<" "<<end_y<<" "<<end_z<<" "<<rmld_data_string<<endl; 
		}
	}
*/


//void writeFile(double concentration_ppmm, double origin_x, double origin_y, double origin_z, double end_x, double end_y, double end_z, string rmld_data_string)
//void writeFile(double concentration_ppmm)

/*
void writeFile(double concentration_ppmm, string rmld_data_string, int statePTU, double stateTilt, \
			   double statePan, double posX, double posY, double posW){
*/

void writeFile(double posX, double posY, double posW, int statePTU, double statePan, double stateTilt, int measure){
	if(historyFile){
		historyFile<<ros::Time::now()<<" "<<posX<<" "<<posY<<" "<<posW<<" "<<statePTU<<" "<<statePan<<" "<<stateTilt<<" "<<measure<<endl; 
		//historyFile <<concentration_ppmm <<endl; 
	}
}

void callbackPTUState(const std_msgs::Int16::ConstPtr& sta){
	//ROS_INFO("PTU status is...%d",sta->data);
	statePTU = sta->data;
}


void callbackPTUJointState(const sensor_msgs::JointState::ConstPtr& jsta){
	/*
	vecJointState  = jsta->position;
	stateTilt = vecJointState[0];
	statePan  = vecJointState[1];
	*/
	stateTilt = jsta->position[0];
	statePan  = jsta->position[1];
}

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
	//std::cout << "Roll: " << (roll)*180/M_PI << ", Pitch: " << (pitch)*180/M_PI << ", Yaw: " << (yaw)*180/M_PI << std::endl;
	posW = (yaw)*180/M_PI;

	ROS_INFO("position (X,Y,W) is... %.2f,%.2f,%.2f",posX,posY,posW);

	/*	
	double roll,pitch,heading;
	btQuaternion q = btQuaternion(enco->pose.pose.orientation.x, \
        enco->pose.pose.orientation.y, enco->pose.pose.orientation.z, \
        enco->pose.pose.orientation.w); 
	//btMatrix3x3(q).getRPY (btScalar &roll, btScalar &pitch, btScalar &heading, unsigned int solution_number=1) const
	tf::Matrix3x3(q).getRPY(roll, pitch, heading); */
	//btMatrix3x3(q).getRPY(roll, pitch, heading);
    	//t = mea->header.stamp;
	
	//btQuaternion q_orig(enco.quat[0], enco.quat[1], enco.quat[2], enco.quat[3]);
	//double roll, pitch, yaw;
	//tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	//posW = enco->pose.pose.position.x;

	//btQuaternion q(target_state->target.transform.rotation.x, target_state->target.transform.rotation.y, target_state->target.transform.rotation.z, target_state->target.transform.rotation.w);
	//double roll, pitch, yaw;
	//tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	
}


void callbackRMLD(rmld_node::rmld_msg mea){
	//ROS_INFO("I heard: [%s]", mea->data.c_str());
	measure  = mea.concentration_ppmm;
	rmld_msg = mea.rmld_data_string;
	  
	//writeFile(mea.concentration_ppmm, mea.origin_x, mea.origin_y, mea.origin_z, mea.end_x, mea.end_y, mea.end_z, mea.rmld_data_string);
	//writeFile(mea.concentration_ppmm,mea.rmld_data_string,statePTU,stateTilt,statePan,posX,posY,posW);
	writeFile(posX,posY,posW,statePTU,statePan,stateTilt,measure);
	//writeFile(mea.concentration_ppmm);

	// Print on screen:
	/*
	cout<<"\nmsg.concentration_ppmm: "<<mea.concentration_ppmm;
	//cout<<"\nmsg.origin_x: "<<mea.origin_x;
	//cout<<"\nmsg.origin_y: "<<mea.origin_y;
	//cout<<"\nmsg.origin_z: "<<mea.origin_z;
	//cout<<"\nmsg.end_x: "<<mea.end_x;
	//cout<<"\nmsg.end_y: "<<mea.end_y;
	//cout<<"\nmsg.end_z: "<<mea.end_z;
	cout<<"\nmsg.rmld_data_string: "<<mea.rmld_data_string<<endl;
	*/
}	

int main(int argc, char **argv)
{
	//string name = argv[1];
	string title;
	ros::init(argc, argv, "Historian");

	
	// ####################### TOPICS ############################
	ros::NodeHandle n;
	openFile(title,file_path,file_name);
	ros::Subscriber sub1 = n.subscribe("/rmld/data", 1000, callbackRMLD);
	//ros::Subscriber sub1 = n.subscribe("/rmld/data","/rmld/data",1000,callbackRMLD);
	//ros::Subscriber sub2 = n.subscribe("/amtec/pan_state", 1000, panCallback);
	//ros::Subscriber sub2 = n.subscribe("/amtec/joint_states", 1000, callbackAmtecState);
	ros::Subscriber sub2 = n.subscribe("/ptu_control/state",1000,callbackPTUState);
	ros::Subscriber sub3 = n.subscribe("/amtec/joint_state",1000,callbackPTUJointState);
	ros::Subscriber sub4 = n.subscribe("/encoder",1000,callbackHuskyPose);

	ros::spin();
	closeFile();
	return 0;
}

