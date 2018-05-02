#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <fstream>
#include <boost/thread.hpp>
#include "ptu_control/commandSweep.h"
#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <math.h>       /* atan */

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//void getPOSES();
void followPOSES();
void gasDETECTION();
int  statusPTU;

// =================== Sweeping Parameters ========================
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle, sample_delay, tilt_angle;
int    num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;

// =================== Planning File Parameters ===================
std::string file_name;
std::string file_path;
//std::string file_path = ros::package::getPath("gasbot_planning");
std::vector<double> vecPoses; //vector contains goals
//double vecGoals; //vector contains goals

// =================== move_base Parameters =======================
std::string plan_frame_name;


// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//                                    PTU STATUS CALLBACK
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
void stateCallback(const std_msgs::Int16::ConstPtr& sta){
	//ROS_INFO("PTU status is...%d",sta->data);
	statusPTU=sta->data;
}

// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//                               GET POSES FROM EXTERNAL FILE
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

void getPOSES(){
	double value;
	std::ifstream filePoses;
	//std::string path = ros::package::getPath("roslib");
	//myFile.open((ros::package::getPath("gasbot_planning")+"/POSES.txt").c_str(),std::ios::app);
	//filePoses.open("/home/husky/ROS_CatkinWS/src/gasbot_planning/POSES.txt",std::ios::app);
	filePoses.open((file_path+"/"+file_name).c_str(), std::ios::app);
	if (filePoses.is_open()){
		std::cout << "File is open."<<std::endl;
		while(filePoses >> value){
			vecPoses.push_back(value);
			std::cout<<"value is "<<value<<std::endl;
     		}
		filePoses.close();
	}
	else std::cout << "Unable to open the file";
}


// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//                                 SCANNING FOR GAS DETECTION
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
void gasDETECTION(){
	ros::NodeHandle n;
  	ros::ServiceClient client1 = n.serviceClient<ptu_control::commandSweep>("/ptu_control/sweep");
  	ros::ServiceClient client2 = n.serviceClient<amtec::GetStatus>("/amtec/get_status");
  
	ptu_control::commandSweep srvSweep;
	//amtec::GetStatus  srvState;
	//rosservice call /ptu_control/sweep "{min_pan: -45.0, max_pan: 45.0, min_tilt: -10.0, max_tilt: -10.0, n_pan: 1, n_tilt: 1, samp_delay: 0.1}"

	// Finding Tilt Angle:
	//--------------------------
	/*
	* 	  | phi
	*  0.904m |      
	*         |         
	*         |--            
	*         |_|_____________________________theta
	* 		    sensing range
	*  phi   = atan(sensing range / 0.904)
	*  theta = atan(0.904 / sensing range)
	*  tilt_angle = 90-phi
	*/
	
	tilt_angle = (atan(sensing_range/offsetY_base_rmld)*(180/M_PI))-90;

	srvSweep.request.min_pan  	= -(FoV/2);        //min_pan_angle;   //-10;
	srvSweep.request.max_pan  	=  (FoV/2);        //max_pan_angle;   // 10;
	srvSweep.request.min_tilt 	= tilt_angle; 	   //min_tilt_angle;  //-10;
	srvSweep.request.max_tilt 	= tilt_angle;      //max_tilt_angle;  //-10;
	srvSweep.request.n_pan    	= num_pan_sweeps;  //  1;
	srvSweep.request.n_tilt   	= num_tilt_sweeps; //  1;
	srvSweep.request.samp_delay 	= sample_delay;    //0.1;
	
	if (client1.call(srvSweep)){
		ROS_INFO("Gas detection in progress ... <%.2f~%.2f,%.2f>",-(FoV/2),(FoV/2),tilt_angle);}
	else{
		ROS_ERROR("Failed to initialize gas scanning.");}
	/*
	if(client2.call(srvState)){     
		ROS_INFO("Got status");
		ROS_INFO("Sum: %f", (float)srvState.response.position_pan);}
	else{
		ROS_ERROR("Failed to get status.");} */
}

// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//                           NAVIGATING THROUGH DESIRED POSES
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
void followPOSES(){
	
	ROS_INFO("Here we go...");
	getPOSES();
	std::cout<<"size of vector "<<vecPoses.size()<<std::endl;

	ros::WallDuration(1).sleep();
	//tell the action client that we want to spin a thread by default
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");}

	//double goalX[6] = {1.0, 1.0, 0.0,-1.0, 0.0,-1.0};
	//double goalY[6] = {0.0, 1.0, 1.0, 0.0,-1.0,-1.0};
	//double goalW[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	//std::vector<double> goalX[4] = {  1.0,  1.0, -1.0, -1.0}; //goalX.size()

	/*
	double goalX[5] = {  1.0,  1.0,   2.0,   2.0,   0.0};
	double goalY[5] = {  0.0,  2.0,   3.0,   3.0,   0.0};
	double goalO[5] = {  0.0, 90.0, 180.0, -90.0,   0.0};
	*/
	
	for (int i=0;i<vecPoses.size()/4;i++){
		move_base_msgs::MoveBaseGoal goal;
		  
		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = plan_frame_name; //"/world"; //"base_link" //"base_footprint";
		goal.target_pose.header.stamp    = ros::Time::now();

		goal.target_pose.pose.position.x 	= vecPoses[(i*4)+0]; //goalX[i];
		goal.target_pose.pose.position.y 	= vecPoses[(i*4)+1]; //goalY[i];
		//goal.target_pose.pose.orientation.w 	= goalW[i]; //vecGoals[(i*3)+2]; //goalW[i];
		//goal.target_pose.pose.orientation 	= tf::createQuaternionMsgFromYaw(goalO[i]*M_PI/180);
		goal.target_pose.pose.orientation 	= tf::createQuaternionMsgFromYaw(vecPoses[(i*4)+2]*M_PI/180);

		//ROS_INFO("Gasbot is moving to pose %i i.e. <%.1fm,%.1fm,%.1fdeg>.",i+1,goalX[i],goalY[i],goalO[i]); 
		ROS_INFO("Gasbot is moving to Pose# %i <%.1fm,%.1fm,%.1fdeg>.",i,vecPoses[(i*4)+0],vecPoses[(i*4)+1],vecPoses[(i*4)+2]); 
		ac.sendGoal(goal);
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Gasbot has reached to desired pose %i",i);
			//ROS_INFO("Wait for a sec....");
			//ros::WallDuration(1).sleep();
			// ============ GAS SENSING ============
			if (vecPoses[(i*4)+3]==1){
				//ROS_INFO("Gas detection in progress....");
				gasDETECTION();
				while(statusPTU!=3){}
				//ros::WallDuration(20).sleep();
				ros::WallDuration(5).sleep();
				while(statusPTU!=0){}
					//ROS_INFO("Hamary ptu ka status...%i\n",global_status);}
				ROS_INFO("Gas detection COMPLETED.");
			}
		}
		else
			ROS_INFO("The Husky failed to reach desired pose.");
	}
}


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
	//paramHandle.param("min_tilt_angle", min_tilt_angle, -10.0);
	//paramHandle.param("max_tilt_angle", max_tilt_angle, -10.0);
	paramHandle.param("sensing_range", sensing_range,    15.0);
	paramHandle.param("offsetY_base_rmld",offsetY_base_rmld,0.904);
	paramHandle.param("field_of_view",FoV, 180.0);
	paramHandle.param("num_pan_sweeps", num_pan_sweeps,     1);
	paramHandle.param("num_tilt_sweeps",num_tilt_sweeps,    1);
	paramHandle.param("sample_delay",   sample_delay,     0.1);
	//paramHandle.getParam("min_pan_angle",min_pan_angle);
	//std::string global_name, relative_name, default_param;
	//paramHandle.param<std::string>("default_param", default_param, "default_value");
	
	// ========= File contains poses or plan =====================
	//std::string file_path = ros::package::getPath("gasbot_planning");
	//paramHandle.param<std::string>("file_name",file_name,"/home/asif/ROS_CatkinWS/src/gasbot_planning/POSES.txt");
	paramHandle.param<std::string>("file_name",file_name,"plan.txt");
	//paramHandle.param<std::string>("file_path",file_path,"/home/husky/ROS_CatkinWS/src/gasbot_planning");
	paramHandle.param<std::string>("file_path",file_path,ros::package::getPath("gasbot_planning"));	

	// ========= move_base Parameters =============================
	paramHandle.param<std::string>("plan_frame_name",plan_frame_name,"/world");


	
  //ros::Rate r(10); // 10 hz
  ros::Subscriber sub1 = n.subscribe("/ptu_control/state",10,stateCallback);
  boost::thread mythread(followPOSES);
  //followPOSES();
  //ros::spinOnce();  
  ros::spin();
  //ros::spinOnce();
  //r.sleep();
return 0;
}



