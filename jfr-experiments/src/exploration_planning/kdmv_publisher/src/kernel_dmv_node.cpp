/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2015, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors:
*********************************************************************/ 



//========================================================================================
//	KernelDM+V node
//	dmvNode.cpp
//	description: implements the KernelDM+V gas distribution mapping algorithm
//		
//		
//		
//	topics published:	
//	services:		
//				
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//	Revision log:
//	version: 1.0	2012/12/08
//========================================================================================

#include "kernel_dmv_node.h"

#include <ros/package.h> // find ros package path

using namespace std;

//--- File Variables
//==========================
std::string conc_FileName, xPts_FileName, yPts_FileName; // file names
std::string conc_FilePath, xPts_FilePath, yPts_FilePath; // file paths
std::vector<double> vecConcent, vecXpoints, vecYpoints; //vector contains goals




//================================================================================
//              READ RECONSTRUCTION FILE
//================================================================================
void readReconstructionFiles(){

	double valueCon, valueX, valueY;
	vecConcent.clear();
	vecXpoints.clear();
	vecYpoints.clear();
	
	std::ifstream fileConcent, fileXpoints, fileYpoints;
		
	//--- Read concentrations
	fileConcent.open((conc_FilePath+"/"+conc_FileName).c_str(), std::ios::app);
	if (fileConcent.is_open()){
		//std::cout << "File is open."<<std::endl;
		ROS_INFO("A file '%s' is to be read \n",(conc_FilePath+"/"+conc_FileName).c_str());
		while(fileConcent >> valueCon){
			vecConcent.push_back(valueCon);
			//std::cout<<"value is "<<valueCon<<std::endl;
     	}
		fileConcent.close();
	}
	else std::cout << "Unable to open concentration file";
	
	//--- Read X-points
	fileXpoints.open((xPts_FilePath+"/"+xPts_FileName).c_str(), std::ios::app);
	if (fileXpoints.is_open()){
		//std::cout << "File is open."<<std::endl;
		ROS_INFO("A file '%s' is to be read \n",(xPts_FilePath+"/"+xPts_FileName).c_str());
		while(fileXpoints >> valueX){
			vecXpoints.push_back(valueX);
			//std::cout<<"value is "<<valueX<<std::endl;
     	}
		fileXpoints.close();
	}
	else std::cout << "Unable to open X points file";
	
	//--- Read Y-points
	fileYpoints.open((yPts_FilePath+"/"+yPts_FileName).c_str(), std::ios::app);
	if (fileYpoints.is_open()){
		//std::cout << "File is open."<<std::endl;
		ROS_INFO("A file '%s' is to be read \n",(yPts_FilePath+"/"+yPts_FileName).c_str());
		while(fileYpoints >> valueY){
			vecYpoints.push_back(valueY);
			//std::cout<<"value is "<<valueY<<std::endl;
     	}
		fileYpoints.close();
	}
	else std::cout << "Unable to open Y points file";
}




void noseCallback(const std_msgs::Float32::ConstPtr& msg)
{
	mutex_nose.lock();
	new_data_nose = true;
	curr_reading = msg->data;
	mutex_nose.unlock();
}



void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	mutex_position.lock();
	new_data_position = true;
	curr_x = msg->pose.pose.position.x;
	curr_y = msg->pose.pose.position.y;
	mutex_position.unlock();
}




//==========================================================================================
//
//					MAIN
//
//==========================================================================================
int main(int argc, char **argv)
{
	
	printf("\n=================================================================");
	printf("\n=	Kernel DM+V Node, Ver %d",NODE_VERSION);
	printf("\n=================================================================\n");
	
	ros::init(argc, argv, "KernelDMVNode");
	ros::NodeHandle param_n("~");
	ros::Rate loop_rate(4);	
	
	ROS_INFO("Im really outside...");
	
	//----------------------------------------------------------------------------
	// Parameter initialization
	
	std::string	frame_id;
	std::string position_topic;
	std::string sensor_topic;
	std::string colormap;
	
	
	
	double map_min_x;
	double map_max_x;
	double map_min_y;
	double map_max_y;
	double cell_size;
	double kernel_size;
	
	double max_sensor_val;
	double min_sensor_val;
	double sensor_offset_x;
	double sensor_offset_y;
	int		n_points_map;
	int		publish_hz;
	
	param_n.param<std::string>("frame_id", frame_id, std::string(DEFAULT_FRAME_ID));
	param_n.param<std::string>("position_topic", position_topic, std::string(DEFAULT_POSITION_TOPIC));
	param_n.param<std::string>("sensor_topic", sensor_topic, std::string(DEFAULT_SENSOR_TOPIC));
	param_n.param<double>("map_min_x", map_min_x,DEFAULT_MAP_MIN_X);
	param_n.param<double>("map_max_x", map_max_x, DEFAULT_MAP_MAX_X);
	param_n.param<double>("map_max_y", map_max_y, DEFAULT_MAP_MAX_Y);
	param_n.param<double>("map_min_y", map_min_y, DEFAULT_MAP_MIN_Y);
	param_n.param<double>("cell_size", cell_size, DEFAULT_CELL_SIZE);
	param_n.param<double>("kernel_size", kernel_size, DEFAULT_KERNEL_SIZE);
	
		
	param_n.param<std::string>("concPath",conc_FilePath,ros::package::getPath("kdmv_publisher")+"/logs/");
	param_n.param<std::string>("xptsPath",xPts_FilePath,ros::package::getPath("kdmv_publisher")+"/logs/");
	param_n.param<std::string>("yptsPath",yPts_FilePath,ros::package::getPath("kdmv_publisher")+"/logs/");
	
	param_n.param<std::string>("concName",conc_FileName,"reconstruction.dat");
	param_n.param<std::string>("xptsName",xPts_FileName,"x_coord.dat");
	param_n.param<std::string>("yptsName",yPts_FileName,"y_coord.dat");
	
	
	
	param_n.param<int>("publish_hz", publish_hz, DEFAULT_PUBLISH_HZ);
	param_n.param<std::string>("colormap", colormap, std::string(DEFAULT_COLORMAP));
	param_n.param<int>("n_points", n_points_map, DEFAULT_N_POINTS_MAP);
	param_n.param<double>("max_sensor_val", max_sensor_val, DEFAULT_MAX_SENSOR_VAL);
	param_n.param<double>("min_sensor_val", min_sensor_val, DEFAULT_MIN_SENSOR_VAL);
	param_n.param<double>("sensor_offset_x", sensor_offset_x, DEFAULT_SENSOR_OFFSET_X);
	param_n.param<double>("sensor_offset_y", sensor_offset_y, DEFAULT_SENSOR_OFFSET_Y);
	
	
	
	ROS_INFO("Algorithm parameters: ");
	ROS_INFO("   - Fixed frame: %s",frame_id.c_str());
	ROS_INFO("   - Position topic: %s",position_topic.c_str());
	ROS_INFO("   - Sensor topic: %s",sensor_topic.c_str());
	ROS_INFO("   - Max X (map): %f",map_max_x);
	ROS_INFO("   - Min X (map): %f",map_min_x);
	ROS_INFO("   - Max Y (map): %f",map_max_y);
	ROS_INFO("   - Min Y (map): %f",map_min_y);
	ROS_INFO("   - Cell size: %f",cell_size);
	ROS_INFO("   - Kernel size: %f",kernel_size);
	ROS_INFO("   - Sensor offset (x): %f",sensor_offset_x);
	ROS_INFO("   - Sensor offset (y): %f",sensor_offset_y);
	ROS_INFO("   - Max sensor value: %f",max_sensor_val);
	ROS_INFO("   - Min sensor value: %f",min_sensor_val);
	ROS_INFO("   - Colormap %s",colormap.c_str());
	ROS_INFO("   - Number of points %d",n_points_map);
	
	
	//------------------------------------------------------------
	//----------------------------------
	// Subscriptions
	//----------------------------------	
	//ros::Subscriber sub_nose = param_n.subscribe(sensor_topic, 1000, noseCallback);
	//ros::Subscriber sub_position = param_n.subscribe(position_topic, 1000, positionCallback);	
	//----------------------------------
	// Advertisements
	//----------------------------------
	ros::Publisher mean_advertise = param_n.advertise<sensor_msgs::PointCloud2>("mean_map", 20);
	ros::Publisher var_advertise = param_n.advertise<sensor_msgs::PointCloud2>("var_map", 20);
	//----------------------------------
	// Initializes the algorithm
	//----------------------------------
	
	gas_map	GDM_map(map_min_x,map_max_x,map_min_y,map_max_y,cell_size,kernel_size,min_sensor_val,max_sensor_val,colormap,frame_id,n_points_map);
	/*
	int count_cycles=0;
	while (ros::ok()){
		if (new_data_nose || new_data_position)
		{
			mutex_nose.lock();
			mutex_position.lock();
			
			GDM_map.addDataPoint(curr_x+sensor_offset_x,curr_y+sensor_offset_y,curr_reading);
			count_cycles++;
			if (count_cycles > publish_hz*4) {
				GDM_map.publishMap(mean_advertise);
				count_cycles=0;
			}
			if (new_data_nose)
				new_data_nose=false;
			if (new_data_position)			
				new_data_nose=false;
			mutex_nose.unlock();
			mutex_position.unlock();
		}
		//ROS_INFO("Spinning");
		ros::spinOnce();
    	loop_rate.sleep();		
	}
	*/
	
		
	ROS_INFO("Im outside...");
	
	while (ros::ok()){
	
	    ROS_INFO("Here we go...");
	    readReconstructionFiles();
	    std::cout<<"map size is: "<<vecConcent.size()<<std::endl;
	    std::cout<<"number of x-points are: "<<vecXpoints.size()<<std::endl;
	    std::cout<<"number of y-points are: "<<vecYpoints.size()<<std::endl;
	    ros::WallDuration(1).sleep();
        
	    for (int i=0;i<vecXpoints.size();i++){
	        for (int j=0;j<vecYpoints.size();j++){
	        
	            curr_x       = vecXpoints[i]*cell_size;
	            curr_y       = vecYpoints[j]*cell_size;
    	        curr_reading = 200.0; //vecConcent[(i*vecXpoints.size())+j];
    	        
    	        GDM_map.addDataPoint(curr_x,curr_y,curr_reading);
    	        //GDM_map.publishMap(mean_advertise);
    	        
	        }
	    }
	    GDM_map.publishMap(mean_advertise);
	    /*
	    curr_x       = 5;
        curr_y       = 5;
        curr_reading = 101.5;        
        GDM_map.addDataPoint(curr_x,curr_y,curr_reading);
        GDM_map.publishMap(mean_advertise);
        */
	    //ros::WallDuration(5).sleep();
	    
	    ROS_INFO("Spinning");
		ros::spinOnce();
    	loop_rate.sleep();
    }
}



