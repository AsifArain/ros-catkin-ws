/*

                  CONFIGURATIONS POSE ARRAY PUBLISHER NODE
              ____________________________________________________


    This is main file to publish conf poses.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    07-Jan-2018
    Version: 0.0
    -------------------------------------------------------------------------


*/


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include "ptu_control2/commandSweep.h"

//#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include <std_msgs/Int64.h>

//#include "plan_execution/common_prelude.h"
//#include "ptu_control2/commandSweep.h"
//#include "amtec/GetStatus.h"

//#include "sensor_msgs/JointState.h" // ptu joint call back

#include <boost/math/constants/constants.hpp>
const double PI = boost::math::constants::pi<double>();

#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>

using namespace std;


//-- visualization variables
//------------------------------
geometry_msgs::PoseArray conf_orientations;
visualization_msgs::Marker conf_positions; 

//--- File Variables
//-------------------------------
double cell_size;
std::vector<double> vec_RobotOrigin,vec_Confs;
std::vector<int> vec_MapSize;

std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements, \
            filename__Confs, \
            experimentTitle, \
            explorationStrategy;

double hard_offset_x, hard_offset_y;

//================================================================================
//              READING CONFIGURATION DATA
//================================================================================
void readDataFiles(){

        
        //ROS_INFO("Reading data files... ");	    
	    std::ifstream file__MapSize, \
	                  file__MapSensorPlacements, \
	                  file__CellSize, \
	                  file__RobotOrigin, \
	                  file__Confs;
        
        
	    //==============================
	    //--- Conf
	    //==============================
	    //ROS_INFO("Conf... ");
	    file__Confs.open((FilePath+filename__Confs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filename__Confs).c_str());
	    double this_conf;
	    vec_Confs.clear();
	    if (file__Confs.is_open()){		    
		    while(file__Confs >> this_conf){
			    vec_Confs.push_back(this_conf);
			    //ROS_INFO("Confs: this_conf %f",this_conf);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_Confs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    file__Confs.close();
		    //ROS_INFO("Confs: total size %zd",vec_Confs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    
	    
	    //==============================
	    //--- Read Cell Size
	    //==============================
	    //ROS_INFO("Cell size... ");
	    file__CellSize.open((FilePath+filename__CellSize).c_str(), std::ios::app);
	    if (file__CellSize.is_open()){
         	file__CellSize >> cell_size;
		    file__CellSize.close();
		    //ROS_INFO("Cell size: %f",cell_size);
	    }	    
	    else std::cout << "Unable to open file for cell size";
	    
	    
	    //==============================
	    //--- Read Robot Origin
	    //==============================
	    //ROS_INFO("Robot origin... ");	    
	    file__RobotOrigin.open((FilePath+filename__RobotOrigin).c_str(), std::ios::app);
	    double this_origin;
	    if (file__RobotOrigin.is_open()){
		    while(file__RobotOrigin >> this_origin){
			    vec_RobotOrigin.push_back(this_origin);
         	}
		    file__RobotOrigin.close();
	    }
	    else std::cout << "Unable to open file robot origin cell size";
	    	    
}





int main (int argc, char** argv){



        printf("\n=================================================================");
	    printf("\n=	         Configuration Placements Publisher Node               ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "conf_publisher_node");
        ros::NodeHandle n ("planned_confs");
        
        // ####################### PARAMETERS ########################
	    ros::NodeHandle paramHandle ("~");
        
        //============================================	
	    //----- Map Info Parameters
	    //============================================
	    //paramHandle.param<std::string>("file_path",FilePath,ros::package::getPath("plan_execution")+"/logs/");
	    paramHandle.param<std::string>("experiment_title",experimentTitle,"prismaforum5-04");
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,"one-step-exploration");
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"prismaforum_origin_conf.dat");
	    paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
        
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
                
        ros::Publisher positions_pub    = n.advertise<visualization_msgs::Marker>("positions", 10);
        ros::Publisher orientations_pub = n.advertise<geometry_msgs::PoseArray>("orientations", 10);
        
        
        ros::Rate r(30);
        
        
        //--- read map info
        //readDataFiles();
        
        
        
        //---------------------------------
        // position marker
        //---------------------------------
        conf_positions.header.frame_id = "/map";        
        //conf_positions.header.stamp = ros::Time::now();
        conf_positions.ns = "env_map_publisher_node";
        conf_positions.action = visualization_msgs::Marker::ADD;
        conf_positions.pose.orientation.w = 1.0;   
        conf_positions.id = 0;
        conf_positions.type = visualization_msgs::Marker::SPHERE_LIST;
        conf_positions.scale.x = 0.25; 
        conf_positions.scale.y = 0.25; 
        conf_positions.scale.z = 0.25; 
        conf_positions.color.r = 0.39f;
        conf_positions.color.g = 0.39f;
        conf_positions.color.b = 0.39f;
        conf_positions.color.a = 1.00;
        
                      
        //---------------------------------
        // orientation marker
        //---------------------------------
        //conf_orientations.poses.clear();
        //conf_orientations.header.stamp = ros::Time::now();
        conf_orientations.header.frame_id = "/map";
        
        
        
        while(ros::ok()){
        
            //--- read map info
            readDataFiles();
            
            conf_positions.points.clear();
            conf_orientations.poses.clear();
            
            
            for (size_t i = 0; i<(vec_Confs.size()/3); ++i){
                
                //float x = ((vec_Confs[i*3+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
                //float y = ((vec_Confs[i*3+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
                float x = vec_Confs[i*3+0];
                float y = vec_Confs[i*3+1];;
                float z = 1.0;
                float o = vec_Confs[i*3+2];
                
                //ROS_INFO("this conf x,y,th are %f,%f,%f",x,y,o);
                
                //-- positions
                geometry_msgs::Point pnt;
                pnt.x = x;
                pnt.y = y;
                pnt.z = z;            
                conf_positions.points.push_back(pnt);
                
                //-- orientation
                geometry_msgs::PoseStamped pos;            
                pos.pose.position.x = x;
                pos.pose.position.y = y;
                pos.pose.position.z = z;
                pos.pose.orientation = tf::createQuaternionMsgFromYaw(o*M_PI/180);
                conf_orientations.poses.push_back(pos.pose);
            }
            
            //-- update time stamps
            conf_positions.header.stamp = ros::Time::now();
            conf_orientations.header.stamp = ros::Time::now();
            
            //-- publish 
            //------------------
            
            positions_pub.publish(conf_positions);
            orientations_pub.publish(conf_orientations);
            
            //ROS_INFO("spinning");
            ros::spinOnce();
            r.sleep();            
            
        }
}


//--------------------- THE END ---------------------//
