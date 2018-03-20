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
geometry_msgs::PoseArray gConfsPoses;
geometry_msgs::PoseArray lConfsPoses;
visualization_msgs::Marker hcsMark; 


//--- File Variables
//-------------------------------
double cell_size;
std::vector<double> vec_RobotOrigin,vec_gConfs,vec_lConfs,vec_Hcs;
std::vector<int> vec_MapSize;

std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements, \
            filename__GlobalConfs, \
            filename__LocalConfs, \
            filename__Hotspots, \
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
	                  file__GlobalConfs, \
	                  file__LocalConfs, \
	                  file__Hotspots;
        
        
	    //==============================
	    //--- Global Conf
	    //==============================
	    //ROS_INFO("Conf... ");
	    file__GlobalConfs.open((FilePath+filename__GlobalConfs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filename__GlobalConfs).c_str());
	    double this_gconf;
	    vec_gConfs.clear();
	    if (file__GlobalConfs.is_open()){		    
		    while(file__GlobalConfs >> this_gconf){
			    vec_gConfs.push_back(this_gconf);
			    //ROS_INFO("Confs: this_gconf %f",this_gconf);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_gConfs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    file__GlobalConfs.close();
		    //ROS_INFO("Confs: total size %zd",vec_gConfs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    
	    
	    //==============================
	    //--- Local Conf
	    //==============================
	    //ROS_INFO("Conf... ");
	    file__LocalConfs.open((FilePath+filename__LocalConfs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filename__LocalConfs).c_str());
	    double this_lconf;
	    vec_gConfs.clear();
	    if (file__LocalConfs.is_open()){		    
		    while(file__LocalConfs >> this_lconf){
			    vec_lConfs.push_back(this_lconf);
			    //ROS_INFO("Confs: this_lconf %f",this_lconf);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_gConfs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    file__LocalConfs.close();
		    //ROS_INFO("Confs: total size %zd",vec_gConfs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    
	    
	    //==============================
	    //--- Hotspots
	    //==============================
	    //ROS_INFO("Hotspots... ");
	    file__Hotspots.open((FilePath+filename__Hotspots).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filename__Hotspots).c_str());
	    double this_hc;
	    vec_gConfs.clear();
	    if (file__Hotspots.is_open()){		    
		    while(file__Hotspots >> this_hc){
			    vec_Hcs.push_back(this_hc);
			    //ROS_INFO("Confs: this_hc %f",this_hc);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_gConfs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    file__Hotspots.close();
		    //ROS_INFO("Confs: total size %zd",vec_gConfs.size());
	    }
	    else std::cout << "Unable to open hcs file";
	    
	    
	    
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
        ros::NodeHandle n ("planned");
        
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
	    
	    
	    //paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
	    paramHandle.param<std::string>("global_conf_file",filename__GlobalConfs,"planned_confs_global.dat");
	    paramHandle.param<std::string>("local_conf_file",filename__LocalConfs,"planned_confs_local.dat");
	    paramHandle.param<std::string>("hotspots_file",filename__Hotspots,"hotspots.dat");
        
        
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
                
        ros::Publisher hcs_pub   = n.advertise<visualization_msgs::Marker>("hotspots", 10);
        ros::Publisher gconf_pub = n.advertise<geometry_msgs::PoseArray>("global_confs", 10);
        ros::Publisher lconf_pub = n.advertise<geometry_msgs::PoseArray>("local_confs", 10);
        
        ros::Rate r(30);
        
        
        //---------------------------------
        // hotspots marker
        //---------------------------------
        hcsMark.header.frame_id = "/map";        
        //hcsMark.header.stamp = ros::Time::now();
        hcsMark.ns = "env_map_publisher_node";
        hcsMark.action = visualization_msgs::Marker::ADD;
        hcsMark.pose.orientation.w = 1.0;   
        hcsMark.id = 0;
        hcsMark.type = visualization_msgs::Marker::SPHERE_LIST;
        hcsMark.scale.x = 0.55; 
        hcsMark.scale.y = 0.55; 
        hcsMark.scale.z = 0.55; 
        hcsMark.color.r = 0.0f;
        hcsMark.color.g = 0.0f;
        hcsMark.color.b = 1.0f;
        hcsMark.color.a = 1.00;
        
         
        //---------------------------------
        // confs poses
        //---------------------------------
        //gConfsPoses.poses.clear();
        //gConfsPoses.header.stamp = ros::Time::now();
        gConfsPoses.header.frame_id = "/map";
        lConfsPoses.header.frame_id = "/map";
        
        
        
        while(ros::ok()){
        
            //--- read map info
            readDataFiles();
            
            hcsMark.points.clear();
            gConfsPoses.poses.clear();
            lConfsPoses.poses.clear();
            
            //ROS_INFO("Im here....%d",vec_gConfs.size()/3);
            
            //-- global confs
            //--------------------
            for (size_t i = 0; i<(vec_gConfs.size()/3); ++i){
                
                float x = ((vec_gConfs[i*3+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
                float y = ((vec_gConfs[i*3+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
                //float x = vec_gConfs[i*3+0];
                //float y = vec_gConfs[i*3+1];;
                float z = 1.0;
                float o = vec_gConfs[i*3+2];
                
                //ROS_INFO("this conf x,y,th are %f,%f,%f",x,y,o);
                                
                //-- orientation
                geometry_msgs::PoseStamped pos;            
                pos.pose.position.x = x;
                pos.pose.position.y = y;
                pos.pose.position.z = z;
                pos.pose.orientation = tf::createQuaternionMsgFromYaw(o*M_PI/180);
                gConfsPoses.poses.push_back(pos.pose);
            }
            
            
            //-- local confs
            //------------------
            for (size_t i = 0; i<(vec_lConfs.size()/3); ++i){
                
                float x = ((vec_lConfs[i*3+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
                float y = ((vec_lConfs[i*3+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
                //float x = vec_lConfs[i*3+0];
                //float y = vec_lConfs[i*3+1];;
                float z = 1.0;
                float o = vec_lConfs[i*3+2];
                
                //ROS_INFO("this conf x,y,th are %f,%f,%f",x,y,o);
                                
                //-- orientation
                geometry_msgs::PoseStamped pos;            
                pos.pose.position.x = x;
                pos.pose.position.y = y;
                pos.pose.position.z = z;
                pos.pose.orientation = tf::createQuaternionMsgFromYaw(o*M_PI/180);
                lConfsPoses.poses.push_back(pos.pose);
            }
            
            //-- hotspots
            //----------------
            for (size_t i = 0; i<(vec_Hcs.size()/2); ++i){
                
                float x = ((vec_Hcs[i*2+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
                float y = ((vec_Hcs[i*2+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
                //float x = vec_Hcs[i*3+0];
                //float y = vec_Hcs[i*3+1];;
                float z = 1.0;
                
                
                //ROS_INFO("this conf x,y,th are %f,%f,%f",x,y,o);
                
                //-- positions
                geometry_msgs::Point pnt;
                pnt.x = x;
                pnt.y = y;
                pnt.z = z;            
                hcsMark.points.push_back(pnt);
                
            }
            
            
            
            //-- update time stamps
            //hcsMark.header.stamp = ros::Time::now();
            //gConfsPoses.header.stamp = ros::Time::now();            
            gConfsPoses.header.stamp = \
            lConfsPoses.header.stamp = \
            hcsMark.header.stamp = ros::Time::now();
            
            
            
            //-- publish 
            //------------------
            
            gconf_pub.publish(gConfsPoses);
            lconf_pub.publish(lConfsPoses);
            hcs_pub.publish(hcsMark);
            
            
            
            //ROS_INFO("spinning");
            ros::spinOnce();
            r.sleep();            
            
        }
}


//--------------------- THE END ---------------------//
