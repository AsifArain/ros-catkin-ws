/*

                  EXPLORATION PLAN PUBLISHER NODE
              ____________________________________________________


    This is main file to publish following exploration plan.
        --> planned configurations (global)
        --> planned configurations (local)
        --> estimated hotspots center (hcs)
    
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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;


//-- visualization variables
//------------------------------
geometry_msgs::PoseArray gConfsPoses;
geometry_msgs::PoseArray lConfsPoses;
//visualization_msgs::Marker hcsMark; 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud hcsCloud;

//--- File Variables
//-------------------------------
double cell_size;
std::vector<double> vec_RobotOrigin,vec_gConfs,vec_lConfs,vec_Hcs;
std::vector<int> vec_MapSize;

std::string FilePath, \
            filenameMapSize, \
            filenameCellSize, \
            filenameRobotOrigin, \
            filenameMapSensorPlacements, \
            filenameGlobalConfs, \
            filenameLocalConfs, \
            filenameHotspots, \
            experimentTitle, \
            explorationStrategy;

double hard_offset_x, hard_offset_y;

double gConfZ;
double lConfZ;
double hcZ;


#define DEFAULT_G_CONF_Z   0.300
#define DEFAULT_L_CONF_Z   0.350
#define DEFAULT_HOT_Z      0.200


//================================================================================
//              READING CONFIGURATION DATA
//================================================================================
void readDataFiles(){

        
        //ROS_INFO("Reading data files... ");	    
	    std::ifstream fileMapSize, \
	                  fileMapSensorPlacements, \
	                  fileCellSize, \
	                  fileRobotOrigin, \
	                  fileGlobalConfs, \
	                  fileLocalConfs, \
	                  fileHotspots;
        
        
	    //==============================
	    //--- Global Conf
	    //==============================
	    //ROS_INFO("Conf... ");
	    fileGlobalConfs.open((FilePath+filenameGlobalConfs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filenameGlobalConfs).c_str());
	    double this_gconf;
	    vec_gConfs.clear();
	    if (fileGlobalConfs.is_open()){		    
		    while(fileGlobalConfs >> this_gconf){
			    vec_gConfs.push_back(this_gconf);
			    //ROS_INFO("Confs: this_gconf %f",this_gconf);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_gConfs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    fileGlobalConfs.close();
		    //ROS_INFO("Confs: total size %zd",vec_gConfs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    
	    
	    //==============================
	    //--- Local Conf
	    //==============================
	    //ROS_INFO("Conf... ");
	    fileLocalConfs.open((FilePath+filenameLocalConfs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filenameLocalConfs).c_str());
	    double this_lconf;
	    vec_lConfs.clear();
	    if (fileLocalConfs.is_open()){		    
		    while(fileLocalConfs >> this_lconf){
			    vec_lConfs.push_back(this_lconf);
			    //ROS_INFO("Confs: this_lconf %f",this_lconf);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_lConfs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    fileLocalConfs.close();
		    //ROS_INFO("Confs: total size %zd",vec_lConfs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    
	    
	    //==============================
	    //--- Hotspots
	    //==============================
	    //ROS_INFO("Hotspots... ");
	    fileHotspots.open((FilePath+filenameHotspots).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filenameHotspots).c_str());
	    double this_hc;
	    vec_Hcs.clear();
	    if (fileHotspots.is_open()){		    
		    while(fileHotspots >> this_hc){
			    vec_Hcs.push_back(this_hc);
			    //ROS_INFO("Confs: this_hc %f",this_hc);
			    //ROS_INFO("Confs: 0th vec conf %f",vec_Hcs);
			    //ros::WallDuration(0.1).sleep();
         	}
		    fileHotspots.close();
		    //ROS_INFO("Confs: total size %zd",vec_Hcs.size());
	    }
	    else std::cout << "Unable to open hcs file";
	    
	    
	    
	    //==============================
	    //--- Read Cell Size
	    //==============================
	    //ROS_INFO("Cell size... ");
	    fileCellSize.open((FilePath+filenameCellSize).c_str(), std::ios::app);
	    if (fileCellSize.is_open()){
         	fileCellSize >> cell_size;
		    fileCellSize.close();
		    //ROS_INFO("Cell size: %f",cell_size);
	    }	    
	    else std::cout << "Unable to open file for cell size";
	    
	    
	    //==============================
	    //--- Read Robot Origin
	    //==============================
	    //ROS_INFO("Robot origin... ");	    
	    fileRobotOrigin.open((FilePath+filenameRobotOrigin).c_str(), std::ios::app);
	    double this_origin;
	    if (fileRobotOrigin.is_open()){
		    while(fileRobotOrigin >> this_origin){
			    vec_RobotOrigin.push_back(this_origin);
         	}
		    fileRobotOrigin.close();
	    }
	    else std::cout << "Unable to open file robot origin cell size";
	    	    
}





int main (int argc, char** argv){



        printf("\n=================================================================");
	    printf("\n=	         Exploration Plan Publisher Node               ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "exploration_plan_publisher_node");
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
	    paramHandle.param<std::string>("cell_size_file",filenameCellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filenameRobotOrigin,"prismaforum_origin_conf.dat");
	    
	    
	    //paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
	    paramHandle.param<std::string>("global_conf_file",filenameGlobalConfs,"planned_confs_global.dat");
	    paramHandle.param<std::string>("local_conf_file",filenameLocalConfs,"planned_confs_local.dat");
	    paramHandle.param<std::string>("hotspots_file",filenameHotspots,"hotspots.dat");
        
        paramHandle.param<double>("global_conf_z",gConfZ,DEFAULT_G_CONF_Z);
	    paramHandle.param<double>("local_conf_z",lConfZ,DEFAULT_L_CONF_Z);
	    paramHandle.param<double>("hotspot_z",hcZ,DEFAULT_HOT_Z);
        
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
                
        ros::Publisher hcs_pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("hotspots",10);
        //ros::Publisher hcs_pub   = n.advertise<visualization_msgs::Marker>("hotspots", 10);
        ros::Publisher gconf_pub = n.advertise<geometry_msgs::PoseArray>("global_confs", 10);
        ros::Publisher lconf_pub = n.advertise<geometry_msgs::PoseArray>("local_confs", 10);
        
        ros::Rate r(30);
        
        
        //---------------------------------
        // hotspots marker
        //---------------------------------
        /*
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
        */
         
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
            
            //hcsMark.points.clear();
            hcsCloud.points.clear();
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
                float z = gConfZ; //0.30; //0.25; //1.0;
                float o = vec_gConfs[i*3+2];
                
                //ROS_INFO("this global conf x,y,th are %f,%f,%f",x,y,o);
                                
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
                float z = lConfZ; //0.35; //0.75; //1.0;
                float o = vec_lConfs[i*3+2];
                
                //ROS_INFO("this local conf x,y,th are %f,%f,%f",x,y,o);
                                
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
                float z = hcZ; //0.20; //0.45; //1.0;
                
                
                //ROS_INFO("this hotspot x,y,z are %f,%f,%f",x,y,z);
                
                //-- positions
                /*
                geometry_msgs::Point pnt;
                pnt.x = x;
                pnt.y = y;
                pnt.z = z;            
                hcsMark.points.push_back(pnt);
                */
                
                //-------------                
                pcl::PointXYZ p;                
                p.x = x; p.y = y; p.z = z;
                
                hcsCloud.points.push_back(p);
                
            }
            
            hcsCloud.height = 1;
            hcsCloud.width = hcsCloud.points.size();
            hcsCloud.header.frame_id = "/map";
            //----------------------
            
            
            //-- update time stamps
            //hcsMark.header.stamp = ros::Time::now();
            //gConfsPoses.header.stamp = ros::Time::now();            
            gConfsPoses.header.stamp = \
            lConfsPoses.header.stamp = ros::Time::now();
            //hcsMark.header.stamp = ros::Time::now();
            
            
            
            ros::Time time_st = ros::Time::now ();
            hcsCloud.header.stamp = time_st.toNSec()/1e3;            
            ros::WallDuration(0).sleep();
            //ROS_INFO("Publishing the new map...");
            
            
            //-- publish 
            //------------------            
            hcs_pub_cloud.publish (hcsCloud.makeShared());
            
            gconf_pub.publish(gConfsPoses);
            lconf_pub.publish(lConfsPoses);
            //hcs_pub.publish(hcsMark);
            
            
            
            //ROS_INFO("spinning");
            ros::spinOnce();
            r.sleep();
            
        }
}


//--------------------- THE END ---------------------//
