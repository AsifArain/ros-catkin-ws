/*

            GAS DISTRIBUTION MAP (GDM) POINTCLOUD (PCL) PUBLISHER NODE
          ______________________________________________________________


    This is main file to publish gas distribution map using pointcloud.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    08-Feb-2017
    Version: 0.0
    -------------------------------------------------------------------------


*/



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/package.h> // find ros package path
#include <boost/filesystem/operations.hpp>
#include <ctime>
#include <iostream>
//#include <chrono>
#include <algorithm>

#include "commandSweepAsif.h" //FIXME
#include "GetStatusAsif.h" //FIXME 
#include "std_msgs/Int16.h"
#include <std_msgs/Int64.h>



using namespace std;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud map_cloud;

//--- Visualization Markers
//==========================
visualization_msgs::Marker map_msg;

//--- File Variables
//==========================
double cell_size;
std::vector<double> vec_RobotOrigin, \
                    vec_Concentration, \
                    vec_Confs, \
                    vec_reconColorR, \
                    vec_reconColorG, \
                    vec_reconColorB, \
                    vec_Xpoints, \
                    vec_Ypoints;
                    
std::vector<int> vec_MapSize;
std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__Reconstruction, \
            filename__reconstructionColorR, \
            filename__reconstructionColorG, \
            filename__reconstructionColorB, \
            filename__XPts, \
            filename__YPts, \
            filename__Confs, \
            experimentTitle, \
            explorationStrategy;

double hard_offset_x, hard_offset_y, high_t;





//--- ROS-topic Variables
//==========================
std::string topicPTUSweepStatus;

//--- PTU/Sweeping Variables
//==========================
int statusPTU;








//================================================================================
//              READ ENVIRONMENT MAP
//================================================================================
void readReconstructionFiles(){

        
        ROS_INFO("Reading environment map... ");	    
	    std::ifstream file__MapSize, \
	                  file__MapConcentration, \
	                  file__reconColorR, \
	                  file__reconColorG, \
	                  file__reconColorB, \
	                  file__CellSize, \
	                  file__RobotOrigin, \
	                  file__Confs, \
	                  file__Xpoints, \
	                  file__Ypoints;
	    
	    //==============================
	    //--- Read X-points
	    //==============================
	    ROS_INFO("Reading x points... ");
	    file__Xpoints.open((FilePath+filename__XPts).c_str(), std::ios::app);
	    double valueX;
	    vec_Xpoints.clear();
	    if (file__Xpoints.is_open()){
		    while(file__Xpoints >> valueX){
			    vec_Xpoints.push_back(valueX);
         	}
		    file__Xpoints.close();
	    }
	    else std::cout << "Unable to open X points file";
	    
	    
	    //==============================
	    //--- Read Y-points
	    //==============================
	    ROS_INFO("Reading y points... ");
	    file__Ypoints.open((FilePath+filename__YPts).c_str(), std::ios::app);
	    double valueY;
	    vec_Ypoints.clear();
	    if (file__Ypoints.is_open()){
		    while(file__Ypoints >> valueY){
			    vec_Ypoints.push_back(valueY);
         	}
		    file__Ypoints.close();
	    }
	    else std::cout << "Unable to open Y points file";
	    
	    
        //==============================
        //--- map size
	    //==============================
	    ROS_INFO("Map size... ");
	    file__MapSize.open((FilePath+filename__MapSize).c_str(), std::ios::app);
	    int this_size;
	    vec_MapSize.clear();
	    if (file__MapSize.is_open()){
		    while(file__MapSize >> this_size){
			    vec_MapSize.push_back(this_size);
			    //ROS_INFO("MapSize: this_size %d",this_size);
         	}
		    file__MapSize.close();
	    }
	    else std::cout << "Unable to open map size file";
        
        	    
	    //==============================
	    //--- concentration map
	    //==============================
	    ROS_INFO("Map... ");
	    file__MapConcentration.open((FilePath+filename__Reconstruction).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__Reconstruction).c_str());
	    double this_map;
	    vec_Concentration.clear();
	    if (file__MapConcentration.is_open()){		    
		    while(file__MapConcentration >> this_map){
			    vec_Concentration.push_back(this_map);
         	}
		    file__MapConcentration.close();
		    //ROS_INFO("MapSensorPlacement: size %zd",vec_Concentration.size());
	    }
	    else std::cout << "Unable to open planning map file";
	    
	    
	    //==============================
	    //--- reconstruction color R
	    //==============================
	    ROS_INFO("Reconstruction Color R... ");
	    file__reconColorR.open((FilePath+filename__reconstructionColorR).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__reconstructionColorR).c_str());
	    int this_colR;
	    vec_reconColorR.clear();
	    if (file__reconColorR.is_open()){		    
		    while(file__reconColorR >> this_colR){
			    vec_reconColorR.push_back(this_colR);
         	}
		    file__reconColorR.close();
		    //ROS_INFO("ReconstructionColorR: size %zd",vec_reconColorR.size());
	    }
	    else std::cout << "Unable to open reconstruction color R file";
	    
	    
	    //==============================
	    //--- reconstruction color G
	    //==============================
	    ROS_INFO("Reconstruction Color G... ");
	    file__reconColorG.open((FilePath+filename__reconstructionColorG).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__reconstructionColorG).c_str());
	    int this_colG;
	    vec_reconColorG.clear();
	    if (file__reconColorG.is_open()){		    
		    while(file__reconColorG >> this_colG){
			    vec_reconColorG.push_back(this_colG);
         	}
		    file__reconColorG.close();
		    //ROS_INFO("ReconstructionColorG: size %zd",vec_reconColorG.size());
	    }
	    else std::cout << "Unable to open reconstruction color G file";
	    
	    
	    //==============================
	    //--- reconstruction color B
	    //==============================
	    ROS_INFO("Reconstruction Color B... ");
	    file__reconColorB.open((FilePath+filename__reconstructionColorB).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__reconstructionColorB).c_str());
	    int this_colB;
	    vec_reconColorB.clear();
	    if (file__reconColorB.is_open()){
		    while(file__reconColorB >> this_colB){
			    vec_reconColorB.push_back(this_colB);
         	}
		    file__reconColorB.close();
		    //ROS_INFO("ReconstructionColorB: size %zd",vec_reconColorB.size());
	    }
	    else std::cout << "Unable to open reconstruction color B file";
	    	    
	    
	    //==============================
	    //--- Read Cell Size
	    //==============================
	    ROS_INFO("Cell size... ");
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
	    ROS_INFO("Robot origin... ");	    
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
    

//================================================================================
//              CALLBACK: PTU SWEEPING STATUS
//================================================================================
void callback___PTUSweepingStatus(const std_msgs::Int16::ConstPtr& sta){
	//ROS_INFO("PTU status at callback is...%d",sta->data);
	statusPTU=sta->data;
}



//================================================================================
//              CREATE GDM CLOUD
//================================================================================
void createMapCloud(){


    //--- read map info
    readReconstructionFiles();
    
    //--------------------------------        
    //--- map cells
    //--------------------------------
    //ROS_INFO("map size is %d,%d",vec_MapSize[0],vec_MapSize[1]);        
    /*
    for (int i = 0; i <= vec_MapSize[0]-1; ++i){        
        for (int j = 0; j <= vec_MapSize[1]-1; ++j){
    */
    
    //ROS_INFO("Im here to create cloud....");
    
    //ROS_INFO("sizes are: %d, %d, %d", vec_Xpoints.size(),vec_Ypoints.size(),vec_Concentration.size());
    //ROS_INFO("sizes are: %d, %d, %d", vec_reconColorR.size(),vec_reconColorG.size(),vec_reconColorB.size());
    
    for (int i=0; i<vec_Xpoints.size(); i++){
        for (int j=0; j<vec_Ypoints.size(); j++){
            
            
            //ROS_INFO("In the loop...."); 
            
            //float x = ((i-vec_RobotOrigin[0])*cell_size)+hard_offset_x;
            //float y = ((j-vec_RobotOrigin[1])*cell_size)+hard_offset_y;                                        
            //float x = ((i-vec_RobotOrigin[0]+1)*cell_size)+hard_offset_x;
            //float y = ((j-vec_RobotOrigin[1]+1)*cell_size)+hard_offset_y;
            
            //float x = ((i-vec_RobotOrigin[0]+0.5-0.0)*cell_size)+hard_offset_x;
            //float y = ((j-vec_RobotOrigin[1]+0.5-0.0)*cell_size)+hard_offset_y;
            
            float x = ((vec_Xpoints[i]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
            float y = ((vec_Ypoints[j]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
            
            float z = 0.5; //0.1
            
            //ROS_INFO("origin are %f,%f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
            //ROS_INFO("i,j are %d,%d",i,j);
            //ROS_INFO("x,y are %f,%f",x,y);
            
            //-- high concentration cell
            //if (vec_Concentration[(i*vec_MapSize[1])+j] > high_t){
            //ROS_INFO("This index: %d", (i*vec_Ypoints.size())+j);
            //ROS_INFO("Concentration here: %f",vec_Concentration[(i*vec_Ypoints.size())+j]);
            //ROS_INFO("RGB here: <%f,%f,%f>",vec_reconColorR[(i*vec_Ypoints.size())+j],vec_reconColorG[(i*vec_Ypoints.size())+j],vec_reconColorB[(i*vec_Ypoints.size())+j]);
            
            if (vec_Concentration[(i*vec_Ypoints.size())+j] > high_t){
                
                
                //ROS_INFO("In color loop1...");
                pcl::PointXYZRGB p;
                //ROS_INFO("In color loop2...");
                p.x = x; p.y = y; p.z = z;
                
                //ROS_INFO("In color loop3...");
                
                //p.r = 255; p.g = 0.3f; p.b = 0.3f;
                p.r = vec_reconColorR[(i*vec_Ypoints.size())+j];
                p.g = vec_reconColorG[(i*vec_Ypoints.size())+j];
                p.b = vec_reconColorB[(i*vec_Ypoints.size())+j];
                
                //ROS_INFO("In color loop4...");
                
                /*
                ROS_INFO("this <r,g,b> is <%f,%f,%f>",vec_reconColorR[(i*vec_MapSize[1])+j],\
                                                      vec_reconColorG[(i*vec_MapSize[1])+j],\
                                                      vec_reconColorB[(i*vec_MapSize[1])+j]);
                */
                //ROS_INFO("In color loop5...");
                map_cloud.points.push_back(p);
                
            }
        }
    }
    
    
    map_cloud.height = 1;
    map_cloud.width = map_cloud.points.size();
    map_cloud.header.frame_id = "/map";
    
    //ROS_INFO("OUT OF CLOUD PUBLISHER...."); 

}

//==========================================================================================
//
//					                    MAIN
//
//==========================================================================================
int main( int argc, char** argv ){
        
        
        printf("\n=================================================================");
	    printf("\n=	           Exploration GDM Publisher Node                      ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "exploration_gdm_publisher_node");
        
        //######## SUBSCRIBE/PUBLISHER #########
        ros::NodeHandle n ("gdm_exploration");
        
        //########## PARAMETERS ################
	    ros::NodeHandle paramHandle ("~");
        
        //============================================	
	    //----- Map Info Parameters
	    //============================================
	    
	    paramHandle.param<std::string>("experiment_title",experimentTitle,"prismaforum5-04");
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,"one-step-exploration");
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    paramHandle.param<std::string>("reconstruction_file",filename__Reconstruction,"reconstruction.dat");
	    paramHandle.param<std::string>("x_points_file",filename__XPts,"x_coord.dat");
	    paramHandle.param<std::string>("y_points_file",filename__YPts,"y_coord.dat");	    
	    paramHandle.param<std::string>("map_size_file",filename__MapSize,"reconstruction_mapsize.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"reconstruction_cellsize.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"reconstruction_origin.dat");
	    paramHandle.param<std::string>("recon_color_r_file",filename__reconstructionColorR,"reconstructionColorR.dat");
	    paramHandle.param<std::string>("recon_color_g_file",filename__reconstructionColorG,"reconstructionColorG.dat");
	    paramHandle.param<std::string>("recon_color_b_file",filename__reconstructionColorB,"reconstructionColorB.dat");
	    
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        paramHandle.param<double>("high_threshould",high_t,100.0);
        paramHandle.param<std::string>("ptu_sweep_topic",topicPTUSweepStatus,"/ptu_control/state");
        
                        
        //============================================
	    // ROS Topic Subscriptions
        //============================================
        //-- PTU STATUS
        ros::Subscriber sub1 = n.subscribe(topicPTUSweepStatus,1000,callback___PTUSweepingStatus);
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
        //-- Map cloud
        ros::Publisher cloud_publisher = n.advertise<sensor_msgs::PointCloud2> ("cloud",10);
        //-- Map message
        ros::Publisher msg_publisher = n.advertise<visualization_msgs::Marker>("msg",10);
        
        ros::Rate r(30);
        
        
        //-- Access time of reconstruction file
        //--------------------------------------------
        boost::filesystem::path gdm_filepath((FilePath+filename__Reconstruction).c_str());
	    //Ref: https://stackoverflow.com/questions/4279164/cboost-file-system-to-return-a-list-of-files-older-than-a-specific-time
	    
	    std::time_t gdm_accesstime_this;
	    std::time_t gdm_accesstime_last;
	    
	    gdm_accesstime_this = boost::filesystem::last_write_time( gdm_filepath );
	    
	    double accesstime_diff;
        
        
        //-- MAP MESSAGE INITIALIZATION
        //--------------------------------
        
        map_msg.header.frame_id = "/base_footprint";
        map_msg.header.stamp = ros::Time::now();
        map_msg.ns = "map_msg_publisherlisher";
        map_msg.action = visualization_msgs::Marker::ADD;
        
        map_msg.pose.position.x = 0.0;
        map_msg.pose.position.y = 2.0;
        map_msg.pose.position.z = 1.0;
        map_msg.pose.orientation.w = 1.0;
        
        map_msg.id = 4;
        map_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        map_msg.scale.z = 5.5;
                
        map_msg.color.r = 1.0f;
        map_msg.color.g = 0.0f;
        map_msg.color.b = 0.0f;
        map_msg.color.a = 1.0;
        
        
        //================================
	    // EXPLORATION WORKING LOOP 
	    //================================
	    
	    ros::spinOnce();
	    	    
	    while (ros::ok()){
	        
	        map_msg.text.clear();
	        	        
	        //-- complete the current scan
	        //=============================================
	        //map_msg.text = "Wait! The map is being computed.";
	        ROS_INFO("Waiting for the scan to be completed.");
	        while(statusPTU!=0){
	            //ROS_INFO("Waiting for the scan to be completed.");
	            ros::WallDuration(2).sleep();
	            ros::spinOnce();
            }
            ros::WallDuration(2).sleep();
            
	        //-- wait untill updated map is arrived
	        //=============================================
	        
	        accesstime_diff = difftime( gdm_accesstime_this,gdm_accesstime_last );
	        //Ref: http://www.cplusplus.com/reference/ctime/difftime/
	        	  
    	    map_msg.text = "Wait! The map is being computed.";
    	    //msg_publisher.publish(map_msg);
	        while (statusPTU==0 && accesstime_diff==0){
	            
	            //ROS_INFO("The map is being computed...");
	            
	            // Text message
                //map_msg.text = "Wait! The map is being computed.";
                msg_publisher.publish(map_msg);
	            
	            //-- publish the current map
	            //ROS_INFO("Publishing the previous map...");
	            //GDM_map.publishMap(mean_advertise);
	        
	            ros::WallDuration(5).sleep();
	            
	            //-- update access time difference
	            gdm_accesstime_this = boost::filesystem::last_write_time( gdm_filepath );
	            accesstime_diff = difftime( gdm_accesstime_this,gdm_accesstime_last );
	            
	            ros::spinOnce();	            
            }
            
            
            //-- publish updated map
	        //=============================================
                        
            map_msg.text = "The updated map is coming.";
            //map_msg.text.clear();
            msg_publisher.publish(map_msg);
            
            
            
            //--------------------------------
            //-- publish map
            //--------------------------------
            
            //-- create cloud
            createMapCloud();
            //-- update last access time (reconstruction file)
            gdm_accesstime_last = gdm_accesstime_this;
            
            //-- publish map cloud
            //Ref: https://answers.ros.org/question/172241/pcl-and-rostime/
            ros::Time time_st = ros::Time::now ();
            map_cloud.header.stamp = time_st.toNSec()/1e3;            
            ros::WallDuration(0).sleep();
            ROS_INFO("Publishing the new map...");
            cloud_publisher.publish (map_cloud.makeShared());
            
	        //-- publish map text (empty)
            map_msg.text = ".";
            //map_msg.text.clear();
            msg_publisher.publish(map_msg);
	        
	        	        
	        //-- ptu is idle
            //=============================================
            ROS_INFO("Publishing the previous map...");
	        while(statusPTU==0){
	            
	            //ROS_INFO("Waiting for the scan.");
	            
	            //-- publish the current map
	            //ROS_INFO("Publishing the previous map...");
	            cloud_publisher.publish (map_cloud.makeShared());

	            ros::WallDuration(1).sleep();
	            ros::spinOnce();
            }
	        
	        
		    ros::spinOnce();
        	r.sleep();
        }
       
        
        
        
        
        
        
        /*
        createMapCloud();
   
        //--------------------------------
        //-- publish map
        //--------------------------------
        
        ROS_INFO("The map is being published...");
                
        while(ros::ok()){
        
        
            //-- publish 
            //------------------
            //Ref: https://answers.ros.org/question/172241/pcl-and-rostime/
            ros::Time time_st = ros::Time::now ();
            map_cloud.header.stamp = time_st.toNSec()/1e3;
            
            cloud_publisher.publish (map_cloud.makeShared());
            
            
                                    
            ros::spinOnce();
            r.sleep();
        }
        */
}

