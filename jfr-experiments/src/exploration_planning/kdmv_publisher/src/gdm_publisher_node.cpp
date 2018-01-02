/*

                         GAS DISTRIBUTION MAP PUBLISHER
              ____________________________________________________


    This is main file to publish gas distribution map based on the 
    implementations of Kernal DM+V gas distribution mapping algorithm.
    
    
    -------------------------------------------------------------------------
    Original Kernal DM+V written by:  Victor Hernandez
    Modified for GDM publication by:  Asif Arain    
    Date:                             27-Nov-2017
    Version:                          0.0
    -------------------------------------------------------------------------


*/


#include "kernel_dmv_node.h"

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

//--- ROS-topic Variables
//==========================
std::string topicPTUSweepStatus;

//--- PTU/Sweeping Variables
//==========================
int statusPTU;


//--- Visualization Markers
//==========================
visualization_msgs::Marker map_msg;


//--- File Variables
//==========================
std::string conc_FileName, xPts_FileName, yPts_FileName; // file names
std::string FilePath; // file path(s)
std::vector<double> vecConcent, vecXpoints, vecYpoints; //vector contains goals

//--- Map Data
//==========================
double cell_size, robot_origin_x, robot_origin_y, map_min_x, map_max_x, map_min_y, map_max_y;
std::vector<double> vecROrigin;

std::string cell_FileName, roff_FileName;



//================================================================================
//              READ MAP INFO
//================================================================================
void readMapInfo(){

        
        ROS_INFO("Reading map info... ");
        
	    double valueX, valueY, valueCell, valueOrigin;
	    vecXpoints.clear();
	    vecYpoints.clear();	
	    std::ifstream fileXpoints, fileYpoints, fileCellSize, fileRobotOrigin;
	
	    //--- Read X-points
	    //==============================
	    ROS_INFO("Reading x points... ");
	    fileXpoints.open((FilePath+xPts_FileName).c_str(), std::ios::app);
	    if (fileXpoints.is_open()){
		    while(fileXpoints >> valueX){
			    vecXpoints.push_back(valueX);
         	}
		    fileXpoints.close();
	    }
	    else std::cout << "Unable to open X points file";
	
	    //--- Read Y-points
	    //==============================
	    ROS_INFO("Reading y points... ");
	    fileYpoints.open((FilePath+"/"+yPts_FileName).c_str(), std::ios::app);
	    if (fileYpoints.is_open()){
		    while(fileYpoints >> valueY){
			    vecYpoints.push_back(valueY);
         	}
		    fileYpoints.close();
	    }
	    else std::cout << "Unable to open Y points file";
	    
	    
	    cell_FileName, roff_FileName;
	    
	    //--- Read Cell Size
	    //==============================
	    ROS_INFO("Reading cell size... ");
	    fileCellSize.open((FilePath+"/"+cell_FileName).c_str(), std::ios::app);
	    if (fileCellSize.is_open()){
		    /*
		    while(fileCellSize >> valueCell){
			    vecYpoints.push_back(valueCell);
         	}
         	*/
         	fileCellSize >> cell_size; //valueCell;
		    fileCellSize.close();
	    }
	    else std::cout << "Unable to open file for cell size";
	    
	    
	    //--- Read Robot Origin
	    //==============================
	    ROS_INFO("Reading robot origin... ");
	    fileRobotOrigin.open((FilePath+"/"+roff_FileName).c_str(), std::ios::app);
	    if (fileRobotOrigin.is_open()){
		    while(fileRobotOrigin >> valueOrigin){
			    vecROrigin.push_back(valueOrigin);
         	}
		    fileRobotOrigin.close();
	    }
	    else std::cout << "Unable to open file robot origin cell size";
	
	    
}





//================================================================================
//              READ RECONSTRUCTION FILE
//================================================================================
void readReconstructionFiles(){

        
        ROS_INFO("Reading map file... ");
        
	    double valueCon, valueX, valueY;
	    vecConcent.clear();
	    vecXpoints.clear();
	    vecYpoints.clear();
	
	    std::ifstream fileConcent, fileXpoints, fileYpoints;
		
	    //--- Read concentrations
	    fileConcent.open((FilePath+"/"+conc_FileName).c_str(), std::ios::app);
	    if (fileConcent.is_open()){
		    while(fileConcent >> valueCon){
			    vecConcent.push_back(valueCon);
         	}
		    fileConcent.close();
	    }
	    else std::cout << "Unable to open concentration file";
	
	    //--- Read X-points
	    fileXpoints.open((FilePath+"/"+xPts_FileName).c_str(), std::ios::app);
	    if (fileXpoints.is_open()){
		    while(fileXpoints >> valueX){
			    vecXpoints.push_back(valueX);
         	}
		    fileXpoints.close();
	    }
	    else std::cout << "Unable to open X points file";
	
	    //--- Read Y-points
	    fileYpoints.open((FilePath+"/"+yPts_FileName).c_str(), std::ios::app);
	    if (fileYpoints.is_open()){
		    while(fileYpoints >> valueY){
			    vecYpoints.push_back(valueY);
         	}
		    fileYpoints.close();
	    }
	    else std::cout << "Unable to open Y points file";
	    
}


//================================================================================
//              CALLBACK: PTU SWEEPING STATUS
//================================================================================
void callback___PTUSweepingStatus(const std_msgs::Int16::ConstPtr& sta){
	//ROS_INFO("PTU status at callback is...%d",sta->data);
	statusPTU=sta->data;
}


//==========================================================================================
//
//					                    MAIN
//
//==========================================================================================
int main(int argc, char **argv)
{
	
	    printf("\n=================================================================");
	    printf("\n=	GDM Publisher Node, Ver %d",NODE_VERSION);
	    printf("\n=================================================================\n");
	
	    ros::init(argc, argv, "gdm_publisher_node");
	    ros::NodeHandle param_n("~");
	    ros::Rate loop_rate(4);	
	    
	    ros::NodeHandle n;
	

	    //----------------------------------------------------------------------------
	    // Parameter initialization
	    //----------------------------------------------------------------------------
	
	    std::string	frame_id;
	    std::string colormap;
	
	    double kernel_size;
	
	    double max_sensor_val;
	    double min_sensor_val;
	    int	n_points_map;
	    int	publish_hz;
	
	    param_n.param<std::string>("frame_id", frame_id, std::string(DEFAULT_FRAME_ID));
	    param_n.param<double>("kernel_size", kernel_size, DEFAULT_KERNEL_SIZE);	
		
	    param_n.param<std::string>("file_path",FilePath,ros::package::getPath("kdmv_publisher")+"/logs/");
	
	    param_n.param<std::string>("concentrations_file_name",conc_FileName,"reconstruction.dat");
	    param_n.param<std::string>("x_points_file_name",xPts_FileName,"x_coord.dat");
	    param_n.param<std::string>("y_points_file_name",yPts_FileName,"y_coord.dat");
	    param_n.param<std::string>("cell_size_file_name",cell_FileName,"cell_size.dat");
	    param_n.param<std::string>("robot_origin_file_name",roff_FileName,"robot_origin.dat");
	
	    param_n.param<std::string>("ptu_sweep_topic",topicPTUSweepStatus,"/ptu_control/state");
	    
	    param_n.param<std::string>("colormap",colormap,std::string(DEFAULT_COLORMAP));
	    param_n.param<int>("n_points",n_points_map,DEFAULT_N_POINTS_MAP);
	    param_n.param<double>("max_sensor_val",max_sensor_val,DEFAULT_MAX_SENSOR_VAL);
	    param_n.param<double>("min_sensor_val",min_sensor_val,DEFAULT_MIN_SENSOR_VAL);
	    
	    
	    	    
	    //============================================
	    // ROS Topic Subscriptions
        //============================================
        //-- PTU STATUS
        ros::Subscriber sub1 = n.subscribe(topicPTUSweepStatus,1000,callback___PTUSweepingStatus);   
        
        
	    //============================================
	    // ROS Topic Publishers
        //============================================
        //-- Mean Map
	    ros::Publisher mean_advertise = n.advertise<sensor_msgs::PointCloud2>("mean_map", 20);
	    //-- Variance Map
	    ros::Publisher var_advertise  = n.advertise<sensor_msgs::PointCloud2>("var_map",  20);
        //-- Map message
        ros::Publisher msg_pub        = n.advertise<visualization_msgs::Marker>("map_msg",20);


	    	    
	    //--- Retrive map data
	    //----------------------------------
	    readMapInfo();	    
	    double robot_offset_x = (vecROrigin[0]-1) * cell_size;
	    double robot_offset_y = (vecROrigin[1]-1) * cell_size;
	    
	    	    
	    boost::filesystem::path gdm_filepath((FilePath+conc_FileName).c_str());
	    //Ref: https://stackoverflow.com/questions/4279164/cboost-file-system-to-return-a-list-of-files-older-than-a-specific-time
	    
	    std::time_t gdm_accesstime_this;
	    std::time_t gdm_accesstime_last;
	    
	    gdm_accesstime_this = boost::filesystem::last_write_time( gdm_filepath );
	    
	    double accesstime_diff;
	    
	    
        //-- MAP MESSAGE
        //=======================
        
        map_msg.header.frame_id = "/map";
        map_msg.header.stamp = ros::Time::now();
        map_msg.ns = "map_msg_publisher";
        map_msg.action = visualization_msgs::Marker::ADD;
        
        map_msg.pose.position.x = 5.0;
        map_msg.pose.position.y = 2.0;
        map_msg.pose.position.z = 1.0;
        map_msg.pose.orientation.w = 1.0;
        
        map_msg.id = 4;
        map_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        map_msg.scale.z = 0.5;
                
        map_msg.color.r = 1.0f;
        map_msg.color.g = 0.0f;
        map_msg.color.b = 0.0f;
        map_msg.color.a = 1.0;
                     
        
        	    
        //=======================
	    // recursive loop
	    //=======================
	    
	    ros::spinOnce();
	    	    
	    while (ros::ok()){
	        
	        
	        map_msg.text.clear();
	        
	        
	        //-- complete the current scan
	        //=============================================
	        map_msg.text = "Wait! The map is being computed.";
	        while(statusPTU!=0){
	            ROS_INFO("Waiting for the scan to be completed.");
	            ros::WallDuration(2).sleep();
	            ros::spinOnce();
            }
	        
	                      
	        
	        //-- wait untill updated map is arrived
	        //=============================================
	        
	        accesstime_diff = difftime( gdm_accesstime_this,gdm_accesstime_last );
	        //Ref: http://www.cplusplus.com/reference/ctime/difftime/
	        	  
    	    map_msg.text = "Wait! The map is being computed.";
    	    //msg_pub.publish(map_msg);
	        while (statusPTU==0 && accesstime_diff==0){
	            
	            ROS_INFO("The map is being computed...");
	            
	            // Text message
                //map_msg.text = "Wait! The map is being computed.";
                msg_pub.publish(map_msg);
	            
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
            
            
            //-- publish empty text message
            
            map_msg.text = "The updated map is coming.";
            //map_msg.text.clear();
            msg_pub.publish(map_msg);
            
            
            //ROS_INFO("Im just after publishing the empty msg.");            
            //ros::WallDuration(10).sleep();
            
                                               
            //-- Retrive map data
	        //=============================================
	        readMapInfo();
	        
	        // -1 to compensate Matlab indices	                
	        map_min_x = ( (*min_element(vecXpoints.begin(),vecXpoints.end())-1) * cell_size) - robot_offset_x - 0; 
	        map_max_x = ( (*max_element(vecXpoints.begin(),vecXpoints.end())-1) * cell_size) - robot_offset_x + 9;
	        map_min_y = ( (*min_element(vecYpoints.begin(),vecYpoints.end())-1) * cell_size) - robot_offset_y - 0;
	        map_max_y = ( (*max_element(vecYpoints.begin(),vecYpoints.end())-1) * cell_size) - robot_offset_y + 9;
	        
	        
	        
	        //-- initialize a new map
            //=============================================
            
	        gas_map GDM_map(map_min_x,\
	                        map_max_x,\
	                        map_min_y,\
	                        map_max_y,\
	                        cell_size,\
	                        kernel_size,\
	                        min_sensor_val,\
	                        max_sensor_val,\
	                        colormap,\
	                        frame_id,\
	                        n_points_map);
	        
	        ROS_INFO("Algorithm parameters: (inside the loop) ");
	        ROS_INFO("   - Fixed frame: %s",frame_id.c_str());
	        ROS_INFO("   - Max X (map): %f",map_max_x);
	        ROS_INFO("   - Min X (map): %f",map_min_x);
	        ROS_INFO("   - Max Y (map): %f",map_max_y);
	        ROS_INFO("   - Min Y (map): %f",map_min_y);
	        ROS_INFO("   - Cell size: %f",cell_size);
	        ROS_INFO("   - Kernel size: %f",kernel_size);
	        ROS_INFO("   - Max sensor value: %f",max_sensor_val);
	        ROS_INFO("   - Min sensor value: %f",min_sensor_val);
	        ROS_INFO("   - Colormap %s",colormap.c_str());
	        ROS_INFO("   - Number of points %d",n_points_map);
	        	        
	        ros::WallDuration(1).sleep();
	        
	        //-- read updated map and update access time
	        ROS_INFO("Reading reconstruction file...");
	        readReconstructionFiles();
	        gdm_accesstime_last = gdm_accesstime_this;
	        	        
            ros::WallDuration(1).sleep();
            
	        //std::cout<<"map size is: "<<vecConcent.size()<<std::endl;
	        ROS_INFO("Current map size and number of <x,y> points are: %zd <%zd,%zd>",\
	                  vecConcent.size(),vecXpoints.size(),vecYpoints.size());
	        //ros::WallDuration(1).sleep();
	        
            
            //-- filling cells with concentration values            
            ROS_INFO("Filling concentrations...");
	        for (int i=0;i<vecXpoints.size();i++){
	            for (int j=0;j<vecYpoints.size();j++){
	                
	                curr_x       = (vecXpoints[i]-1)*cell_size;
	                curr_y       = (vecYpoints[j]-1)*cell_size;
        	        curr_reading = vecConcent[(i*vecXpoints.size())+j];
        	        //ROS_INFO("<x,y,concentration> are: <%f,%f,%f>",curr_x,curr_y,curr_reading);
        	        
        	        GDM_map.addDataPoint(curr_x-robot_offset_x,curr_y-robot_offset_y,curr_reading);
        	        //GDM_map.publishMap(mean_advertise);
	            }
	        }
	        
	        ros::WallDuration(1).sleep();
	        
	        //-- publish the updated map
	        ROS_INFO("Publishing the new map...");
	        GDM_map.publishMap(mean_advertise);
	        	        
	        //-- publish empty text message
            map_msg.text = ".";
            //map_msg.text.clear();
            msg_pub.publish(map_msg);
	        
	        	        
	        //-- ptu is idle
            //=============================================
	        while(statusPTU==0){
	            
	            //ROS_INFO("Waiting for the scan.");
	            
	            //-- publish the current map
	            ROS_INFO("Publishing the previous map...");
	            GDM_map.publishMap(mean_advertise);

	            ros::WallDuration(1).sleep();
	            ros::spinOnce();
            }
	        
	        
		    ros::spinOnce();
        	loop_rate.sleep();
        }
       

}



