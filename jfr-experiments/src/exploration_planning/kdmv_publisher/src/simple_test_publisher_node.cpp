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
std::vector<double> vec_RobotOrigin;

std::string cell_FileName, roff_FileName;


double hard_offset_x, hard_offset_y;

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
	    fileYpoints.open((FilePath+yPts_FileName).c_str(), std::ios::app);
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
	    fileCellSize.open((FilePath+cell_FileName).c_str(), std::ios::app);
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
	    fileRobotOrigin.open((FilePath+roff_FileName).c_str(), std::ios::app);
	    if (fileRobotOrigin.is_open()){
		    while(fileRobotOrigin >> valueOrigin){
			    vec_RobotOrigin.push_back(valueOrigin);
			    ROS_INFO("robot origin: %f",valueOrigin);
         	}
		    fileRobotOrigin.close();
		    
		    ROS_INFO("Reading robot origin completed... ");
		    
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
	    fileConcent.open((FilePath+conc_FileName).c_str(), std::ios::app);
	    if (fileConcent.is_open()){
		    while(fileConcent >> valueCon){
			    vecConcent.push_back(valueCon);
			    /*
			    if (valueCon > 0.1){
			        
			        ROS_INFO("con value: %f",valueCon);
			        //ros::WallDuration(0.2).sleep();
		        }
		        */
         	}
		    fileConcent.close();
	    }
	    else std::cout << "Unable to open concentration file";
	
	    //--- Read X-points
	    fileXpoints.open((FilePath+xPts_FileName).c_str(), std::ios::app);
	    if (fileXpoints.is_open()){
		    while(fileXpoints >> valueX){
			    vecXpoints.push_back(valueX);
         	}
		    fileXpoints.close();
	    }
	    else std::cout << "Unable to open X points file";
	
	    //--- Read Y-points
	    fileYpoints.open((FilePath+yPts_FileName).c_str(), std::ios::app);
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
	    param_n.param<std::string>("robot_origin_file_name",roff_FileName,"prismaforum5_origin_conf.dat");
	
	    param_n.param<std::string>("ptu_sweep_topic",topicPTUSweepStatus,"/ptu_control/state");
	    
	    param_n.param<std::string>("colormap",colormap,std::string(DEFAULT_COLORMAP));
	    param_n.param<int>("n_points",n_points_map,DEFAULT_N_POINTS_MAP);
	    param_n.param<double>("max_sensor_val",max_sensor_val,DEFAULT_MAX_SENSOR_VAL);
	    param_n.param<double>("min_sensor_val",min_sensor_val,DEFAULT_MIN_SENSOR_VAL);
	    
	    param_n.param<double>("hard_offset_x",hard_offset_x,0.0);
        param_n.param<double>("hard_offset_y",hard_offset_y,0.0);
	    
	    	    
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
        


	    	    
	    //--- Retrive map data
	    //----------------------------------
	    readMapInfo();	    
	    double robot_offset_x = (vec_RobotOrigin[0]-1) * cell_size;
	    double robot_offset_y = (vec_RobotOrigin[1]-1) * cell_size;
	    
                     
        
        	    
        //=======================
	    // recursive loop
	    //=======================
	    
	    ros::spinOnce();
	    	    
	    while (ros::ok()){
            
            //-- Retrive map data
	        //=============================================
	        readMapInfo();
	        
	        // -1 to compensate Matlab indices	                
	        map_min_x = ( (*min_element(vecXpoints.begin(),vecXpoints.end())-1) * cell_size) - robot_offset_x - 5; 
	        map_max_x = ( (*max_element(vecXpoints.begin(),vecXpoints.end())-1) * cell_size) - robot_offset_x + 5;
	        map_min_y = ( (*min_element(vecYpoints.begin(),vecYpoints.end())-1) * cell_size) - robot_offset_y - 5;
	        map_max_y = ( (*max_element(vecYpoints.begin(),vecYpoints.end())-1) * cell_size) - robot_offset_y + 5;
	        
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
	        ROS_INFO("   - Min X (map): %f",map_min_x);
	        ROS_INFO("   - Max X (map): %f",map_max_x);
	        ROS_INFO("   - Min Y (map): %f",map_min_y);
	        ROS_INFO("   - Max Y (map): %f",map_max_y);
	        ROS_INFO("   - Cell size: %f",cell_size);
	        ROS_INFO("   - Kernel size: %f",kernel_size);
	        ROS_INFO("   - Max sensor value: %f",max_sensor_val);
	        ROS_INFO("   - Min sensor value: %f",min_sensor_val);
	        ROS_INFO("   - Colormap %s",colormap.c_str());
	        ROS_INFO("   - Number of points %d",n_points_map);
	        	        
	        //-- read updated map and update access time
	        ROS_INFO("Reading reconstruction file...");
	        readReconstructionFiles();
	        
	                    
	        //std::cout<<"map size is: "<<vecConcent.size()<<std::endl;
	        ROS_INFO("Current map size and number of <x,y> points are: %zd <%zd,%zd>",\
	                  vecConcent.size(),vecXpoints.size(),vecYpoints.size());
	        //ros::WallDuration(1).sleep();
	        
            
            //-- filling cells with concentration values            
            ROS_INFO("Filling concentrations...");
	        for (int i=0;i<vecXpoints.size();i++){
	            for (int j=0;j<vecYpoints.size();j++){
	                

                    	                
	                float x = ((vecXpoints[i]-vec_RobotOrigin[0]+0.0-1.0)*cell_size)+hard_offset_x;
	                float y = ((vecYpoints[j]-vec_RobotOrigin[1]+0.0-1.0)*cell_size)+hard_offset_y;
	                float r = vecConcent[(i*vecYpoints.size())+j];

                    /*
	                if (r > 0.1){
	                    
                        int ind = (i*vecYpoints.size())+j;
                        ROS_INFO("<i,j,ind> are: <%d,%d,%d>",i,j,ind);
                        
                        ROS_INFO("<x,y,r> are: <%f,%f,%f>",x,y,r);
                        ros::WallDuration(0.1).sleep();
                    }
                    */
                    GDM_map.addDataPoint(x,y,r);
	                
	                //curr_x       = (vecXpoints[i]-1)*cell_size;
	                //curr_y       = (vecYpoints[j]-1)*cell_size;
        	        
        	        //float this_x = (vecXpoints[i]-1)*cell_size;
        	        //float this_y = (vecYpoints[j]-1)*cell_size;
        	        
        	        
        	        //int this_ind = (i*vecYpoints.size())+j;
        	        
        	        
        	        //float this_read = vecConcent[this_ind];
        	        
        	        //curr_reading = vecConcent[(i*vecYpoints.size())+j];
        	        
        	        //ROS_INFO("<x,y,concentration> are: <%f,%f,%f>",curr_x,curr_y,curr_reading);
        	        //ROS_INFO("concentration file index is: %f",(i*vecXpoints.size())+j);
        	        //ROS_INFO("concentration file index is: %d",this_ind);
        	        
        	        
        	        //ROS_INFO("<x,y,ind,conc> are: <%f,%f,%d,%f>",this_x,this_y,this_ind,this_read);
        	        //ROS_INFO("<i,j,ind> are: <%d,%d,%d>",i,j,this_ind);
        	        
        	        //ros::WallDuration(1).sleep();
        	        
        	        //GDM_map.addDataPoint(curr_x-robot_offset_x,curr_y-robot_offset_y,curr_reading);
        	        //GDM_map.publishMap(mean_advertise);
	            }
	        }
	        
	        ros::WallDuration(1).sleep();
	        
	        //-- publish the updated map
	        ROS_INFO("Publishing the new map...");
	        GDM_map.publishMap(mean_advertise);
	        
		    ros::spinOnce();
        	loop_rate.sleep();
        }
}



