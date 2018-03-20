/*

            GAS DISTRIBUTION MAP (GDM) POINTCLOUD (PCL) PUBLISHER NODE
               DURING THE EXPLORATION FOR GAS EMISSION MONITORING
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


using namespace std;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud map_cloud;


//--- Visualization Markers
//==========================
visualization_msgs::Marker marker_none, \
                           marker_low, \
                           marker_high, \
                           marker_orig, \
                           marker_conf_sphere, \
                           marker_conf_arrow, \
                           marker_ar;


//--- File Variables
//==========================
double cell_size;
std::vector<double> vec_RobotOrigin,vec_Concentration,vec_Confs,vec_reconColorR,vec_reconColorG,vec_reconColorB;
std::vector<int> vec_MapSize;
std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements, \
            filename__reconstructionColorR, \
            filename__reconstructionColorG, \
            filename__reconstructionColorB, \
            filename__Confs;

double hard_offset_x, hard_offset_y, high_t;

//================================================================================
//              READ RECONSTRUCTION FILES
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
	                  file__Confs;
	    
        
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
	    //--- Reconstruction magnitudes
	    //==============================
	    ROS_INFO("Map... ");
	    file__MapConcentration.open((FilePath+filename__MapSensorPlacements).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__MapSensorPlacements).c_str());
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
//              CREATE MAP CLOUD
//================================================================================
void createMapCloud(){    


        
    //--- read map info
    readReconstructionFiles();
    
    
    //PointCloud map_cloud;
    
    
    //--------------------------------        
    //--- map cells
    //--------------------------------
    //ROS_INFO("map size is %d,%d",vec_MapSize[0],vec_MapSize[1]);        
    for (int i = 0; i <= vec_MapSize[0]-1; ++i){        
        for (int j = 0; j <= vec_MapSize[1]-1; ++j){            
            
            float x = ((i-vec_RobotOrigin[0]+0.5-0.0)*cell_size)+hard_offset_x;
            float y = ((j-vec_RobotOrigin[1]+0.5-0.0)*cell_size)+hard_offset_y;
            float z = 0.5; //0.1
            
            //ROS_INFO("origin are %f,%f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
            //ROS_INFO("i,j are %d,%d",i,j);
            //ROS_INFO("x,y are %f,%f",x,y);
                                      
            
            //-- high concentration cell
            if (vec_Concentration[(i*vec_MapSize[1])+j] > high_t){
                
                                    
                pcl::PointXYZRGB pointToPush;
                
                pointToPush.x = x; pointToPush.y = y; pointToPush.z = z;
                pointToPush.r = vec_reconColorR[(i*vec_MapSize[1])+j];
                pointToPush.g = vec_reconColorG[(i*vec_MapSize[1])+j];
                pointToPush.b = vec_reconColorB[(i*vec_MapSize[1])+j];
                
                /*
                ROS_INFO("this <r,g,b> is <%f,%f,%f>",vec_reconColorR[(i*vec_MapSize[1])+j],\
                                                      vec_reconColorG[(i*vec_MapSize[1])+j],\
                                                      vec_reconColorB[(i*vec_MapSize[1])+j]);
                */
            }
        }
    }
    
    
    map_cloud.height = 1;
    map_cloud.width = map_cloud.points.size();
    map_cloud.header.frame_id = "/map";
        
        


}

//================================================================================
//                      MAIN
//================================================================================
int main( int argc, char** argv ){
        
        
        printf("\n=================================================================");
	    printf("\n=	            Exploration GDM Publisher Node                     ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "exploration_gdm_publisher_node");
        ros::NodeHandle n;
        
        // ####################### PARAMETERS ########################
	    ros::NodeHandle paramHandle ("~");
        
        //============================================	
	    //----- Map Info Parameters
	    //============================================
	    paramHandle.param<std::string>("file_path",FilePath,ros::package::getPath("plan_execution")+"/logs/");
	    paramHandle.param<std::string>("map_file",filename__MapSensorPlacements,"reconstruction.dat");
	    paramHandle.param<std::string>("map_size_file",filename__MapSize,"reconstruction_mapsize.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"reconstruction_cellsize.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"reconstruction_origin.dat");
	    //paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
	    paramHandle.param<std::string>("recon_color_r_file",filename__reconstructionColorR,"reconstructionColorR.dat");
	    paramHandle.param<std::string>("recon_color_g_file",filename__reconstructionColorG,"reconstructionColorG.dat");
	    paramHandle.param<std::string>("recon_color_b_file",filename__reconstructionColorB,"reconstructionColorB.dat");
	    
    
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        paramHandle.param<double>("high_threshould",high_t,500.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================        
        ros::Publisher map_publisher = n.advertise<sensor_msgs::PointCloud2> ("gdm_pcl", 1);
        
        
        ros::Rate r(30);        
        
        
        
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
            map_publisher.publish (map_cloud.makeShared());
                                    
            ros::spinOnce();
            r.sleep();
        }
}

