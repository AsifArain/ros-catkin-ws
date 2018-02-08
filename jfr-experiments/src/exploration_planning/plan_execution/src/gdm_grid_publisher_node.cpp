/*

                     ENVIRONMENT MAP PUBLISHER NODE
              ____________________________________________________


    This is main file to publish environment map.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    07-Dec-2017
    Version: 0.0
    -------------------------------------------------------------------------


*/



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>


using namespace std;

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
std::vector<double> vec_RobotOrigin,vec_Concentration,vec_Confs;
std::vector<int> vec_MapSize;
std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements, \
            filename__Confs;

double hard_offset_x, hard_offset_y, high_t;

//================================================================================
//              READ ENVIRONMENT MAP
//================================================================================
void readEnvironmentMap(){

        
        ROS_INFO("Reading environment map... ");	    
	    std::ifstream file__MapSize, \
	                  file__MapConcentration, \
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
	    //--- concentration map
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
	    //--- Conf
	    //==============================
	    /*
	    ROS_INFO("Conf... ");
	    file__Confs.open((FilePath+filename__Confs).c_str(), std::ios::app);
	    //ROS_INFO("Conf file: %s",(FilePath+filename__Confs).c_str());
	    double this_conf;
	    vec_Confs.clear();
	    if (file__Confs.is_open()){		    
		    while(file__Confs >> this_conf){
			    vec_Confs.push_back(this_conf);
			    //ROS_INFO("Confs: this_conf %f",this_conf);
         	}
		    file__Confs.close();
		    //ROS_INFO("Confs: size %zd",vec_Confs.size());
	    }
	    else std::cout << "Unable to open conf file";
	    */
	    
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
//                      MAIN
//================================================================================
int main( int argc, char** argv ){
        
        
        printf("\n=================================================================");
	    printf("\n=	   Gas High Concentrtion Distribution Map Publisher Node       ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "conc_map_publisher_node");
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
    
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        paramHandle.param<double>("high_threshould",high_t,500.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
        //ros::Publisher map_marker = n.advertise<visualization_msgs::Marker>("placement_map", 10);
        ros::Publisher map_marker = n.advertise<visualization_msgs::Marker>("conc_map", 10);
        
        ros::Rate r(30);
        
        
        //--- read map info
        readEnvironmentMap();
        
        
        //---------------------------------
        // VISUALIZATION MARKERS
        //---------------------------------
        
        
        //--- Marker initialization
        //--------------------------------
        /*
        marker_none.header.frame_id = "/map";        
        marker_none.header.stamp = ros::Time::now();
        marker_none.ns = "conc_map_publisher_node";
        marker_none.action = visualization_msgs::Marker::ADD;
        marker_none.pose.orientation.w = 1.0;   
        marker_none.id = 0;      
        marker_none.type = visualization_msgs::Marker::POINTS;
        marker_none.scale.x = 0.45; //cell_size; //0.2;
        marker_none.scale.y = 0.45; //cell_size; //0.2;
        marker_none.color.r = 0.0f;
        marker_none.color.g = 0.7f;
        marker_none.color.b = 0.0f;
        marker_none.color.a = 0.25;
        */
        
        
        //--- Marker high concentration
        //--------------------------------        
        marker_high.header.frame_id = "/map";        
        marker_high.header.stamp = ros::Time::now();
        marker_high.ns = "conc_map_publisher_node";
        marker_high.action = visualization_msgs::Marker::ADD;
        marker_high.pose.orientation.w = 1.0;   
        marker_high.id = 1;
        marker_high.type = visualization_msgs::Marker::POINTS;
        marker_high.scale.x = 0.45; //cell_size; //0.2;
        marker_high.scale.y = 0.45; //cell_size; //0.2;
        marker_high.color.r = 0.7f;
        marker_high.color.g = 0.0f;
        marker_high.color.b = 0.0f;
        marker_high.color.a = 0.25; //0.75
        
        
        //--- Marker low concentration
        //--------------------------------
        marker_low.header.frame_id = "/map";        
        marker_low.header.stamp = ros::Time::now();
        marker_low.ns = "conc_map_publisher_node";
        marker_low.action = visualization_msgs::Marker::ADD;
        marker_low.pose.orientation.w = 1.0;   
        marker_low.id = 2;      
        marker_low.type = visualization_msgs::Marker::POINTS;
        marker_low.scale.x = 0.45; //cell_size; //0.2;
        marker_low.scale.y = 0.45; //cell_size; //0.2;
        marker_low.color.r = 0.0f;
        marker_low.color.g = 0.7f;
        marker_low.color.b = 0.0f;
        marker_low.color.a = 0.25; //0.25
        
        
        //--- Marker initialization (map origin)
        //--------------------------------
        /*
        marker_orig.header.frame_id = "/map";        
        marker_orig.header.stamp = ros::Time::now();
        marker_orig.ns = "conc_map_publisher_node";
        marker_orig.action = visualization_msgs::Marker::ADD;
        marker_orig.pose.orientation.w = 1.0;   
        marker_orig.id = 3;
        marker_orig.type = visualization_msgs::Marker::POINTS;
        marker_orig.scale.x = 0.45; //cell_size; //0.2;
        marker_orig.scale.y = 0.45; //cell_size; //0.2;
        marker_orig.color.r = 0.0f;
        marker_orig.color.g = 0.0f;
        marker_orig.color.b = 1.0f;
        marker_orig.color.a = 1.00;
        */
        
        //--- Marker initialization (confs)
        //--------------------------------        
        /*
        marker_conf_sphere.header.frame_id = "/map";        
        marker_conf_sphere.header.stamp = ros::Time::now();
        marker_conf_sphere.ns = "conc_map_publisher_node";
        marker_conf_sphere.action = visualization_msgs::Marker::ADD;
        marker_conf_sphere.pose.orientation.w = 1.0;   
        marker_conf_sphere.id = 4;
        marker_conf_sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_conf_sphere.scale.x = 0.25; //cell_size; //0.2;
        marker_conf_sphere.scale.y = 0.25; //cell_size; //0.2;
        marker_conf_sphere.scale.z = 0.25; //cell_size; //0.2;
        marker_conf_sphere.color.r = 0.0f;
        marker_conf_sphere.color.g = 0.0f;
        marker_conf_sphere.color.b = 1.0f;
        marker_conf_sphere.color.a = 1.00;
        */
        
        
        //--- Marker initialization (confs)
        //--------------------------------        
        /*
        marker_conf_arrow.header.frame_id = "/map";        
        marker_conf_arrow.header.stamp = ros::Time::now();
        marker_conf_arrow.ns = "conc_map_publisher_node";
        marker_conf_arrow.action = visualization_msgs::Marker::ADD;
        marker_conf_arrow.pose.orientation.w = 1.0;   
        marker_conf_arrow.id = 5;
        marker_conf_arrow.type = visualization_msgs::Marker::ARROW;
        marker_conf_arrow.scale.x = 0.25; //cell_size; //0.2;
        marker_conf_arrow.scale.y = 0.25; //cell_size; //0.2;
        marker_conf_arrow.scale.z = 0.25; //cell_size; //0.2;
        marker_conf_arrow.color.r = 0.0f;
        marker_conf_arrow.color.g = 0.0f;
        marker_conf_arrow.color.b = 1.0f;
        marker_conf_arrow.color.a = 1.00;
        */
        
        
        //--------------------------------        
        //--- map cells
        //--------------------------------
        //ROS_INFO("map size is %d,%d",vec_MapSize[0],vec_MapSize[1]);        
        for (int i = 0; i <= vec_MapSize[0]-1; ++i){        
            for (int j = 0; j <= vec_MapSize[1]-1; ++j){
            
                //float x = ((i-vec_RobotOrigin[0])*cell_size)+hard_offset_x;
                //float y = ((j-vec_RobotOrigin[1])*cell_size)+hard_offset_y;                                        
                //float x = ((i-vec_RobotOrigin[0]+1)*cell_size)+hard_offset_x;
                //float y = ((j-vec_RobotOrigin[1]+1)*cell_size)+hard_offset_y;
                float x = ((i-vec_RobotOrigin[0]+0.5-0.0)*cell_size)+hard_offset_x;
                float y = ((j-vec_RobotOrigin[1]+0.5-0.0)*cell_size)+hard_offset_y;
                float z = 0.5; //0.1
                
                //ROS_INFO("origin are %f,%f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
                //ROS_INFO("i,j are %d,%d",i,j);
                //ROS_INFO("x,y are %f,%f",x,y);
                                          
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                
                //-- high concentration cell
                if (vec_Concentration[(i*vec_MapSize[1])+j] > high_t){
                    marker_high.points.push_back(p);
                }
                //-- low concentration cell
                else{
                    marker_low.points.push_back(p);
                }
            }
        }
        
        
        //--------------------------------        
        //--- confs
        //--------------------------------
        /*
        //ROS_INFO("map size is %d,%d",vec_MapSize[0],vec_MapSize[1]);        
        for (int i = 0; i<(vec_Confs.size()/3); ++i){        
            
            float x = ((vec_Confs[i*3+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size)+hard_offset_x;
            float y = ((vec_Confs[i*3+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size)+hard_offset_y;
            
            //float x = vec_Confs[i*3+0]/2;
            //float y = vec_Confs[i*3+1]/2;
            float o = vec_Confs[i*3+2]/2;            
            float z = 1.0;
            
            //ROS_INFO("this conf x,y are %f,%f",x,y);
                                      
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            
            marker_conf_sphere.points.push_back(p);
        }
        */
                
        //--------------------------------
        //-- map origin (troubleshooting)
        //--------------------------------
        /*        
        //float qr_x = 135.0, qr_y = 031.0;
        //float qr_x = 291.0, qr_y = 63.0;
        float qr_x = 291.0, qr_y = 63.0;
        
        //float or_x = 135.0, or_y = 031.0;
        float or_x = vec_RobotOrigin[0], or_y = vec_RobotOrigin[1];
        
        float x = ((qr_x-or_x+0.5-0.0)*cell_size);
        float y = ((qr_y-or_y+0.5-0.0)*cell_size);                                    
        float z = 1.8;
                
        //ROS_INFO("origin vector is %f,%f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
        //ROS_INFO("x,y of origin are %f,%f",x,y);
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        marker_orig.points.push_back(p);
        */              
        
        //--------------------------------
        //-- publish map
        //--------------------------------
        
        ROS_INFO("The map is being published...");
                
        while(ros::ok()){
        
        
            //-- publish 
            //------------------
            
            map_marker.publish(marker_low);
            map_marker.publish(marker_high);
                                    
            ros::spinOnce();
            r.sleep();
        }
}

