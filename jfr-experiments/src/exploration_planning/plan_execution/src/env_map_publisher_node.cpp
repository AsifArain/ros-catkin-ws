/*

                     SENSOR PLACEMENTS MAP PUBLISHER NODE
              ____________________________________________________


    This is main file to publish sensor placements map.
    
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
visualization_msgs::Marker map_points, map_points1, map_points2;


//--- File Variables
//==========================
double cell_size;
std::vector<double> vec_RobotOrigin;
std::vector<int> vec_MapSize,vec_MapSensorPlacements;
std::string FilePath, filename__MapSize, filename__CellSize, filename__RobotOrigin, filename__MapSensorPlacements;

double hard_offset_x, hard_offset_y;

//================================================================================
//              READ ENVIRONMENT MAP
//================================================================================
void readEnvironmentMap(){

        
        ROS_INFO("Reading environment map... ");	    
	    std::ifstream file__MapSize, file__MapSensorPlacements, file__CellSize, file__RobotOrigin;
	    
        
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
			    ROS_INFO("MapSize: this_size %d",this_size);
         	}
		    file__MapSize.close();
	    }
	    else std::cout << "Unable to open map size file";
        
        	    
	    //==============================
	    //--- map sensor placements
	    //==============================
	    ROS_INFO("Sensor placements map... ");
	    file__MapSensorPlacements.open((FilePath+filename__MapSensorPlacements).c_str(), std::ios::app);
	    //ROS_INFO("Map file: %s",(FilePath+filename__MapSensorPlacements).c_str());
	    int this_map;
	    vec_MapSensorPlacements.clear();
	    if (file__MapSensorPlacements.is_open()){		    
		    while(file__MapSensorPlacements >> this_map){
			    vec_MapSensorPlacements.push_back(this_map);
         	}
		    file__MapSensorPlacements.close();
		    ROS_INFO("MapSensorPlacement: size %zd",vec_MapSensorPlacements.size());
	    }
	    else std::cout << "Unable to open planning map file";
	    
	    
	    //==============================
	    //--- Read Cell Size
	    //==============================
	    ROS_INFO("Cell size... ");
	    file__CellSize.open((FilePath+filename__CellSize).c_str(), std::ios::app);
	    if (file__CellSize.is_open()){
         	file__CellSize >> cell_size;
		    file__CellSize.close();
		    ROS_INFO("Cell size: %f",cell_size);
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
	    printf("\n=	            Environment Map Publisher Node                     ");
	    printf("\n=================================================================\n");        
        
        ros::init(argc, argv, "sensor_placements_map_publisher_node");
        ros::NodeHandle n;
        
        // ####################### PARAMETERS ########################
	    ros::NodeHandle paramHandle ("~");
        
        //============================================	
	    //----- Map Info Parameters
	    //============================================
	    paramHandle.param<std::string>("file_path",FilePath,ros::package::getPath("plan_execution")+"/logs/");
	    paramHandle.param<std::string>("map_file",filename__MapSensorPlacements,"prismaforum_map_conf.dat");
	    paramHandle.param<std::string>("map_size_file",filename__MapSize,"prismaforum_mapsize_conf.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"prismaforum_origin_conf.dat");
    
        paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
        
        
        //============================================
	    //----- ROS Topic Publishers
        //============================================
        //ros::Publisher map_marker = n.advertise<visualization_msgs::Marker>("placement_map", 10);
        ros::Publisher map_marker = n.advertise<visualization_msgs::Marker>("env_map", 10);
        
        ros::Rate r(30);
        
        
        //--- read map info
        readEnvironmentMap();
        
        
        //---------------------------------
        // VISUALIZATION MARKERS
        //---------------------------------
        
        
        //--- Marker initialization
        //--------------------------------        
        map_points.header.frame_id = "/map";        
        map_points.header.stamp = ros::Time::now();
        map_points.ns = "env_map_publisher_node";
        map_points.action = visualization_msgs::Marker::ADD;
        map_points.pose.orientation.w = 1.0;   
        map_points.id = 0;      
        map_points.type = visualization_msgs::Marker::POINTS;
        map_points.scale.x = 0.45; //cell_size; //0.2;
        map_points.scale.y = 0.45; //cell_size; //0.2;
        map_points.color.r = 0.0f;
        map_points.color.g = 0.7f;
        map_points.color.b = 0.0f;
        map_points.color.a = 0.25;
        
        
        //--- Marker initialization (unoccupied)
        //--------------------------------
        map_points1.header.frame_id = "/map";        
        map_points1.header.stamp = ros::Time::now();
        map_points1.ns = "env_map_publisher_node";
        map_points1.action = visualization_msgs::Marker::ADD;
        map_points1.pose.orientation.w = 1.0;   
        map_points1.id = 1;      
        map_points1.type = visualization_msgs::Marker::POINTS;
        map_points1.scale.x = 0.45; //cell_size; //0.2;
        map_points1.scale.y = 0.45; //cell_size; //0.2;
        map_points1.color.r = 0.0f;
        map_points1.color.g = 0.7f;
        map_points1.color.b = 0.0f;
        map_points1.color.a = 0.50; //0.25
        
        
        //--- Marker initialization (occupied)
        //--------------------------------        
        map_points2.header.frame_id = "/map";        
        map_points2.header.stamp = ros::Time::now();
        map_points2.ns = "sensor_placements_map_publisher_node";
        map_points2.action = visualization_msgs::Marker::ADD;
        map_points2.pose.orientation.w = 1.0;   
        map_points2.id = 2;      
        map_points2.type = visualization_msgs::Marker::POINTS;
        map_points2.scale.x = 0.45; //cell_size; //0.2;
        map_points2.scale.y = 0.45; //cell_size; //0.2;
        map_points2.color.r = 0.7f;
        map_points2.color.g = 0.0f;
        map_points2.color.b = 0.0f;
        map_points2.color.a = 0.50; //0.75
        
        
        //ROS_INFO("Fill cells....");
        
        //--- Unoccupied cells
        //--------------------------------
        //ROS_INFO("map size is %d,%d",vec_MapSize[0],vec_MapSize[1]);        
        for (int i = 0; i < vec_MapSize[0]; ++i){        
            for (int j = 0; j < vec_MapSize[1]; ++j){
            
                //float x = ((i-vec_RobotOrigin[0])*cell_size)+hard_offset_x;
                //float y = ((j-vec_RobotOrigin[1])*cell_size)+hard_offset_y;                                        
                float x = ((i-vec_RobotOrigin[0]+1)*cell_size)+hard_offset_x;
                float y = ((j-vec_RobotOrigin[1]+1)*cell_size)+hard_offset_y;                                    
                float z = 0.5; //0.1
                
                //ROS_INFO("origin are %f,%f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
                //ROS_INFO("i,j are %d,%d",i,j);
                //ROS_INFO("x,y are %f,%f",x,y);
                                          
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                    
                if (vec_MapSensorPlacements[(i*vec_MapSize[1])+j] == 1){                    
                    //map_points.points.push_back(p);                    
                    map_points1.points.push_back(p);
                    
                    //ros::WallDuration(0.1).sleep();
                }
                else{
                    map_points2.points.push_back(p);
                }
            }
        }
            
        ROS_INFO("The map is being published...");
        
        while(ros::ok()){
        
            //-- publish 
            //------------------
            //map_marker.publish(map_points);            
            map_marker.publish(map_points1);
            map_marker.publish(map_points2);
            
            //ROS_INFO("Spinning");
            ros::spinOnce();
            r.sleep();            
        }
}

