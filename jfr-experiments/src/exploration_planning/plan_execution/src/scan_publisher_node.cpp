/*

                            SCAN PUBLISHER NODE
              ____________________________________________________


    This is main file to publish field of view at RVIZ.

    -------------------------------------------------------------------------
        Author:  Asif Arain
        Date:    06-Nov-2017
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
#include "ptu_control2/commandSweep.h"

#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include <std_msgs/Int64.h>

//#include "plan_execution/common_prelude.h"
//#include "ptu_control2/commandSweep.h"
//#include "amtec/GetStatus.h"

#include "sensor_msgs/JointState.h" // ptu joint call back

#include <boost/math/constants/constants.hpp>
const double PI = boost::math::constants::pi<double>();
#include <geometry_msgs/PoseArray.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

using namespace std;

//--- Pose/localization Variables
//==========================
double posX,posY,posW;


//--- ROS-topic Variables
//==========================
std::string topicLocalization, \
            topicPTUSweepStatus, \
            topicPTUJointStatus, \
            topicRMLDReadings, \
            topicCurrentGoal, \
            topicRVIZGoal;

//--- PTU/Sweeping Variables
//==========================
double min_pan_angle, \
       max_pan_angle, \
       min_tilt_angle, \
       max_tilt_angle, \
       sample_delay, \
       tilt_angle;
int    statusPTU, num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
vector<double> vecJointState;
double anglePan,angleTilt;

//--- Visualization Markers
//==========================
visualization_msgs::Marker conf_points, \
                           conf_spheres, \
                           fov_strip, \
                           beam_line, \
                           path_strip;


geometry_msgs::PoseArray conf_orns;


void callback___Localization();
void publish____ScanArea();
void callback___PTUSweepingStatus();
void callback___PTUJointAngles();

geometry_msgs::PoseStamped currentPose;

double travelingPointsDist;

//--- Log File Variables
//==========================
ofstream fileTravelingPath,fileExecutedConfs;
string FilePath, filenameTravelingPath, filenameExecutedConfs;
string experimentTitle;
string explorationStrategy;

//visualization_msgs::Marker scan_marker;

#define	DEFAULT_TOPIC_LOCALIZATION  "/ndt_mcl"
//#define	PI 3.14159265
#define	DEFAULT_SENSING_RANGE       15.0
#define	DEFAULT_FOV                 90.0

#define	DEFAULT_TOPIC_PTU_SWEEP_STATUS  "/ptu_control/state"
#define	DEFAULT_TOPIC_PTU_JOINT_STATUS  "/amtec/joint_states"

#define	DEFAULT_TRAVELING_POINTS_DISTANCE  0.5

//================================================================================
//              CALLBACK: LOCALIZATION
//================================================================================
void callback___Localization(const nav_msgs::Odometry::ConstPtr& enco){

	//-- Position
	posX = enco->pose.pose.position.x;
	posY = enco->pose.pose.position.y;

	//-- Orientation
	tf::Quaternion q(enco->pose.pose.orientation.x, enco->pose.pose.orientation.y, \
	enco->pose.pose.orientation.z, enco->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	posW = (yaw)*180/M_PI;

	//-- Print info
	//ROS_INFO("position (X,Y,W) is... %.2f,%.2f,%.2f",posX,posY,posW);


	currentPose.pose.position = enco->pose.pose.position;
	currentPose.pose.orientation = enco->pose.pose.orientation;

}


//================================================================================
//              CALLBACK: PTU SWEEPING STATUS
//================================================================================
void callback___PTUSweepingStatus(const std_msgs::Int16::ConstPtr& sta){
	//ROS_INFO("PTU status at callback is...%d",sta->data);
	statusPTU=sta->data;
}


//================================================================================
//              CALLBACK: PTU JOINT ANGLES
//================================================================================
void callback___PTUJointAngles(const sensor_msgs::JointState::ConstPtr& jsta){

	/*
	vecJointState  = jsta->position;
	angleTilt = vecJointState[0];
	anglePan  = vecJointState[1];
	*/
	angleTilt = jsta->position[0];
	anglePan  = jsta->position[1];

	//-- Print info
	//ROS_INFO("Pan Angle %.2f, Tilt Angle %.2f",anglePan,angleTilt);

}


//================================================================================
//              CALLBACK: PTU JOINT ANGLES
//================================================================================
double fEuclideanDist(double x1, double y1, double x2, double y2){

	double x = x1 - x2; //calculating number to square in next step
	double y = y1 - y2;
	double dist;

	dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
	dist = sqrt(dist);

	return dist;
}



//================================================================================
//              READ ENVIRONMENT MAP
//================================================================================
void readExecutedPlanFromFile(){

        ROS_INFO("Reading executed plan from a file... ");
        
	    std::ifstream fileTravelingPath,fileExecutedConfs;
	    double x,y,t;
	    
        //==============================
        //  traveling path
	    //==============================	    
	    fileTravelingPath.open((FilePath+filenameTravelingPath).c_str(),std::ios::app);
	    //double x,y;
	    if (fileTravelingPath.is_open()){
		    while(fileTravelingPath >> x >> y){
		        geometry_msgs::Point pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.5; 
                path_strip.points.push_back(pt);
                ROS_INFO("(from file) Pt <%f,%f> added to traveling path.",pt.x,pt.y);
         	}
		    fileTravelingPath.close();
	    }
	    else ROS_INFO("Unable to open path file");
        
        
        //==============================
        //  configurations
	    //==============================	    
	    fileExecutedConfs.open((FilePath+filenameExecutedConfs).c_str(),std::ios::app);
	    //double x,y,t;
	    if (fileExecutedConfs.is_open()){
		    while(fileExecutedConfs >> x >> y >> t){
		        
		        //-- position
                geometry_msgs::Point pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.5;
                conf_points.points.push_back(pt);
                conf_spheres.points.push_back(pt);
                ROS_INFO("(from file) Pt <%f,%f> added to conf points.",pt.x,pt.y);

                //-- pose 
                geometry_msgs::PoseStamped ps;
                ps.pose.position.x = x;
                ps.pose.position.y = y;
                ps.pose.position.z = 0.5;
                ps.pose.orientation = tf::createQuaternionMsgFromYaw(t*M_PI/180); //currentPose.pose.orientation; //
                conf_orns.poses.push_back(ps.pose);
                ROS_INFO("(from file) Pt <%f,%f,%f> added to conf pose.",ps.pose.position.x,ps.pose.position.y,t);
         	}
		    fileExecutedConfs.close();
	    }
	    else ROS_INFO("Unable to open path file");
        	    
	    
	    	    
}
    


//================================================================================
//              WRITING TRAVELING PATH POINTS TO A FILE
//================================================================================
void writeTravelingPathFile(double x, double y){
    
    ROS_INFO("In path writing function....");
    
    //===== Create File    
    fileTravelingPath.open((FilePath+filenameTravelingPath).c_str(),std::ios::app);
    
	if (fileTravelingPath){
        
        //-- write data
        //fileTravelingPath<<x<<" "<<y<<endl;
        fileTravelingPath << std::fixed << std::setprecision(4) << x << " " << y << endl;
        //-- Print info
		ROS_INFO("Point <%.2f,%.2f> is written to traveling path file",x,y);
	}
	else ROS_INFO("Unable to open path file");
	
	fileTravelingPath.close();
}


//================================================================================
//              WRITING EXECUTED CONFIGURATIONS TO A FILE
//================================================================================
void writeExecutedConfsFile(double x,double y,double t){
    
    ROS_INFO("In conf writing function....");
        
    //===== Create File
    fileExecutedConfs.open((FilePath+filenameExecutedConfs).c_str(),std::ios::app);
    
	if (fileExecutedConfs){
        
        //-- write data
        fileExecutedConfs << std::fixed << std::setprecision(4) << x << " " << y << " " << t << endl;
        //-- Print info
		ROS_INFO("Point <%.2f,%.2f,%.2f> is written to conf file",x,y,t);
	}
	else ROS_INFO("Unable to open conf file");
	
	fileExecutedConfs.close();
}


//================================================================================
//              PUBLISH SCAN AREA
//================================================================================
void publish____ScanArea(){


        //--- clear variables
        fov_strip.points.clear();
        beam_line.points.clear();


        if(statusPTU!=0){


            //--- Field of view
            //=========================================

            //-- Point at the origin
            geometry_msgs::Point p0;
            p0.x = posX+0.0;
            p0.y = posY+0.0;
            p0.z = 0.904;
            fov_strip.points.push_back(p0);

            //-- Circumference points
            for (float i = posW-(FoV/2.0); i < posW+(FoV/2.0); ++i){

                float x = posX + sensing_range * cos(i * PI/180);
                float y = posY + sensing_range * sin(i * PI/180);
                float z = 0.1;
                geometry_msgs::Point p1;
                p1.x = x;
                p1.y = y;
                p1.z = z;
                fov_strip.points.push_back(p1);
            }


            //-- Back to the origin
            fov_strip.points.push_back(p0);

            //--- Optical Beam
            //=========================================
            //ROS_INFO("Beam angle: %f",((posW*PI/180)+anglePan)*180/PI);

            //-- initial point
            float xi = posX + 0.21 * cos((posW*PI/180)+anglePan);
            float yi = posY + 0.21 * sin((posW*PI/180)+anglePan);
            float zi = 0.904;
            geometry_msgs::Point pi;
            pi.x = xi;
            pi.y = yi;
            pi.z = zi;

            //-- end point
            float xe = posX + sensing_range * cos((posW*PI/180)+anglePan);
            float ye = posY + sensing_range * sin((posW*PI/180)+anglePan);
            float ze = 0.1;
            geometry_msgs::Point pe;
            pe.x = xe;
            pe.y = ye;
            pe.z = ze;

            //-- beam line
            beam_line.points.push_back(pi);
            beam_line.points.push_back(pe);

            
            //--- Traveling path
            //=========================================
            
            //-- check that this point is not already in the list
            //if ( path_strip.points[path_strip.points.size()-1].x != posX &&\
            //     path_strip.points[path_strip.points.size()-1].y != posY){
            
            //ROS_INFO("Traveling path loop...");
            
            if( path_strip.points.empty() ){
            
                //-- this origin
                geometry_msgs::Point po;
                po.x = posX;
                po.y = posY;
                po.z = 0.5; //0.1;
                //conf_points.points.push_back(po);
                //conf_spheres.points.push_back(po);
                path_strip.points.push_back(po);
                
                writeTravelingPathFile(po.x,po.y);
                
                ROS_INFO("(if) Pt <%f,%f> added to traveling path.",po.x,po.y);
                
            }                
            else{
            
                if ( path_strip.points.back().x != posX &&\
                     path_strip.points.back().y != posY){

                    
                    //-- this origin
                    geometry_msgs::Point po;
                    po.x = posX;
                    po.y = posY;
                    po.z = 0.5; //0.1;
                    //conf_points.points.push_back(po);
                    //conf_spheres.points.push_back(po);
                    path_strip.points.push_back(po);
                    
                    writeTravelingPathFile(po.x,po.y);
                    
                    ROS_INFO("(else) Pt <%f,%f> added to traveling path.",po.x,po.y);

                 }
             }

            //--- Configurations
            //=========================================

            //-- check that this conf is not already in the list
            
            //if ( conf_spheres.points[conf_spheres.points.size()-1].x != posX &&\
            //     conf_spheres.points[conf_spheres.points.size()-1].y != posY){

            //ROS_INFO("Conf loop...");
            
            if( conf_spheres.points.empty() ){
                
                //-- position
                geometry_msgs::Point pt;
                pt.x = posX;
                pt.y = posY;
                pt.z = 0.5;
                conf_points.points.push_back(pt);
                conf_spheres.points.push_back(pt);                

                //-- pose
                geometry_msgs::PoseStamped ps;
                ps.pose.position.x = posX;
                ps.pose.position.y = posY;
                ps.pose.position.z = 0.5;
                ps.pose.orientation = currentPose.pose.orientation;
                conf_orns.poses.push_back(ps.pose);
                                
                //-- quaternion to degree
	            tf::Quaternion q(currentPose.pose.orientation.x, currentPose.pose.orientation.y, \
	            currentPose.pose.orientation.z, currentPose.pose.orientation.w);
	            tf::Matrix3x3 m(q);
	            double roll,pitch,yaw;
	            m.getRPY(roll,pitch,yaw);
	            double t = (yaw)*180/M_PI;
	            
	            writeExecutedConfsFile(ps.pose.position.x,ps.pose.position.y,t);
                ROS_INFO("(if) Pose <%f,%f,%f> added to conf.",ps.pose.position.x,ps.pose.position.y,t);
            }
            else
            {                
                //ROS_INFO("Last conf <%f,%f>",conf_spheres.points.back().x,conf_spheres.points.back().y);
                
                if ( conf_spheres.points.back().x != posX &&\
                     conf_spheres.points.back().y != posY){

                    //-- position
                    geometry_msgs::Point pt;
                    pt.x = posX;
                    pt.y = posY;
                    pt.z = 0.5;
                    conf_points.points.push_back(pt);
                    conf_spheres.points.push_back(pt);                

                    //-- pose
                    geometry_msgs::PoseStamped ps;
                    ps.pose.position.x = posX;
                    ps.pose.position.y = posY;
                    ps.pose.position.z = 0.5;
                    ps.pose.orientation = currentPose.pose.orientation;
                    conf_orns.poses.push_back(ps.pose);
                                    
                    //-- quaternion to degree
                    tf::Quaternion q(currentPose.pose.orientation.x, currentPose.pose.orientation.y, \
                    currentPose.pose.orientation.z, currentPose.pose.orientation.w);
                    tf::Matrix3x3 m(q);
                    double roll,pitch,yaw;
                    m.getRPY(roll,pitch,yaw);
                    double t = (yaw)*180/M_PI;
                    
                    writeExecutedConfsFile(ps.pose.position.x,ps.pose.position.y,t);
                    ROS_INFO("(else) Pose <%f,%f,%f> added to conf.",ps.pose.position.x,ps.pose.position.y,t);
                }
            }


            /*
            int check_flage = 1;
            for (int i = 0; i < conf_points.points.size(); ++i){
                if (conf_points.points[i].x == posX && \
                    conf_points.points[i].y == posY){
                    check_flage = 0;
                    break;
                }
            }

            //-- if not, add to the list
            if (check_flage == 1){

                //-- this origin
                geometry_msgs::Point po;
                po.x = posX;
                po.y = posY;
                po.z = 0.5; //0.1;
                conf_points.points.push_back(po);
                conf_spheres.points.push_back(po);
                path_strip.points.push_back(po);


                //-- conf orientation
                geometry_msgs::PoseStamped pos;
                pos.pose.position.x = posX;
                pos.pose.position.y = posY;
                pos.pose.position.z = 0.5; //0.1;
                pos.pose.orientation = currentPose.pose.orientation; //tf::createQuaternionMsgFromYaw(o*M_PI/180);
                conf_orns.poses.push_back(pos.pose);

            }
            */            
        }
        
                
        
        
        //-- Traveling path (points between two confs)
        //-----------------------------------------------
        
        if( path_strip.points.empty() ){
            
            ROS_INFO("Path vector is empty....");
            
            //-- this point
            geometry_msgs::Point p;
            p.x = posX;
            p.y = posY;
            p.z = 0.5; //0.1;
            //conf_points.points.push_back(po);
            //conf_spheres.points.push_back(po);
            path_strip.points.push_back(p);
            
            writeTravelingPathFile(p.x,p.y);
            
            ROS_INFO("(if) Pt <%f,%f> added to traveling path.",p.x,p.y);
        }
        else
        {                        
            double lastX = path_strip.points.back().x;
            double lastY = path_strip.points.back().y;
            double thisX = posX;
            double thisY = posY;
            
            double dist = fEuclideanDist(lastX,lastY,thisX,thisY);
            
            ROS_INFO("Last pt: <%f,%f>, this pt: <%f,%f>, dist: %f",lastX,lastY,thisX,thisY,dist);
            
            if ( dist >= travelingPointsDist ){
                
                //-- this point
                geometry_msgs::Point p;
                p.x = posX;
                p.y = posY;
                p.z = 0.5; //0.1;
                //conf_points.points.push_back(po);
                //conf_spheres.points.push_back(po);
                path_strip.points.push_back(p);
                
                writeTravelingPathFile(p.x,p.y);
                
                ROS_INFO("(else) Pt <%f,%f> added to traveling path.",p.x,p.y);
                
            }
        }
        
        //-----------------------


}



//================================================================================
//                      MAIN
//================================================================================
int main( int argc, char** argv ){


        printf("\n=================================================================");
	    printf("\n=	             Scan Publisher Node                               ");
	    printf("\n=================================================================\n");

        ros::init(argc, argv, "scan_publisher_node");
        ros::NodeHandle n ("scan_pub");

        // ####################### PARAMETERS ########################
        ros::NodeHandle paramHandle ("~");


        //============================================
	    //----- Parameters
	    //============================================
	    
	    paramHandle.param<std::string>("experiment_title",experimentTitle,"prismaforum5-04");
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,"one-step-exploration");
	    	    	    
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    
	    paramHandle.param<std::string>("traveling_path_file",filenameTravelingPath,"traveling_path.dat");
	    paramHandle.param<std::string>("executed_confs_file",filenameExecutedConfs,"executed_confs.dat");
	    
	    paramHandle.param("sensing_range",sensing_range,DEFAULT_SENSING_RANGE);
	    paramHandle.param("field_of_view",FoV,DEFAULT_FOV);
	    paramHandle.param("traveling_points_distance",travelingPointsDist,DEFAULT_TRAVELING_POINTS_DISTANCE);
	    
	    



        //============================================
        //----- ROS Topic Names
        //============================================
        paramHandle.param<std::string>("localization_topic",topicLocalization,DEFAULT_TOPIC_LOCALIZATION);
        paramHandle.param<std::string>("ptu_sweep_topic",   topicPTUSweepStatus, DEFAULT_TOPIC_PTU_SWEEP_STATUS);
        paramHandle.param<std::string>("ptu_joint_topic",   topicPTUJointStatus, DEFAULT_TOPIC_PTU_JOINT_STATUS);


        //============================================
	    //----- ROS Topic Subscriptions
        //============================================
        //-- PTU STATUS
        ros::Subscriber sub1 = n.subscribe(topicPTUSweepStatus, 1000,callback___PTUSweepingStatus);
        //-- Localization
        ros::Subscriber sub2 = n.subscribe(topicLocalization,   1000,callback___Localization);
        //-- PTU JOINT
    	ros::Subscriber sub3 = n.subscribe(topicPTUJointStatus, 1000,callback___PTUJointAngles);

        //============================================
	    //----- ROS Topic Publishers
        //============================================
        //ros::Publisher scan_marker = n.advertise<visualization_msgs::Marker>("scan_area", 10);
        ros::Publisher sweep_pub = n.advertise<visualization_msgs::Marker>("sweep", 10);
        ros::Publisher pos_pub = n.advertise<visualization_msgs::Marker>("positions", 10);
        ros::Publisher orn_pub = n.advertise<geometry_msgs::PoseArray>("orientations", 10);
        ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path", 10);

        ros::Rate r(30);



        //---------------------------------
        // VISUALIZATION MARKERS
        //---------------------------------


        //--- Marker initialization
        //--------------------------------

        conf_spheres.header.frame_id = \
        path_strip.header.frame_id = \
        conf_points.header.frame_id = \
        fov_strip.header.frame_id = \
        beam_line.header.frame_id = "/map";

        //conf_spheres.header.stamp = \
        path_strip.header.stamp = \
        conf_points.header.stamp = \
        fov_strip.header.stamp = \
        beam_line.header.stamp = ros::Time::now();

        conf_spheres.ns = \
        path_strip.ns = \
        conf_points.ns = \
        fov_strip.ns = \
        beam_line.ns = "scan_publisher_node";

        conf_spheres.action = \
        path_strip.action = \
        conf_points.action = \
        fov_strip.action = \
        beam_line.action = visualization_msgs::Marker::ADD;

        conf_spheres.pose.orientation.w = \
        path_strip.pose.orientation.w = \
        conf_points.pose.orientation.w = \
        fov_strip.pose.orientation.w = \
        beam_line.pose.orientation.w = 1.0;


        //--- Marker IDs
        //--------------------------------

        conf_points.id  = 0;
        fov_strip.id    = 1;
        beam_line.id    = 2;
        path_strip.id   = 3;
        conf_spheres.id = 4;


        //--- Marker types
        //--------------------------------

        conf_spheres.type = visualization_msgs::Marker::SPHERE_LIST;
        conf_points.type  = visualization_msgs::Marker::POINTS;
        fov_strip.type    = visualization_msgs::Marker::LINE_STRIP;
        beam_line.type    = visualization_msgs::Marker::LINE_LIST;
        path_strip.type   = visualization_msgs::Marker::LINE_STRIP;

        //--- Marker width/scale
        //--------------------------------

        // conf sphare list
        conf_spheres.scale.x = 0.50;
        conf_spheres.scale.y = 0.50;
        conf_spheres.scale.z = 0.50;


        // POINTS markers use x and y scale for width/height respectively
        conf_points.scale.x = 0.20;
        conf_points.scale.y = 0.20;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        fov_strip.scale.x  = 0.15;
        beam_line.scale.x  = 0.15;
        path_strip.scale.x = 0.10;


        //--- Marker color and tranparency
        //--------------------------------

        // spheres are dark gray
        conf_spheres.color.r = 0.6902; //0.40;
        conf_spheres.color.g = 0.7686; //0.40;
        conf_spheres.color.b = 0.8706; //0.40;
        conf_spheres.color.a = 1.00;

        // Points are green
        conf_points.color.r = 0.7f;
        conf_points.color.g = 0.7f;
        conf_points.color.b = 0.7f;
        conf_points.color.a = 1.00;


        // Line list is red
        beam_line.color.r = 1.0;
        beam_line.color.a = 1.0;

        // Line strip is blue
        fov_strip.color.b = 1.0;
        fov_strip.color.a = 0.5;

        // path strip is gray
        path_strip.color.r = 0.5;
        path_strip.color.g = 0.5;
        path_strip.color.b = 0.5;
        path_strip.color.a = 0.5;


        //---------------------------------
        // orientation marker
        //---------------------------------
        //conf_orns.poses.clear();
        //conf_orns.header.stamp = ros::Time::now();
        conf_orns.header.frame_id = "/map";
        
        
        //---
        //geometry_msgs::Point p;
        //p.x = posX;
        //p.y = posY;
        //p.z = 0.5; //0.1;        
        //path_strip.points.push_back(p);
        //--------
        
        ros::WallDuration(1).sleep();
        ros::spinOnce();
        r.sleep();
        ROS_INFO("First localization position: <%.2f,%.2f,%.2f>",posX,posY,posW);        
        ros::WallDuration(5).sleep();
        
        //ros::spinOnce();
        //r.sleep();
        //ROS_INFO("Second localization position: <%.2f,%.2f,%.2f>",posX,posY,posW);        
        //ros::WallDuration(5).sleep();
        
        readExecutedPlanFromFile();
        
        
        
        

        while(ros::ok()){



                publish____ScanArea();


                //-- update time stamps
                conf_spheres.header.stamp = \
                path_strip.header.stamp = \
                conf_points.header.stamp = \
                fov_strip.header.stamp = \
                beam_line.header.stamp = \
                conf_orns.header.stamp = ros::Time::now();




                //-- publish
                //------------------
                /*
                scan_marker.publish(fov_strip);
                scan_marker.publish(beam_line);
                //scan_marker.publish(conf_points);
                scan_marker.publish(path_strip);
                scan_marker.publish(conf_spheres);

                orientations_pub.publish(conf_orns);
                */


                sweep_pub.publish(fov_strip);
                sweep_pub.publish(beam_line);

                pos_pub.publish(conf_spheres);
                orn_pub.publish(conf_orns);

                path_pub.publish(path_strip);


                //ROS_INFO("Spinning");
                ros::spinOnce();
		        r.sleep();

	    }
	    
	    if (!ros::ok()){
	        ROS_INFO("GOING DOWN....");
        }
	    

}

