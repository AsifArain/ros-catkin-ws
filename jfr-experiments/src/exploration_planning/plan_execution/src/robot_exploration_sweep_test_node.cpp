/*

               ROBOT EXPLORATION FOR GAS EMISSION MONITORING
               _____________________________________________


    This is main file to execute planned sensing configurations using either 
    Active (one-step) Exploration strategy or Planned-Adoptive (two-step)   
    Exploration strategy.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    27-Oct-2017
    Version: 0.0
    -------------------------------------------------------------------------


*/

//#include "common_prelude.h"
//#include "robot_prelude.h"
//#include "common_functions.h"


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <fstream>
#include <boost/thread.hpp>
#include "ptu_control2/commandSweep.h"
#include "amtec/GetStatus.h"
#include "std_msgs/Int16.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <math.h>       /* atan */

//#include "gasbot_planning/ExecutedConfNum.h"
//#include "gasbot_planning/AddTwoInts.h"
#include "roscpp_tutorials/TwoInts.h"
#include <sys/stat.h> // mkdir
#include "sensor_msgs/JointState.h" // ptu joint call back
#include <nav_msgs/Odometry.h> // localization
#include <rmld_node/rmld_msg.h> // ppm data
//#include <string> // num to string

#include <std_msgs/String.h>
//#include <geometry_msgs/PoseStamped.h>



//----- Gas Sampling Parameters
//============================================
    #define	DEFAULT_SENSING_RANGE       15.0
    #define	DEFAULT_OFFSETY_BASE_RMLD   0.904
    #define	DEFAULT_FOV                 180.0
    #define	DEFAULT_NUM_PAN_SWEEPS      1
    #define	DEFAULT_NUM_TILT_SWEEPS     1
    #define	DEFAULT_SAMPLE_DELAY        0.1
	
//----- ROS Service Names
//============================================
    #define	DEFAULT_SERVICE_PTU_SWEEP_COMMAND  "/ptu_control/sweep"
    #define	DEFAULT_SERVICE_PTU_JOINT_STATUS   "/amtec/get_status"
    
    
using namespace std;
    

//--- PTU/Sweeping Variables
//==========================
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle, sample_delay, tilt_angle;
int    num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
//vector<double> vecJointState;
//double anglePan,angleTilt;


//--- ROS-Service Variables
//==========================
std::string srvPTUSweepCommand, srvPTUJointStatus;

//--- File Variables
//==========================
/*
double cell_size;
std::vector<double> vec_RobotOrigin;
std::vector<int> vec_MapSize,\
                 vec_MapSensorPlacements,\
                 vec_Confs;
                 
std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements, \
            filename__Confs;

double hard_offset_x, hard_offset_y;
*/





//================================================================================
//              GAS SAMPLING
//================================================================================
void perform____GasSampling(){

	    ros::NodeHandle nSampling;      	
      	ros::ServiceClient client1 = nSampling.serviceClient<ptu_control2::commandSweep>(srvPTUSweepCommand);
      	ros::ServiceClient client2 = nSampling.serviceClient<amtec::GetStatus>(srvPTUJointStatus);
        
        ROS_INFO("srvPTUSweepCommand: %s",srvPTUSweepCommand.c_str());
        
	    ptu_control2::commandSweep srvSweep;
	    amtec::GetStatus  srvState;
	    //rosservice call /ptu_control/sweep "{min_pan: -45.0, max_pan: 45.0, min_tilt: -10.0, max_tilt: -10.0, n_pan: 1, n_tilt: 1, samp_delay: 0.1}"

	    // Finding Tilt Angle:
	    //--------------------------
	    /*
	    * 	      | phi
	    *  0.904m |      
	    *         |         
	    *         |--            
	    *         |_|_____________________________theta
	    * 		    sensing range
	    *  phi   = atan(sensing range / 0.904)
	    *  theta = atan(0.904 / sensing range)
	    *  tilt_angle = 90-phi
	    */
	
	    tilt_angle = (atan(sensing_range/offsetY_base_rmld)*(180/M_PI))-90;
	    
	    
	    ROS_INFO("parameters for Service request:%f,%f,%f,%f,%d,%d,%f",\
	             -(FoV/2),\
	             (FoV/2),\
	             tilt_angle, \
  	             tilt_angle, \
	             num_pan_sweeps,\
	             num_tilt_sweeps,\
	             sample_delay);
	             

	    srvSweep.request.min_pan  	= -(FoV/2);        //min_pan_angle;
	    srvSweep.request.max_pan  	=  (FoV/2);        //max_pan_angle;
	    srvSweep.request.min_tilt 	= tilt_angle; 	   //min_tilt_angle;
	    srvSweep.request.max_tilt 	= tilt_angle;      //max_tilt_angle;
	    srvSweep.request.n_pan    	= num_pan_sweeps;  //  1;
	    srvSweep.request.n_tilt   	= num_tilt_sweeps; //  1;
	    srvSweep.request.samp_delay = sample_delay;    //0.1;
	    
	    ROS_INFO("Num of pan sweeps: %d,%d",num_pan_sweeps,srvSweep.request.n_pan);
	    
	    ROS_INFO("Service request:%f,%f,%f,%f,%d,%d,%f",\
	             srvSweep.request.min_pan,\
	             srvSweep.request.max_pan,\
	             srvSweep.request.min_tilt, \
  	             srvSweep.request.max_tilt, \
	             srvSweep.request.n_pan,\
	             srvSweep.request.n_tilt,\
	             srvSweep.request.samp_delay);
	
	    ROS_INFO("Im here.... aacha");
	    //client1.call(srvSweep);
	    //ROS_INFO("dosri line...");
	    
	    
	    if (client1.call(srvSweep)){
		    ROS_INFO("Gas measurements are progress ... <%.2f~%.2f,%.2f>",-(FoV/2),(FoV/2),tilt_angle);
	    }
	    else{
		    ROS_ERROR("Failed to initialize gas scanning.");
	    }
	    /*
	    if(client2.call(srvState)){     
		    ROS_INFO("Got status");
		    ROS_INFO("Sum: %f", (float)srvState.response.position_pan);}
	    else{
		    ROS_ERROR("Failed to get status.");
	    } 
		*/
}


//================================================================================
//              EXECUTE PLANNED CONFIGURATIONS 
//================================================================================
void execute____PlannedConfigurations(){
	
	
	perform____GasSampling();
	//int conf_num = 0;
	
	
}



//================================================================================
//                      MAIN
//================================================================================


int main(int argc, char** argv){

        printf("\n=================================================================");
	    printf("\n=	            Robot Exploration Node                             ");
	    printf("\n=================================================================\n");   

        ros::init(argc, argv, "robot_plan_execution");
        ros::NodeHandle n;

	    // ####################### PARAMETERS ########################
	    ros::NodeHandle paramHandle ("~");

	    //============================================
	    //----- Gas Sampling Parameters
	    //============================================
	    //paramHandle.param("min_pan_angle",    min_pan_angle,    DEFAULT_MIN_PAN_ANGLE);
	    //paramHandle.param("max_pan_angle",    max_pan_angle,    DEFAULT_MAX_PAN_ANGLE);
	    paramHandle.param("sensing_range",    sensing_range,    DEFAULT_SENSING_RANGE);
	    paramHandle.param("offsetY_base_rmld",offsetY_base_rmld,DEFAULT_OFFSETY_BASE_RMLD);
	    paramHandle.param("field_of_view",    FoV,              DEFAULT_FOV);
	    paramHandle.param("num_pan_sweeps",   num_pan_sweeps,   DEFAULT_NUM_PAN_SWEEPS);
	    paramHandle.param("num_tilt_sweeps",  num_tilt_sweeps,  DEFAULT_NUM_TILT_SWEEPS);
	    paramHandle.param("sample_delay",     sample_delay,     DEFAULT_SAMPLE_DELAY);
	    
	    ROS_INFO("xx paramHandle sensing_range %f",sensing_range);
	    ROS_INFO("xx paramHandle FoV %f",FoV);
	    ROS_INFO("xx paramHandle num_pan_sweeps %d",num_pan_sweeps);
	    ROS_INFO("xx paramHandle num_tilt_sweeps %d",num_tilt_sweeps);
	    ROS_INFO("xx paramHandle sample_delay %f",sample_delay);
	    
	    /*
    	//============================================
	    //----- File contains poses or plan
	    //============================================
	    paramHandle.param<std::string>("file_plan",plan_FileName,DEFAULT_PLAN_FILENAME);
	    //paramHandle.param<std::string>("FilePath",FilePath,ros::package::getPath("plan_execution"));	

	    //============================================	
	    //----- Navignation Parameters
	    //============================================
	    paramHandle.param<std::string>("plan_frame_name",plan_frame_name,DEFAULT_PLAN_FRAME_NAME);
        
        
        //============================================
        //----- Historian (default) Parameters -----
        //============================================
	    // Gas Measurements
	    paramHandle.param("gas_log_threshold",GasMeasureThreshold,DEFAULT_GAS_MEASURE_THRESHOLD);
	    paramHandle.param("ptu_operation_status",OperationStatus,DEFAULT_OPERATION_STATUS);
	    // Log File
	    //paramHandle.param<std::string>("path",log_FileName,ros::package::getPath("gasbot_historian")+"/History/");
	    paramHandle.param<std::string>("experiment_title",experimentTitle,DEFAULT_EXPERIMENT_TITLE);
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,DEFAULT_EXPLORATION_STRATEGY);

	    */
	    	    
	    //============================================	
	    //----- Map Info Parameters
	    //============================================
	    /*
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    //paramHandle.param<std::string>("map_file",filename__MapSensorPlacements,"prismaforum_map_conf.dat");
	    //paramHandle.param<std::string>("map_size_file",filename__MapSize,"prismaforum_mapsize_conf.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"prismaforum_origin_conf.dat");
	    //paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
	    */
	    
	    
	    
	    
	    //============================================	
	    //----- ROS Topic Names
	    //============================================
	    /*
	    paramHandle.param<std::string>("localization_topic",topicLocalization,   DEFAULT_TOPIC_LOCALIZATION);
	    paramHandle.param<std::string>("ptu_sweep_topic",   topicPTUSweepStatus, DEFAULT_TOPIC_PTU_SWEEP_STATUS);
	    paramHandle.param<std::string>("ptu_joint_topic",   topicPTUJointStatus, DEFAULT_TOPIC_PTU_JOINT_STATUS);
	    paramHandle.param<std::string>("rmld_topic",        topicRMLDReadings,   DEFAULT_TOPIC_RMLD_READINGS);
	    */
	    //paramHandle.param<std::string>("current_goal_topic",topicCurrentGoal,    DEFAULT_TOPIC_CURRENT_GOAL);
	    
	    
	    //============================================	
	    //----- ROS Service Names
	    //============================================
	    paramHandle.param<std::string>("ptu_sweep_command_srv", srvPTUSweepCommand, DEFAULT_SERVICE_PTU_SWEEP_COMMAND);
	    paramHandle.param<std::string>("ptu_joint_angles_srv",  srvPTUJointStatus,  DEFAULT_SERVICE_PTU_JOINT_STATUS);
	    
        
        
        //============================================
	    //----- ROS Topic Subscriptions
	    //============================================
	    //-- PTU STATUS
        //ros::Subscriber sub1 = n.subscribe(topicPTUSweepStatus, 1000,callback___PTUSweepingStatus); 
        //-- Localization
        //ros::Subscriber sub2 = n.subscribe(topicLocalization,   1000,callback___Localization);
        //-- RMLD
        //ros::Subscriber sub3 = n.subscribe(topicRMLDReadings,   1000,callback___RMLD);	
    	//-- PTU JOINT
    	//ros::Subscriber sub4 = n.subscribe(topicPTUJointStatus, 1000,callback___PTUJointAngles);
        //-- CURRENT GOAL
    	//ros::Subscriber sub6 = n.subscribe(topicCurrentGoal,  1000,callbackCurrentGoal);
	    
	    
  
        //============================================
        //----- ROS Topic Publishers
        //============================================
        //ros::Publisher  pub_conf = n.advertise<std_msgs::Int16>("/plan_execution/conf_num", 1000);

        //============================================
        //----- ROS Services
        //============================================
        //ros::ServiceServer srv_conn = n.advertiseService("executed_conf_num",service____ExecutedConfNumber);

        //boost::thread mythread(execute____PlannedConfigurations);
        //ros::spin();
        
        
        ros::Rate loop_rate(10);

        perform____GasSampling();
        ros::spinOnce();
        loop_rate.sleep();
        
               
        
        
        
        return 0;
}



