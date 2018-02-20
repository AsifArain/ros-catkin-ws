/*

              HUMAN-EXPERT EXPLORATION FOR GAS EMISSION MONITORING
              ____________________________________________________


    This is main file to execute sensing configurations planned by a human 
    expert.
    
    -------------------------------------------------------------------------
    Author:  Asif Arain
    Date:    27-Oct-2017
    Version: 0.0
    -------------------------------------------------------------------------


*/

//#include "common_prelude.h"
//#include "common_functions.h"


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <fstream>
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
#include "roscpp_tutorials/TwoInts.h"
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

//================================================================================
//              GAS SAMPLING
//================================================================================
void perform____GasSampling(){

	    ros::NodeHandle nSampling;      	
      	ros::ServiceClient client1 = nSampling.serviceClient<ptu_control2::commandSweep>(srvPTUSweepCommand);
      	ros::ServiceClient client2 = nSampling.serviceClient<amtec::GetStatus>(srvPTUJointStatus);
      
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
	    
	    
	    ROS_INFO("human parameters for Service request:%f,%f,%f,%f,%d,%d,%f",\
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
	    
	    ROS_INFO("human Num of pan sweeps: %d,%d",num_pan_sweeps,srvSweep.request.n_pan);
	    
	    ROS_INFO("human Service request:%f,%f,%f,%f,%d,%d,%f",\
	             srvSweep.request.min_pan,\
	             srvSweep.request.max_pan,\
	             srvSweep.request.min_tilt, \
  	             srvSweep.request.max_tilt, \
	             srvSweep.request.n_pan,\
	             srvSweep.request.n_tilt,\
	             srvSweep.request.samp_delay);
	
	    //client1.call(srvSweep);
	    /*
	    if (client1.call(srvSweep)){
		    ROS_INFO("human Gas measurements are progress ... <%.2f~%.2f,%.2f>",-(FoV/2),(FoV/2),tilt_angle);
	    }
	    else{
		    ROS_ERROR("Failed to initialize gas scanning.");
	    }
	    */
	    
	    
	    if(client2.call(srvState)){     
		    ROS_INFO("Got status");
		    ROS_INFO("position_pan:  %f", (float)srvState.response.position_pan);
		    ROS_INFO("position_tilt: %f", (float)srvState.response.position_tilt);
		    ROS_INFO("velocity_pan:  %f", (float)srvState.response.velocity_pan);
		    ROS_INFO("velocity_tilt: %f", (float)srvState.response.velocity_tilt);
		    
	    }
	    else{
		    ROS_ERROR("Failed to get status.");
        } 
		
}




//================================================================================
//                      MAIN
//================================================================================

int main(int argc, char** argv){
        ros::init(argc, argv, "ptu_sweep_request_node");
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
        
        //ROS_INFO("xx paramHandle min_pan_angle %f",min_pan_angle);
	    //ROS_INFO("xx paramHandle max_pan_angle %f",max_pan_angle);
	    ROS_INFO("xx paramHandle sensing_range %f",sensing_range);
	    ROS_INFO("xx paramHandle FoV %f",FoV);
	    ROS_INFO("xx paramHandle num_pan_sweeps %d",num_pan_sweeps);
	    ROS_INFO("xx paramHandle num_tilt_sweeps %d",num_tilt_sweeps);
	    ROS_INFO("xx paramHandle sample_delay %f",sample_delay);
	    
        
	        
	    
	    
	    //============================================	
	    //----- ROS Service Names
	    //============================================
	    paramHandle.param<std::string>("ptu_sweep_command_srv", srvPTUSweepCommand, DEFAULT_SERVICE_PTU_SWEEP_COMMAND);
	    paramHandle.param<std::string>("ptu_joint_angles_srv",  srvPTUJointStatus,  DEFAULT_SERVICE_PTU_JOINT_STATUS);
	    
        
        
        
        ros::Rate loop_rate(10);

        perform____GasSampling();
        ros::spinOnce();
        loop_rate.sleep();
        
        return 0;
}



