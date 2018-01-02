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

#include "common_prelude.h"
#include "robot_prelude.h"
#include "common_functions.h"





//================================================================================
//              GET POSES FROM EXTERNAL FILE
//================================================================================
void get________PlannedPosesFromFile(){

	double value;
	vecPoses.clear();
	
	std::ifstream filePoses;
	//std::string path = ros::package::getPath("roslib");
	//myFile.open((ros::package::getPath("gasbot_planning")+"/POSES.txt").c_str(),std::ios::app);
	//filePoses.open("/home/husky/ROS_CatkinWS/src/gasbot_planning/POSES.txt",std::ios::app);
	filePoses.open((plan_FilePath+"/"+plan_FileName).c_str(), std::ios::app);
	if (filePoses.is_open()){
		std::cout << "File is open."<<std::endl;
		ROS_INFO("A file '%s' is to be read \n",(plan_FilePath+"/"+plan_FileName).c_str());
		while(filePoses >> value){
			vecPoses.push_back(value);
			std::cout<<"value is "<<value<<std::endl;
 		}
		filePoses.close();
	}
	else std::cout << "Unable to open the file";
}

//================================================================================
//              EXECUTE PLANNED CONFIGURATIONS 
//================================================================================
void execute____PlannedConfigurations(){
	
	//int conf_num = 0;
	
	//-- create log directory
    //------------------------
	create_____LogDirectory();
	
    	
	while(ros::ok()){
    
        //-- obtaining poses
        //---------------------
        ROS_INFO("Obtaining planned poses to execute...");
        get________PlannedPosesFromFile();
        std::cout<<"size of vector "<<vecPoses.size()<<std::endl;
            num_of_conf = vecPoses.size()/4;
        std::cout<<"Number of conf to execute "<<num_of_conf<<std::endl;
            
        if (num_of_conf > conf_num){

            ros::WallDuration(1).sleep();
            //tell the action client that we want to spin a thread by default
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
                
            //-- wait for the action server to come up
            //--------------------------------------------
            while(!ac.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
            }

            for (int i=conf_num;i<vecPoses.size()/4;i++){
                move_base_msgs::MoveBaseGoal goal;
                  
                //we'll send a goal to the robot to move 1 meter forward
                goal.target_pose.header.frame_id = plan_frame_name; //"/world"; //"base_link" //"base_footprint";
                goal.target_pose.header.stamp    = ros::Time::now();

                goal.target_pose.pose.position.x 	= vecPoses[(i*4)+0]; //goalX[i];
                goal.target_pose.pose.position.y 	= vecPoses[(i*4)+1]; //goalY[i];
                //goal.target_pose.pose.orientation.w 	= goalW[i]; //vecGoals[(i*3)+2]; //goalW[i];
                //goal.target_pose.pose.orientation 	= tf::createQuaternionMsgFromYaw(goalO[i]*M_PI/180);
                goal.target_pose.pose.orientation 	= tf::createQuaternionMsgFromYaw(vecPoses[(i*4)+2]*M_PI/180);

                //ROS_INFO("Gasbot is moving to pose %i i.e. <%.1fm,%.1fm,%.1fdeg>.",i+1,goalX[i],goalY[i],goalO[i]); 
                ROS_INFO("Gasbot is moving to Pose# %i <%.1fm,%.1fm,%.1fdeg>.",i,vecPoses[(i*4)+0],vecPoses[(i*4)+1],vecPoses[(i*4)+2]); 
                ac.sendGoal(goal);
                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	                ROS_INFO("Gasbot has reached to desired pose %i",i+1);
	                //ROS_INFO("Wait for a sec....");
	                //ros::WallDuration(1).sleep();
	                // ============ GAS SENSING ============
	                if (vecPoses[(i*4)+3]==1){
	                        
                        //-- update conf num
                        //conf_num++;
                        
                        //-- create log file
                        //------------------------
                        create_____LogFile();
                        
                        //-- Gas sampling
                        //------------------------
                        ROS_INFO("Gas sampling is in progress....");
                        perform____GasSampling();
		                
		                //-- Wait untill sampling is completed
		                //------------------------
		                //ROS_INFO("Line 180, PTU status is %d",statusPTU);
		                while(statusPTU!=3){
	                        //ROS_INFO("Line 182, PTU status is not 3... it is %d",statusPTU);
                            ros::WallDuration(1).sleep();
                        }
                        //ROS_INFO("Line 185, PTU status is %d",statusPTU);
		                //ros::WallDuration(20).sleep();
		                ros::WallDuration(5).sleep();
		                while(statusPTU!=0){
	                        //ROS_INFO("Line 189, PTU status is not 0... it is %d",statusPTU);
                            ros::WallDuration(1).sleep();
                        }
                        //ROS_INFO("Line 192, PTU status is %d",statusPTU);
				        //ROS_INFO("Hamary ptu ka status...%i\n",global_status);}
		                ROS_INFO("Gas detection COMPLETED.");
		                
		                //-- update conf num
		                //------------------------
                        conf_num++;
	                    
                        //-- close log file
                        //------------------------
		                close______LogFile();
	                }
                }
                else{
                	ROS_INFO("The Husky failed to reach desired pose.");
            	}
            }
        
        //pub_conf.publish(conf_msg);
        ros::WallDuration(5).sleep();
        }
        else{                    
            ROS_INFO("No configuration to execute.....");
            ros::WallDuration(5).sleep();
        }
    }
}



//================================================================================
//                      MAIN
//================================================================================


int main(int argc, char** argv){
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
	    
	    	
    	//============================================
	    //----- File contains poses or plan
	    //============================================
	    paramHandle.param<std::string>("plan_FileName",plan_FileName,DEFAULT_PLAN_FILENAME);
	    paramHandle.param<std::string>("plan_FilePath",plan_FilePath,ros::package::getPath("plan_execution"));	

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

	    
	    
	    //============================================	
	    //----- ROS Topic Names
	    //============================================
	    paramHandle.param<std::string>("localization_topic",topicLocalization,   DEFAULT_TOPIC_LOCALIZATION);
	    paramHandle.param<std::string>("ptu_sweep_topic",   topicPTUSweepStatus, DEFAULT_TOPIC_PTU_SWEEP_STATUS);
	    paramHandle.param<std::string>("ptu_joint_topic",   topicPTUJointStatus, DEFAULT_TOPIC_PTU_JOINT_STATUS);
	    paramHandle.param<std::string>("rmld_topic",        topicRMLDReadings,   DEFAULT_TOPIC_RMLD_READINGS);
	    //paramHandle.param<std::string>("current_goal_topic",topicCurrentGoal,    DEFAULT_TOPIC_CURRENT_GOAL);
        
        
        //============================================
	    //----- ROS Topic Subscriptions
	    //============================================
	    //-- PTU STATUS
        ros::Subscriber sub1 = n.subscribe(topicPTUSweepStatus, 1000,callback___PTUSweepingStatus); 
        //-- Localization
        ros::Subscriber sub2 = n.subscribe(topicLocalization,   1000,callback___Localization);
        //-- RMLD
        ros::Subscriber sub3 = n.subscribe(topicRMLDReadings,   1000,callback___RMLD);	
    	//-- PTU JOINT
    	ros::Subscriber sub4 = n.subscribe(topicPTUJointStatus, 1000,callback___PTUJointAngles);
        //-- CURRENT GOAL
    	//ros::Subscriber sub6 = n.subscribe(topicCurrentGoal,  1000,callbackCurrentGoal);
	    
	    
  
        //============================================
        //----- ROS Topic Publishers
        //============================================
        //ros::Publisher  pub_conf = n.advertise<std_msgs::Int16>("/plan_execution/conf_num", 1000);

        //============================================
        //----- ROS Services
        //============================================
        ros::ServiceServer srv_conn = n.advertiseService("executed_conf_num",service____ExecutedConfNumber);

        boost::thread mythread(execute____PlannedConfigurations);
        //followPOSES();
        //ros::spinOnce();  
        ros::spin();
        //ros::spinOnce();
        //r.sleep();
return 0;
}



