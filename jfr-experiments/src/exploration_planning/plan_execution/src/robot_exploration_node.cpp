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


//--- File Variables
//==========================
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


//================================================================================
//              READ ENVIRONMENT MAP
//================================================================================
void readEnvironmentMap(){

        ROS_INFO("Reading data for robot exploration node... ");
        
        
        //ROS_INFO("Reading environment map... ");	    
	    std::ifstream file__MapSize, \
	                  file__MapSensorPlacements, \
	                  file__CellSize, \
	                  file__RobotOrigin;
	    
        
	    
	    //==============================
	    //--- Read Cell Size
	    //==============================
	    //ROS_INFO("Cell size... ");
	    file__CellSize.open((FilePath+filename__CellSize).c_str(), std::ios::app);
	    if (file__CellSize.is_open()){
         	file__CellSize >> cell_size;
		    file__CellSize.close();
	    }
	    else std::cout << "Unable to open file for cell size";
	    
	    
	    //==============================
	    //--- Read Robot Origin
	    //==============================
	    //ROS_INFO("Robot origin... ");	    
	    file__RobotOrigin.open((FilePath+filename__RobotOrigin).c_str(), std::ios::app);
	    //ROS_INFO("This origin file %s",(FilePath+filename__RobotOrigin).c_str());
	    double this_origin;
	    if (file__RobotOrigin.is_open()){
		    while(file__RobotOrigin >> this_origin){
			    vec_RobotOrigin.push_back(this_origin);
			    //ROS_INFO("This origin %f",this_origin);
         	}
		    file__RobotOrigin.close();
	    }
	    else std::cout << "Unable to open file robot origin cell size";
	    	    
}



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
	filePoses.open((FilePath+plan_FileName).c_str(), std::ios::app);
	if (filePoses.is_open()){
		//std::cout << "File is open."<<std::endl;
		//ROS_INFO("A file '%s' is to be read \n",(FilePath+plan_FileName).c_str());
		while(filePoses >> value){
			vecPoses.push_back(value);
			//std::cout<<"value is "<<value<<std::endl;
 		}
		filePoses.close();
	}
	else std::cout << "Unable to open the file";
}

//================================================================================
//              EXECUTE PLANNED CONFIGURATIONS 
//================================================================================
void execute____PlannedConfigurations(){
	
	
	//perform____GasSampling(); // dummy
	//int conf_num = 0;
	
	//-- create log directory
    //------------------------
	create_____LogDirectory();
	
	
	//-- read environment map info
    //------------------------
	readEnvironmentMap();
	
    	
	while(ros::ok()){
    
        //-- obtaining poses
        //---------------------
        ROS_INFO("Obtaining planned poses to execute...");
        get________PlannedPosesFromFile();
        
        //std::cout<<"size of vector "<<vecPoses.size()<<std::endl;
        
        num_of_conf = vecPoses.size()/3;
        //std::cout<<"Number of conf to execute: "<<num_of_conf<<std::endl;        
        ROS_INFO("Number of conf to execute: %d",num_of_conf);        
        ROS_INFO("Initial conf num (of this reading): %d",conf_num);
            
        if (num_of_conf >= conf_num){

            ros::WallDuration(1).sleep();
            
            ROS_DEBUG("Before move base ....");
            //tell the action client that we want to spin a thread by default
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
            ROS_DEBUG("After move base ....");
            
            
            //-- wait for the action server to come up
            //--------------------------------------------
            while(!ac.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
            }
            
            ROS_DEBUG("After wakeup ....");

            for (int i=(conf_num-1);i<vecPoses.size()/3;i++){
                
                //ROS_INFO("In the loop....");
                
                //ROS_INFO("first %f",vecPoses[i*3+0]); 
                //ROS_INFO("second %f",vec_RobotOrigin[0]);
                //ROS_INFO("third %f",cell_size);
                
                
                float conf_x = ((vecPoses[i*3+0]-vec_RobotOrigin[0]+0.5-1.0)*cell_size);
                float conf_y = ((vecPoses[i*3+1]-vec_RobotOrigin[1]+0.5-1.0)*cell_size);                
                float conf_t = vecPoses[i*3+2];
                
                ROS_INFO("This conf <%f,%f,%f>",conf_x,conf_y,conf_t);
                ROS_INFO("This vecPoses <%f,%f,%f>",vecPoses[i*3+0],vecPoses[i*3+1],vecPoses[i*3+2]);
                
                //--- goal pose
                //--------------------
                move_base_msgs::MoveBaseGoal goal;
                
                goal.target_pose.header.frame_id  = plan_frame_name; //"/world"; //"base_link" //"base_footprint";
                goal.target_pose.header.stamp     = ros::Time::now();
                
                goal.target_pose.pose.position.x  = conf_x;
                goal.target_pose.pose.position.y  = conf_y;                
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(conf_t*M_PI/180);
                
                                
                //--- sending goal
                //--------------------
                
                ROS_INFO("Gasbot is moving to Pose# %i <%.1fm,%.1fm,%.1fdeg>.",i+1,conf_x,conf_y,conf_t); 
                ac.sendGoal(goal);
                ac.waitForResult();
                
                //ROS_INFO("Move gasbot to Pose# %i <%.1fm,%.1fm,%.1fdeg>.",i,conf_x,conf_y,conf_t); 
                
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	                
	                ROS_INFO("Gasbot has reached to the desired pose# %i",i+1);
	                //ROS_INFO("Wait for a sec....");
	                //ros::WallDuration(1).sleep();
	                
	                
	                // ============ GAS SENSING ============
	                //if (vecPoses[(i*4)+3]==1){
	                        
                        //-- update conf num
                        //conf_num++;
                        
                        //-- create log file
                        //------------------------
                        create_____LogFile();
                        
                        //-- Gas sampling
                        //------------------------
                        ROS_INFO("Gas sampling in progress....");
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
		                ROS_INFO("Gas sampling COMPLETED.");
		                
		                
	                    
                        //-- close log file
                        //------------------------
		                close______LogFile();
		                
		                //-- update conf num
		                //------------------------
                        conf_num++;
                        
	                //}
                }
                else{
                	ROS_WARN("Gasbot failed to reach the desired pose.");
            	}
            }
        
        //pub_conf.publish(conf_msg);
        ros::WallDuration(5).sleep();
        }
        else{                    
            ROS_INFO("No more planned configurations to execute.....");
            ros::WallDuration(5).sleep();
        }
    }
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

	    paramHandle.param("initial_conf_num",conf_num,DEFAULT_CONF_NUM);
	    	    
	    //============================================	
	    //----- Map Info Parameters
	    //============================================
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    //paramHandle.param<std::string>("map_file",filename__MapSensorPlacements,"prismaforum_map_conf.dat");
	    //paramHandle.param<std::string>("map_size_file",filename__MapSize,"prismaforum_mapsize_conf.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"prismaforum_origin_conf.dat");
	    //paramHandle.param<std::string>("conf_file",filename__Confs,"robot_plan_detection.dat");
	    
	    
	    //============================================	
	    //----- ROS Topic Names
	    //============================================
	    paramHandle.param<std::string>("localization_topic",topicLocalization,   DEFAULT_TOPIC_LOCALIZATION);
	    paramHandle.param<std::string>("ptu_sweep_topic",   topicPTUSweepStatus, DEFAULT_TOPIC_PTU_SWEEP_STATUS);
	    paramHandle.param<std::string>("ptu_joint_topic",   topicPTUJointStatus, DEFAULT_TOPIC_PTU_JOINT_STATUS);
	    paramHandle.param<std::string>("rmld_topic",        topicRMLDReadings,   DEFAULT_TOPIC_RMLD_READINGS);
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



