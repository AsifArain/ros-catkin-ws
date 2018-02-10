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

#include "common_prelude.h"
#include "human_prelude.h"
#include "common_functions.h"
#include <visualization_msgs/Marker.h>


//--- File Variables
//==========================
double cell_size;

std::vector<double> vec_RobotOrigin;
std::vector<int> vec_MapSize,vec_MapSensorPlacements;

std::string FilePath, \
            filename__MapSize, \
            filename__CellSize, \
            filename__RobotOrigin, \
            filename__MapSensorPlacements;
            
double hard_offset_x, hard_offset_y;


//--- Visualization Markers
//==========================
visualization_msgs::Marker p_msg, p_pnt, p_pt1,p_pt2,p_pt3;

//-- ROS publishers
//==========================
ros::Publisher placement_pub;



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
         	}
		    file__MapSize.close();
	    }
	    else std::cout << "Unable to open map size file";
        
        	    
	    //==============================
	    //--- map sensor placements
	    //==============================
	    ROS_INFO("Sensor placements map... ");
	    file__MapSensorPlacements.open((FilePath+filename__MapSensorPlacements).c_str(), std::ios::app);
	    int this_map;
	    vec_MapSensorPlacements.clear();
	    if (file__MapSensorPlacements.is_open()){		    
		    while(file__MapSensorPlacements >> this_map){
			    vec_MapSensorPlacements.push_back(this_map);
         	}
		    file__MapSensorPlacements.close();
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
//              EXECUTE PLANNED CONFIGURATIONS (HUMAN-SELECTED)
//================================================================================
void execute____PlannedConfigurations(){
        
        //-- create log directory
        //------------------------
	    create_____LogDirectory();
	    
	    
	    //--- read environment map
	    //----------------------------------
	    readEnvironmentMap();	    
	    //double robot_offset_x = (vec_RobotOrigin[0]-1) * cell_size;
	    //double robot_offset_y = (vec_RobotOrigin[1]-1) * cell_size;
	    
	    //ROS_INFO("Environment file reading is done....");
	    
	    //ROS_INFO("Origin is.... %f, %f",vec_RobotOrigin[0],vec_RobotOrigin[1]);
	    
	    int robot_offset_x_cell = vec_RobotOrigin[0]-1;
	    int robot_offset_y_cell = vec_RobotOrigin[1]-1;
	    
	    //ROS_INFO("Environment file reading is doneeeee....");
	    
	    
	    //-- initialization of text message
	    //----------------------------------	    
        p_msg.header.frame_id = "/mcl_pose"; //"/map";        
        p_msg.header.stamp = ros::Time::now();
        p_msg.ns = "placement_msg_publisher";
        p_msg.action = visualization_msgs::Marker::ADD;
        p_msg.pose.position.x = 1.0;
        p_msg.pose.position.y = 1.0;
        p_msg.pose.position.z = 1.0;
        p_msg.pose.orientation.w = 1.0;        
        p_msg.id = 0;
        p_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;        
        p_msg.scale.z = 0.5;        
        p_msg.color.r = 1.0f;
        p_msg.color.g = 0.0f;
        p_msg.color.b = 0.0f;
        p_msg.color.a = 1.0;
        
        
        //-- initialization of placement points
	    //---------------------------------------
        p_pnt.header.frame_id = "/map";        
        p_pnt.header.stamp = ros::Time::now();
        p_pnt.ns = "placement_msg_publisher";
        p_pnt.action = visualization_msgs::Marker::ADD;
        p_pnt.pose.orientation.w = 1.0;        
        p_pnt.id = 1;
        p_pnt.type = visualization_msgs::Marker::POINTS;        
        p_pnt.scale.x = 0.1;
        p_pnt.scale.y = 0.1;
        p_pnt.color.r = 1.0f;
        p_pnt.color.g = 0.0f;
        p_pnt.color.b = 0.0f;
        p_pnt.color.a = 1.0;
        
        
        
        //-- initialization of placement points1
	    //---------------------------------------	    
        p_pt1.header.frame_id = "/map";        
        p_pt1.header.stamp = ros::Time::now();
        p_pt1.ns = "placement_msg_publisher";
        p_pt1.action = visualization_msgs::Marker::ADD;
        p_pt1.pose.orientation.w = 1.0;        
        p_pt1.id = 2;
        p_pt1.type = visualization_msgs::Marker::POINTS;        
        p_pt1.scale.x = 0.1;
        p_pt1.scale.y = 0.1;
        p_pt1.color.r = 1.0f;
        p_pt1.color.g = 0.0f;
        p_pt1.color.b = 0.0f;
        p_pt1.color.a = 1.0;
        
        
        //-- initialization of placement points2
	    //---------------------------------------	    
        p_pt2.header.frame_id = "/map";        
        p_pt2.header.stamp = ros::Time::now();
        p_pt2.ns = "placement_msg_publisher";
        p_pt2.action = visualization_msgs::Marker::ADD;
        p_pt2.pose.orientation.w = 1.0;        
        p_pt2.id = 3;
        p_pt2.type = visualization_msgs::Marker::POINTS;        
        p_pt2.scale.x = 0.2;
        p_pt2.scale.y = 0.2;
        p_pt2.color.r = 0.0f;
        p_pt2.color.g = 1.0f;
        p_pt2.color.b = 0.0f;
        p_pt2.color.a = 1.0;
        
        //-- initialization of placement points3
	    //---------------------------------------	    
        p_pt3.header.frame_id = "/map";        
        p_pt3.header.stamp = ros::Time::now();
        p_pt3.ns = "placement_msg_publisher";
        p_pt3.action = visualization_msgs::Marker::ADD;
        p_pt3.pose.orientation.w = 1.0;        
        p_pt3.id = 4;
        p_pt3.type = visualization_msgs::Marker::POINTS;        
        p_pt3.scale.x = 0.1;
        p_pt3.scale.y = 0.1;
        p_pt3.color.r = 0.0f;
        p_pt3.color.g = 0.0f;
        p_pt3.color.b = 1.0f;
        p_pt3.color.a = 1.0;
        
	    //-- publish an empty message
	    //----------------------------------
	    p_msg.text = ".";
        placement_pub.publish(p_msg);
	    
	    
	    //----------------------------------
	    // lets follow human plan
	    //----------------------------------
	    
        while(ros::ok()){
                                    
            //-----------------------------------------------------
            //-- get goal from rviz
            //-----------------------------------------------------
            ROS_INFO("Waiting for a desired pose from a human expert...");
            geometry_msgs::PoseStampedConstPtr rviz_goal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topicRVIZGoal);
                        
            //REF: http://smrpn.blogspot.se/2014/09/call-any-subscriber-to-topic-only-once.html
            //REF: https://github.com/ros/ros_comm/issues/488
            ROS_INFO("Hurrah..... I have received my goal!!!!");            
                        
            //-- Orientation (Quaternion to degrees)
            tf::Quaternion q(rviz_goal->pose.orientation.x,\
                             rviz_goal->pose.orientation.y,\
                             rviz_goal->pose.orientation.z,\
                             rviz_goal->pose.orientation.w);
            tf::Matrix3x3 m(q);
            double goal_roll, goal_pitch, goal_yaw;
            m.getRPY(goal_roll,goal_pitch,goal_yaw);
            double goal_yaw_dg = (goal_yaw)*180/M_PI;
            
                        
            //-----------------------------------------------------
            //-- validating the goal position for safe navigation
            //-----------------------------------------------------
            
            double rviz_position_x = rviz_goal->pose.position.x; //(vecXpoints[i]-1)*cell_size;
            double rviz_position_y = rviz_goal->pose.position.y; //(vecYpoints[j]-1)*cell_size;
            
            
            //-- publish cell point
            //--------------------------
            /*
            float x = rviz_position_x;
            float y = rviz_position_y;                                        
            float z = 0.7;                                
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            p_pt3.points.push_back(p);        
            placement_pub.publish(p_pt3);
            */
            
            
            int rviz_cell_x = roundf(rviz_position_x/cell_size)+robot_offset_x_cell;
            int rviz_cell_y = roundf(rviz_position_y/cell_size)+robot_offset_y_cell;
            
            int this_ind = (rviz_cell_x*vec_MapSize[1])+rviz_cell_y;
            int rviz_occ_info = vec_MapSensorPlacements[this_ind];
            
            
            if (rviz_occ_info == 1){
            
                ROS_INFO("I am moving to the pose <%.1fm,%.1fm,%.1fdeg>....",\
                     rviz_goal->pose.position.x,rviz_goal->pose.position.y,goal_yaw_dg);
                
                
                //-- publish message
                //--------------------------
                p_msg.text = "It is a safe position.";
	            placement_pub.publish(p_msg);
	            
	            
	            
                //-- publish cell point
                //--------------------------
                /*
                float x = ((rviz_cell_x-vec_RobotOrigin[0]+1)*cell_size)+hard_offset_x;
                float y = ((rviz_cell_y-vec_RobotOrigin[1]+1)*cell_size)+hard_offset_y;
                float z = 0.5;                                    
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                p_pt2.points.push_back(p);            
	            placement_pub.publish(p_pt2);
	            */
                
                
                
                //-----------------------------------------------------
                //-- translate rviz goal into move_base goal
                //-----------------------------------------------------
                move_base_msgs::MoveBaseGoal goal;
                
                goal.target_pose.header.frame_id = plan_frame_name;
                goal.target_pose.header.stamp    = ros::Time::now();
                goal.target_pose.pose 	         = rviz_goal->pose;
                
                            
                //-----------------------------------------------------
                //-- wake up move_base
                //-----------------------------------------------------
                ros::WallDuration(1).sleep();
                //tell the action client that we want to spin a thread by default
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
                
                //-- wait for the action server to come up
                while(!ac.waitForServer(ros::Duration(5.0))){
                        ROS_INFO("Waiting for the move_base action server to come up");
                }
                
                //-----------------------------------------------------
                //-- send goal
                //-----------------------------------------------------
                ac.sendGoal(goal);
                ac.waitForResult();
                
                //-----------------------------------------------------
                //-- sampling the environment
                //-----------------------------------------------------
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    
                    ROS_INFO("Gasbot has reached to desired pose...");
                    
                    //-- create log file
                    //------------------------
                    create_____LogFile();
		                            
                    //-- Gas sampling
                    //------------------------
                    ROS_INFO("Gas sampling is in progress....");
                    perform____GasSampling();
                    
                    //-- Wait untill sampling is completed
                    //------------------------
                    while(statusPTU!=3){
                        ros::WallDuration(1).sleep();
                    }
                    ros::WallDuration(5).sleep();
                    while(statusPTU!=0){
                        ros::WallDuration(1).sleep();
                    }
                    ROS_INFO("Gas measurements are completed.");
                    
                    //-- Update conf num
                    //------------------------
                    conf_num = conf_num+1;
                    //conf_msg.data = conf_num;
                    
                    //pub_conf.publish(conf_msg);
                    
                    //-- close log file
                    //------------------------
                    close______LogFile();
                    
                }
                else{
                    ROS_INFO("Failed to reach the desired pose.");
                }
            
            }
            else{
            
                	            
	            ROS_INFO("Select a safe sensing position.");
                
                //-- publish message
                //--------------------------
                p_msg.text = "Select a safe sensing position.";
	            placement_pub.publish(p_msg);	    
                
                
                //-- publish cell point
                //--------------------------
                /*
                float x = ((rviz_cell_x-vec_RobotOrigin[0]+1)*cell_size)+hard_offset_x;
                float y = ((rviz_cell_y-vec_RobotOrigin[1]+1)*cell_size)+hard_offset_y;                
                float z = 0.6;                                    
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                p_pt1.points.push_back(p);            
                placement_pub.publish(p_pt1);
                */
            }
            
        }
}


//================================================================================
//                      MAIN
//================================================================================

int main(int argc, char** argv){
        ros::init(argc, argv, "human_plan_execution");
        ros::NodeHandle n;

	    // ####################### PARAMETERS ########################
	    ros::NodeHandle paramHandle ("~");


        //============================================
	    //----- Gas Sampling Parameters
	    //============================================
	    paramHandle.param("min_pan_angle",    min_pan_angle,    DEFAULT_MIN_PAN_ANGLE);
	    paramHandle.param("max_pan_angle",    max_pan_angle,    DEFAULT_MAX_PAN_ANGLE);
	    paramHandle.param("sensing_range",    sensing_range,    DEFAULT_SENSING_RANGE);
	    paramHandle.param("offsetY_base_rmld",offsetY_base_rmld,DEFAULT_OFFSETY_BASE_RMLD);
	    paramHandle.param("field_of_view",    FoV,              DEFAULT_FOV);
	    paramHandle.param("num_pan_sweeps",   num_pan_sweeps,   DEFAULT_NUM_PAN_SWEEPS);
	    paramHandle.param("num_tilt_sweeps",  num_tilt_sweeps,  DEFAULT_NUM_TILT_SWEEPS);
	    paramHandle.param("sample_delay",     sample_delay,     DEFAULT_SAMPLE_DELAY);	    
        
               
	
        //============================================	
	    //----- Navignation Parameters
	    //============================================
	    paramHandle.param<std::string>("plan_frame_name",plan_frame_name,DEFAULT_PLAN_FRAME_NAME);

        //============================================
        //----- Historian (default) Parameters -----
        //============================================
	    // Gas Measurements
	    paramHandle.param("threshold",GasMeasureThreshold,DEFAULT_GAS_MEASURE_THRESHOLD);
	    paramHandle.param("operation",OperationStatus,    DEFAULT_OPERATION_STATUS);
	    // Log File
	    //paramHandle.param<std::string>("log_file_path",log_FilePath,ros::package::getPath("plan_execution")+"/logs/human_exploration/");
	    paramHandle.param<std::string>("experiment_title",    experimentTitle,    DEFAULT_EXPERIMENT_TITLE);
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,DEFAULT_EXPLORATION_STRATEGY);
        
        //============================================	
	    //----- Localization Parameters
	    //============================================
	    //paramHandle.param<std::string>("localization_topic",localization_topic,DEFAULT_LOCALIZATION_TOPIC);
	    
	    
	    //============================================	
	    //----- Map Info Parameters
	    //============================================
	    //paramHandle.param<std::string>("file_path",FilePath,ros::package::getPath("plan_execution")+"/logs/");
	    paramHandle.param<std::string>("experiment_title",experimentTitle,"prismaforum5-04");
	    paramHandle.param<std::string>("exploration_strategy",explorationStrategy,"one-step-exploration");
	    paramHandle.param<std::string>("file_path",FilePath,\
	    ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/");
	    paramHandle.param<std::string>("map_file",filename__MapSensorPlacements,"prismaforum_map_conf.dat");
	    paramHandle.param<std::string>("map_size_file",filename__MapSize,"prismaforum_mapsize_conf.dat");	    
	    paramHandle.param<std::string>("cell_size_file",filename__CellSize,"prismaforum_cellsize_conf.dat");
	    paramHandle.param<std::string>("robot_origin_file",filename__RobotOrigin,"prismaforum_origin_conf.dat");
    
	    paramHandle.param<double>("hard_offset_x",hard_offset_x,0.0);
        paramHandle.param<double>("hard_offset_y",hard_offset_y,0.0);
	    
	    //============================================	
	    //----- ROS Topic Names
	    //============================================
	    paramHandle.param<std::string>("localization_topic",topicLocalization,   DEFAULT_TOPIC_LOCALIZATION);
	    paramHandle.param<std::string>("ptu_sweep_topic",   topicPTUSweepStatus, DEFAULT_TOPIC_PTU_SWEEP_STATUS);
	    paramHandle.param<std::string>("ptu_joint_topic",   topicPTUJointStatus, DEFAULT_TOPIC_PTU_JOINT_STATUS);
	    paramHandle.param<std::string>("rmld_topic",        topicRMLDReadings,   DEFAULT_TOPIC_RMLD_READINGS);
	    //paramHandle.param<std::string>("current_goal_topic",topicCurrentGoal,    DEFAULT_TOPIC_CURRENT_GOAL);
	    paramHandle.param<std::string>("rviz_goal_topic",   topicRVIZGoal,       DEFAULT_TOPIC_RVIZ_GOAL);
	    
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
        placement_pub = n.advertise<visualization_msgs::Marker>("placement_msg", 10);
        
        //============================================
        //----- ROS Services
        //============================================
        ros::ServiceServer srv_conf = n.advertiseService("executed_conf_num",service____ExecutedConfNumber);

        
        ros::Rate loop_rate(10);

        //conf_msg.data = 0;
        //pub_conf.publish(conf_msg);
        //Ref: https://answers.ros.org/question/53865/publisher-an-integer/

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //
        //                      FOLLOW HUMAN PLAN
        //
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        boost::thread mythread(execute____PlannedConfigurations);
        
        //ros::spinOnce();
        ros::spin();
        //loop_rate.sleep();
        
        
        //ros::spinOnce();  
        //ros::spin();
        //ros::spinOnce();
        //r.sleep();
        return 0;
}



