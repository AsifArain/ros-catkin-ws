#include "pc_builder.h"

//############################################
//		Topic callbacks
//############################################
void handlePTUStateTopic(const std_msgs::Int16::ConstPtr& data_in)
{
	if (!new_state_arrived) {
		ptu_current_state = data_in->data;
		new_state_arrived=true;	
	}
}


//############################################
//
//		     MAIN
//
//############################################
int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;

	
	bool scan_in_process=false;

	ptu_current_state = IDLE;
	ptu_last_state=IDLE;

	//----- Subscriptions --------//
	
	ros::Subscriber ptu_state_subscriber = n.subscribe("ptu_control/state", 1000, handlePTUStateTopic);



	ros::service::waitForService("assemble_scans");
  	ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  	laser_assembler::AssembleScans laser_assembler_srv;


	ros::Publisher	pc_topic=n.advertise<sensor_msgs::PointCloud2>("pc_buider/pc", 20);
	//----- Node Initialization -------//

	loadNodeParameters(n);
	new_state_arrived = false;

	ros::Rate loop_rate(refresh_rate_hz);

	ros::Time last_state_update_received,current_time;
	last_state_update_received=ros::Time::now();
	//----- Loop -----------------//

	while (ros::ok()){

		
		current_time=ros::Time::now();
		ros::Duration delta_time;
		delta_time = current_time-last_state_update_received;

		if(delta_time.toSec() > SCAN_FINISHED_S && scan_in_process)
		{
			scan_in_process=false;
			publishPointCloud(laser_assembler_srv,pc_topic,client);
		}
		if(new_state_arrived){
			last_state_update_received=ros::Time::now();
			if(ptu_current_state==SCANNING_MODEL && ptu_last_state != SCANNING_MODEL && !scan_in_process){
				ROS_INFO(">>>>>>>>Scanning for model starts here<<<<<<<<<<<");
				laser_assembler_srv.request.begin   = ros::Time::now();	
				scan_in_process=true;
			}
			else if (scan_in_process&&ptu_current_state!=SCANNING_MODEL && ptu_last_state == SCANNING_MODEL){
				publishPointCloud(laser_assembler_srv,pc_topic,client);	
				scan_in_process=false;
			}

			ptu_last_state = ptu_current_state;
			new_state_arrived=false;
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}
}




//*********************************************
//*	Initializes the node's parameters
//*********************************************
void	loadNodeParameters(ros::NodeHandle private_nh)
{

	std::string parameter_name;

	parameter_name=std::string(NODE_NAME)+std::string("/sampling_rate");
	private_nh.param(parameter_name.c_str(), refresh_rate_hz, DEFAULT_LOOP_RATE_HZ);

	parameter_name=std::string(NODE_NAME)+std::string("/fixed_frame");
	private_nh.param(parameter_name.c_str(), fixed_frame, std::string(DEFAULT_FIXED_FRAME));

	parameter_name=std::string(NODE_NAME)+std::string("/lidar_topic");
	private_nh.param(parameter_name.c_str(), scan_lidar_name, std::string(DEFAULT_TOPIC));

	parameter_name=std::string(NODE_NAME)+std::string("/ptu_state_topic");
	private_nh.param(parameter_name.c_str(), ptu_state_topic, std::string(DEFAULT_PTU_TOPIC));


  	ROS_INFO("=============================");
	ROS_INFO("Node Configuration:");
	ROS_INFO("sampling_rate set to %f",refresh_rate_hz);
	ROS_INFO("Lidar topic: %s",scan_lidar_name.c_str());
	ROS_INFO("Fixed frame: %s",fixed_frame.c_str());
	ROS_INFO("PTU state topic: %s",ptu_state_topic.c_str());
}

void publishPointCloud(laser_assembler::AssembleScans laser_assembler_srv, ros::Publisher pc_topic, ros::ServiceClient client){
	ROS_INFO("<<<<Scan finished>>>>");

	laser_assembler_srv.request.end   = ros::Time::now();

	if (client.call(laser_assembler_srv)){
    		printf("Got cloud with %u points\n", laser_assembler_srv.response.cloud.points.size());
		sensor_msgs::PointCloud2 cloud2_msg;
		sensor_msgs::convertPointCloudToPointCloud2(laser_assembler_srv.response.cloud, cloud2_msg);
		// condition/filter comes with response		

		cloud2_msg.header.frame_id=fixed_frame.c_str();
		cloud2_msg.header.stamp=ros::Time::now();
		
		pc_topic.publish(cloud2_msg);
	}
}

	

