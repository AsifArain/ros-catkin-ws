#include "octo_raytrace.h"




//==================================================
//=	  	   Callbacks
//==================================================


//RMLD topic			
void handleRMLDTopic(const rmld_node::rmld_msg::ConstPtr& msg_in){
	last_RMLD_PPMM=msg_in->concentration_ppmm;
	rmld_to_process=true;

}

//PTU state
void handlePTUStateTopic(const std_msgs::Int16::ConstPtr& data_in)
{
	ptu_current_state = data_in->data;

	new_state_arrived=true;	
}

//PC state

//Victor: This comes from the velodine
/*
void handlePCTopic(const sensor_msgs::PointCloud2::ConstPtr& data_in)
{
	// Fixes with Victor:
  	pcl::PCLPointCloud2 pcl_pc2;
    	pcl_conversions::toPCL(*data_in,pcl_pc2);
    	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
	cloudpcl=*temp_cloud;
	
	//pcl::fromROSMsg(*data_in,cloudpcl);
	//Victor: Triggers the octomap model built
	cloud_to_process=true;
	model_available=false;
}
*/
void handleVelodynePCTopic(const sensor_msgs::PointCloud2::ConstPtr& data_in)
{
	// Fixes with Victor:
  	pcl::PCLPointCloud2 pcl_pc2;
    	pcl_conversions::toPCL(*data_in,pcl_pc2);
    	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
	cloudpcl=*temp_cloud;
	
/*	pcl::fromROSMsg(*data_in,cloudpcl);*/
	//Victor: Triggers the octomap model built
	cloud_to_process=true;
	model_available=false;
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

	loadNodeParameters(n);
	ros::Rate loop_rate(refresh_rate_hz);


	//----- Subscriptions --------//
	ros::Subscriber rmld_conc = n.subscribe(rmld_topic.c_str(), 1000,handleRMLDTopic);
	ros::Subscriber ptu_state_subscriber = n.subscribe("ptu_control/state", 1000, handlePTUStateTopic);
	//ros::Subscriber pc_subscriber = n.subscribe(pc_topic.c_str(), 1000, handlePCTopic);
	
	ros::Subscriber pc_subscriber = n.subscribe(pc_topic.c_str(), 1000, handleVelodynePCTopic);
	//ros::Subscriber velodyne_sub = n.subscribe<PointCloud>("velodyne_points", 1, handleVelodynePCTopic);



	


	//----- Advertise
	ros::Publisher	rmld_ray_pub = n.advertise<visualization_msgs::Marker>("raytrace/vis_ray",100);
	ros::Publisher	rmld_topic=n.advertise<octo_raytrace::raytrace_rmld>("raytrace/data", 100);
	
	//----- Initializations ------//
	cloud_to_process=false;	
	rmld_to_process=false;
	model_available=false;
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;

	std::ofstream file_log;
	if (data_recording_enabled){
  		file_log.open (log_file_name.c_str());
	}

	/*
	sleep for some time
		while !cloud to process
		if cloud to procees
			create model.
			model_available=1.
		
	*/

	if(cloud_to_process) {
				
			ROS_INFO("Wait: Cloud model is in process...");
			model_tree = new octomap::OcTree(voxel_size);
			octomap::point3d	origin(0,0,0);
			octomap::Pointcloud octomap_pcl;
			//Victor: This part generates the octomap model
			octomap::pointcloudPCLToOctomap(cloudpcl, octomap_pcl);
			//OctomapROS::pointcloudPCLToOctomap(cloudpcl, octomap_pcl); // Asif
 			model_tree->insertScan(octomap_pcl,origin);
			cloud_to_process=false;
			model_available=true;
			ROS_INFO("Cloud model is ready!");
		}

	while (ros::ok()){

		//Victor: This part of the code tells me if I have a new point cloud to process
		/*		
		if(cloud_to_process) {
				
			ROS_INFO("will build the model");
			model_tree = new octomap::OcTree(voxel_size);
			octomap::point3d	origin(0,0,0);
			octomap::Pointcloud octomap_pcl;
			//Victor: This part generates the octomap model
			octomap::pointcloudPCLToOctomap(cloudpcl, octomap_pcl);
			//OctomapROS::pointcloudPCLToOctomap(cloudpcl, octomap_pcl); // Asif
 			model_tree->insertScan(octomap_pcl,origin);
			cloud_to_process=false;
			model_available=true;
			ROS_INFO("I have the model");
		}
		*/

		//rmld_to_process=true;
		//Victor: Make sure to define when to trigger this
		//if(rmld_to_process && model_available){
		if(rmld_to_process && model_available && ptu_current_state==3){
			bool tf_flag=true;
   			try{
				tf_listener.waitForTransform(fixed_frame.c_str(), rmld_frame.c_str(),ros::Time::now(), \
				ros::Duration(FRAME_WAIT_TIME));
      				tf_listener.lookupTransform(fixed_frame.c_str(),rmld_frame.c_str(),ros::Time(0), transform);
    			}
    	
			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
				tf_flag=false;
    			}

			if(tf_flag){
				
				tf::Quaternion q(transform.getRotation().x(),\
				transform.getRotation().y(),transform.getRotation().z(),\
				transform.getRotation().w());

				double yaw,pitch,roll;
				tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

				Eigen::Affine3f tr_matrix;
				tr_matrix=pcl::getTransformation(0,0,0,roll,pitch,yaw);

				pcl::PointXYZ direction_vector,dir_xyz;
				//pcl::PointT dir_xyz,point_xyz;
				direction_vector.x=1;
				direction_vector.y=0;
				direction_vector.z=0;	
				dir_xyz=pcl::transformPoint(direction_vector,tr_matrix);
				octomap::point3d	dir_vector(dir_xyz.x,dir_xyz.y,dir_xyz.z);
				octomap::point3d	origin_point(transform.getOrigin().x(), transform.getOrigin().y(),\
					transform.getOrigin().z());
				octomap::point3d end_point;
				bool result=model_tree->castRay(origin_point,dir_vector,end_point,true);

				if (result){
					//ROS_INFO("Travel: (%f,%f,%f)->(%f,%f,%f)",\
					transform.getOrigin().x(),transform.getOrigin().y(),\
					transform.getOrigin().z(),end_point.x(),end_point.y(),end_point.z());
					ros::Time current_time;
					current_time=ros::Time::now();

					if (data_recording_enabled){
						file_log << current_time<<";"<<transform.getOrigin().x()<<";"<<transform.getOrigin().y()<<";"<<transform.getOrigin().z()<<";"
							<<end_point.x()<<";"<<end_point.y()<<";"<<end_point.z()<<";"<<last_RMLD_PPMM<<"\n";
					}
					if(display_rays) {
						publishRayTrace(rmld_ray_pub, origin_point, end_point,fixed_frame,last_RMLD_PPMM);

						octo_raytrace::raytrace_rmld 	rmldM;
						rmldM.startX=origin_point.x();
						rmldM.startY=origin_point.y();
						rmldM.startZ=origin_point.z();

						rmldM.endX=end_point.x();
						rmldM.endY=end_point.y();
						rmldM.endZ=end_point.z();

						rmldM.ppmm=last_RMLD_PPMM;
						rmld_topic.publish(rmldM);

					}
				}
				
					

			}

			rmld_to_process=false;
		}

		 
		ptu_last_state=ptu_current_state;
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (data_recording_enabled){
		file_log.close();
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

	parameter_name=std::string(NODE_NAME)+std::string("/rmld_frame");
	private_nh.param(parameter_name.c_str(), rmld_frame, std::string(DEFAULT_RMLD_FRAME));

	parameter_name=std::string(NODE_NAME)+std::string("/fixed_frame");
	private_nh.param(parameter_name.c_str(), fixed_frame, std::string(DEFAULT_FIXED_FRAME));

	parameter_name=std::string(NODE_NAME)+std::string("/pc_topic");
	//parameter_name=std::string(NODE_NAME)+std::string("/velodyne_points");
	private_nh.param(parameter_name, pc_topic, std::string(DEFAULT_PC_TOPIC));

	parameter_name=std::string(NODE_NAME)+std::string("/rmld_topic");
	private_nh.param(parameter_name, rmld_topic, std::string(DEFAULT_RMLD_TOPIC));

	parameter_name=std::string(NODE_NAME)+std::string("/voxel_size");
	private_nh.param(parameter_name, voxel_size,DEFAULT_VOXEL);

	
	parameter_name=std::string(NODE_NAME)+std::string("/publish_model");
	private_nh.param(parameter_name, publish_model, DEFAULT_MODEL_PUB);

	parameter_name=std::string(NODE_NAME)+std::string("/display_rays");
	private_nh.param(parameter_name, display_rays, DEFAULT_DISPLAY_RAYS);

	parameter_name=std::string(NODE_NAME)+std::string("/output_log");
	private_nh.param(parameter_name, log_file_name, std::string(DEFAULT_FILE_NAME));

	if (strcmp(log_file_name.c_str(),DEFAULT_FILE_NAME)!=0){
		data_recording_enabled=true;
	}
	else{
		data_recording_enabled=false;
	}


  	ROS_INFO("=============================");
	ROS_INFO("Node Configuration:");
	ROS_INFO("sampling_rate set to %f",refresh_rate_hz);
	ROS_INFO("PC topic: %s",pc_topic.c_str());
	ROS_INFO("RMLD frame: %s",rmld_frame.c_str());
	ROS_INFO("RMLD topic: %s",rmld_topic.c_str());
	ROS_INFO("fixed frame: %s",fixed_frame.c_str());
	ROS_INFO("model publish enabled: %d",publish_model);
	ROS_INFO("display rays enabled: %d",display_rays);
	ROS_INFO("Voxel size: %f",voxel_size);

	if (data_recording_enabled) {
		ROS_INFO("Recording location %s",log_file_name.c_str());

	}

}	


void publishRayTrace(ros::Publisher rmld_ray_pub, octomap::point3d startp, octomap::point3d endp,std::string fixed_frame,double conc){
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = fixed_frame.c_str();
		line_list.header.stamp = ros::Time::now();
		line_list.ns = NODE_NAME;
		line_list.id = 1;
		line_list.scale.x = 0.02;
		line_list.scale.y = 0.02;
		line_list.scale.z = 0.02;

	
		line_list.pose.orientation.x = 0;
		line_list.pose.orientation.y = 0;
		line_list.pose.orientation.z = 0;
		line_list.pose.orientation.w = 1.0;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
					
		if (conc > 600){
		line_list.color.r = 1;
		line_list.color.g = 0;
		line_list.color.b = 0;
    		line_list.color.a = 1.0;
		}
		else{
		line_list.color.r = 0;
		line_list.color.g = 1;
		line_list.color.b = 0;
    		line_list.color.a = 1.0;
		}


    		line_list.lifetime = ros::Duration(0.5);
    	
    		line_list.action = visualization_msgs::Marker::ADD;						
		geometry_msgs::Point p;
		p.x = startp.x();
      		p.y = startp.y();
		p.z = startp.z();//+0.25;
		line_list.points.push_back(p);
		p.x = endp.x();
      		p.y = endp.y();
		p.z = endp.z();//+0.25;
		line_list.points.push_back(p);
		
		rmld_ray_pub.publish(line_list);
    	
}

