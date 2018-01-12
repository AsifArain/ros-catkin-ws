//#include "common_prelude.h"
//#include "robot_exploration.h"
//#include "human_exploration.h"

//================================================================================
//              CREATE LOG DIRECTORY
//================================================================================
void create_____LogDirectory(){

    
        //===== Date and Time
        time_t date_temp;       	// Stores seconds elapsed since 01-01-1970
        struct tm *date_format; 	// Saves in Date format
        char date_out[25];     		// Output string

        time(&date_temp);
        date_format = localtime(&date_temp);
        strftime(date_out, 25, "%Y-%m-%d-%H:%M:%S", date_format);

        char date_date[11];
        strftime(date_date, 11, "%Y-%m-%d%H:%M:%S", date_format);
        string date_str(date_date);
        string date_time_(date_out);

        //===== Log Directory
        //log_FilePath = ros::package::getPath("plan_execution")+\
        //               "/logs/"+experimentTitle+\
        //               "/"+explorationStrategy+"/"+date_time_; //"/"+explorationStrategy+"/"+date_time_+"/";
                       
                       
        std::string Path1 = ros::package::getPath("plan_execution")+"/logs/"; 
        std::string Path2 = Path1+experimentTitle;
        std::string Path3 = Path2+"/"+explorationStrategy+"/";
        //std::string Path4 = Path3+"/"+date_time_+"/";
        //std::string Path4 = ros::package::getPath("plan_execution")+"/logs/"+experimentTitle+"/"+explorationStrategy+"/"+date_time_;
        
        if (mkdir(Path1.c_str(),0777)==-1){};
        if (mkdir(Path2.c_str(),0777)==-1){};
        if (mkdir(Path3.c_str(),0777)==-1){};
        //if (mkdir(Path4.c_str(),0777)==-1){};
        
        
        log_FilePath = Path3;
        
        
        //===== Create Directory
        if (mkdir(log_FilePath.c_str(),0777)==-1){ 
        
            //===== Print info
            ROS_INFO("Following logs directory is created:\n%s\n",log_FilePath.c_str());
        }
        else{
            //===== Print info
            ROS_INFO("Following logs directory either already exists or it is not created:\n%s\n",log_FilePath.c_str());
        }

        
}





//================================================================================
//              CREATE LOG FILE
//================================================================================
void create_____LogFile(){
    
        //===== File name	
        //string filename = "measurements_"+std::to_string (conf_num)+".dat";
        log_FileName = "measurements_conf"+boost::lexical_cast<std::string>(conf_num+1)+".dat";

        //===== Create File
        historyFile.open((log_FilePath+log_FileName).c_str());

        //===== Print info
        ROS_INFO("A log file '%s' has been created in the following directory \n%s",\
                 log_FileName.c_str(),log_FilePath.c_str());
}



//================================================================================
//              CLOSE LOG FILE
//================================================================================
void close______LogFile(){
        if(historyFile){
	        historyFile.close();
        }
}




//================================================================================
//              CALLBACK: PTU SWEEPING STATUS
//================================================================================
void callback___PTUSweepingStatus(const std_msgs::Int16::ConstPtr& sta){
        //ROS_INFO("PTU status is...%d",sta->data);
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
}

//================================================================================
//              SERVICE: EXECUTED CONFIGURATION NUMBER
//================================================================================
bool service____ExecutedConfNumber(roscpp_tutorials::TwoInts::Request  &req,
    roscpp_tutorials::TwoInts::Response &res){
        
        //res.sum = req.a + req.b;
        res.sum = conf_num;
        //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
        //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
        return true;
}


//================================================================================
//              WRITING DATA TO THE LOG FILE
//================================================================================
void write______LogFile(double posX, \
                        double posY, \
                        double posW, \
                        double anglePan, \
                        double angleTilt, \
                        double measure){
                                
	if(historyFile){
        	
        //--- Date and Time
        //---------------------------
                
        time_t date_temp;       	// Stores seconds elapsed since 01-01-1970
        struct tm *date_format; 	// Saves in Date format
        char date_out[25];     		// Output string

        time(&date_temp);
        date_format = localtime(&date_temp);
        strftime(date_out, 25, "%Y-%m-%d-%H:%M:%S", date_format);

        char date_date[11];
        strftime(date_date, 11, "%Y-%m-%d%H:%M:%S", date_format);
        string date_str(date_date);
        string date_time_(date_out);

        //--- Fill log file
        //---------------------------
        historyFile<<conf_num<<" "<<date_time_<<" "<<posX<<" "<<posY<<" "<<posW<<" "<<anglePan<<" "<<angleTilt<<" "<<measure<<endl;
		//historyFile<<golS<<" "<<golX<<" "<<golY<<" "<<golW<<" "<<posX<<" "<<posY<<" "<<posW<<" "<<statePTU<<" "<<anglePan<<" "<<angleTilt<<" "<<measure<<endl;

		//--- Print info
		//---------------------------
		ROS_INFO("Pose <%.2f,%.2f,%.2f>; Pan-Tilt Angles <%.2f,%.2f>; Measurement %.2f;",\
			  posX,posY,posW,anglePan,angleTilt,measure);
	}
}




//================================================================================
//              CALLBACK: RMLD MEASUREMENTS
//================================================================================
void callback___RMLD(rmld_node::rmld_msg mea){

	    measure  = mea.concentration_ppmm;
	    rmld_msg = mea.rmld_data_string;	

	    // Writing to the file
	    // Note: It is obvious that the file will be updated only 
	    //       if a gas measurement is available.
	    //measure >= 50.0 &&
	    /*
	    if (measure >= GasMeasureThreshold && (statePTU==OperationStatus || OperationStatus==5)){	
		    write______LogFile(posX,posY,posW,statePTU,anglePan,angleTilt,measure);
		    //write______LogFile(golS,golX,golY,golW,posX,posY,posW,statePTU,anglePan,angleTilt,measure);
	    }
	    */
	
	    
	
	    if (    historyFile && \
	            measure >= GasMeasureThreshold && \
	            (statusPTU==OperationStatus || OperationStatus==5)){	
		
		    //ROS_INFO("Writing in progress...");
		    write______LogFile(posX,posY,posW,anglePan,angleTilt,measure);
	    }
}


//================================================================================
//              GAS SAMPLING
//================================================================================
void perform____GasSampling(){

	    ros::NodeHandle n;      	
      	ros::ServiceClient client1 = n.serviceClient<ptu_control2::commandSweep>(srvPTUSweepCommand);
      	ros::ServiceClient client2 = n.serviceClient<amtec::GetStatus>(srvPTUJointStatus);
      
	    ptu_control2::commandSweep srvSweep;
	    //amtec::GetStatus  srvState;
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

	    srvSweep.request.min_pan  	= -(FoV/2);        //min_pan_angle;
	    srvSweep.request.max_pan  	=  (FoV/2);        //max_pan_angle;
	    srvSweep.request.min_tilt 	= tilt_angle; 	   //min_tilt_angle;
	    srvSweep.request.max_tilt 	= tilt_angle;      //max_tilt_angle;
	    srvSweep.request.n_pan    	= num_pan_sweeps;  //  1;
	    srvSweep.request.n_tilt   	= num_tilt_sweeps; //  1;
	    srvSweep.request.samp_delay = sample_delay;    //0.1;
	
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
		    ROS_ERROR("Failed to get status.");} 
		*/
}

