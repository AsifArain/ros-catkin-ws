//=============================================================
//	Node Name: ptu_control.cpp
//	Version:	01
//	Date:		August 12th, 2013
//	Description: Integrates joystick ptu control
//		as well as service requests from other modules
//==============================================================


#include "ptu_control.h"
//-------------------------------------------------------
// 	Joystick callback
//-------------------------------------------------------
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	if (joy->buttons[reset_button]==1 && (!is_scan_ptu_requested && !is_gas_scan_requested)){
		is_reset_ptu_requested=true;
		ROS_INFO("Here");
	}
	else if (joy->buttons[scan_button]==1 && (!is_scan_ptu_requested && !is_gas_scan_requested)){
		is_scan_ptu_requested=true;
	}

	else if (joy->buttons[gas_scan_button]==1 && (!is_scan_ptu_requested && !is_gas_scan_requested)){
		is_gas_scan_requested=true;
	}
	else {

		if ((lock_trigger && joy->axes[lock_button] > 0 && is_lock_ptu_requested)  || (!lock_trigger && joy->buttons[lock_button]==0 && is_lock_ptu_requested) )
			is_lock_ptu_requested=false;
		else if((lock_trigger && joy->axes[lock_button] <=0) || (!lock_trigger && joy->buttons[lock_button]==1 ) ){
				
			is_lock_ptu_requested=true;
			ptu_horizontal_request=joy->axes[3];
			ptu_vertical_request=joy->axes[4];
		}	
	}
	
}

//-------------------------------------------------------
// 	Pan callback
//-------------------------------------------------------
void panCallback(const amtec::AmtecState::ConstPtr& data_in)
{
	pan_location_deg=rad2deg(data_in->position);
	
}

//-------------------------------------------------------
// 	tilt callback
//-------------------------------------------------------
void tiltCallback(const amtec::AmtecState::ConstPtr& data_in)
{
	tilt_location_deg=rad2deg(data_in->position);
}


//-------------------------------------------------------
// 	Service callback
//-------------------------------------------------------
bool sweepRequestCallback(ptu_control::commandSweep::Request  &req, ptu_control::commandSweep::Response& resp)
{
	min_pan_srv= req.min_pan;
	min_tilt_srv=req.min_tilt;
	max_pan_srv=req.max_pan;
	max_tilt_srv=req.max_tilt;
	n_pan_srv=req.n_pan;
	n_tilt_srv=req.n_tilt;
	samp_delay_srv=req.samp_delay;
	service_running=true;
	return true;
}

//******************************************************************
//******************************************************************
//* 			Main Function				   *
//******************************************************************
//******************************************************************
int main(int argc, char **argv)
{
	ROS_INFO("*****************************************************");
	ROS_INFO("     %s Node starting...",DEVICE_NAME);
	ROS_INFO("*****************************************************");



	ros::init(argc, argv, DEVICE_NAME);
  	ros::NodeHandle n;
	ros::Publisher state_pub = n.advertise<std_msgs::Int16>("/ptu_control/state", 1000);
	ros::ServiceServer service = n.advertiseService("/ptu_control/sweep", sweepRequestCallback);
	retrieveLaunchParameters(n);
	
	ros::Subscriber sub1 = n.subscribe(joystick_topic.c_str(), 1000, joyCallback);
	ros::Subscriber sub2 = n.subscribe("/amtec/pan_state", 1000, panCallback);
	ros::Subscriber sub3 = n.subscribe("/amtec/tilt_state", 1000, tiltCallback);


	ros::ServiceClient client = n.serviceClient<amtec::SetPosition>("/amtec/set_position");
	ros::ServiceClient clientStatus = n.serviceClient<amtec::GetStatus>("/amtec/get_status");
	amtec::SetPosition srv;
	

	ros::Rate loop_rate(10);

	service_running=false;
	is_gas_scan_requested=false;
	is_lock_ptu_requested=false;
	is_reset_ptu_requested=false;
	is_scan_ptu_requested=false;
	amtec::GetStatus  srvState;
	controller_state=IDLE;
	resetPTUPosition(state_pub, client,clientStatus);
	while (ros::ok()){
		
	
		if(joystick_control_enabled && !service_running) {
			
			//Joystick requests
			if(is_reset_ptu_requested){
				ROS_INFO("Going to (0,0) position");
				resetPTUPosition(state_pub, client,clientStatus);
				is_reset_ptu_requested=false;
			}

			else if(is_gas_scan_requested){
				ROS_INFO("Scanning for gas");
				//AZIF: if condition to perform or not the initian scan
				if (enable_environment_scan){
				 	performSceneScan(state_pub, scan_min_tilt_deg, scan_max_tilt_deg, client,clientStatus);
				}
				sleep(1);
				performGasScan(state_pub, scan_min_tilt_deg, scan_max_tilt_deg, scan_min_pan_deg, scan_max_pan_deg, n_pan_joy, n_tilt_joy,samp_delay_joy,client,clientStatus);
				sleep(1);
				resetPTUPosition(state_pub, client,clientStatus);
				is_gas_scan_requested=false;
			}

			else if(is_scan_ptu_requested){
				ROS_INFO("Scene scanning");

				performSceneScan(state_pub, scan_min_tilt_deg, scan_max_tilt_deg, client,clientStatus);
				is_scan_ptu_requested=false;
			}

			else if(is_lock_ptu_requested){
				bool info_received=false;
				double target_pan_deg=0;
				double target_tilt_deg=0;

				amtec::GetStatus  srvState;
				do{
					if(clientStatus.call(srvState)){
						info_received=true;
						target_pan_deg=rad2deg(srvState.response.position_pan);	
						target_tilt_deg=rad2deg(srvState.response.position_tilt);	
					}
					else{
						info_received=false;
					}
					sleep(0.1);
				}while(!info_received);
				//ROS_INFO("--- PTU locked, position %f,%f",target_pan_deg,target_tilt_deg);
				

				
				if (ptu_horizontal_request > 0){
					target_pan_deg = target_pan_deg + interp(angle_increments_deg,fabs(ptu_horizontal_request),N_DELTA_DEG);
					//target_pan_deg=deg2rad(target_pan_deg);
					//target_tilt_deg=deg2rad(target_tilt_deg);

					srv.request.position_pan=deg2rad(target_pan_deg);
					srv.request.position_tilt=deg2rad(target_tilt_deg);
					client.call(srv);
					sleep(JOYSTICK_CONTROL_DELAY);
				}
				if(ptu_horizontal_request < 0){
					target_pan_deg = target_pan_deg - interp(angle_increments_deg,fabs(ptu_horizontal_request),N_DELTA_DEG);
					//target_pan_deg=deg2rad(target_pan_deg);
					//target_tilt_deg=deg2rad(target_tilt_deg);

					srv.request.position_pan=deg2rad(target_pan_deg);
					srv.request.position_tilt=deg2rad(target_tilt_deg);
					client.call(srv);
					sleep(JOYSTICK_CONTROL_DELAY);
				}
				if (ptu_vertical_request > 0){
					target_tilt_deg = target_tilt_deg - interp(angle_increments_deg,fabs(ptu_vertical_request),N_DELTA_DEG);
					//target_pan_deg=deg2rad(target_pan_deg);
					//target_tilt_deg=deg2rad(target_tilt_deg);

					srv.request.position_pan=deg2rad(target_pan_deg);
					srv.request.position_tilt=deg2rad(target_tilt_deg);
					client.call(srv);
					sleep(JOYSTICK_CONTROL_DELAY);
				}
				if(ptu_vertical_request < 0){
					target_tilt_deg = target_tilt_deg + interp(angle_increments_deg,fabs(ptu_vertical_request),N_DELTA_DEG);
					//target_pan_deg=deg2rad(target_pan_deg);
					//target_tilt_deg=deg2rad(target_tilt_deg);

					srv.request.position_pan=deg2rad(target_pan_deg);
					srv.request.position_tilt=deg2rad(target_tilt_deg);
					client.call(srv);
					sleep(JOYSTICK_CONTROL_DELAY);
				}
				//ROS_INFO("Requested PTU LOCATION %f,%f",target_pan_deg,target_tilt_deg);

			}

		}

		//Service requestes//
		else if(service_running) {
			ROS_INFO("Service requested");
			ROS_INFO("Pan %f,%f,%d",min_pan_srv,max_pan_srv,n_pan_srv);
			ROS_INFO("Tilt %f,%f,%d",min_tilt_srv,max_tilt_srv,n_tilt_srv);


			//AZIF: if condition here
			if (enable_environment_scan){
				performSceneScan(state_pub, min_tilt_srv, max_tilt_srv, client,clientStatus);
			}

			sleep(0.5);
			performGasScan(state_pub, min_tilt_srv, max_tilt_srv, min_pan_srv, max_pan_srv, n_pan_srv, n_tilt_srv, samp_delay_srv,client,clientStatus);
			sleep(0.5);
			resetPTUPosition(state_pub, client,clientStatus);
			service_running=false;
		}


		std_msgs::Int16	 msg_state;
		msg_state.data=controller_state;
		state_pub.publish(msg_state);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}



//-------------------------------------------------------
// 			Parameter retrieval
//-------------------------------------------------------
void	retrieveLaunchParameters(ros::NodeHandle n){
	char param_name[100];

	std::string 	param_string;
	double		param_double;
	int		param_int;
	bool		param_bool;


	// AZIF: Set a parameter to enable/disable the environment scan /enable_environment_scan
	
	//---- Enable/disable Environment Scan ----//
	sprintf(param_name,"%s/enable_environment_scan",DEVICE_NAME);
	if(n.getParam(param_name,param_bool)) {  
		enable_environment_scan=param_double;
	}
	else {
		enable_environment_scan=DEFAULT_ENABLE_ENV_SCAN;
	}

	//---- min tilt scan ----//
	sprintf(param_name,"%s/min_tilt_scan",DEVICE_NAME);
	if(n.getParam(param_name,param_double)) {  
		scan_min_tilt_deg=param_double;
	}
	else {
		scan_min_tilt_deg=DEFAULT_SCAN_MIN_TILT_DEG;
	}

	//---- max tilt scan ----//
	sprintf(param_name,"%s/max_tilt_scan",DEVICE_NAME);
	if(n.getParam(param_name,param_double)) {  
		scan_max_tilt_deg=param_double;
	}
	else {
		scan_max_tilt_deg=DEFAULT_SCAN_MAX_TILT_DEG;
	}


	//---- min pan scan ----//
	sprintf(param_name,"%s/min_pan_scan",DEVICE_NAME);
	if(n.getParam(param_name,param_double)) {  
		scan_min_pan_deg=param_double;
	}
	else {
		scan_min_pan_deg=DEFAULT_SCAN_MIN_PAN_DEG;
	}

	//---- max pan scan ----//
	sprintf(param_name,"%s/max_pan_scan",DEVICE_NAME);
	if(n.getParam(param_name,param_double)) {  
		scan_max_pan_deg=param_double;
	}
	else {
		scan_max_pan_deg=DEFAULT_SCAN_MAX_PAN_DEG;
	}
	//---- scan button ----//
	sprintf(param_name,"%s/scan_button",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		scan_button=param_int;
	}
	else {
		scan_button=DEFAULT_SCAN_BUTTON;
	}

	//---- reset button ----//
	sprintf(param_name,"%s/reset_button",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		reset_button=param_int;
	}
	else {
		reset_button=DEFAULT_RESET_BUTTON;
	}


	//---- gas scan button ----//
	sprintf(param_name,"%s/gas_button",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		gas_scan_button=param_int;
	}
	else {
		gas_scan_button=DEFAULT_GAS_SCAN_BUTTON;
	}


	//---- lock button ----//
	sprintf(param_name,"%s/lock_button",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		lock_button=param_int;
	}
	else {
		lock_button=DEFAULT_LOCK_BUTTON;
	}

	//---- lock trigger ----//
	sprintf(param_name,"%s/lock_trigger",DEVICE_NAME);
	if(n.getParam(param_name,param_bool)) {  
		lock_trigger=param_bool;
	}
	else {
		lock_trigger=DEFAULT_LOCK_TRIGGER;
	}

	//---- joystick topic ----//
	sprintf(param_name,"%s/joystick_topic",DEVICE_NAME);
	if(n.getParam(param_name,param_string)) {  
		joystick_topic=param_string;
	}
	else {
		joystick_topic=DEFAULT_JOYSTICK_TOPIC;
	}


	//---- Joystick enabled ----//
	sprintf(param_name,"%s/joystick_control",DEVICE_NAME);
	if(n.getParam(param_name,param_bool)) {  
		joystick_control_enabled=param_bool;
	}
	else {
		joystick_control_enabled=DEFAULT_JOYSTICK_ENABLED;
	}

	//---- pan points ----//
	sprintf(param_name,"%s/n_pan_points",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		n_pan_joy=param_int;
	}
	else {
		n_pan_joy=DEFAULT_N_PAN;
	}

	//---- tilt points ----//
	sprintf(param_name,"%s/n_tilt_points",DEVICE_NAME);
	if(n.getParam(param_name,param_int)) {  
		n_tilt_joy=param_int;
	}
	else {
		n_tilt_joy=DEFAULT_N_TILT;
	}

	//---- sample delay ----//
	sprintf(param_name,"%s/samp_delay",DEVICE_NAME);
	if(n.getParam(param_name,param_double)) {  
		samp_delay_joy=param_double;
	}
	else {
		samp_delay_joy=GAS_SWEEP_DELAY;
	}

}


//-------------------------------------------------------
// 	Scene Scan
//-------------------------------------------------------
void	performSceneScan(ros::Publisher state_pub,double min_tilt_deg, double max_tilt_deg, ros::ServiceClient client,ros::ServiceClient clientStatus)
{
	amtec::SetPosition srv;
	amtec::GetStatus  srvState;
	ROS_INFO("Scan sweep requested");
			
	int tilt_i=0;
	int retry_count=0;
	bool pos_reached=false;
	//Just for the demo ocasion

	min_tilt_deg=-5;
	max_tilt_deg=10;	

	///////////////////////////


	int increments_tilt=20;
	double delta_tilt_deg=(max_tilt_deg-min_tilt_deg+10)/increments_tilt;
	//ROS_INFO("%f,%f",max_tilt_deg,min_tilt_deg);

	srv.request.position_pan=0;
	srv.request.position_tilt=deg2rad(max_tilt_deg+5);
	client.call(srv);	
	

	
	do{
		
		sleep(0.5);
		
		if(clientStatus.call(srvState)) {
			retry_count++;
			ROS_INFO("Moving");
			if(fabs(rad2deg(srvState.response.position_tilt)-max_tilt_deg) <= 0.8 && fabs(rad2deg(srvState.response.position_pan)) <= 0.8) {
				pos_reached=true;
			}

			else if (retry_count>20) {
				ROS_INFO("Warning: PTU not responding. Retrying");
				srv.request.position_pan=0;
				srv.request.position_tilt=deg2rad(max_tilt_deg);
				client.call(srv);
				sleep(1);
				retry_count=0;	
			} 
		}
			
		else {
			ROS_INFO("Warning: PTU not responding. Retrying");
			srv.request.position_pan=0;
			srv.request.position_tilt=deg2rad(max_tilt_deg);
			client.call(srv);
			sleep(1);
		     }		
	}while(!pos_reached);

	

	double target_tilt_deg=0;//max_tilt_deg;
	
	do{

		target_tilt_deg=target_tilt_deg-delta_tilt_deg;
		srv.request.position_pan=0;
		srv.request.position_tilt=deg2rad(target_tilt_deg);
		client.call(srv);
		tilt_i=tilt_i+1;

		std_msgs::Int16	 msg_state;
		msg_state.data=SCANNING_MODEL;
		state_pub.publish(msg_state);
		//ROS_INFO("Requested tilt %f",target_tilt_deg);
		sleep(0.1);

						
	}while (tilt_i<=increments_tilt);
	
	do{
		clientStatus.call(srvState);
		sleep(0.5);
	}while(fabs(srvState.response.velocity_tilt) > 0.01);
	std_msgs::Int16	 msg_state;
	msg_state.data=IDLE;
	state_pub.publish(msg_state);
	
}

//-------------------------------------------------------
// 	Gas Scan
//-------------------------------------------------------

void	performGasScan(ros::Publisher state_pub, double min_tilt_deg, double max_tilt_deg, double min_pan_deg, double max_pan_deg, int n_pan, int n_tilt, double samp_delay,ros::ServiceClient client,ros::ServiceClient clientStatus)
{

	bool pos_reached=false;
	int retry_count=0;

	amtec::SetPosition srv;
	amtec::GetStatus  srvState;
	srv.request.position_pan=deg2rad(min_pan_deg);
	srv.request.position_tilt=deg2rad(min_tilt_deg);
	client.call(srv);    
	
	do{
		
		sleep(0.5);
		
		if(clientStatus.call(srvState)) {
			retry_count++;
			//ROS_INFO("Moving");
			if(fabs(rad2deg(srvState.response.position_tilt)-min_tilt_deg) <= 0.8 && fabs(rad2deg(srvState.response.position_pan)-min_pan_deg) <= 0.8) {
				pos_reached=true;
			}

			else if (retry_count>20) {
				ROS_INFO("Warning: PTU not responding 1st. Retrying");
				srv.request.position_pan=deg2rad(min_pan_deg);
				srv.request.position_tilt=deg2rad(min_tilt_deg);
				client.call(srv);
				sleep(1);
				retry_count=0;	
			} 
		}
			
		else {
			ROS_INFO("Warning: PTU not responding 2nd. Retrying");
			srv.request.position_pan=deg2rad(min_pan_deg);
			srv.request.position_tilt=deg2rad(min_tilt_deg);
			client.call(srv);
			sleep(1);
		     }		
	}while(!pos_reached);



	int tilt_i=0;
	int pan_i=0;
	int current_movement=TO_RIGHT;

	double	requested_pan_deg;
	double	requested_tilt_deg;	
	
	requested_pan_deg=min_pan_deg;
	requested_tilt_deg=min_tilt_deg;



	float delta_pan_deg=fabs(max_pan_deg-min_pan_deg)/n_pan;
	float delta_tilt_deg=fabs(max_tilt_deg-min_tilt_deg)/n_tilt;
	//AZIF: Changes might go here if(n_tilt=1 && n_pan==1)

	do{
		if(current_movement==TO_RIGHT){

			srv.request.position_pan=deg2rad(requested_pan_deg);
			srv.request.position_tilt=deg2rad(requested_tilt_deg);
			client.call(srv);   
			do{
				clientStatus.call(srvState);
				sleep(0.1);
			}while(fabs(srvState.response.velocity_tilt) > 0.0 || fabs(srvState.response.velocity_pan) > 0.0); 
			//ROS_INFO("Scanning at %f,%f",requested_pan_deg,requested_tilt_deg);
			requested_pan_deg=requested_pan_deg+delta_pan_deg;
			pan_i=pan_i+1;

			if (pan_i > n_pan) {
				current_movement=TO_LEFT;
				requested_pan_deg=requested_pan_deg-delta_pan_deg;
				requested_tilt_deg=requested_tilt_deg+delta_tilt_deg;
				tilt_i=tilt_i+1;
			}
					
			std_msgs::Int16	 msg_state;
			msg_state.data=SCANNING_GAS;
			state_pub.publish(msg_state);
			sleep(samp_delay);
		}
		else{
			srv.request.position_pan=deg2rad(requested_pan_deg);
			srv.request.position_tilt=deg2rad(requested_tilt_deg);
			client.call(srv);
			do{
				clientStatus.call(srvState);
				sleep(0.1);
			}while(fabs(srvState.response.velocity_tilt) > 0.0 || fabs(srvState.response.velocity_pan) > 0.0);

			//ROS_INFO("Scanning at %f,%f",requested_pan_deg,requested_tilt_deg);
			requested_pan_deg=requested_pan_deg-delta_pan_deg;
			pan_i=pan_i-1;

			if (pan_i <= 0) {
				current_movement=TO_RIGHT;
				requested_pan_deg=requested_pan_deg+delta_pan_deg;
				requested_tilt_deg=requested_tilt_deg+delta_tilt_deg;
				tilt_i=tilt_i+1;
			}

			std_msgs::Int16	 msg_state;
			msg_state.data=SCANNING_GAS;
			state_pub.publish(msg_state);
			sleep(samp_delay);
		}
	}while (tilt_i<=n_tilt);

	do{
		clientStatus.call(srvState);
		sleep(0.5);
	}while(fabs(srvState.response.velocity_tilt) > 0.01);

	std_msgs::Int16	 msg_state;
	msg_state.data=SCANNING_GAS;
	state_pub.publish(msg_state);

}
//-------------------------------------------------------
// 	PTU reset position
//-------------------------------------------------------
void	resetPTUPosition(ros::Publisher state_pub, ros::ServiceClient client,ros::ServiceClient clientStatus)
{
	amtec::SetPosition srv;
	amtec::GetStatus  srvState;

	srv.request.position_pan=0;
	srv.request.position_tilt=0;
	client.call(srv);    
	int retry_count=0;
	bool pos_reached=false;
	
	do{
		sleep(0.5);

		if (clientStatus.call(srvState)){
			//ROS_INFO("Gotcha %f,%f",rad2deg(srvState.response.position_tilt),rad2deg(srvState.response.position_pan));
			retry_count++;
			if(fabs(rad2deg(srvState.response.position_tilt)) <= 0.8 && fabs(rad2deg(srvState.response.position_pan)) <= 0.8){
				pos_reached=true;
			}
			else if (retry_count>20) {
				ROS_INFO("Warning: PTU not responding Marco thinks we are here. Retrying");
				srv.request.position_pan=0;
				srv.request.position_tilt=0;
				client.call(srv);
				sleep(1);
				retry_count=0;	
			}
		}
		else{
			ROS_INFO("Warning: PTU not responding Ricarrdo thinks we are here. Retrying");
			srv.request.position_pan=0;
			srv.request.position_tilt=0;
			client.call(srv);
			sleep(1);			
		}
					
		std_msgs::Int16	 msg_state;
		msg_state.data=RESET;
		state_pub.publish(msg_state);
		ROS_INFO("To reset position");			
	} while(!pos_reached);

}



/****************************************************
 *	Interpolation function
 ****************************************************/
double interp( lookup_table* c, double x, int n )
{
    int i;

    for( i = 0; i < n-1; i++ )
    {
        if ( c[i].x <= x && c[i+1].x >= x )
        {
            double diffx = x - c[i].x;
            double diffn = c[i+1].x - c[i].x;

            return c[i].y + ( c[i+1].y - c[i].y ) * diffx / diffn; 
        }
    }

    return 0; // Not in Range
}



