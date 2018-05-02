//=============================================================
//	Node Name: ptu_control.h
//	Version:	01
//	Date:		August 12th, 2013
//	Description: Integrates joystick ptu control
//		as well as service requests from other modules
//==============================================================

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "SetPosition.h"
#include "AmtecState.h"
#include "std_msgs/Int16.h"
#include "commandSweep.h"
#include "GetStatus.h"

// Constants and calibrations

#define		IDLE				0
#define		MOVING_JOYSTICK			1
#define		SCANNING_MODEL			2
#define		SCANNING_GAS			3
#define		RESET				4


#define		DEVICE_NAME			"ptu_control"


#define		PI					3.1416
#define		deg2rad(x)				x*PI/180
#define		rad2deg(x)				x*180/PI

#define		DEFAULT_N_PAN				5
#define		DEFAULT_N_TILT				5
#define		DEFAULT_SCAN_MIN_TILT_DEG		-10
#define		DEFAULT_SCAN_MAX_TILT_DEG		 10
#define		DEFAULT_SCAN_MIN_PAN_DEG		-10
#define		DEFAULT_SCAN_MAX_PAN_DEG		 10		
#define		DEFAULT_SCAN_BUTTON			  5
#define		DEFAULT_RESET_BUTTON			  6
#define		DEFAULT_GAS_SCAN_BUTTON			  1	

#define		DEFAULT_LOCK_TRIGGER			true
#define		DEFAULT_LOCK_BUTTON			  2

#define		DEFAULT_JOYSTICK_TOPIC			"/teleop/joy"

#define		DEFAULT_JOYSTICK_ENABLED		false

#define		N_DELTA_DEG				11
#define 	DEFAULT_ENABLE_ENV_SCAN			false


#define		TO_LEFT					0
#define		TO_RIGHT				1
#define		GAS_SWEEP_DELAY				0.5

#define		JOYSTICK_CONTROL_DELAY			0.5

bool		is_gas_scan_requested;
bool		is_lock_ptu_requested;
bool		is_reset_ptu_requested;
bool		is_scan_ptu_requested;
double		ptu_horizontal_request;
double		ptu_vertical_request;
bool		lock_trigger;
bool 		enable_environment_scan;

double		scan_min_tilt_deg;
double		scan_max_tilt_deg;

double		scan_min_pan_deg;
double		scan_max_pan_deg;

int		scan_button;
int		lock_button;
int		reset_button;
int		gas_scan_button;
std::string	joystick_topic;

double		pan_location_deg;
double		tilt_location_deg;
bool		joystick_control_enabled;
bool		service_running;
int		controller_state;		

double		min_pan_srv;
double		max_pan_srv;

double		min_tilt_srv;
double		max_tilt_srv;

int		n_pan_srv;
int		n_tilt_srv;

double		samp_delay_srv;

int		n_pan_joy;
int		n_tilt_joy;

double		samp_delay_joy;

typedef struct { double x; double y; } lookup_table;

lookup_table angle_increments_deg[N_DELTA_DEG] = 
{
	{0,	0},
	{0.1,	1},
	{0.2,	1.5}, 
	{0.3,	2}, 
	{0.4,	2.5},
	{0.5,	3},
	{0.6,	3.5},
	{0.7,	4}, 
	{0.8,	6}, 
	{0.9,	7},
	{1.0,	8}
};    

void		retrieveLaunchParameters(ros::NodeHandle);

double		interp( lookup_table* , double, int);


void performSceneScan(ros::Publisher,double, double, ros::ServiceClient,ros::ServiceClient);
void performGasScan(ros::Publisher, double, double, double, double, int, int, double,ros::ServiceClient,ros::ServiceClient);
void resetPTUPosition(ros::Publisher, ros::ServiceClient,ros::ServiceClient);

