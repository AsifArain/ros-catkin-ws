/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2011, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors:
*********************************************************************/ 


  #include "gas_map.h"
  #include "ros/ros.h"
  #include "std_msgs/Float32.h"

  #include "nav_msgs/Odometry.h"
 
  #include <fstream>
  #include <iostream>
  #include <sstream>
  #include <string>
  #include <math.h>
  //#include <cv_bridge/CvBridge.h>

  #include <boost/thread/mutex.hpp>
  #include <boost/math/constants/constants.hpp>
  #include <visualization_msgs/MarkerArray.h>
  
  
  #define NODE_VERSION		3

  using namespace std;

  //constants
  //display width (pixels)
  #define DISPLAY_WIDTH 250
  //display height (pixels)
  #define DISPLAY_HEIGHT 150

		double MAX_CONCENTRATION=0.02;



  //nose displacement along robot vertical axis (m)
  #define NOSE_POSITION_U 0.3
  //nose displacement along robot horizontal axis (m)
  #define NOSE_POSITION_V 0.0
  //Number of grids to be displayed (x-axis)
  #define DISPLAY_GRIDS_COLS 8
  //Number of grids to be displayed (7-axis)
  #define DISPLAY_GRIDS_ROWS 5

//-------------------------------------------------------
//	Default values
//-------------------------------------------------------
	#define	DEFAULT_FRAME_ID		    "/gas_map" 
	#define	DEFAULT_POSITION_TOPIC		"/odom"
	#define	DEFAULT_SENSOR_TOPIC		"/mox_sensor"
	#define	DEFAULT_MAP_MAX_X		    100
	#define	DEFAULT_MAP_MIN_X		    0
	#define	DEFAULT_MAP_MAX_Y		    100
	#define	DEFAULT_MAP_MIN_Y		    0
	#define	DEFAULT_CELL_SIZE		    0.5
	#define	DEFAULT_KERNEL_SIZE		    1
	#define	DEFAULT_COLORMAP		    "jet"
	#define	DEFAULT_SENSOR_OFFSET_X		0
	#define	DEFAULT_SENSOR_OFFSET_Y		0
	#define	DEFAULT_MAX_SENSOR_VAL		0
	#define	DEFAULT_MIN_SENSOR_VAL		0
	#define	DEFAULT_N_POINTS_MAP		500
	#define	DEFAULT_PUBLISH_HZ			15
	

  //global variables
 // const double PI = boost::math::constants::pi<double>();
  //const float sigma_omega = 1/(2*PI*pow(kernel_size,2));
 /*   to move
  cv::Mat weights;
  cv::Mat confidence;
  cv::Mat weightedReadings;
  cv::Mat weightedVariance;
  cv::Mat meanMap;
  cv::Mat varianceMap;
  cv::Mat kernelCoefficients;
  cv::Mat robotTrajectory;
  */
  boost::mutex mutex_nose;
  boost::mutex mutex_position;
  bool new_data_nose = false;
  bool new_data_position = false;
  float curr_reading = 0;
  float curr_x = 0;
  float curr_y = 0;
  float global_mean = 1;
  float global_variance = 1;
  float num_samples = 0;

  //for color maps in rviz.
  void publishmarker(float x, float y,float size,float r,float g,float b);
  void color_map(std::string colormap, int val);
  ros::Publisher pub_marker_mean, pub_marker_var;
  visualization_msgs::Marker draw;
  geometry_msgs::Point p;
  std::string COLORMAP="jet";
  double colr=0;
  double colg=0;
  double colb=0;
