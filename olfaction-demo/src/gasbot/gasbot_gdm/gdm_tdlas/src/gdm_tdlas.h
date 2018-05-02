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

#include "rmld_msg.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map.h"
#include <stdio.h>
#include <string.h>
#include <geometry_msgs/Point.h>

#include <boost/thread/mutex.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/math/constants/constants.hpp>
#include <qpOASES.hpp>


#define	LOOP_RATE_HZ 50
#define	NODE_NAME "gdm_tdlas" 


//experiments in the landfill 02
//double MIN_X = -4;//-5.30;
//double MAX_X = 6.00;
//double MIN_Y = -3.09;
//double MAX_Y = 15.00;
//double MIN_Z = 0;
//double MAX_Z = 1.00;
//double CELL_SIZE = 1;
//double MAX_CONC = 500;

std::string RMLD_TOPIC = "localized_RMLD/rmld_data2";
std::string FIXED_FRAME ="/pcl_model";


//experiments in the basement, demo
//double MIN_X = 3.1;
//double MAX_X = 12.0;
//double MIN_Y  = -2.0;
//double MAX_Y = 2.0;
//double MIN_Z = 0.0;
//double MAX_Z = 1.5;
//double CELL_SIZE = 1.0;
//double MAX_CONC = 500;




//experiments in the landfill 03
double MIN_X =-2;
double MAX_X = 4;//6;
double MIN_Y = -3;
double MAX_Y = 10;
double MIN_Z = 0.0;
double MAX_Z = 1.0;
double CELL_SIZE = 1.0;
double MAX_CONC = 500;

geometry_msgs::Point lastPoint_Origin;
geometry_msgs::Point lastPoint_End;

bool	new_data_arrived;
float	measurementPPMM;
GdmTDLAS::GasMap *gas_map;

boost::mutex mutex_rmld;
