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

#include "gdm_tdlas.h"
#include "std_msgs/Float32.h"


USING_NAMESPACE_QPOASES
using namespace std;

void rmldInCallback(const localized_RMLD::rmld_msg::ConstPtr& msg_in){
	if(!new_data_arrived){

		measurementPPMM = msg_in->concentration_ppmm;

		lastPoint_Origin.x = msg_in->origin_x;
		lastPoint_Origin.y = msg_in->origin_y;
		lastPoint_Origin.z = msg_in->origin_z;
		lastPoint_End.x = msg_in->end_x;
		lastPoint_End.y = msg_in->end_y;
		lastPoint_End.z = msg_in->end_z;

		new_data_arrived = true;
	}
}

void updateGradient(real_t *g, std::vector<int> *idx, std::vector<float> *val, float ppmm){
	for (unsigned int i = 0; i < idx->size(); i++){
		g[idx->at(i)] -= 2 * ppmm * val->at(i);
	}
}

void printSparseMatrixInfo(vector<sparse_int_t> *H_jc, vector<sparse_int_t> *H_ir, vector<real_t> *H_val){
	printf("\nColumns vector:\n");
	for (unsigned int i = 0; i < H_jc->size(); i++)
		printf("%d\t",H_jc->at(i));
	printf("\nRows vector:\n");
	for (unsigned int i = 0; i < H_ir->size(); i++)
		printf("%d\t",H_ir->at(i));
	printf("\nValues vector:\n");
	for (unsigned int i = 0; i < H_val->size(); i++)
		printf("%4.2f\t",H_val->at(i));
	printf("\n");
}

void updateHessian(vector<sparse_int_t> *H_jc, vector<sparse_int_t> *H_ir, vector<real_t> *H_val, std::vector<int> *idx, std::vector<float> *val){	
		
	//index columns
	for (unsigned int i = 0; i < idx->size(); i++){
		//index rows		
		for (unsigned int j = 0; j < idx->size(); j++){
			bool already_present = false;
			real_t val_update = val->at(i)*val->at(j);
			
			//new insertion
			//ROS_INFO("INSERTION");
			//ROS_INFO("H_jc size: %d",H_jc->size());
			//ROS_INFO("idx i : %d",idx->at(i));
			//ROS_INFO("idx i + 1 : %d",(idx->at(i)+1));
			//ROS_INFO("H_jc idx i : %d",H_jc->at(idx->at(i)));
			//ROS_INFO("H_jc idx i + 1 : %d",H_jc->at(idx->at(i)+1));
		
			for (int c = H_jc->at(idx->at(i)); c < H_jc->at(idx->at(i)+1); c++){
				if (H_ir->at(c) == idx->at(j)){
					already_present = true;
					H_val->at(c) += val_update;
					break;
				}
			}
			
			//add new element
			if (already_present == false){
				vector <sparse_int_t>::iterator i_int = H_ir->begin();
				vector <real_t>::iterator i_real = H_val->begin();
				unsigned int offset = 0;
				bool offsetOK = false;				
				for(int k = H_jc->at(idx->at(i)); k < H_jc->at(idx->at(i)+1) ; k++){
					if (idx->at(j) < H_ir->at(k)){
						offset = k;
						offsetOK = true;
						break;
					}				
				}
				
				if (offsetOK == false){
					offset = H_jc->at(idx->at(i)+1);	
				}
				
				H_ir->insert(i_int + offset,idx->at(j));
				H_val->insert(i_real + offset,val_update);

				for (unsigned int c = idx->at(i)+1; c < H_jc->size(); c++){
					H_jc->at(c)++;
				}
			}				
		}
	}
}

int main(int argc, char **argv)
{
	//Ros initialization
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;

	//Parameters
	ros::NodeHandle param_n("~");

	param_n.getParam("MIN_X", MIN_X);
	param_n.getParam("MAX_X", MAX_X);
	param_n.getParam("MIN_Y", MIN_Y);
	param_n.getParam("MAX_Y", MAX_Y);
	param_n.getParam("MIN_Z", MIN_Z);
	param_n.getParam("MAX_Z", MAX_Z);
	param_n.getParam("CELL_SIZE", CELL_SIZE);
	param_n.getParam("MAX_CONC", MAX_CONC);
	param_n.getParam("FIXED_FRAME", FIXED_FRAME);
	param_n.getParam("RMLD_TOPIC", RMLD_TOPIC);

	//Subscriptions
	ros::Subscriber sub = n.subscribe(RMLD_TOPIC, 1000, rmldInCallback);

	//ros::Publisher	tdlas_advertise=n.advertise<visualization_msgs::MarkerArray>("gdm_tdlas", 20);
	ros::Publisher tdlas_advertise = n.advertise<sensor_msgs::PointCloud2>("gdm_tdlas", 20);
	//Global initializations
	ros::Rate loop_rate(LOOP_RATE_HZ);
	new_data_arrived = false;
	gas_map = new GdmTDLAS::GasMap(MIN_X,MAX_X,MIN_Y,MAX_Y,MIN_Z,MAX_Z,CELL_SIZE,MAX_CONC,FIXED_FRAME);	
	const unsigned int tot_num_cells = gas_map->getNumCells(0) * gas_map->getNumCells(1) * gas_map->getNumCells(2);
	double cputime = 0.1;
	int nWSR = 100000;
	float objValOffset = 0.0;	

	SQProblem qp(tot_num_cells,0);
	returnValue rv;
	
	vector<sparse_int_t> H_jc(tot_num_cells+1);
	vector<sparse_int_t> H_ir(tot_num_cells);
	vector<real_t> H_val(tot_num_cells);
	real_t g[tot_num_cells];
	real_t lb[tot_num_cells];
	vector<real_t> xOpt(tot_num_cells);

	//initialize gradient and lower bounds
	for (unsigned int i = 0; i < tot_num_cells; i++){
		g[i] = 0;
		lb[i] = 0;
		H_jc[i] = i;
		H_ir[i] = i;
		H_val[i] = 0.000001;
	}

	H_jc[tot_num_cells] = tot_num_cells;

	SymSparseMat *H = new SymSparseMat(tot_num_cells, tot_num_cells, &H_ir[0], &H_jc[0], &H_val[0]);
	H->createDiagInfo();

	//solve first QP
	qp.setPrintLevel(PL_NONE);
	rv = qp.init( H,g,NULL,lb,NULL,NULL,NULL,nWSR,&cputime);		

	switch(rv){
		case SUCCESSFUL_RETURN:
			if (qp.getPrimalSolution(&xOpt[0]) == RET_QP_NOT_SOLVED){
				ROS_INFO("Map initialization failed!");
				return 1;
			}
			else{
				gas_map->updateMap(&xOpt);
				gas_map->publish(tdlas_advertise);
				ROS_INFO("Map initialized!");
			}
			break;
		case RET_MAX_NWSR_REACHED:
			ROS_INFO("Map initialization: max working set recalculation reached!");	
			qp.reset();
			break;
		case RET_INIT_FAILED:
			ROS_INFO("Map initialization failed!");
			return 1;
		default:
			ROS_INFO("Map initialization: Unhandled return value!");
			break;
	}
	cputime = 1.0;
	nWSR = 100000;

	gas_map->printInfo();

	//Main loop
	while (ros::ok()){
		//ROS_INFO("Spinning");
	
		if(new_data_arrived){
			
			mutex_rmld.lock();

			//when octomap is unable to project a ray in the 3d model or
			//when there is tf missing, the rmld driver returns -1 as an
			//integral concentration measurement
			//ROS_INFO("From [%4.2f,%4.2f,%4.2f] to [%4.2f,%4.2f,%4.2f] = %fPPMM", \
				lastPoint_Origin.x,lastPoint_Origin.y,lastPoint_Origin.z, \
				lastPoint_End.x,lastPoint_End.y,lastPoint_End.z, \
				measurementPPMM);

			vector<int> idx;
			vector<float> val;		
			if(gas_map->CalculateNewMeasurementRow(lastPoint_Origin, lastPoint_End, measurementPPMM, &idx, &val)){
				//update matrices
				updateGradient(g,&idx,&val,measurementPPMM);					
				updateHessian(&H_jc,&H_ir,&H_val,&idx,&val);
				objValOffset += measurementPPMM*measurementPPMM;
	
				delete H;

				H = new SymSparseMat(tot_num_cells, tot_num_cells, &H_ir[0], &H_jc[0], &H_val[0]);
				H->createDiagInfo();
			
				rv = qp.hotstart( H,g,NULL,&lb[0],NULL,NULL,NULL,nWSR,&cputime);		

				switch(rv){
					case SUCCESSFUL_RETURN:
						if (qp.getPrimalSolution(&xOpt[0]) == RET_QP_NOT_SOLVED)
							ROS_INFO("Map update failed!");
						else{
							gas_map->updateMap(&xOpt);
							gas_map->publish(tdlas_advertise);
							ROS_INFO("Map updated successfully in %4.4f seconds!",cputime);
							gas_map->printMapData();
						}
						break;
					case RET_MAX_NWSR_REACHED:
						ROS_INFO("Map update: max working set recalculation reached!");	
						qp.reset();
						break;
					case RET_HOTSTART_FAILED:
						ROS_INFO("Map update failed!");
					default:
						ROS_INFO("Map update: unhandled return value!");
						break;
				}
				cputime = 0.1;
				nWSR = 10000;
				
			}

			new_data_arrived=false;
			mutex_rmld.unlock();
		//--------------------------------------------------------------//
		//--------------------------------------------------------------//

		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	delete gas_map;
	delete H;

  	return 0;	
}
