

#include "map.h"
#include "math.h"

using namespace std;
using namespace GdmTDLAS;

//==========================================================================================================
GasMap::GasMap(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float cell_size, float max_conc, std::string frame){

	this->m_min_x = min_x;
	this->m_max_x = max_x;
	this->m_min_y = min_y;
	this->m_max_y = max_y;
	this->m_min_z = min_z;
	this->m_max_z = max_z;
	this->m_cell_size = cell_size;
	this->m_max_conc = max_conc;

	this->m_num_cells.clear();
	this->m_num_cells.push_back(ceil((max_x - min_x) / cell_size));
	this->m_num_cells.push_back(ceil((max_y - min_y) / cell_size));
	this->m_num_cells.push_back(ceil((max_z - min_z) / cell_size));


	this->m_cell_coords_x.clear();
	for (int i = 0; i < this->m_num_cells[0]; i++){
		this->m_cell_coords_x.push_back(min_x + i*cell_size);
	}

	this->m_cell_coords_y.clear();
	for (int i = 0; i < this->m_num_cells[1]; i++){
		this->m_cell_coords_y.push_back(min_y + i*cell_size);
	}

	this->m_cell_coords_z.clear();
	for (int i = 0; i < this->m_num_cells[2]; i++){
		this->m_cell_coords_z.push_back(min_z + i*cell_size);
	}

	//build the data structure
	this->m_map.clear();
	for (int k = 0; k < this->m_num_cells[2]; k++){
		for (int j = 0; j < this->m_num_cells[1]; j++){
			for (int i = 0; i < this->m_num_cells[0]; i++){	
				float x_center = min_x + i*cell_size + cell_size / 2;
				float y_center = min_y + j*cell_size + cell_size / 2;
				float z_center = min_z + k*cell_size + cell_size / 2;
				this->m_map.push_back(Cell(x_center,y_center,z_center,cell_size));
			}
		}
	}
	this->m_frame = frame;

	//build the template pointclouds

	for (int t = 0; t < NUM_CELL_TEMPLATES; t++){
		float conc = (float)t / (float)NUM_CELL_TEMPLATES;
		pcl::PointCloud<pcl::PointXYZRGB> curr_pc;
		curr_pc.width = round(conc * MAX_POINTS_PER_CELL);
		curr_pc.height = 1;
		curr_pc.points.resize (curr_pc.width * curr_pc.height);
	
		for (size_t j = 0; j < curr_pc.points.size(); j++)
		{
		    curr_pc.points[j].x = - this->m_cell_size / 2.0 +  this->m_cell_size * rand() / (RAND_MAX + 1.0f);
		    curr_pc.points[j].y = - this->m_cell_size / 2.0 +  this->m_cell_size * rand() / (RAND_MAX + 1.0f);
		    curr_pc.points[j].z = - this->m_cell_size / 2.0 +  this->m_cell_size * rand() / (RAND_MAX + 1.0f);
		    curr_pc.points[j].r = conc*255;
		    curr_pc.points[j].g = conc*255;
		    curr_pc.points[j].b = conc*255;
		}
		
		template_cells[t] = curr_pc;
	}

}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
int GasMap::getNumCells(int dimension){
	return this->m_num_cells[dimension];
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
bool GasMap::CalculateNewMeasurementRow(geometry_msgs::Point start, geometry_msgs::Point end, float ppmm, std::vector<int> *idx, std::vector<float> *val){
	vector<Intersection> intersections;

	float tot_l = sqrt(pow(end.x - start.x,2) + pow(end.y - start.y,2) + pow(end.z - start.z,2));
	
	if (tot_l == 0 || ppmm == -1){
		//ROS_INFO("Beam length equal to zero!");
		return false;
	}
	

	if ((start.x < this->m_min_x && end.x < this->m_min_x) || (start.x > this->m_max_x && end.x > this->m_max_x) || (start.y < this->m_min_y && end.y < this->m_min_y) || (start.y > this->m_max_y && end.y > this->m_max_y) || (start.z < this->m_min_z && end.z < this->m_min_z) || (start.z > this->m_max_z && end.z > this->m_max_z)){
		ROS_INFO("Measurement out of bounds!");
		return false;
	}

	//ROS_INFO("Complete beam from [%4.2f , %4.2f , %4.2f] to [%4.2f , %4.2f , %4.2f]",start.x,start.y,start.z,end.x,end.y,end.z);
	ProjectEndpointsInBounds(&start, &end);
	//ROS_INFO("Chopped beam from [%4.2f , %4.2f , %4.2f] to [%4.2f , %4.2f , %4.2f]",start.x,start.y,start.z,end.x,end.y,end.z);

	intersections.clear();	

	float start_x, start_y, start_z, end_x, end_y, end_z;

	if (start.x < end.x){
		start_x = start.x;
		start_y = start.y;
		start_z = start.z;
		end_x = end.x;
		end_y = end.y;
		end_z = end.z;
	}else{
		start_x = end.x;
		start_y = end.y;
		start_z = end.z;
		end_x = start.x;
		end_y = start.y;
		end_z = start.z;
	}

	//float start_x = min(start.x,end.x);
	//float end_x = max(start.x,end.x);

	//ROS_INFO("X intersections");
	for (unsigned int i = 0; i < this->m_cell_coords_x.size() ; i++){
		if (this->m_cell_coords_x[i] > start_x && m_cell_coords_x[i] < end_x){
			Intersection in;
			in.m_x = m_cell_coords_x[i];
			in.m_t = (in.m_x - start_x)/(end_x - start_x);
			in.m_y = (1 - in.m_t) * start_y + in.m_t * end_y;
			in.m_z = (1 - in.m_t) * start_z + in.m_t * end_z;
			
			float x_cell = (in.m_x - this->m_min_x) / this->m_cell_size;
			float y_cell = (in.m_y - this->m_min_y) / this->m_cell_size;
			float z_cell = (in.m_z - this->m_min_z) / this->m_cell_size;			

			//ROS_INFO("Coords: %4.2f , %4.2f , %4.2f",in.m_x,in.m_y,in.m_z);
			//ROS_INFO("Cell: %4.2f , %4.2f , %4.2f",x_cell,y_cell,z_cell);		
	
			in.m_idx_y = floor(y_cell);
			in.m_idx_z = floor(z_cell);

			if (in.m_x > 0)
				in.m_idx_x = floor(x_cell) - 1;
			else
				in.m_idx_x = floor(x_cell );

			intersections.push_back(in);	
		}	
	}

	if (start.y < end.y){
		start_x = start.x;
		start_y = start.y;
		start_z = start.z;
		end_x = end.x;
		end_y = end.y;
		end_z = end.z;
	}else{
		start_x = end.x;
		start_y = end.y;
		start_z = end.z;
		end_x = start.x;
		end_y = start.y;
		end_z = start.z;
	}

	//float start_y = min(start.y,end.y);
	//float end_y = max(start.y,end.y);

	//ROS_INFO("Y intersections");
	for (unsigned int i = 0; i < this->m_cell_coords_y.size() ; i++){
		if (this->m_cell_coords_y[i] > start_y && m_cell_coords_y[i] < end_y){
			Intersection in;
			in.m_y = m_cell_coords_y[i];
			in.m_t = (in.m_y - start_y)/(end_y - start_y);
			in.m_x = (1 - in.m_t) * start_x + in.m_t * end_x;
			in.m_z = (1 - in.m_t) * start_z + in.m_t * end_z;
			
			float x_cell = (in.m_x - this->m_min_x) / this->m_cell_size;
			float y_cell = (in.m_y - this->m_min_y) / this->m_cell_size;	
			float z_cell = (in.m_z - this->m_min_z) / this->m_cell_size;
			
			//ROS_INFO("Coords: %4.2f , %4.2f , %4.2f",in.m_x,in.m_y,in.m_z);
			//ROS_INFO("Cell: %4.2f , %4.2f , %4.2f",x_cell,y_cell,z_cell);

			in.m_idx_x = floor(x_cell);
			in.m_idx_z = floor(z_cell);		

			if (in.m_y > 0)
				in.m_idx_y = floor(y_cell) - 1;
			else
				in.m_idx_y = floor(y_cell);

			intersections.push_back(in);	
		}	
	}

	if (start.z < end.z){
		start_x = start.x;
		start_y = start.y;
		start_z = start.z;
		end_x = end.x;
		end_y = end.y;
		end_z = end.z;
	}else{
		start_x = end.x;
		start_y = end.y;
		start_z = end.z;
		end_x = start.x;
		end_y = start.y;
		end_z = start.z;
	}

	//float start_z = min(start.z,end.z);
	//float end_z = max(start.z,end.z);
	
	//ROS_INFO("Z intersections");
	for (unsigned int i = 0; i < this->m_cell_coords_z.size() ; i++){
		if (this->m_cell_coords_z[i] > start_z && m_cell_coords_z[i] < end_z){
			Intersection in;
			in.m_z = m_cell_coords_z[i];
			in.m_t = (in.m_z - start_z)/(end_z - start_z);
			in.m_x = (1 - in.m_t) * start_x + in.m_t * end_x;
			in.m_y = (1 - in.m_t) * start_y + in.m_t * end_y;
			
			float x_cell = (in.m_x - this->m_min_x) / this->m_cell_size;
			float y_cell = (in.m_y - this->m_min_y) / this->m_cell_size;	
			float z_cell = (in.m_z - this->m_min_z) / this->m_cell_size;

			//ROS_INFO("T: %4.2f",in.m_t);
			//ROS_INFO("Coords: %4.2f , %4.2f , %4.2f",in.m_x,in.m_y,in.m_z);
			//ROS_INFO("Cell: %4.2f , %4.2f , %4.2f",x_cell,y_cell,z_cell);

			in.m_idx_x = floor(x_cell);
			in.m_idx_y = floor(y_cell);		

			if (in.m_z > 0)
				in.m_idx_z = floor(z_cell) - 1;
			else
				in.m_idx_z = floor(z_cell);

			intersections.push_back(in);	
		}	
	}

	std::sort(intersections.begin(), intersections.end(), Intersection::SortIntersectionPredicate);
	
	for (unsigned int i = 0; i < intersections.size() ; i++){
		float l = 0;
		
		if (i == 0)
			l = intersections[i].m_t * tot_l;
		else
			l = (intersections[i].m_t - intersections[i-1].m_t) * tot_l;

		//ROS_INFO("I t: %4.2f",intersections[i].m_t);
		//ROS_INFO("I X: %d",intersections[i].m_idx_x);
		//ROS_INFO("I Y: %d",intersections[i].m_idx_y);
		//ROS_INFO("I Z: %d",intersections[i].m_idx_z);
		//ROS_INFO("idx: %d",intersections[i].m_idx_z * this->getNumCells(0) * this->getNumCells(1) + intersections[i].m_idx_y * this->getNumCells(0) + intersections[i].m_idx_x);

		idx->push_back(intersections[i].m_idx_z * this->getNumCells(0) * this->getNumCells(1) + intersections[i].m_idx_y * this->getNumCells(0) + intersections[i].m_idx_x);
		val->push_back(l);
	}
	
	//insert also the value in the cell containing the end of the beam
	float x_cell = (end.x - this->m_min_x) / this->m_cell_size;
	float y_cell = (end.y - this->m_min_y) / this->m_cell_size;
	float z_cell = (end.z - this->m_min_z) / this->m_cell_size;
	int last_x,last_y,last_z;

	//ROS_INFO("X cell: %4.2f",x_cell);
	//ROS_INFO("Y cell: %4.2f",y_cell);
	//ROS_INFO("Z cell: %4.2f",z_cell);

	if (x_cell> 0)
		last_x = floor(x_cell);
	else
		last_x = ceil(x_cell);

	if (y_cell> 0)
		last_y = floor(y_cell);
	else
		last_y = ceil(y_cell);

	if (z_cell> 0)
		last_z = floor(z_cell);
	else
		last_z = ceil(z_cell);

	//ROS_INFO("Last X: %d",last_x);
	//ROS_INFO("Last Y: %d",last_y);
	//ROS_INFO("Last Z: %d",last_z);
	//ROS_INFO("idx: %d",last_z * this->getNumCells(0) * this->getNumCells(1) + last_y * this->getNumCells(0) + last_x);
	
	idx->push_back(last_z * this->getNumCells(0) * this->getNumCells(1) + last_y * this->getNumCells(0) + last_x);

	if (intersections.size() == 0)
		val->push_back(tot_l);	
	else	
		val->push_back((1 - intersections[intersections.size()-1].m_t)*tot_l);

	//ROS_INFO("IDX:");
	//for (int i=0; i < idx->size(); i++){
	//	printf("%d\t",idx->at(i));
	//}	
	//printf("\n");

	//TODO: this needs to be removed
	for (int i=0; i < idx->size(); i++){
		if (idx->at(i) < 0 || idx->at(i) >= this->getNumCells(0) * this->getNumCells(1) * this->getNumCells(2)){
			ROS_INFO("Problem in parsing the measurement!");
			return false;
		}		
	}

	return true;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
void GasMap::ProjectEndpointsInBounds(geometry_msgs::Point *start, geometry_msgs::Point *end){
	if (start->x < this->m_min_x){
		float t = (this->m_min_x - start->x)/(end->x - start->x);
		if (t<0) t = -t;
		start->y = (1 - t) * start->y + t * end->y;
		start->z = (1 - t) * start->z + t * end->z;
		start->x = this->m_min_x;
	}

	if (start->x > this->m_max_x){
		float t = (start->x - this->m_max_x)/(end->x - start->x);
		if (t<0) t = -t;
		start->y = (1 - t) * start->y + t * end->y;
		start->z = (1 - t) * start->z + t * end->z;
		start->x = this->m_max_x;
	}

	if (end->x < this->m_min_x){
		float t = (this->m_min_x - start->x)/(end->x - start->x);
		if (t<0) t = -t;		
		end->y = (1 - t) * start->y + t * end->y;
		end->z = (1 - t) * start->z + t * end->z;
		end->x = this->m_min_x;
	}

	if (end->x > this->m_max_x){
		float t = (start->x - this->m_max_x)/(end->x - start->x);
		if (t<0) t = -t;
		end->y = (1 - t) * start->y + t * end->y;
		end->z = (1 - t) * start->z + t * end->z;
		end->x = this->m_max_x;
	}

	if (start->y < this->m_min_y){
		float t = (this->m_min_y - start->y)/(end->y - start->y);
		if (t<0) t = -t;
		start->x = (1 - t) * start->x + t * end->x;
		start->z = (1 - t) * start->z + t * end->z;
		start->y = this->m_min_y;
	}

	if (start->y > this->m_max_y){
		float t = (start->y - this->m_max_y)/(end->y - start->y);
		if (t<0) t = -t;
		start->x = (1 - t) * start->x + t * end->x;
		start->z = (1 - t) * start->z + t * end->z;
		start->y = this->m_max_y;
	}

	if (end->y < this->m_min_y){
		float t = (this->m_min_y - start->y)/(end->y - start->y);
		if (t<0) t = -t;
		end->x = (1 - t) * start->x + t * end->x;
		end->z = (1 - t) * start->z + t * end->z;
		end->y = this->m_min_y;
	}

	if (end->y > this->m_max_y){
		float t = (start->y - this->m_max_y)/(end->y - start->y);
		if (t<0) t = -t;
		end->x = (1 - t) * start->x + t * end->x;
		end->z = (1 - t) * start->z + t * end->z;
		end->y = this->m_max_y;
	}

	if (start->z < this->m_min_z){
		float t = (this->m_min_z - start->z)/(end->z - start->z);
		if (t<0) t = -t;
		start->y = (1 - t) * start->y + t * end->y;
		start->x = (1 - t) * start->x + t * end->x;
		start->z = this->m_min_z;
	}

	if (start->z > this->m_max_z){
		float t = (start->z - this->m_max_z)/(end->z - start->z);
		if (t<0) t = -t;
		start->y = (1 - t) * start->y + t * end->y;
		start->x = (1 - t) * start->x + t * end->x;
		start->z = this->m_max_z;
	}

	if (end->z < this->m_min_z){
		float t = (this->m_min_z - start->z)/(end->z - start->z);
		if (t<0) t = -t;
		end->y = (1 - t) * start->y + t * end->y;
		end->x = (1 - t) * start->x + t * end->x;
		end->z = this->m_min_z;
	}

	if (end->z > this->m_max_z){
		float t = (start->z - this->m_max_z)/(end->z - start->z);
		if (t<0) t = -t;
		end->y = (1 - t) * start->y + t * end->y;
		end->x = (1 - t) * start->x + t * end->x;
		end->z = this->m_max_z;
	}

}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
bool GasMap::updateMap(std::vector<real_t> *new_concentrations){
	
	if (new_concentrations->size() != this->m_map.size()){
		ROS_INFO("Dimension of the map does not match with dimension of calculated concentrations");
		return 1;
	}
	for (unsigned int i = 0; i < new_concentrations->size(); i++){
		float conc = new_concentrations->at(i);
		unsigned char idx_r = (unsigned char)round(conc / this->m_max_conc * 64.0); 
		unsigned char idx_g = (unsigned char)round(conc / this->m_max_conc * 64.0); 
		unsigned char idx_b = (unsigned char)round(conc / this->m_max_conc * 64.0);

		idx_r = min(idx_r,(unsigned char)63);
		idx_g = min(idx_g,(unsigned char)63);
		idx_b = min(idx_b,(unsigned char)63);
		this->m_map[i].setColor(red[idx_r],green[idx_g],blue[idx_b]);
		this->m_map[i].setConcentration(min(conc / this->m_max_conc,0.99f));
	}

	return 0;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
void GasMap::publish(ros::Publisher &tdlas_advertise){
	vector<Cell>::iterator it;
	visualization_msgs::MarkerArray array;
	int aux=0;

	array.markers.clear();
	
	sensor_msgs::PointCloud2 cloud_pub;
	pcl::PointCloud<pcl::PointXYZRGB> acc_cloud;

	for(it=this->m_map.begin();it!=this->m_map.end();it++ ){
		if (it->getConcentration() >0){
			pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
			int idx = floor((float)NUM_CELL_TEMPLATES * it->getConcentration());
			temp_cloud = template_cells[idx];		

			for (size_t i = 0; i < temp_cloud.points.size(); i++)
			{
		    		temp_cloud.points[i].x += it->getX();
		    		temp_cloud.points[i].y += it->getY();
		    		temp_cloud.points[i].z += it->getZ();
			}
		
			if(aux==0){
				acc_cloud=temp_cloud;
				aux++;
			}
			else{
				acc_cloud+=temp_cloud;
			}
		}
		
	 }
	 
	pcl::toROSMsg(acc_cloud, cloud_pub);	
	cloud_pub.header.frame_id = this->m_frame.c_str(); 
	cloud_pub.header.stamp = ros::Time::now();
	tdlas_advertise.publish(cloud_pub);	
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
void GasMap::printInfo(){
	printf("-------------------------------------\n");
	printf("Minimum x coordinate: %4.2f\n",this->m_min_x);
	printf("Maximum x coordinate: %4.2f\n",this->m_max_x);
	printf("Minimum y coordinate: %4.2f\n",this->m_min_y);
	printf("Maximum y coordinate: %4.2f\n",this->m_max_y);
	printf("Minimum z coordinate: %4.2f\n",this->m_min_z);
	printf("Maximum z coordinate: %4.2f\n",this->m_max_z);
	printf("Cell size: %4.2f\n",this->m_cell_size);	
	printf("Number of cells x dimension: %d\n",this->getNumCells(0));	
	printf("Number of cells y dimension: %d\n",this->getNumCells(1));
	printf("Number of cells z dimension: %d\n",this->getNumCells(2));
	printf("-------------------------------------\n");
	printf("Cell x coordinates:\n");
	for (unsigned int i = 0; i < this->m_cell_coords_x.size(); i++){
		printf("%4.2f\t",this->m_cell_coords_x[i]);	
	}
	printf("\nCell y coordinates:\n");
	for (unsigned int i = 0; i < this->m_cell_coords_y.size(); i++){
		printf("%4.2f\t",this->m_cell_coords_y[i]);	
	}
	printf("\nCell z coordinates:\n");
	for (unsigned int i = 0; i < this->m_cell_coords_z.size(); i++){
		printf("%4.2f\t",this->m_cell_coords_z[i]);	
	}
	printf("\n");
}
//---------------------------------------------------------------------------------------------------------

void GasMap::printMapData(){
	vector<Cell>::iterator it;
	printf("\n-------------------------------------------------------");
	for(it=this->m_map.begin();it!=this->m_map.end();it++ ){
		float connc=it->getConcentration();
		connc=connc*MAX_CONC2;
		printf("\n %f,%f,%f,%f",it->getX(),it->getY(),it->getZ(),connc);
	}
	printf("\n");
	//for (unsigned int i=0; i < this->m_cell_coords_x.size();i++){
	//	printf("\n (x,y,z),conc %f",this->m_map[i].getConcentration());
	//}
}






