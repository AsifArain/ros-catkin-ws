#include "map.h"
#include "math.h"

using namespace std;
using namespace GdmTDLAS;

//==========================================================================================================
GasMap::GasMap(float min_x, float max_x, float min_y, float max_y, float cell_size, float max_conc, std::string frame){

	this->m_min_x = min_x;
	this->m_max_x = max_x;
	this->m_min_y = min_y;
	this->m_max_y = max_y;
	this->m_cell_size = cell_size;
	this->m_max_conc = max_conc;

	this->m_rows = ceil((max_y - min_y) / cell_size);
	this->m_cols = ceil((max_x - min_x) / cell_size);

	this->m_cell_coords_y.clear();
	for (int i = 0; i < this->m_rows; i++){
		this->m_cell_coords_y.push_back(min_y + i*cell_size);
	}

	this->m_cell_coords_x.clear();
	for (int i = 0; i < this->m_cols; i++){
		this->m_cell_coords_x.push_back(min_x + i*cell_size);
	}

	//build the data structure
	this->m_map.clear();
	for (int j = 0; j < this->m_cols; j++){
		for (int i = 0; i < this->m_rows; i++){	
			float x_center = min_x + j*cell_size + cell_size / 2;
			float y_center = min_y + i*cell_size + cell_size / 2;
			this->m_map.push_back(Cell(x_center,y_center,cell_size));
		}
	}

	this->m_frame = frame;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
int GasMap::getRows(){
	return this->m_rows;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
int GasMap::getCols(){
	return this->m_cols;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
bool GasMap::CalculateNewMeasurementRow(geometry_msgs::Point start, geometry_msgs::Point end, float ppmm, std::vector<int> *idx, std::vector<float> *val){
	float start_x = min(start.x,end.x);
	float end_x = max(start.x,end.x);	
	vector<Intersection> intersections;

	float tot_l = sqrt(pow(end.x - start.x,2) + pow(end.y - start.y,2));
	
	if (tot_l == 0 || ppmm == -1){
		ROS_INFO("Beam length equal to zero!");
		return false;
	}
	
	if (start.x < m_min_x || end.x < m_min_x || start.x > m_max_x || end.x > m_max_x || start.y < m_min_y || end.y < m_min_y || start.y > m_max_y || end.y > m_max_y){
		ROS_INFO("Measurement out of bounds!");
		return false;
	}

	intersections.clear();	

	for (unsigned int i = 0; i < this->m_cell_coords_x.size() ; i++){
		if (this->m_cell_coords_x[i] > start_x && m_cell_coords_x[i] < end_x){
			Intersection in;
			in.m_x = m_cell_coords_x[i];
			in.m_t = (in.m_x - start.x)/(end.x - start.x);
			in.m_y = (1 - in.m_t) * start.y + in.m_t * end.y;
			
			float x_cell = (in.m_x - this->m_min_x) / this->m_cell_size;
			float y_cell = (in.m_y - this->m_min_y) / this->m_cell_size;			

			in.m_idx_row = floor(y_cell);

			if (in.m_x > 0)
				in.m_idx_col = floor(x_cell) - 1;
			else
				in.m_idx_col = floor(x_cell );

			intersections.push_back(in);	
		}	
	}

	float start_y = min(start.y,end.y);
	float end_y = max(start.y,end.y);

	for (unsigned int i = 0; i < this->m_cell_coords_y.size() ; i++){
		if (this->m_cell_coords_y[i] > start_y && m_cell_coords_y[i] < end_y){
			Intersection in;
			in.m_y = m_cell_coords_y[i];
			in.m_t = (in.m_y - start.y)/(end.y - start.y);
			in.m_x = (1 - in.m_t) * start.x + in.m_t * end.x;
			
			float x_cell = (in.m_x - this->m_min_x) / this->m_cell_size;
			float y_cell = (in.m_y - this->m_min_y) / this->m_cell_size;	

			in.m_idx_col = floor(x_cell);		

			if (in.m_y > 0)
				in.m_idx_row = floor(y_cell) - 1;
			else
				in.m_idx_row = floor(y_cell);

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

		idx->push_back(intersections[i].m_idx_col * this->getRows() + intersections[i].m_idx_row);
		val->push_back(l);
	}
	
	//insert also the value in the cell containing the end of the beam
	float x_cell = (end.x - this->m_min_x) / this->m_cell_size;
	float y_cell = (end.y - this->m_min_y) / this->m_cell_size;
	int last_col,last_row;

	if (x_cell> 0)
		last_col = floor(x_cell);
	else
		last_col = ceil(x_cell);

	if (y_cell> 0)
		last_row = floor(y_cell);
	else
		last_row = ceil(y_cell);
	
	idx->push_back(last_col * this->getRows() + last_row);

	if (intersections.size() == 0)
		val->push_back(tot_l);	
	else	
		val->push_back((1 - intersections[intersections.size()-1].m_t)*tot_l);
	
	return true;
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
bool GasMap::updateMap(std::vector<real_t> *new_concentrations){
	
	if (new_concentrations->size() != this->m_map.size())
		return 1;

	for (unsigned int i = 0; i < new_concentrations->size(); i++){
		float conc = new_concentrations->at(i);
		unsigned char idx_r = (unsigned char)round(conc / this->m_max_conc * 64.0); 
		unsigned char idx_g = (unsigned char)round(conc / this->m_max_conc * 64.0); 
		unsigned char idx_b = (unsigned char)round(conc / this->m_max_conc * 64.0);

		idx_r = min(idx_r,(unsigned char)63);
		idx_g = min(idx_g,(unsigned char)63);
		idx_b = min(idx_b,(unsigned char)63);
		this->m_map[i].setColor(red[idx_r],green[idx_g],blue[idx_b]);
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

	for(it=this->m_map.begin();it!=this->m_map.end();it++ ){

   		visualization_msgs::Marker marker;
		marker.header.frame_id = this->m_frame.c_str();
    		marker.header.stamp = ros::Time::now();
		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		marker.lifetime = ros::Duration();
		marker.id=aux++;

		it->publish(marker);
		array.markers.push_back(marker);
	 }

	tdlas_advertise.publish(array);

	
}
//---------------------------------------------------------------------------------------------------------

//==========================================================================================================
void GasMap::printInfo(){
	printf("-------------------------------------\n");
	printf("Minimum x coordinate: %4.2f\n",this->m_min_x);
	printf("Maximum x coordinate: %4.2f\n",this->m_max_x);
	printf("Minimum y coordinate: %4.2f\n",this->m_min_y);
	printf("Maximum y coordinate: %4.2f\n",this->m_max_y);
	printf("Cell size: %4.2f\n",this->m_cell_size);	
	printf("Number of rows: %d\n",this->m_rows);	
	printf("Number of columns: %d\n",this->m_cols);	
	printf("-------------------------------------\n");
	printf("Cell x coordinates:\n");
	for (unsigned int i = 0; i < this->m_cell_coords_x.size(); i++){
		printf("%4.2f\t",this->m_cell_coords_x[i]);	
	}
	printf("\nCell y coordinates:\n");
	for (unsigned int i = 0; i < this->m_cell_coords_y.size(); i++){
		printf("%4.2f\t",this->m_cell_coords_y[i]);	
	}
	printf("\n");
}
//---------------------------------------------------------------------------------------------------------





