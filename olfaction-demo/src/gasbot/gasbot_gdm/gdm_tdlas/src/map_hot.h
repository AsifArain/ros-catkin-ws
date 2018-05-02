#ifndef _MAP_H
#define _MAP_H

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include "cell.h"
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

namespace GdmTDLAS{

class GasMap{

	private:
		/** \brief minumum x coordinate of the map */
		float m_min_x;
		/** \brief maximum x coordinate of the map */
		float m_max_x;
		/** \brief minumum y coordinate of the map */
		float m_min_y;
		/** \brief maximum y coordinate of the map */
		float m_max_y;
		/** \brief size of the cell of the map */
		float m_cell_size;
		/** \brief maximum gas concentration (useful for visualization purposes)*/
		float m_max_conc;
		/** \brief number of rows of the map */
		int m_rows;
		/** \brief number of the columns of the map */
		int m_cols;
		/** \brief Vector representing the floor */
		std::vector<float> m_cell_coords_x;
		/** \brief Vector representing the floor */
		std::vector<float> m_cell_coords_y;
		/** \brief Vector representing the floor */
		std::vector<Cell> m_map;
		/** \brief Coordinate frame of the map */
		std::string m_frame;

	public:
		/** \brief Default constructor */
		GasMap(float min_x, float max_x, float min_y, float max_y, float cell_size, float max_conc, std::string frame);
		/**
		 * Getter method
		 * @return number of rows
		 */
		int getRows();
		/**
		 * Getter method
		 * @return number of columns
		 */
		int getCols();
		/**
		 * Sets the color of the cell to be visualized
		 * @param start beam start point
		 * @param end beam end point
		 * @param row vector containing the length traversed in each cell by the measurement at hand
		 * @param reading from the TDLAS sensor
		 * @return boolean value, TRUE success - FALSE failure
		 */
		bool CalculateNewMeasurementRow(geometry_msgs::Point start, geometry_msgs::Point end, float ppmm, std::vector<int> *idx, std::vector<float> *val);
		/**
		 * Updates the map according to the new model
		 * @param array of concentrations
		 * @return boolean value, TRUE success - FALSE failure
		 */
		bool updateMap(std::vector<real_t> *new_concentrations);		
		/**
		 * Publishes the map for visualization in a given frame
		 * @param array
		 * @param frame 
		 */
		void publish(ros::Publisher &tdlas_advertise);
		/**
		 * Print at screen information about the map 
		 */
		void printInfo();
};

class Intersection{
	public:
		float m_x;
		float m_y;
		float m_t;
		int m_idx_row;
		int m_idx_col;
		static bool SortIntersectionPredicate(const Intersection& i1, const Intersection& i2){return i1.m_t < i2.m_t;}
};

//colormap: JET
//static const float red[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0625,0.1250,0.1875,0.2500,0.3125,0.3750,0.4375,0.5000,0.5625,0.6250,0.6875,0.7500,0.8125,0.8750,0.9375,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,0.9375,0.8750,0.8125,0.7500,0.6875,0.6250,0.5625,0.5000};

//static const float green[64] = {0,0,0,0,0,0,0,0,0.0625,0.1250,0.1875,0.2500,0.3125,0.3750,0.4375,0.5000,0.5625,0.6250,0.6875,0.7500,0.8125,0.8750,0.9375,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,0.9375,0.8750,0.8125,0.7500,0.6875,0.6250,0.5625,0.5000,0.4375,0.3750,0.3125,0.2500,0.1875,0.1250,0.0625,0,0,0,0,0,0,0,0,0};

//static const float blue[64] = {0.5625,0.6250,0.6875,0.7500,0.8125,0.8750,0.9375,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,0.9375,0.8750,0.8125,0.7500,0.6875,0.6250,0.5625,0.5000,0.4375,0.3750,0.3125,0.2500,0.1875,0.1250,0.0625,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//colormap: HOT 
static const float red[64] = {1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,0.9583,0.9167,0.8750,0.8333,0.7917,0.7500,0.7083,0.6667,0.6250,0.5833,0.5417,0.5000,0.4583,0.4167,0.3750,0.3333,0.2917,0.2500,0.2083,0.1667,0.1250,0.0833,0.0417};

static const float green[64] = {1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,0.9583,0.9167,0.8750,0.8333,0.7917,0.7500,0.7083,0.6667,0.6250,0.5833,0.5417,0.5000,0.4583,0.4167,0.3750,0.3333,0.2917,0.2500,0.2083,0.1667,0.1250,0.0833,0.0417,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static const float blue[64] = {1.0000,0.9375,0.8750,0.8125,0.7500,0.6875,0.6250,0.5625,0.5000,0.4375,0.3750,0.3125,0.2500,0.1875,0.1250,0.0625,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
}


#endif
