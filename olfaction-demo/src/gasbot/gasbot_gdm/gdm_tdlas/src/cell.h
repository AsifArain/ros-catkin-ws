
#ifndef _CELL_H
#define _CELL_H

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
//#include "pcl/ros/conversions.h" //asif
//#include <pcl/point_cloud.h> //asif
//#include </opt/ros/indigo/include/pcl_ros/io/pcd_io.h>  //asif
#include <pcl_ros/io/pcd_io.h>
//#include <pcd_io.h>
#include <pcl/point_types.h>
//#include 	"pcl/common/transform.h"
//#include 	"LinearMath/btTransform.h"

// Asif
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>



#define CELL_DEFAULT_R 0.0		// default RED component
#define CELL_DEFAULT_G 0.0		// default GREEN component
#define CELL_DEFAULT_B 0.0		// default BLUE component

namespace GdmTDLAS{

class Cell{
	private:
		float size;				// radius of the cell
		float x;				// x position
		float y;				// y position
		float z;				// z position
		float conc;				// gas concentration
		float r;				// red color component
		float g;				// green color component
		float b;				// blue color component
	
		//geometry_msgs::Point vertex[8];		// vertices constituting the cell
	public:
		/**
		 * Constructor
		 * @param x x position (metrical coordinates)
		 * @param y y position (metrical coordinates)
		 * @param z z position (metrical coordinates)
		 * @param size size of the cell
		 */
		Cell(float x, float y, float z, float size);
		/**
		 * Sets the color of the cell to be visualized
		 * @param r red component
		 * @param g green component
		 * @param b blue component
		 * @return integer code, typically zero
		 */
		int setColor(float r, float g, float b);
		/**
		 * Sets the concentration of a cell
		 * @param conc gas concentration
		 * @return integer code, typically zero
		 */
		int setConcentration(float conc);
		/**
		 * Gets the concentration of a cell
		 * @return concentration of the cell
		 */
		float getConcentration();
		/**
		 * Gets the x coordinate of the center of a cell
		 * @return x coordinate
		 */
		float getX();
		/**
		 * Gets the y coordinate of the center of a cell
		 * @return y coordinate
		 */
		float getY();
		/**
		 * Gets the z coordinate of the center of a cell
		 * @return z coordinate
		 */
		float getZ();
		/**
		 * Printing function
		 */
		void print();

		/**
		 * Publishes the point cloud associated to the cell
		 * @param point_cloud
		 */
		//void publish(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);
};

}

#endif
