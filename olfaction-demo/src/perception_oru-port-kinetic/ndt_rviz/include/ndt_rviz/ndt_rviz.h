#pragma once

#include <ndt_map/ndt_map.h>
#include <ndt_mcl/3d_ndt_mcl.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <ndt_rviz/utils.h>

namespace ndt_visualisation {

  // Contains a set of useful functions to generate markers from NDT related classes.

  // Some helper functions to convert to the position etc. utilized in the markers.
  inline geometry_msgs::Point toPointFromTF (const tf::Vector3& p)
  {
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    return pt;
  }
  
  inline geometry_msgs::Point toPointFromEigen (const Eigen::Vector3d &p) 
  {
    geometry_msgs::Point pt;
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    return pt;
  }

  void assignDefault(visualization_msgs::Marker &m)
  {
    m.header.frame_id = "/world";
    m.scale.x = 1;
    m.scale.y = 1;
    m.scale.z = 1;
    m.lifetime = ros::Duration(60.);
  }
  
  void assignColor(visualization_msgs::Marker &m, int color)
  {
    double r,g,b = 0.;
    int color_mod = color % 3;
    
    switch (color_mod)
      {
      case 0:
      r = 1.; g = 0.; b = 0.;
      break;
      case 1:
	r = 0.; g = 1.; b = 0.;
	break;
      case 2:
	r = 0.; g = 0.; b = 1.;
	break;
	
      default:
	r = 0.5; g = 0.5; b = 0.5;
	break;
      };
    
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.3;
    
    if (color < 0) {
      m.color.r = 1.;
      m.color.g = 1.;
      m.color.b = 1.;
      m.color.a = 1.;
    }
  }
  
  /* typedef std::vector<std_msgs::ColorRGBA> ColorVec; */
  /* ColorVec initColors () */
  /* { */
  /*   ColorVec colors(2); */
  /*   colors[0].r = 0.5; */
  /*   colors[0].g = 1.0; */
  /*   colors[0].a = 1.0; */
  /*   colors[1].r = 1.0; */
  /*   colors[1].g = 1.0; */
  /*   colors[1].a = 1.0; */
  /*   return colors; */
  /* } */

  // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
  inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells/*, tf::Pose& pose*/, const visualization_msgs::Marker &marker)
    {
      visualization_msgs::Marker m = marker;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

        //	cells[i]->rescaleCovariance(); // needed?

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(p1));
	  m.points.push_back(toPointFromEigen(p2));

	  //	  tf::Vector3 p1_, p2_;
	  //	  tf::vectorEigenToTF (p1, p1_);
	  //	  tf::vectorEigenToTF (p2, p2_);
	  //	  m.points.push_back(toPointFromTF(pose*p1_));
	  //	  m.points.push_back(toPointFromTF(pose*p2_));
	}
      }
      return m;
      
    }

 // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells, const Eigen::Affine3d &pose, const visualization_msgs::Marker &marker)
    {
      visualization_msgs::Marker m = marker;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(pose*p1));
	  m.points.push_back(toPointFromEigen(pose*p2));
	}
      }
      return m;
      
    }

   inline visualization_msgs::Marker markerNDTCells (std::vector<lslgeneric::NDTCell*> cells) 
    {
      visualization_msgs::Marker m;
      assignDefault(m);
      m.ns = "NDTCells";
      return markerNDTCells(cells, m);
    }

inline visualization_msgs::Marker markerNDTCells( lslgeneric::NDTMap &map, int id, const std::string &name) {
    visualization_msgs::Marker m;
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    visualization_msgs::Marker ret = markerNDTCells(cells, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
    return  ret;
}

inline visualization_msgs::Marker markerNDTCells( lslgeneric::NDTMap &map, const Eigen::Affine3d &pose, int id, const std::string &name) {
    visualization_msgs::Marker m;
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    visualization_msgs::Marker ret = markerNDTCells(cells, pose, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
    return  ret;
}

  inline visualization_msgs::Marker markerNDTCells (lslgeneric::NDTMap &map, int id) 
  {
    return  markerNDTCells(map, id, std::string("NDTMap"));
  }

                                                  
 // Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
void markerNDTCells2 (std::vector<lslgeneric::NDTCell*> cells, const Eigen::Affine3d &pose, visualization_msgs::Marker &m)
    {
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.02;

      for (size_t i = 0; i < cells.size(); i++)	{
	if(!cells[i]->hasGaussian_) continue;

	Eigen::Vector3d mean = cells[i]->getMean();
	Eigen::Matrix3d evecs = cells[i]->getEvecs();
	Eigen::Vector3d evals = cells[i]->getEvals();
	for (size_t j = 0; j < 3; j++) {
	  double scale = evals(j);
	  if (scale < 0.0001)
	    continue;
	  scale = sqrt(scale);
	  Eigen::Vector3d offset = evecs.col(j) * scale;
	  Eigen::Vector3d p1 = mean - offset;
	  Eigen::Vector3d p2 = mean + offset;
	  m.points.push_back(toPointFromEigen(pose*p1));
	  m.points.push_back(toPointFromEigen(pose*p2));
	}
      }
    }

void markerNDTCells2( lslgeneric::NDTMap &map, const Eigen::Affine3d &pose, int id, const std::string &name, visualization_msgs::Marker &m) {
    assignDefault(m);
    assignColor(m, id);
    m.id = id;
    m.ns = name;
    std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
    markerNDTCells2(cells, pose, m);
    for (std::vector<lslgeneric::NDTCell*>::const_iterator it = cells.begin(); it != cells.end(); it++) {
      delete *it;
    }
}

  //! Draw correspondance lines between the NDTCells
  inline visualization_msgs::Marker markerCellVectorCorrespondances (lslgeneric::NDTMap &map1, lslgeneric::NDTMap &map2, const std::vector<std::pair<int, int> > &corr) 
    {
      visualization_msgs::Marker m;
      assignDefault(m);
      m.ns = "NDTCellCorrs";
      m.id = 1;
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.scale.x = 0.005;
      m.color.r = m.color.a = 1.0;
      m.color.g = 0.2;

      for (size_t i = 0; i < corr.size(); i++) {
	
	lslgeneric::NDTCell* cell1 = map1.getCellIdx(corr[i].first);
	lslgeneric::NDTCell* cell2 = map2.getCellIdx(corr[i].second);
	if (cell1 == NULL || cell2 == NULL) {
	  ROS_WARN("Failed to get cells using NDTMap::getCellIdx()!");
	  continue;
	}
	m.points.push_back(toPointFromEigen(cell1->getMean()));
	m.points.push_back(toPointFromEigen(cell2->getMean()));
      }
      return m;
  }

// Code from user skohlbrecher
void drawCovariance(const Eigen::Vector2d& mean, const Eigen::Matrix2d& covMatrix, visualization_msgs::Marker &marker)
{
     marker.pose.position.x = mean[0];
     marker.pose.position.y = mean[1];
     
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covMatrix);
     
     const Eigen::Vector2d& eigValues (eig.eigenvalues());
     const Eigen::Matrix2d& eigVectors (eig.eigenvectors());
     
     float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

     marker.type = visualization_msgs::Marker::CYLINDER;
     
    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);
    
    marker.scale.x = lengthMajor;
    marker.scale.y = lengthMinor;
    marker.scale.z = 0.001;
    
    marker.pose.orientation.w = cos(angle*0.5);
    marker.pose.orientation.z = sin(angle*0.5);
}

void drawCovariance(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covMatrix, visualization_msgs::Marker &marker)
{
     Eigen::Vector2d tmp_mean = mean.head(2);
     Eigen::Matrix2d tmp_cov = covMatrix.block(0,0,2,2);
     drawCovariance(tmp_mean, tmp_cov, marker);
}

inline visualization_msgs::Marker markerMeanCovariance2d(const Eigen::Vector3d &mean, const Eigen::Matrix3d &cov, double scale, int id, int color) {
  visualization_msgs::Marker m;
  assignDefault(m);
  m.ns = "mean_cov";
  m.id = id;
  m.color.r = m.color.a = 1.0;
  m.color.g = 0.2;
  if (color >= 0) {
    assignColor(m, color);
  }
  Eigen::MatrixXd cov_scaled = cov*scale;
  
  drawCovariance(mean, cov_scaled, m);
  return m;
}

visualization_msgs::Marker markerParticlesNDTMCL3D(const NDTMCL3D &mcl, int color, const std::string& ns) {
  visualization_msgs::Marker m;
  assignDefault(m);
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.005;
  m.color.g = m.color.a = 1.0;
  m.color.r = 0.6; m.color.b = 0.8;
  if (color >= 0) {
    assignColor(m, color);
  }
  for (unsigned int i = 0; i < mcl.pf.size(); i++) {
    const Eigen::Affine3d &T = mcl.pf.pcloud[i].T;
    m.points.push_back(toPointFromEigen(T.translation()));
    // Length of the line -> 0.3.
    Eigen::Vector3d p2 = T*Eigen::Vector3d(0.3, 0., 0.);
    m.points.push_back(toPointFromEigen(p2));
  }  
  return m;
}


} // namespace

