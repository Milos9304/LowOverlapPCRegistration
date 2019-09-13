#ifndef ICPH
#define ICPH

#include <Eigen/Dense>
#include "Registrator.hpp"

Eigen::Matrix4f registerPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);

#endif
