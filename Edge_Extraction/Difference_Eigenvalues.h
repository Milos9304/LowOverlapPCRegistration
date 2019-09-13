#ifndef HEADERDIFFEIGEN_H
#define HEADERDIFFEIGEN_H

//#include <tuple>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

//std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr*, int> extract_edges(char*);
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr&, const std::string&, bool = false);


#endif
