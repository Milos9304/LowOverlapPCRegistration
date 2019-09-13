#ifndef FILTERH
#define FILTERH

#include <pcl/io/ply_io.h>
//#include <pcl/filters/median_filter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/statistical_outlier_removal.h>


void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string, bool);
void applyGaussianKernel(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string, bool);

#endif
