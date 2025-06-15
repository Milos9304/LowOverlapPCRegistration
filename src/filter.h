#ifndef FILTERH
#define FILTERH

#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/median_filter.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/convolution_3d.h>

void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string, bool);
void applyGaussianKernel(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string, bool);

#endif
