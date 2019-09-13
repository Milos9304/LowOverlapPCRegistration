#ifndef RESWRITERH
#define RESWRITERH

#include <iostream>
#include <fstream>
//#include <vector>
//#include <cmath>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

//#include "line_descriptor.h"

class ResultWriter{

    public:
        
        ResultWriter(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);
        void transformDynamicPointCloud(Eigen::Matrix4f);
        void saveTransformedPointClouds(std::string);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getStaticPointCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getDynamicPointCloud();


    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcA;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcB;

};

#endif
