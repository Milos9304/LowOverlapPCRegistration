#include "icp.h"

Eigen::Matrix4f registerPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pcA, pcl::PointCloud<pcl::PointXYZ>::Ptr pcB){

        Registrator::Ptr registrator (new Registrator());
        registrator->setNumKSearchNeighbors(100);
        registrator->setDescriptorRadius(1.2);
        registrator->setSubsamplingRadius(0.2);
        registrator->setConsensusInlierThreshold(0.2);
        registrator->setConsensusMaxIterations(120);
        registrator->setICPMaxIterations(200);
        registrator->setICPMaxCorrespondenceDistance(0.07);
        registrator->setResidualThreshold(0.1);
        registrator->setTargetCloud(pcA);
        registrator->setSourceCloud(pcB);

        return registrator->performRegistration("both"); //correspondence / icp / both

}
