#include "resultWriter.h"

ResultWriter::ResultWriter(pcl::PointCloud<pcl::PointXYZ>& pcA, pcl::PointCloud<pcl::PointXYZ>& pcB){

    this -> pcA = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this -> pcB = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
   
    pcl::copyPointCloud<pcl::PointXYZ>(pcA, *(this -> pcA));
    pcl::copyPointCloud<pcl::PointXYZ>(pcB, *(this -> pcB));

}

void ResultWriter::transformDynamicPointCloud(Eigen::Matrix4f T){

    /*Apply transformation T on PointCloud B*/
    pcl::transformPointCloud(*(this -> pcB), *(this -> pcB), T);
}

void ResultWriter::saveTransformedPointClouds(std::string filename){

    pcl::PointCloud<pcl::PointXYZRGB> pcAColored;
    pcl::copyPointCloud(*(this -> pcA), pcAColored);
    for(auto& point: pcAColored.points){
        point.r = 255;
        point.g =   0;
        point.b =   0;
    }

    pcl::PointCloud<pcl::PointXYZRGB> pcBColored;
    pcl::copyPointCloud(*(this -> pcB), pcBColored);
    for(auto& point: pcBColored.points){
        point.r = 255;
        point.g = 255;
        point.b = 255;
    }

    pcl::PointCloud<pcl::PointXYZRGB> mergedCloud = pcAColored;
    mergedCloud += pcBColored;

    pcl::PLYWriter writer;
    writer.write(filename, mergedCloud, true, false);

}

pcl::PointCloud<pcl::PointXYZ>::Ptr ResultWriter::getStaticPointCloud(){

    return this -> pcA;

} 

pcl::PointCloud<pcl::PointXYZ>::Ptr ResultWriter::getDynamicPointCloud(){

    return this -> pcB;

}
