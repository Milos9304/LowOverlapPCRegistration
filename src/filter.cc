#include "filter.h"

//Statistical filter parameters
#define MEAN_K 150//50 //number of nearest neighbors
#define STD_DEV_MULT 0.05 //standard deviation multiplicator, see PCL docs

//Median filter parameters
#define STRIDE 0.001
#define WINDOW_SIZE 70

#define SIGMA 6

//#include <pcl/filters/voxel_grid_occlusion_estimation.h>
//#include <pcl/filters/shadowpoints.h>

void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputname, bool writeFile){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(MEAN_K);
    sor.setStddevMulThresh(STD_DEV_MULT);
    sor.filter(*cloud);

    if (!writeFile)
        return;

    pcl::PLYWriter writePLY;
    pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
    pcl::copyPointCloud(*cloud, cloudRGB);
    
    for (size_t i = 0; i < cloudRGB.points.size(); ++i) {
        cloudRGB.points[i].r = 255;
        cloudRGB.points[i].g = 0;
        cloudRGB.points[i].b = 0;
    }
  
    writePLY.write(outputname, cloudRGB,  true, false);
    std::cout << outputname << " written\n";

}

void applyGaussianKernel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputname, bool writeFile){

    /*pcl::MedianFilter<pcl::PointXYZ> mf;
    mf.setInputCloud(cloud);
    mf.setMaxAllowedMovement(STRIDE);
    mf.setWindowSize(WINDOW_SIZE);
    mf.filter(*cloud);

    */
    //Set up the KDTree 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>); 
    (*kdtree).setInputCloud(cloud);

    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
    (*kernel).setSigma(SIGMA);
    (*kernel).setThresholdRelativeToSigma(SIGMA);
    
    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> c3d;
    c3d.setKernel(*kernel);
    c3d.setInputCloud(cloud);
    c3d.setSearchMethod(kdtree);
    c3d.setRadiusSearch(0.03);//0.01			
    c3d.convolve(*cloud);
    
    if (!writeFile)
        return;

    pcl::PLYWriter writePLY;
    pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
    pcl::copyPointCloud(*cloud, cloudRGB);
    
    for (size_t i = 0; i < cloudRGB.points.size(); ++i) {
        cloudRGB.points[i].r = 255;
        cloudRGB.points[i].g = 0;
        cloudRGB.points[i].b = 0;
    }
  
    writePLY.write(outputname, cloudRGB,  true, false);
    std::cout << outputname << " written\n";
} 
