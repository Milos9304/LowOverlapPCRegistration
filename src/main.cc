#include <chrono>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>

#include "../Edge_Extraction/Difference_Eigenvalues.h"
#include "../hough3d-code/hough3dlines.h"
#include "../hough3d-code/vector3d.h"
#include "icp.h"
#include "filter.h"
#include "findTransformation.h"
#include "intersection.h"
#include "resultWriter.h"
#include "line_descriptor.h"
#include "boundingBox.h"

#include <pcl/segmentation/cpc_segmentation.h>


using namespace boost::system;
namespace filesys = boost::filesystem;

bool calculateOverlappingRatio = false;

int main(int argc, char *argv[]){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
   
    const std::string input_fileA = argv[1];
    const std::string input_fileB = argv[2];
    const std::string edges_output_file = argv[3];
    const std::string lines_output_file = argv[4];
    const std::string filtered_statistical_output_file = argv[5];
    const std::string filtered_median_output_file = argv[6];
    const std::string merged_output_file = argv[7];

    if( pcl::io::loadPLYFile (input_fileA, *cloudA) == -1 ){
    
        std::cerr << "File not found\n";
        return -1;
    }

    std::cout << input_fileA << " loaded.\n";

    if(  pcl::io::loadPLYFile (input_fileB, *cloudB) == -1 ){
    
        std::cerr << "File not found\n";
        return -1;
    }

    std::cout << input_fileB << " loaded.\n";

    //copy original PointClouds to a buffer to use them in the final merging step
    ResultWriter resultWriter(*cloudA, *cloudB);

    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;
 
    //3D convolve
    begin = std::chrono::steady_clock::now();
       applyGaussianKernel(cloudA, filtered_median_output_file, true);    
       applyGaussianKernel(cloudB, filtered_median_output_file+"2.ply", true);  
    end = std::chrono::steady_clock::now();
    std::cout << "3D convolve:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;

    //Extract edges
    begin = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr extractedEdgesA = extract_edges(cloudA, edges_output_file, true);   
        pcl::PointCloud<pcl::PointXYZ>::Ptr extractedEdgesB = extract_edges(cloudB, edges_output_file+"2.ply", true);  
    end = std::chrono::steady_clock::now();
    std::cout << "Edge extraction:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;

    //Filter
    begin = std::chrono::steady_clock::now();
        applyStatisticalOutlierFilter(extractedEdgesA, filtered_statistical_output_file, true);    
        applyStatisticalOutlierFilter(extractedEdgesB, filtered_statistical_output_file+"2.ply", true);  
    end = std::chrono::steady_clock::now();
    std::cout << "Filtering:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;   begin = std::chrono::steady_clock::now();
    
    //Center pointcloud A on top of XY plane
    
        //Calculate bounding box
        const size_t sizeA =  extractedEdgesA -> points.size();

        BoundingBox bbA, bbB;
     
        bbA.setCoefficients(extractedEdgesA);

        const float xShift = (bbA.minX + bbA.maxX) / 2.0;
        const float yShift = (bbA.minY + bbA.maxY) / 2.0;
     
        std::cout << "Shifting by X: " << -xShift << " Y:" << -yShift << " Z:" << -bbA.minZ << std::endl; 
 
        //Center cloud A
        for(size_t i = 0; i < sizeA; ++i){
        
            extractedEdgesA -> points[i].x -= xShift;
            extractedEdgesA -> points[i].y -= yShift;
            extractedEdgesA -> points[i].z -= bbA.minZ;

         }

        //Center cloud B
        for(size_t i = 0; i < extractedEdgesB -> points.size(); ++i){
    
            extractedEdgesB -> points[i].x -= xShift;
            extractedEdgesB -> points[i].y -= yShift;
            extractedEdgesB -> points[i].z -= bbA.minZ;     

         }

        bbA.minX -= xShift;
        bbA.maxX -= xShift;
        bbA.minY -= yShift;
        bbA.maxY -= yShift;
        bbA.maxZ -= bbA.minZ;
        bbA.minZ  = 0;

    end = std::chrono::steady_clock::now();
    std::cout << "Centering pointclouds:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl; 

    //Hough transform
    begin = std::chrono::steady_clock::now();
        std::vector<Vector3d> pcA;
        std::vector<Vector3d> pcB;

        for (size_t i = 0; i < extractedEdgesA -> points.size (); ++i)
            pcA.push_back(Vector3d(extractedEdgesA->points[i].x, extractedEdgesA->points[i].y, extractedEdgesA->points[i].z));

         for (size_t i = 0; i < extractedEdgesB -> points.size (); ++i)
            pcB.push_back(Vector3d(extractedEdgesB->points[i].x, extractedEdgesB->points[i].y, extractedEdgesB->points[i].z));       

        Eigen::MatrixXf detected_linesA = hough_transform(pcA, lines_output_file, true);
        Eigen::MatrixXf detected_linesB = hough_transform(pcB, lines_output_file+"2.ply", true);

    end = std::chrono::steady_clock::now();
    std::cout << "Hough transform:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;

    //x y score
    std::tuple<float, float, float> maxTranslation = std::tuple<float, float, float>(0, 0, -1);
    float maximizingAngle;
    
    begin = std::chrono::steady_clock::now();
    std::vector<float> possibleRotations = findPossibleRotations(detected_linesA, detected_linesB);
    end = std::chrono::steady_clock::now();
    std::cout << "Rotation angle search:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;

    begin = std::chrono::steady_clock::now();
    for(const auto angle: possibleRotations){

       std::cout << "Trying angle " << angle << "..." << std::endl;    

        pcl::PointCloud<pcl::PointXYZ>::Ptr tempB(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*extractedEdgesB, *tempB, calculateTMatrix(angle, 0, 0));        
        
        bbB.setCoefficients(tempB);
   
        //bbA.print();
        //bbB.print();
 
        //USEFUL FOR DEBUGGING
        //pcl::PLYWriter w;
        //w.write("experimental1.ply", *extractedEdgesA, true, false);
        //w.write("experimental2.ply", *tempB, true, false);     

       std::tuple<float, float, float> translation = findTranslation(angle, detected_linesA, detected_linesB, bbA, bbB);//min_y, max_y, min_yB, max_yB ); 
        
        if(std::get<2>(maxTranslation) < std::get<2>(translation)){
            maxTranslation = translation;
            maximizingAngle = angle;
        }
 
    }
    end = std::chrono::steady_clock::now();
    std::cout << "Transformation search:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;

    float tX = std::get<0>(maxTranslation);
    float tY = std::get<1>(maxTranslation);
 
    std::cout << "Transformation parameters:    angle: " << maximizingAngle << "        X: " << tX << "         Y: " << tY << std::endl; 
    
    //PerformTransformation based on line matching
    
    resultWriter.saveTransformedPointClouds( merged_output_file+"original_config.ply" );    
 
    //BUG: sometimes these 2 lines need to be commented out, sometimes not. We suggest comparing to experimental2.ply to find out whether to comment them out or not.
    //resultWriter.transformDynamicPointCloud( calculateTMatrix(0, -xShift, -yShift, -bbA.minZ) );
    resultWriter.transformDynamicPointCloud( calculateTMatrix(maximizingAngle, 0, 0) );
    //resultWriter.transformDynamicPointCloud( calculateTMatrix(0, xShift, yShift, bbA.minZ) );    

    resultWriter.transformDynamicPointCloud( calculateTMatrix(0, 0, tY) ); 
    resultWriter.saveTransformedPointClouds( merged_output_file+"rotated_ty.ply" );   

    resultWriter.transformDynamicPointCloud( calculateTMatrix(0, tX, 0) ); 
    resultWriter.saveTransformedPointClouds( merged_output_file+"rotated_ty_tx.ply" ); 

    resultWriter.saveTransformedPointClouds( merged_output_file+"before_icp.ply" );    
 
    pcl::transformPointCloud(*extractedEdgesA, *extractedEdgesA, calculateTMatrix(0, xShift, yShift, bbA.minZ) );
    pcl::transformPointCloud(*extractedEdgesB, *extractedEdgesB, calculateTMatrix(maximizingAngle, tX + xShift, tY + yShift, bbA.minZ) );
   

    IntersectionInterface ii;
    ii.setStaticPointCloud(extractedEdgesA);
    ii.setDynamicPointCloud(extractedEdgesB);
    pcl::CropHull<pcl::PointXYZ> intersectionFilter = ii.getIntersectionFilter();

    pcl::PointCloud<pcl::PointXYZ>::Ptr iA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr iB(new pcl::PointCloud<pcl::PointXYZ>);

    intersectionFilter.setInputCloud( resultWriter.getStaticPointCloud() );
    intersectionFilter.filter(*iA);

    intersectionFilter.setInputCloud( resultWriter.getDynamicPointCloud() );
    intersectionFilter.filter(*iB);
 
    pcl::PLYWriter w;
    //USEFUL FOR DEBUGGING
    //w.write("hullA.ply", *iA, true, false);
    //w.write("hullB.ply", *iB, true, false);
    //std::cout << "hull files written" << std::endl; 

    std::cout << "Running ICP matching..." << std::endl;
    begin = std::chrono::steady_clock::now();
        Eigen::Matrix4f icpCorrectionTransformation = registerPointClouds(iA, iB);
    end = std::chrono::steady_clock::now();
    std::cout << "ICP matching finished:  " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "s" << std::endl;
    
    resultWriter.transformDynamicPointCloud( icpCorrectionTransformation );
    resultWriter.saveTransformedPointClouds( merged_output_file );    
    std::cout << "ICP corrected pointcloud written" << std::endl; 

    if(calculateOverlappingRatio){

        std::cout << "Calculating overlapping ratio..." << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr correctedEdgesB(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*extractedEdgesB, *correctedEdgesB, icpCorrectionTransformation);
 
        IntersectionInterface iiOverlap;
        iiOverlap.setStaticPointCloud(extractedEdgesA);
        iiOverlap.setDynamicPointCloud(correctedEdgesB);
        double intersectionVolume = iiOverlap.getIntersectionHull().getTotalVolume();

        pcl::ConvexHull<pcl::PointXYZ> hullTotal;

        pcl::PointCloud<pcl::PointXYZ>::Ptr mergedPC(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloudB, *mergedPC, icpCorrectionTransformation * calculateTMatrix(maximizingAngle, tX + xShift, tY + yShift, bbA.minZ));
       
        *mergedPC += *cloudA;

        hullTotal.setInputCloud(mergedPC);
        double totalVolume = hullTotal.getTotalVolume();

        std::cout << "Intersection volume: " << intersectionVolume << std::endl;
        std::cout << "Total volume: " << totalVolume << std::endl;

        std::cout << "Overlapping ratio: " << intersectionVolume / totalVolume << std::endl;

    }


}
