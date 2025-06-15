/*
 * EigenvalueCovariance.cpp
 *
 *  Created on: May 6, 2015
 *      Author: dbazazian
 *  
 *  Modified by: Milos Prokop, August 2019
 */
#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
//#include <cstdlib>
//#include <pcl/io/io.h>
#include <pcl/common/common_headers.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/shot_omp.h>
#include "pcl/features/fpfh.h"
#include <pcl/io/ply_io.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <pcl/filters/statistical_outlier_removal.h>

#include "Difference_Eigenvalues.h"

#define KNN 20  

using namespace std;
using namespace Eigen;

//Sigma threshold
//float threshold = 0.05;

//int main (int argc, char*argv[]){eeeeeeeeeeee
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& outputname, bool writeFile ){

    pcl::PointCloud<pcl::PointXYZ>::Ptr Normals (new pcl::PointCloud<pcl::PointXYZ>);
    Normals->resize(cloud->size());

    // K nearest neighbor search
    int KNumbersNeighbor = 10; // numbers of neighbors 7 , 120
    std::vector<int> NeighborsKNSearch(KNumbersNeighbor);
    std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighbor);

    int* NumbersNeighbor = new  int [cloud ->points.size ()];
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    pcl::PointXYZ searchPoint;

    double* SmallestEigen = new  double [cloud->points.size() ];
    double* MiddleEigen = new  double [cloud->points.size() ];
    double* LargestEigen = new  double [cloud->points.size() ];

    //double* DLS = new  double [cloud->points.size() ];
    //double* DLM = new  double [cloud->points.size() ];
    //double* DMS = new  double [cloud->points.size() ];
    double* Sigma = new  double [cloud->points.size() ];

    // std::vector<double> SmallestEigen;
    // std::vector<double> MiddleEigen;
    // std::vector<double> LargestEigen;
    //
    // std::vector<double> DLS;
    // std::vector<double> DML;
    // std::vector<double> DMS;

    //  ************ All the Points of the cloud *******************
    for (size_t i = 0; i < cloud ->points.size (); ++i) {

        searchPoint.x =   cloud->points[i].x;
        searchPoint.y =   cloud->points[i].y;
        searchPoint.z =   cloud->points[i].z;

        if ( kdtree.nearestKSearch (searchPoint, KNumbersNeighbor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
                 NumbersNeighbor[i]= NeighborsKNSearch.size (); }
            else { NumbersNeighbor[i] = 0; }

        float Xmean; float Ymean; float Zmean;
        float sum= 0.00;

        // Computing Covariance Matrix
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += cloud->points[ NeighborsKNSearch[ii] ].x;
        
        Xmean = sum / NumbersNeighbor[i];
            
        sum= 0.00;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += cloud->points[NeighborsKNSearch[ii] ].y;
        Ymean = sum / NumbersNeighbor[i];
            
        sum= 0.00;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += cloud->points[NeighborsKNSearch[ii] ].z;
        Zmean = sum / NumbersNeighbor[i];

        float CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].x - Xmean )  );
        CovXX = sum / ( NumbersNeighbor[i]-1);
            
        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
        sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
        CovXY = sum / ( NumbersNeighbor[i]-1);
        CovYX = CovXY ;

        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
        sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
        CovXZ= sum / ( NumbersNeighbor[i]-1); 
        CovZX = CovXZ;

        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
        CovYY = sum / ( NumbersNeighbor[i]-1);

        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
        CovYZ = sum / ( NumbersNeighbor[i]-1);
        CovZY = CovYZ;

        sum = 0.00 ;
        for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii)
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].z - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
        CovZZ = sum / ( NumbersNeighbor[i]-1);

        // Computing Eigenvalue and EigenVector
        Matrix3f Cov;
        Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

        SelfAdjointEigenSolver<Matrix3f> eigensolver(Cov);
        if (eigensolver.info() != Success) abort();

        double EigenValue1 = eigensolver.eigenvalues()[0];
        double EigenValue2 = eigensolver.eigenvalues()[1];
        double EigenValue3 = eigensolver.eigenvalues()[2];

        double Smallest = 0.00; double Middle = 0.00; double Largest= 0.00;

        if (EigenValue1 < EigenValue2)
            Smallest =  EigenValue1;
        else
            Smallest = EigenValue2;

        if (EigenValue3 < Smallest)
            Smallest =  EigenValue3;


        if(EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
              Smallest = EigenValue1;
        if(EigenValue2 <= EigenValue3) {Middle = EigenValue2; Largest = EigenValue3;}
        else {Middle = EigenValue3; Largest = EigenValue2;}
        }

        if(EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
        {
              Largest = EigenValue1;
        if(EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
        else {Smallest = EigenValue3; Middle = EigenValue2;}
        }

        if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
        {
              Middle = EigenValue1;
        if(EigenValue2 >= EigenValue3){Largest = EigenValue2; Smallest = EigenValue3;}
        else{Largest = EigenValue3; Smallest = EigenValue2;}
        }

        SmallestEigen[i]= Smallest ;
        MiddleEigen[i]= Middle;
        LargestEigen[i]= Largest;

        //DLS[i] = std::abs ( SmallestEigen[i] / LargestEigen[i]) ;          // std::abs ( LargestEigen[i] -  SmallestEigen[i] ) ;
        //DLM[i] = std::abs ( MiddleEigen[i] /  LargestEigen[i]) ;             // std::abs (  LargestEigen[i] - MiddleEigen[i] ) ;
        //DMS[i] = std::abs ( SmallestEigen[i] / MiddleEigen[i]) ;       // std::abs ( MiddleEigen[i] -  SmallestEigen[i] ) ;
        
        Sigma[i] = (SmallestEigen[i] ) / ( SmallestEigen[i] + MiddleEigen[i] + LargestEigen[i] ) ;
    } 

    //std::cout<< " Computing Sigma is Done! " << std::endl;
    // Color Map For the difference of the eigen values

    double MaxD = 0.00;
    double MinD = std::numeric_limits<double>::max();
      
    int Ncolors = 256;

    for (size_t i = 0; i < cloud ->points.size (); ++i) {

          if (Sigma [i] < MinD)
              MinD= Sigma [i];

          if (Sigma[i] > MaxD)
              MaxD = Sigma [i];
    }

    //std::cout<< " Minimum is :" << MinD<< std::endl;
    //std::cout<< " Maximum  is :" << MaxD << std::endl;

	/*
	  // computing the standard deviation
	 double ss = 0.00 ;
	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  ss += Sigma [i] ;}
	  double avg = ss / cloud ->points.size () ;
	  ss = 0.00 ;
	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  ss += (Sigma [i] -  avg ) * (  Sigma [i] -  avg ) ;}
	  double stddvtion =   sqrt (  ss  /  ( cloud ->points.size () - 1 )  ) ;

	  std::cout<< " Standard Deviation is :" << stddvtion << std::endl;

	   MaxD = ( 2 )* stddvtion;
	  //MaxD = 10* stddvtion;

	  // Color table
		double line;
		double code[Ncolors][3];
	   ifstream colorcode ( "/Path/TO/ArtificialPointClouds/JetColorDensity/ColorCodes256.txt" );
	   //store color codes in array
	    int i=0,j=0;
	    while( colorcode>> line ) {
	    code[i][j]=line;
	    j++;
	    if (j == 3)
	    i++;
	}
	    code[1][0] = 0;
	    code[1][1] = 0;
	    code[1][2] = 135.468;



	    // jet color map

	int level = 0;
	float step = ( ( MaxD -  MinD) / Ncolors ) ;
	    for (size_t i = 0; i < cloud ->points.size (); ++i) {
    if (  SmallestEigen [i] <= MaxD ) {
                level = floor( (SmallestEigen [i] - MinD ) /  step ) ;

                cloud->points[i].r = code[ level ][0];
                cloud->points[i].g =  code[ level ][1];
                cloud->points[i].b =  code[ level ][2];
    } // if sigma less than Max
                }
    */

    int ePointsCount = 0;


    int level = 0;
    float step = ( ( MaxD -  MinD) / Ncolors ) ;
    //  level = floor( (Sigma [i] - MinD ) /  step ) ;
    float threshold = ( MinD + ( 6 * step) );
    //float threshold=0.1;
    //std::cout << ( MinD + ( 6 * step) ) << "\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr edgePoints (new pcl::PointCloud<pcl::PointXYZ>);
    
    for (size_t i = 0; i < cloud -> points.size(); ++i) {
        if ( Sigma [i] > threshold ) {

            edgePoints->points.push_back( cloud->points[i] );
            ePointsCount++;
    
        }
    }
    if ( writeFile ){

        pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
        pcl::copyPointCloud(*edgePoints, cloudRGB);

        for (size_t i = 0; i < cloudRGB.points.size(); ++i) {
            cloudRGB.points[i].r = 255;
            cloudRGB.points[i].g = 0;
            cloudRGB.points[i].b = 0;
        }

        pcl::PLYWriter writePLY;
        writePLY.write(outputname, cloudRGB,  true, false);
        std::cout << "File " << outputname << " written.\n";

    }


/*
   //DELETE
   pcl::PLYWriter writePLY;
   pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
   pcl::copyPointCloud(*edgePoints,cloudRGB);

   for (size_t i = 0; i < cloudRGB.points.size(); ++i) {
                cloudRGB.points[i].r = 255;
                cloudRGB.points[i].g = 0;
                cloudRGB.points[i].b = 0;

   }
  
  writePLY.write(outputname, cloudRGB,  true, false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (edgePoints);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.05);
  sor.filter (*filtered);

  pcl::copyPointCloud(*filtered,cloudRGB);
  writePLY.write(outputname+"_filtered.ply", cloudRGB,  true, false);
    
*/

    // writing the Sigma on the disk
    //	    	   		std::ofstream ofsSigma;
    //	    	   		ofsSigma.open("/Path/TO/SigmaDragon.txt");
    //	    	            for (size_t i = 0; i < cloud ->points.size (); ++i) {
    //	    	            	ofsSigma << Sigma [i]<< ","<< std::endl ;
    //	    	                    }
    //  	pcl::PLYWriter writePLY;
    //  	writePLY.write (OUTPUT_FILE, *cloud,  false);

    //return 0;

  return edgePoints;
}
