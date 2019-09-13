#include "boundingBox.h"

void BoundingBox::setCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

        this -> minX = cloud -> points[0].x;
        this -> minY = cloud -> points[0].y;

        this -> maxX = cloud -> points[0].x;
        this -> maxY = cloud -> points[0].y;

        this -> minZ = cloud -> points[0].z;
        this -> maxZ = cloud -> points[0].z;

        for(size_t i = 0; i < cloud -> points.size(); ++i){
       
            if( this -> minX > cloud -> points[i].x ) this -> minX = cloud -> points[i].x;
            if( this -> minY > cloud -> points[i].y ) this -> minY = cloud -> points[i].y;
            if( this -> minZ > cloud -> points[i].z ) this -> minZ = cloud -> points[i].z;

            if( this -> maxX < cloud -> points[i].x ) this -> maxX = cloud -> points[i].x;
            if( this -> maxY < cloud -> points[i].y ) this -> maxY = cloud -> points[i].y;
            if( this -> maxZ < cloud -> points[i].z ) this -> maxZ = cloud -> points[i].z;

         }
}

void BoundingBox::print(){
    
    std:: cout << "-------------------------------" << std::endl;
    std:: cout << this -> minX << "     " << this -> maxX << std::endl;
    std:: cout << this -> minY << "     " << this -> maxY << std::endl;
    std:: cout << this -> minZ << "     " << this -> maxZ << std::endl;
    std:: cout << "-------------------------------" << std::endl;

}

