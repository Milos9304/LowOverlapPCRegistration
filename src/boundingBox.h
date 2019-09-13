#ifndef BOUNDBH
#define BOUNDBH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BoundingBox{

    public:

        float minX;
        float maxX;
        float minY;
        float maxY;
        float minZ;
        float maxZ;

        void setCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr);
        void print();
};

#endif
