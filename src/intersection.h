#ifndef INTERSECTH
#define INTERSECTH

#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include <fcl/narrowphase/collision.h>

class IntersectionInterface{

    octomap::OcTree *pcA;
    octomap::OcTree *pcB;

    public:
        //IntersectionInterface();
        void setStaticPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
        void setDynamicPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
        pcl::ConvexHull<pcl::PointXYZ> getIntersectionHull();
        pcl::CropHull<pcl::PointXYZ> getIntersectionFilter();

    private:
        void buildOctoTree(pcl::PointCloud<pcl::PointXYZ>::Ptr, octomap::OcTree*);

};
#endif
