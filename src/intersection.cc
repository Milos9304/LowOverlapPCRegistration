#include "intersection.h"

#define OCTREE_GRANULARITY 1//(float, smaller = finer)
#define MAX_INTERSECTS 500

using namespace std;
using namespace octomap;

void calcThresholdedNodes(const OcTree tree,
                          unsigned int& num_thresholded,
                          unsigned int& num_other)
{
  num_thresholded = 0;
  num_other = 0;

  for(OcTree::tree_iterator it = tree.begin_tree(), end=tree.end_tree(); it!= end; ++it){
    if (tree.isNodeAtThreshold(*it))
      num_thresholded++;
    else
      num_other++;
  }
}

void outputStatistics(const OcTree tree){
  unsigned int numThresholded, numOther;
  calcThresholdedNodes(tree, numThresholded, numOther);
  size_t memUsage = tree.memoryUsage();
  unsigned long long memFullGrid = tree.memoryFullGrid();
  size_t numLeafNodes = tree.getNumLeafNodes();

  cout << "Tree size: " << tree.size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  cout << endl;
}

//IntersectionInterface::IntersectionInterface() : pcA(0.1), pcB(0.1){}

void IntersectionInterface::setStaticPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){

    this -> pcA = new OcTree(OCTREE_GRANULARITY);
    buildOctoTree(pointCloud, this -> pcA);

}

void IntersectionInterface::setDynamicPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){
   
    this -> pcB = new OcTree(OCTREE_GRANULARITY);
    buildOctoTree(pointCloud, this -> pcB);

}

void IntersectionInterface::buildOctoTree(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, OcTree *ocTree){

    octomap::Pointcloud octocloud;
    //OcTree ocTree(0.1);
    for (int i = 0; i < pointCloud -> points.size(); i++) {
           // cout << "X:" << pointCloud->points[i].x << endl;

            point3d endpoint(pointCloud -> points[i].x, pointCloud -> points[i].y, pointCloud -> points[i].z);
            // tree.updateNode(endpoint, true);
            octocloud.push_back(endpoint);
    }

    point3d sensorOrigin(0,0,0);

    ocTree->insertPointCloud(octocloud, sensorOrigin);
    outputStatistics(*ocTree);

    //ocTree.write("labak-rgbd.ot");
    
}

pcl::ConvexHull<pcl::PointXYZ> IntersectionInterface::getIntersectionHull(){

    fcl::OcTree<float>* treeA = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(pcA));
    fcl::CollisionObject<float> tree_objA( (std::shared_ptr<fcl::CollisionGeometry<float>>(treeA)) );

    fcl::OcTree<float>* treeB = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(pcB));
    fcl::CollisionObject<float> tree_objB( (std::shared_ptr<fcl::CollisionGeometry<float>>(treeB)) );

    
    fcl::CollisionRequest<float> request(MAX_INTERSECTS, true, 1, true);
    fcl::CollisionResult<float> result;

    fcl::collide(&tree_objA, &tree_objB, request, result);

    std::vector<fcl::Contact<float>> contacts;
    result.getContacts(contacts);

    std::cout << "size: " << contacts.size() << std::endl;
    std::cout << "isCollision: " << result.isCollision() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr markerCloud( new pcl::PointCloud<pcl::PointXYZ>() ); 

    for(const auto& c : contacts)
            markerCloud -> push_back(pcl::PointXYZ(c.pos[0], c.pos[1], c.pos[2]));

    pcl::ConvexHull<pcl::PointXYZ> convexHull;	

    convexHull.setInputCloud(markerCloud);

    return convexHull;

}

pcl::CropHull<pcl::PointXYZ> IntersectionInterface::getIntersectionFilter(){
    
    std::cout<<"finding intersection region...\n";

    pcl::ConvexHull<pcl::PointXYZ> convexHull = this -> getIntersectionHull();

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> hullPoints(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::Vertices> hullPolygons;
    convexHull.reconstruct(*hullPoints, hullPolygons);	
    
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    
    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(hullPoints);

    return cropHullFilter;

} 
