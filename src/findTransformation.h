#ifndef FINDTRANSFORMATIONH
#define FINDTRANSFORMATIONH

#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "line_descriptor.h"
#include "parabolicCylinder.h"
#include "boundingBox.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>


std::vector<float> findPossibleRotations(Eigen::MatrixXf&, Eigen::MatrixXf&);

//Returns X, Y, SCORE of translation with highest score
//IN: angle, detected lines A, detected lines B, BoundingBox of A, BoundingBox of B 
std::tuple<float, float, float> findTranslation(float, Eigen::MatrixXf&, Eigen::MatrixXf&, BoundingBox, BoundingBox);

//IN: angle, x, y
Eigen::Matrix4f calculateTMatrix(float, float, float, float z=0);

//IN: angle, x, y, z

#endif

