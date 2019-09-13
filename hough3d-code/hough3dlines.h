#ifndef H3DLINESH
#define H3DLINESH

#include <vector>
#include <Eigen/Dense>
#include "../src/line_descriptor.h"
#include "vector3d.h"

Eigen::MatrixXf hough_transform(const std::vector<Vector3d>&, const std::string&, bool=false);

#endif

