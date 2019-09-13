#include "line_descriptor.h"

LineDescriptor::LineDescriptor(int pointCount, Vector3d& origin, Vector3d& direction){

    this -> pointCount = pointCount;
    this -> origin = origin;
    this -> direction = direction;

}
