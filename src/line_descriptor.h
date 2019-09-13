#ifndef LINEDESCRH
#define LINEDESCRH

#include "../hough3d-code/vector3d.h"

struct LineDescriptor{

    int pointCount;
    Vector3d origin;
    Vector3d direction;

    LineDescriptor(int, Vector3d&, Vector3d&);

};

#endif
