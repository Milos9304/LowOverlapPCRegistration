#ifndef PARCYLH
#define PARCYLH

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "decisionTreeParabolic.h"

/* 
   n x 3 array
   for each line it contains coefficients A, B, C required for further calculation (see doc. ?)
*/ 
class ParabolicCylinder{

    public:
        //k, h
        ParabolicCylinder(float, float);
        void createCoefficientsTable(Eigen::MatrixXf&);
        void setStaticPointCloud(Eigen::MatrixXf&);
        float getScoreTx(float, float);
        float getScoreTy(Eigen::MatrixXf&, float, float);
        void updateCache(Eigen::MatrixXf&, float);
        std::tuple<float, float> getXStaticBounds();
        std::tuple<float, float> getXCachedBounds();

    private:
        float k, h, width;
        float twelve_k_to_four;
        float four_k_to_four;
        Eigen::MatrixXf coefficientsB;
        Eigen::MatrixXf staticIntersections;
        Eigen::MatrixXf xyCached;
        DecisionTreeParabolic* decisionTreeX;
        DecisionTreeParabolic* decisionTreeY;

        Eigen::MatrixXf _createCoefficientsTable(Eigen::MatrixXf&, bool);

	// This function implements approximation to parabolic arc length function, namely the Equation A6 in the corresponding publication
        float parabolicDistance(float, float);
};

#endif
