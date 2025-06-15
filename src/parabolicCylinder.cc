#include "parabolicCylinder.h"

#include <iostream>

/*//makes every direction vector face upwards, i.e. ensures dz >= 0 for every line
void normalizeDirections(Eigen::MatrixXf& lines){

    Eigen::VectorXf dz = lines.col(5); 

    for(size_t i = 0; i < lines.rows(); ++i){

        if(dz[i] < 0)
            lines.block(i, 3, 1, 3) *= -1;
    }

}*/

ParabolicCylinder::ParabolicCylinder(float k, float h){

    this -> k = k;
    this -> h = h;   
    this -> width = 2*sqrt(h)/k;

    this -> four_k_to_four = 4 * pow(k, 4);
    this -> twelve_k_to_four = 3 * this -> four_k_to_four; 

    std::cout << "Parabolic cylinder parameters:        k=" << k << "   h=" << h << std::endl; 

}

void ParabolicCylinder::createCoefficientsTable(Eigen::MatrixXf& lines){

    _createCoefficientsTable(lines, false);

}

std::tuple<float, float> ParabolicCylinder::getXStaticBounds(){

    return std::tuple<float, float>(staticIntersections.col(0).minCoeff(), staticIntersections.col(0).maxCoeff());

}

std::tuple<float, float> ParabolicCylinder::getXCachedBounds(){

    float defaultVal = xyCached(0,0);
    Eigen::ArrayXf col = xyCached.col(0).array().isNaN().select(defaultVal, xyCached.col(0));

    return std::tuple<float, float>(col.minCoeff(), col.maxCoeff());

}

void ParabolicCylinder::setStaticPointCloud(Eigen::MatrixXf& lines_unfiltered){

    Eigen::MatrixXf coeffs = _createCoefficientsTable(lines_unfiltered, true);
    Eigen::ArrayXf t_unfiltered = coeffs.col(1).array() + coeffs.col(3).array().sqrt(); 

    const float Xthreshold = this -> width;
    const float Tthreshold = sqrt(pow(this->width,2)+pow(Xthreshold, 2));
 
    const int filteredSize =  (t_unfiltered < Tthreshold).select(Eigen::VectorXi::Ones(t_unfiltered.rows()), Eigen::VectorXi::Zero(t_unfiltered.rows())).sum();

    std::cout << filteredSize;

    Eigen::ArrayXf t(filteredSize);
    Eigen::MatrixXf lines(filteredSize, lines_unfiltered.cols());
    
    int index = 0;

    for(size_t i = 0; i < t_unfiltered.rows(); ++i){
        if(t_unfiltered(i) < Tthreshold){
            t(index) = t_unfiltered(i);
            lines.row(index) << lines_unfiltered.row(i);
            index++;
        }

    }
    
    std::cout << "Filtered size: " << t.rows() << std::endl;


    //calculates intersections with pointcloud A
    this -> staticIntersections = lines.leftCols(3).array() + lines.rightCols(3).array() * t.replicate(1,3).array();

    std::vector<CoordAndT> leftSideX;
    std::vector<CoordAndT> rightSideX;

    std::vector<CoordAndT> leftSideZ;
    std::vector<CoordAndT> rightSideZ;

    for(size_t i = 0; i < this->staticIntersections.rows(); ++i){

        Eigen::ArrayXf row = this ->staticIntersections.row(i).array();
        if(row[1] < 0){
            leftSideX.push_back(CoordAndT(row[0], t(i)));
            leftSideZ.push_back(CoordAndT(row[2], 0));
        }
        else{
            rightSideX.push_back(CoordAndT(row[0], t(i)));
            rightSideZ.push_back(CoordAndT(row[2], 0));
        }
    }

    std::sort(leftSideX.begin(), leftSideX.end(), [](const CoordAndT& a, const CoordAndT& b){return std::get<0>(a) < std::get<0>(b);});
    std::sort(rightSideX.begin(), rightSideX.end(), [](const CoordAndT& a, const CoordAndT& b){return std::get<0>(a) < std::get<0>(b);});   

    std::sort(leftSideZ.begin(), leftSideZ.end(), [](const CoordAndT& a, const CoordAndT& b){return std::get<0>(a) < std::get<0>(b);});
    std::sort(rightSideZ.begin(), rightSideZ.end(), [](const CoordAndT& a, const CoordAndT& b){return std::get<0>(a) < std::get<0>(b);});
  
    this -> decisionTreeX = new DecisionTreeParabolic(leftSideX.data(), leftSideX.data()+leftSideX.size()-1, rightSideX.data(), rightSideX.data()+rightSideX.size()-1); 
    //We note that populating decisionTreeY with leftSideZ is not a bug. See comment on line 199 for explanation. 
    this -> decisionTreeY = new DecisionTreeParabolic(leftSideZ.data(), leftSideZ.data()+leftSideZ.size()-1, rightSideZ.data(), rightSideZ.data()+rightSideZ.size()-1);

}

Eigen::MatrixXf ParabolicCylinder::_createCoefficientsTable(Eigen::MatrixXf& lines, bool linesA){

//    normalizeDirections(lines);
    
    //lines: Ox Oy Oz Dx Dy Dz
    //table: A B C D

    float one_over_k_squared = 1 / ( k * k );

    const int rows = lines.rows();
   
    Eigen::MatrixXf table(rows, 4);
    
    Eigen::VectorXf one_over_dy = Eigen::ArrayXf::Ones(rows).cwiseQuotient(lines.col(4).array());
    Eigen::VectorXf one_over_k2dy2 = one_over_k_squared * one_over_dy.array().square();
    Eigen::VectorXf dz_over_k2dy2 = lines.col(5).cwiseProduct(one_over_k2dy2);
    
    //A
    table.col(0) = -one_over_dy;
    
    //B 
    table.col(1) = lines.col(1).cwiseProduct(table.col(0)) - 0.5*dz_over_k2dy2;

    Eigen::VectorXf oy_over_dy = lines.col(1).cwiseProduct(one_over_dy);

    //C
    table.col(2) = -table.col(0).cwiseProduct(dz_over_k2dy2);

    //D
    table.col(3) = lines.col(1).cwiseProduct(table.col(2))+0.25*dz_over_k2dy2.cwiseProduct(dz_over_k2dy2)+(this->h*Eigen::VectorXf::Ones(rows)-lines.col(2)).cwiseProduct(one_over_k2dy2);

    if(!linesA)
        this -> coefficientsB = table;

    return table; 

}

void ParabolicCylinder::updateCache(Eigen::MatrixXf& lines, float ty){

    Eigen::VectorXf t = (coefficientsB.col(0)*ty + coefficientsB.col(1)).array()+(coefficientsB.col(2)*ty+coefficientsB.col(3)).array().sqrt();    
    this -> xyCached = lines.block(0, 1, lines.rows(), 2).array() + t.replicate(1,2).array()*lines.block(0, 3, lines.rows(), 2).array();
    //this -> cacheEntry = ty;

}

float ParabolicCylinder::getScoreTx(float tx, float min_sigma){

    float score = 1;

    for(size_t i = 0; i < this -> xyCached.rows(); ++i){

        Eigen::VectorXf row = this -> xyCached.row(i);
        if(std::isnan(row[1]))
            continue;

        CoordAndT nearestPoint = decisionTreeX -> inference(row[1], row[0]+tx);
        
        float distance = this -> parabolicDistance(row[0]+tx, std::get<0>(nearestPoint));

        score *= 1 + 0.1*exp(-0.5/ (pow( min_sigma+min_sigma* std::get<1>(nearestPoint)/(this->width*2), 2))   *pow(distance,2));

    }
    
    return score;
}

float ParabolicCylinder::getScoreTy(Eigen::MatrixXf& lines, float ty, float min_sigma){

    //const float sigma = 0.00015/3;//0.001/3;
    //const float sigmaSquared = sigma * sigma;


    Eigen::VectorXf t = (coefficientsB.col(0)*ty + coefficientsB.col(1)).array()+(coefficientsB.col(2)*ty+coefficientsB.col(3)).array().sqrt();    
    Eigen::MatrixXf yz = lines.block(0, 1, lines.rows(), 2).array() + t.replicate(1,2).array()*lines.block(0, 4, lines.rows(), 2).array();

    float score = 1;

    for(size_t i = 0; i < yz.rows(); ++i){

        Eigen::VectorXf row = yz.row(i);
        if(std::isnan(row[1]))
            continue;

        // We note that the choice of the second coordinate of 'row' is not a bug; rather, it corresponds to a non-conventional feature of the dataset at the time of writing the code, where point coordinates were stored in the order (X, Z, Y). As a result, 'row[1]' accesses the Z-coordinate. Similarly, 'decisionTreeY' is initialized on line 110 with 'leftSideZ', meaning that its internal representation and inference are performed over Z-values. While this contradicts the convention used in the paper (where Z is the fixed axis), the implementation remains correct due to consistent usage of Z in both matching and scoring. The naming remains for legacy reasons. 
	CoordAndT nearestPoint = decisionTreeY -> inference(row[0], row[1]);
        float distance = this -> parabolicDistance(row[1], std::get<0>(nearestPoint));

        score *= 1 + 0.1*exp(-0.5/(pow( min_sigma+min_sigma*t(i)/(this->width*2), 2))*pow(distance,2));

    }
    
    return score; 
}

// This function implements approximation to parabolic arc length function, namely the Equation A6 in the corresponding publication
float ParabolicCylinder::parabolicDistance(float y1, float y2){

    float c1 = 1 + four_k_to_four * pow(y2, 2);
    float c2 = 1 + four_k_to_four * pow(y1, 2);

    return (c1 * sqrt(c1) - c2 * sqrt(c2)) / twelve_k_to_four; 

}
