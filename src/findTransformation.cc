#include "findTransformation.h"

#include <fstream>

using namespace Eigen;

//DELETE

#include <vector>
#include <algorithm>
#include <numeric>

//DELETE
template<typename T>
T stdev(const std::vector<T> &vec)
{
    size_t sz = vec.size();
    if (sz == 1)
        return 0.0;

    // Calculate the mean
    T mean = std::accumulate(vec.begin(), vec.end(), 0.0) / sz;

    // Now calculate the variance
    auto variance_func = [&mean, &sz](T accumulator, const T& val)
    {
        return accumulator + ((val - mean)*(val - mean) / (sz - 1));
    };

    return sqrt(std::accumulate(vec.begin(), vec.end(), 0.0, variance_func));
}

void _removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove){

    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    //if( rowToRemove < numRows )
    matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void _removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.rightCols(numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

int** _cartesianProduct(int sizeA, int sizeB){

    //sizeA*sizeB rows, 2 columns
    int** array = (int**) malloc(sizeof(int*) * sizeA * sizeB + sizeof(int) * 2 * sizeA * sizeB); 

    int* ptr = (int*)(array+sizeA*sizeB);
    for(int i = 0; i < sizeA * sizeB; ++i)
        array[i] = (ptr + 2 * i);

    for(int i = 0; i < sizeA; ++i)
        for(int k = 0; k < sizeB; ++k){
            array[i*sizeB+k][0]=i;
            array[i*sizeB+k][1]=k;
        }

    return array; 

}

//makes every direction vector face upwards, i.e. ensures dz >= 0 for every line
void normalizeDirections(Eigen::MatrixXf& lines){

    Eigen::VectorXf dz = lines.col(5); 

    for(size_t i = 0; i < lines.rows(); ++i){

        if(dz[i] < 0)
            lines.block(i, 3, 1, 3) *= -1;
    }

}

Eigen::Matrix4f calculateTMatrix(float angle, float x, float y, float z){

    Eigen::Matrix4f rotation;
    rotation << cos(angle), -sin(angle), 0, 0,
                sin(angle),  cos(angle), 0, 0,
                         0,           0, 1, 0,
                         0,           0, 0, 1;

    Eigen::Matrix4f translation;
    translation <<  1, 0, 0, x,
                    0, 1, 0, y,
                    0, 0, 1, z,
                    0, 0, 0, 1;

    return translation*rotation;

}

std::tuple<float, float, float> findTranslation(float rotationB, MatrixXf& linesA, MatrixXf& linesB, BoundingBox bbA, BoundingBox bbB){//float maxY, float maxZ, float minAY, float maxAY, float minBY, float maxBY){

    float min_sigma = 0.00015;

    const int rowsA = linesA.rows();
    const int rowsB = linesB.rows();

    Eigen::Vector3d normalXY;
    //normalXY << 0, 0, 1;

    Matrix3f rotationZ;
    rotationZ << cos(rotationB), -sin(rotationB), 0,
                 sin(rotationB),  cos(rotationB), 0,
                              0,               0, 1;

    //MatrixXf directionsA = linesA.rightCols(3).transpose();
    MatrixXf directionsB = linesB.rightCols(3) * rotationZ.transpose();

    //MatrixXf originsA = linesA.block(0, 1, rowsA, 3).transpose();    
    MatrixXf originsB = linesB.block(0, 1, rowsB, 3) * rotationZ.transpose();      

    MatrixXf xyA = linesA.rightCols(6);
    MatrixXf xyB(directionsB.rows(), 6);
    xyB << originsB, directionsB;

        std::ofstream ofs;
        ofs.open("rotatedLines.data", std::ofstream::out | std::ofstream::trunc);

        for( int i = 0; i < xyB.rows(); ++i ){

            ofs << "5 ";
            ofs << xyB.row(i)[0]<<" ";
            ofs << xyB.row(i)[1]<<" ";
            ofs << xyB.row(i)[2]<<" ";
            ofs << xyB.row(i)[3]<<" ";
            ofs << xyB.row(i)[4]<<" ";
            ofs << xyB.row(i)[5]<<" ";
            ofs <<"\n";

        }

        ofs.close();


    //MAKE SURE "A" IS CENTERED!!! (we put cylinder at origin)
    //h > maxZ
    //k=sqrt(h)/maxY

    normalizeDirections(xyA);
    normalizeDirections(xyB);

    ParabolicCylinder parabolicCylinder(sqrt(bbA.maxZ)/(bbA.maxY), bbA.maxZ*1.3);

    parabolicCylinder.setStaticPointCloud(xyA);
    parabolicCylinder.createCoefficientsTable(xyB);

    float maxScoreY = 1;
    float maxTy = bbA.minY - bbB.maxY;

    std::cout << "     Y from: " << bbA.minY-bbB.maxY<<"   to: " << bbA.maxY-bbB.minY <<std::endl;

    std::vector<std::tuple<float, float>> results; 
    std::cout<< "WARNING: lower number of propagations. see findTransformation.cc"<< std::endl; 

    std::vector<std::tuple<float, float>> filteredY;
    
    //TODO

//UNCOMMENT FOR SIGMA SEARCH
//    for(; ; min_sigma += 0.000001){
        
        filteredY.clear();
        results.clear();

        std::cout << "-----------------------" << std::endl;
        std::cout << "SIGMA: " << min_sigma << std::endl;
        for(float ty = bbA.minY - bbB.maxY; ty < bbA.maxY - bbB.minY; ty += 0.0001){ //0.001

            //std::cout << ty << std::endl;
            float score = parabolicCylinder.getScoreTy(xyB, ty, min_sigma);
            if(maxScoreY < score){
                maxScoreY = score;
                maxTy = ty;
            }
            results.push_back(std::tuple<float, float>(score, ty));    
        } 
    

        std::copy_if(results.begin(), results.end(), std::back_inserter(filteredY), [maxScoreY](std::tuple<float, float> tuple){return std::get<0>(tuple)>= maxScoreY * 0.8;} );
        //sort in descending order by score
        std::sort(filteredY.begin(), filteredY.end(), [](std::tuple<float, float> tA, std::tuple<float, float> tB){return std::get<0>(tA) > std::get<0>(tB);});

        //take first 5 most probable if there are more than that
        filteredY.resize(std::min((int)filteredY.size(), 5));
        for(auto f: filteredY)
            std::cout << std::get<1>(f) << "        " << std::get<0>(f)<<std::endl;
//    }

    std::cout << "     X from: " << bbA.minX - bbB.maxX << "    to: " << bbA.maxX - bbB.minX << std::endl;

    results.clear();
    std::vector<std::tuple<float, float, float>> resultsFinal;

    float maxScoreX = 1;
    float maxTx = bbA.minX - bbB.maxX;
          maxTy = std::get<1>(filteredY[0]);

//UNCOMMENT FOR SIGMA SEARCH    
//    for(min_sigma = 0; ; min_sigma += 0.000000001){
        
        std::cout << "-----------------------" << std::endl;
        std::cout << "SIGMA: " << min_sigma << std::endl;

      for(auto ty : filteredY){
           
            maxScoreX = -1;

            parabolicCylinder.updateCache(xyB, std::get<1>(ty));

            for(float tx = bbA.minX - bbB.maxX; tx < bbA.maxX - bbB.minX; tx += 0.005){        

                float score = parabolicCylinder.getScoreTx(tx, min_sigma);
                if(maxScoreX < score * std::get<0>(ty)){
                    maxScoreX = score * std::get<0>(ty);
                    maxTx = tx;
                    maxTy = std::get<1>(ty);
                }
            
            }
            const float eps=0.004999;
            const float minX = maxTx-eps;
            const float maxX = maxTx+eps;

            for(float tx = minX; tx < maxX; tx += 0.000001){

                float score = parabolicCylinder.getScoreTx(tx, min_sigma);
                if(maxScoreX < score * std::get<0>(ty)){
                    maxScoreX = score * std::get<0>(ty);
                    maxTx = tx;
                    maxTy = std::get<1>(ty);
                }

            }
            
            resultsFinal.push_back(std::tuple<float, float, float>(maxTx, maxTy, maxScoreX)); 
            std::cout << maxTx << "         " << maxTy << "         " << maxScoreX << std::endl;

        }

//    }

    std::sort(resultsFinal.begin(), resultsFinal.end(),
         [](const std::tuple<float, float, float>& tA, const std::tuple<float, float, float>& tB){return std::get<2>(tA) > std::get<2>(tB);}
    );

    std::tuple<float, float, float> finalTransformation = resultsFinal[0];

    std::cout << "     Max score: " << std::get<2>(finalTransformation) << "  Max tx:  " << std::get<0>(finalTransformation) << "  Max ty:  "<< std::get<1>(finalTransformation) << std::endl << std::endl; 
    
    return finalTransformation;
}

std::vector<float> findPossibleRotations(MatrixXf& linesA, MatrixXf& linesB){

    //means our gaussians are sensitive to 0.998-1 cosine similarity 
    const float epsilon = 0.002;
    
    const float sigma = epsilon/3;

    const float twoPi=2*M_PI;

    const int rowsA = linesA.rows();
    const int rowsB = linesB.rows();

    Matrix3f rotationZ;
    MatrixXf rotatedDirectionsB;

    MatrixXf directionsA = linesA.rightCols(3).transpose();
    MatrixXf directionsB = linesB.rightCols(3).transpose();

    int** cartesianProduct = _cartesianProduct(rowsA, rowsB);

    float maxScore = 1;
    float maximizingAngle = 0;

    std::vector<std::tuple<float, float>> results;

    for(float angle = 0; angle < twoPi; angle+=0.001){ //0.001

        //Create rotation matrix around Z axis
        rotationZ << cos(angle), -sin(angle), 0,
                     sin(angle),  cos(angle), 0,
                              0,           0, 1;

        rotatedDirectionsB = rotationZ * directionsB;

        float score = 1;

        for(int i = 0; i < rowsA * rowsB; ++i){

            int aIndex = cartesianProduct[i][0];
            int bIndex = cartesianProduct[i][1];
    
            //Assuming directions are unit
            float cosine = abs(directionsA.col(aIndex).dot( rotatedDirectionsB.col(bIndex) ));
            if(cosine > M_PI)
                cosine = M_PI-cosine;

            //Evaluate gaussian, centered at 1 with sdev sigma
            score *= 1 + 0.01*exp(-0.5*pow(((cosine - 1)/sigma), 2));

        }

        results.push_back(std::tuple<float, float>(score, angle));
    
        if(score > maxScore){
            maxScore = score;
            maximizingAngle = angle;
        }
        //std::cout << score << std::endl;

    }

    std::vector<std::tuple<float, float>> filtered;
    std::copy_if(results.begin(), results.end(), std::back_inserter(filtered), [maxScore](std::tuple<float, float> tuple){return std::get<0>(tuple)>= maxScore * 0.8;} );
    
    //sort in descending order by score
    std::sort(filtered.begin(), filtered.end(), [](std::tuple<float, float> tA, std::tuple<float, float> tB){return std::get<0>(tA) > std::get<0>(tB);});

    //take first 20 most probable if there are more than that
    //results.resize(std::min((int)results.size(), 3));

    for(auto f: filtered)
        std::cout << std::get<1>(f) << "        " << std::get<0>(f)<<std::endl;

    std::vector<float> possibleRotations;

    while(filtered.size() > 0){
        
        auto it = filtered.begin();
        float head = std::get<1>(*filtered.begin());
        
        possibleRotations.push_back(head);
        float mirrored = head - M_PI;
        mirrored = mirrored >= 0 ? mirrored : mirrored + 2 * M_PI;
        possibleRotations.push_back(mirrored);

        while (it != filtered.end()){

            if( abs(std::get<1>(*it) - head) < 0.1 || abs(std::get<1>(*it) - mirrored) < 0.1)
                    it = filtered.erase(it);
            else
                ++it;
        }

   }
   
    std::cout<<"------\n"; 
    for(auto f: possibleRotations)
        std::cout << f <<std::endl;

    free(cartesianProduct);

    return possibleRotations;
}
