//
// hough3dlines.cpp
//     Main program implementing the iterative Hough transform
//
// Original author:  Tilman Schramke, Christoph Dalitz
// Modified by: Milos Prokop
// Date:    2018-03-24
// License: see License-BSD2
//

//#include "vector3d.h"
#include "pointcloud.h"
#include "hough.h"

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include "hough3dlines.h"
//#include "../src/line_descriptor.h"

using Eigen::MatrixXf;
using namespace std;

// usage message
const char* usage = "Usage:\n"
  "\though3dlines [options] <infile>\n"
  "Options (defaults in brackets):\n"
  "\t-o <outfile>   write results to <outfile> [stdout]\n"
  "\t-dx <dx>       step width in x'y'-plane [0]\n"
  "\t               when <dx>=0, it is set to 1/64 of total width\n"
  "\t-nlines <nl>   maximum number of lines returned [0]\n"
  "\t               when <nl>=0, all lines are returned\n"
  "\t-minvotes <nv> only lines with at least <nv> points are returned [0]\n"
  "\t-v             be verbose and print Hough space size to stdout\n"
  "\t-vv            be even more verbose and print Hough lines (before LSQ)\n"
  "Version:\n"
  "\t1.1 from 2018-03-24\n";



template<class Matrix>
void write_binary(const char* filename, const Matrix& matrix){
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
}
template<class Matrix>
void read_binary(const char* filename, Matrix& matrix){
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
}

// orthogonal least squares fit with libeigen
// rc = largest eigenvalue
//
double orthogonal_LSQ(const PointCloud &pc, Vector3d* a, Vector3d* b){
  double rc = 0.0;

  // anchor point is mean value
  *a = pc.meanValue();

  // copy points to libeigen matrix
  Eigen::MatrixXf points = Eigen::MatrixXf::Constant(pc.points.size(), 3, 0);
  for (int i = 0; i < points.rows(); i++) {
    points(i,0) = pc.points.at(i).x;
    points(i,1) = pc.points.at(i).y;
    points(i,2) = pc.points.at(i).z;
  }

  // compute scatter matrix ...
  MatrixXf centered = points.rowwise() - points.colwise().mean();
  MatrixXf scatter = (centered.adjoint() * centered);

  // ... and its eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatter);
  Eigen::MatrixXf eigvecs = eig.eigenvectors();

  // we need eigenvector to largest eigenvalue
  // libeigen yields it as LAST column
  b->x = eigvecs(0,2); b->y = eigvecs(1,2); b->z = eigvecs(2,2);
  rc = eig.eigenvalues()(2);

  return (rc);
}

Eigen::MatrixXf hough_transform(const std::vector<Vector3d> &input_points, const std::string& outputfilename, bool writeFile){    
    //int main(int argc, char ** argv) {

    // default values for command line options
    double opt_dx = 0.03;//0.05;//0.0;
    int opt_nlines = 160;//80;//0;
    int opt_minvotes = 200;//300//500;//0;
    int opt_verbose = 0;
    char* infile_name = NULL;
    char* outfile_name = NULL;

    // number of icosahedron subdivisions for direction discretization
    int granularity = 4;
    int num_directions[7] = {12, 21, 81, 321, 1281, 5121, 20481};

    // IO files
    //FILE* infile = NULL;
    FILE* outfile = stdout;

    // bounding box of point cloud
    Vector3d minP, maxP, minPshifted, maxPshifted;
    // diagonal length of point cloud
    double d;

    PointCloud X;
    X.points = input_points;
    /*for( aut  const& element : input_points ){

    X.points.push_back(Vector3d(element[0], element[1], element[2]));

    }*/


    // center cloud and compute new bounding box
    //X.getMinMax3D(&minP, &maxP);
    //d = (maxP-minP).norm();
    
    //if (d == 0.0)
    //    fprintf(stderr, "Error: all points in point cloud identical\n");
    //
    //X.shiftToOrigin();
    X.getMinMax3D(&minPshifted, &maxPshifted);

    // estimate size of Hough space
    /*if (opt_dx == 0.0)
        opt_dx = d / 64.0;
    else if (opt_dx >= d) {
        fprintf(stderr, "Error: dx too large\n");
        return NULL;// 1;
    }*/
    
    std::cout << "Step size: " << opt_dx << std::endl;

    double num_x = floor(d / opt_dx + 0.5);
    double num_cells = num_x * num_x * num_directions[granularity];
    if (opt_verbose) {
    printf("info: x'y' value range is %f in %.0f steps of width dx=%f\n",
           d, num_x, opt_dx);
    printf("info: Hough space has %.0f cells taking %.2f MB memory space\n",
           num_cells, num_cells * sizeof(unsigned int) / 1000000.0);
    }

    // first Hough transform
    Hough* hough;
    try {
        hough = new Hough(minPshifted, maxPshifted, opt_dx, granularity);
    } 
    catch (const std::exception &e) {
        
        fprintf(stderr, "Error: cannot allocate memory for %.0f Hough cells (%.2f MB)\n", num_cells, (double(num_cells) / 1000000.0) * sizeof(unsigned int));
        


        while(1);



        //return NULL;// 2;
    
    }
    hough->add(X);

    // print header info if necessary
    /*if (opt_outformat == format_gnuplot) {
    fprintf(outfile, "set datafile separator ','\n"
            "set parametric\n"
            "set xrange [%f:%f]\n"
            "set yrange [%f:%f]\n"
            "set zrange [%f:%f]\n"
            "set urange [%f:%f]\n"
            "splot '%s' using 1:2:3 with points palette",
            minP.x, maxP.x, minP.y, maxP.y, minP.z, maxP.z,
            -d, d, infile_name);
    }*/

    // iterative Hough transform
    // (Algorithm 1 in IPOL paper)
    PointCloud Y;	// points close to line
    double rc;
    unsigned int nvotes;
    int nlines = 0;

    std::vector<LineDescriptor> detectedLines;

    do {
        
        Vector3d a; // anchor point of line
        Vector3d b; // direction of line

        hough -> subtract(Y); // do it here to save one call

        nvotes = hough -> getLine(&a, &b);
        
        if (opt_verbose > 1) {
          
            Vector3d p = a + X.shift;
            printf("info: highest number of Hough votes is %i for the following "
                 "line:\ninfo: a=(%f,%f,%f), b=(%f,%f,%f)\n",
                 nvotes, p.x, p.y, p.z, b.x, b.y, b.z);
        }

        X.pointsCloseToLine(a, b, opt_dx, &Y);

        rc = orthogonal_LSQ(Y, &a, &b);
        if ( rc==0.0 )
            break;

        X.pointsCloseToLine(a, b, opt_dx, &Y);
        nvotes = Y.points.size();

        if ( nvotes < (unsigned int)opt_minvotes )
            break;

        rc = orthogonal_LSQ(Y, &a, &b);
        if (rc==0.0) break;

        a = a + X.shift;

        nlines++;

        fprintf(outfile, "npoints=%lu, a=(%f,%f,%f), b=(%f,%f,%f)\n", Y.points.size(), a.x, a.y, a.z, b.x, b.y, b.z); 
        detectedLines.push_back( LineDescriptor(Y.points.size(), a, b) );        
        X.removePoints(Y);

        //DELETE
        //break;



    } while ((X.points.size() > 1) && 
           ((opt_nlines == 0) || (opt_nlines > nlines)));
    
    Eigen::MatrixXf detectedLinesOut(detectedLines.size(),7);
    for(size_t i = 0; i < detectedLines.size(); ++i){

        detectedLinesOut(i, 0) = detectedLines[i].pointCount;
        detectedLinesOut(i, 1) = detectedLines[i].origin.x;
        detectedLinesOut(i, 2) = detectedLines[i].origin.y;   
        detectedLinesOut(i, 3) = detectedLines[i].origin.z;
        detectedLinesOut(i, 4) = detectedLines[i].direction.x;
        detectedLinesOut(i, 5) = detectedLines[i].direction.y;
        detectedLinesOut(i, 6) = detectedLines[i].direction.z;
   }

    std::string ofn = outputfilename+"load.data";
    write_binary(ofn.c_str(), detectedLinesOut);
    std::cout<<"data written";

    if (writeFile){

        std::ofstream ofs;
        ofs.open(outputfilename, std::ofstream::out | std::ofstream::trunc);

        for( auto const& line : detectedLines ){

            ofs << line.pointCount<<" ";
            ofs << line.origin.x<<" ";
            ofs << line.origin.y<<" ";
            ofs << line.origin.z<<" ";
            ofs << line.direction.x<<" ";
            ofs << line.direction.y<<" ";
            ofs << line.direction.z<<" ";
            ofs <<"\n";

        }

        ofs.close();

    }

    // clean up
    delete hough;
    if (outfile_name) fclose(outfile);

    return detectedLinesOut;// 0;
}
