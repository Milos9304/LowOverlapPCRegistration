## Dependencies:
    fcl
    octomap
    libccd
    qhull
    pcl
    Eigen
### Optional
    OpenMP

## Dataset
Place folders A1 B1 C1 D1 E1 into 8thFloor folder.
Dataset available soon for download.

## Building
    cmake .
    make
    
## Running
    An example of a run to merge scans A and B.
    ./run.sh A1 high B1
    
    Choose high, medium, low to choose between subsampled scans with radius 0.5cm ,1cm and 1.5cm respectively or omit the parameter to run the algorithm on original resolution.
    
    All experiments were performed with *high* settings.
    
    Read output of the program to find out what result files have been written.
    
    For further information about input and output files see script run.sh
    
