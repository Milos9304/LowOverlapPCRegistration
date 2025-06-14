Source code of point cloud registration tool robust against cases with low overlapping regions.
This is a support code to a publication https://www.mdpi.com/2072-4292/12/1/61

**Ensure _git-lfs_ is installed on your system before cloning this repository!**

The following are required for sucessful compilation
## Dependencies:
    fcl
    octomap
    libccd
    qhull
    pcl
    Eigen

If the following is found in the system, the compiled program will take advantage of paralel execution.
### Optional
    OpenMP

## Dataset
If you want to re-run experiments from the paper, move folders A1 B1 C1 D1 E1 into 8thFloor folder. Each directory contains an original scan of the area as depicted in Figure 3 in the paper. The three subsampled versions with densities 0.5cm (**high**), 1cm (**medium**), 1.5cm (**low**) are included.

## Building
    cmake .
    make
    
## Running
An example of a run to merge scans A and B which were subsampled with **high** density.

    ./run.sh A1 high B1
Or

    ./run.sh A1 B1
    
to run point cloud registration without any prior subsampling. Note, that **high** density performed the best in our experiments and was used to obtain results in the paper. The other densities are included for experimental purposes.

Read output of the program to find out what result files have been written.

For further information about input and output files or how to include custom PLY formatted scans see script run.sh .
