cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 14)
message(${CMAKE_CXX_FLAGS_RELEASE})
project(LowOverlapMerge)

find_package(PCL 1.7 REQUIRED COMPONENTS common io filters segmentation surface visualization) #visualization)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED NO_MODULE)
find_package(FCL REQUIRED)
find_package(CCD REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(ICP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/PointCloudRegistrationTool/include/)
set(ICP_SOURCE_DIR ${PROJECT_SOURCE_DIR}/PointCloudRegistrationTool/src/)
set(ICP_SRC_FILES ${ICP_SOURCE_DIR}/Registrator.cpp)

#set(OCTOMAP_LIB_FILES /usr/local/lib/liboctomap.1.9.0.dylib /usr/local/lib/liboctomath.1.9.0.dylib /usr/local/lib/libfcl.0.6.0.dylib /usr/local/lib/libccd.2.0.dylib)

set(PROJECT_SOURCE_FILES src/main.cc Edge_Extraction/Difference_Eigenvalues.cpp hough3d-code/hough3dlines.cpp hough3d-code/pointcloud.cpp hough3d-code/sphere.cpp hough3d-code/vector3d.cpp hough3d-code/hough.cpp src/resultWriter.cc src/line_descriptor.cc src/filter.cc src/findTransformation.cc src/parabolicCylinder.cc src/decisionTreeParabolic.cc src/intersection.cc src/icp.cc src/boundingBox.cc)

find_package(OpenMP)

if(OPENMP_FOUND)
	message("OpenMP found")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${ICP_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})
add_executable(mapMerge ${PROJECT_SOURCE_FILES} ${ICP_SRC_FILES})
target_link_libraries(mapMerge ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIB_FILES} ${OCTOMAP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES})
