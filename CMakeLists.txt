cmake_minimum_required(VERSION 3.0.2)
project(hypascore_localization LANGUAGES CXX)

# compiler options
add_compile_options(-O3 -DNDEBUG)
set(CMAKE_CXX_STANDARD 17)

# catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)
catkin_package()

# IBEX
if(NOT(TARGET Ibex::ibex) AND NOT(TARGET Ibex::gaol) AND NOT(TARGET Ibex::gdtoa) AND NOT(TARGET Ibex::soplex))
  find_package(IBEX REQUIRED)
  message(STATUS "Found IBEX version ${IBEX_VERSION}")
endif()
get_target_incdirs (IBEX_INCLUDE_DIRS Ibex::ibex)
message(STATUS "IBEX include directories: ${IBEX_INCLUDE_DIRS}")

# CODAC
find_package(CODAC REQUIRED)
message(STATUS "Found Codac version ${CODAC_VERSION}")

# OpenCV
set(OpenCV_DIR /home/user/workspaces/libraries/opencv/build) # Set directory if built locally
find_package(OpenCV REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED)

# BOOST 
find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)

# g2o
LIST(APPEND CMAKE_MODULE_PATH /home/user/workspaces/libraries/g2o/cmake_modules)
find_package(G2O REQUIRED)
# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)
# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)
FIND_G2O_LIBRARY(G2O_OPENGL_LIBRARY opengl_helper)
# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)
# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)
set(G2O_USE_VENDORED_CERES true)
SET(G2O_LIBS 
${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE}
${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} 
${G2O_SOLVER_STRUCTURE_ONLY} ${G2O_SOLVER_EIGEN} ${G2O_TYPES_DATA} ${G2O_TYPES_ICP} ${G2O_TYPES_SBA}
${G2O_TYPES_SCLAM2D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D} ${G2O_OPENGL_LIBRARY})

# OpenMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
  ${IBEX_INCLUDE_DIRS}
  ${CODAC_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

add_executable(hypascore_localization_node 
                                      # high-level executables
                                      src/hypascore_localization_node.cpp 
                                      src/PoseEstimationSystem.cpp
                                      # Visual odoemtry object 
                                      src/visual_odometry/VisualOdometry.cpp

                                      # objects for localization
                                      # LiDAR object 
                                      src/localization/LidarData.cpp
                                      # GPS object
                                      src/localization/GpsData.cpp
                                      # Map object
                                      src/localization/CMMap.cpp
                                      # global localization
                                      src/localization/coarse/IntervalPose.cpp
                                      src/localization/coarse/CoarseLocalizer.cpp
                                      # tracking
                                      src/localization/refined/HoughAccumulator.cpp
                                      src/localization/refined/RefinedLocalizer.cpp

                                      # contractors 
                                      src/contractors/Ctc3DRotation.cpp 
                                      src/contractors/Ctc3DTransform.cpp
                                      src/contractors/Ctc3DStereoVO.cpp
                                      src/contractors/CtcBoundaryLine.cpp
                                      src/contractors/CtcNoCross.cpp
                                      src/contractors/CtcRangeBearingNoCross.cpp
                                      src/contractors/CtcRaster.cpp
                                      src/contractors/SepPolygon.cpp
                                      src/contractors/CtcRange.cpp)
target_link_libraries(hypascore_localization_node PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBS} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${G2O_LIBS})

# Test Scripts to visualize the operation of the applied contractors
# Pose Contractors
add_executable(test_ctc3drotation_node src/test/test_Ctc3DRotation.cpp src/contractors/Ctc3DRotation.cpp)
target_link_libraries(test_ctc3drotation_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX)
add_executable(test_ctc3dtransform_node src/test/test_Ctc3DTransform.cpp src/contractors/Ctc3DTransform.cpp  src/contractors/Ctc3DRotation.cpp)
target_link_libraries(test_ctc3dtransform_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX)
add_executable(test_ctc3dstereovo_node src/test/test_Ctc3DStereoVO.cpp src/contractors/Ctc3DStereoVO.cpp src/contractors/Ctc3DRotation.cpp)
target_link_libraries(test_ctc3dstereovo_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX)
# Basic contractors
# CtcRange
add_executable(test_ctcrange_node src/test/test_CtcRange.cpp src/contractors/CtcRange.cpp)
target_link_libraries(test_ctcrange_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})
# CtcBoundaryLine
add_executable(test_ctcboundaryline_node src/test/test_CtcBoundaryLine.cpp src/contractors/CtcBoundaryLine.cpp)
target_link_libraries(test_ctcboundaryline_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})
# CtcNoCross
add_executable(test_ctcnocross_node src/test/test_CtcNoCross.cpp src/contractors/CtcNoCross.cpp)
target_link_libraries(test_ctcnocross_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})
# CtcRangeBearingNoCross
add_executable(test_ctcrangebearingnocross_node src/test/test_CtcRangeBearingNoCross.cpp src/contractors/CtcRangeBearingNoCross.cpp src/contractors/CtcNoCross.cpp)
target_link_libraries(test_ctcrangebearingnocross_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})
# SepPolygon
add_executable(test_seppolygon_node src/test/test_SepPolygon.cpp src/contractors/SepPolygon.cpp src/contractors/CtcBoundaryLine.cpp)
target_link_libraries(test_seppolygon_node PUBLIC ${catkin_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})