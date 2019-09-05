cmake_minimum_required(VERSION 2.8.3)
project(erl_graph_slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES erl_graph_slam
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#################### Find GTSAM components ######################
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
####################  GTSAM End ######################

#################### Posegraph Node ######################
add_executable(posegraph src/posegraph.cpp)
target_link_libraries(posegraph 
  gtsam
  ${catkin_LIBRARIES}
)
target_link_libraries (posegraph Eigen3::Eigen)

#################### Test Gtsam Pose2SLAM Example ######################
add_executable(Pose2SLAM src/Pose2SLAM.cpp)
target_link_libraries(Pose2SLAM 
  gtsam
  ${catkin_LIBRARIES}
)
target_link_libraries (Pose2SLAM Eigen3::Eigen)

#################### Test Gtsam OdometryExample Example ######################
add_executable(OdometryExample src/OdometryExample.cpp)
target_link_libraries(OdometryExample 
  gtsam
  ${catkin_LIBRARIES}
)
target_link_libraries (Pose2SLAM Eigen3::Eigen)

