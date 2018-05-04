cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(icp_module)

find_package(PCL 1.5 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  pcl_ros)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_executable (icp_module src/icp_module.cpp)
target_link_libraries (icp_module ${PCL_LIBRARIES})
target_link_libraries(icp_module ${catkin_LIBRARIES})

