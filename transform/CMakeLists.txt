cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(transform_pcl)

find_package(PCL 1.5 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  pcl_ros)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable (transform src/transform.cpp)
target_link_libraries (transform ${PCL_LIBRARIES})
target_link_libraries(transform ${catkin_LIBRARIES})
