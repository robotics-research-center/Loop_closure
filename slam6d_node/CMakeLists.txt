cmake_minimum_required(VERSION 2.8.3)
project(pcd_extract)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp sensor_msgs nav_msgs)
find_package(Boost REQUIRED COMPONENTS signals)


generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
#include_directories(Boost_INCLUDE_DIRS)

add_executable(pcd_extract src/pcd_extract.cpp)
add_executable(pcd_sync src/pcd_sync.cpp)

target_link_libraries(pcd_extract ${catkin_LIBRARIES} )
target_link_libraries(pcd_sync ${catkin_LIBRARIES} )