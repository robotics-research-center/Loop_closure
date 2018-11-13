cmake_minimum_required (VERSION 3.5.1)
project (icp)
find_package (PCL 1.7 REQUIRED)
set (CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cloud_assemble cloud_assemble.cpp)

target_link_libraries(cloud_assemble ${PCL_LIBRARIES})