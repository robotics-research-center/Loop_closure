cmake_minimum_required(VERSION 2.8)
project(icp_and_variants)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(registration_icp registration_icp.cpp)
target_link_libraries(registration_icp ${PCL_LIBRARIES})