cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})

add_executable (filter1 filter1.cpp)
target_link_libraries (filter1 ${PCL_LIBRARIES})