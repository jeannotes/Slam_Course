cmake_minimum_required(VERSION 2.8)
project(demo)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(demo main.cpp)
target_link_libraries(demo ${CERES_LIBRARIES})