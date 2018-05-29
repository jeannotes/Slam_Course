cmake_minimum_required( VERSION 2.8 )
project( useEigen )

# method 1
# include_directories("/usr/include/eigen3")

# method 2 use find find_package
# reference https://blog.csdn.net/u011092188/article/details/61425924

add_executable(useEigen code.cpp)

find_package(Eigen3 REQUIRED)
message(${EIGEN3_INCLUDE_DIR})
include_directories("${EIGEN3_INCLUDE_DIR}")

target_link_libraries(useEigen ${EIGEN3_LIBRARIES})