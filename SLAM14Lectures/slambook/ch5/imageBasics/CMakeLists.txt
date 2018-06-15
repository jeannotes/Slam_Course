cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 2.4 REQUIRED)
#find_package( OpenCV REQUIRED PATHS "/usr/local/opencv-3.4.1/share/OpenCV")
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(${OpenCV_INCLUDE_DIRS})
add_executable( imageBasics imageBasics.cpp )
# 链接OpenCV库
target_link_libraries( imageBasics ${OpenCV_LIBS} )