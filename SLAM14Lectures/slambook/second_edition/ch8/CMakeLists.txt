cmake_minimum_required(VERSION 2.8)
project(ch8)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} ) 
# gflags
find_package(GFlags REQUIRED)

## glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

include_directories( "/usr/include/eigen3" )

set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

add_executable(direct_method direct_semidense.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS}  ${G2O_LIBS}  ${GLOG_LIBRARIES})

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS}  ${G2O_LIBS}  ${GLOG_LIBRARIES})

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS}  ${G2O_LIBS}  ${GLOG_LIBRARIES})