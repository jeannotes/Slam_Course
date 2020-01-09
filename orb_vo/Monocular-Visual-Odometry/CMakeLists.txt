cmake_minimum_required( VERSION 2.8 )
project ( myslam )

# compile settings
#set( CMAKE_CXX_COMPILER "gcc" )
#set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_BUILD_TYPE "Release" ) # *** stack smashing detected ***: <unknown> terminated
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )


#SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)


list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# gflags
find_package(GFlags REQUIRED)

## glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# set ccache (for speeding up)
#find_program(CCACHE_FOUND ccache) 
#if(CCACHE_FOUND) 
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
#endif(CCACHE_FOUND) 
#set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 

############### Dependencies ######################

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED ) ## note!! if the csparse is not found, please install: sudo apt-get install libsuitesparse-dev
include_directories( ${G2O_INCLUDE_DIRS} )
include_directories( ${CSPARSE_INCLUDE_DIR} )
# PCL 
find_package( PCL REQUIRED ) 
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Combine the above libraries
set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES} # If "make install" is good, use this line
    libSophus.so # If "make install" failed, copy files manully to usr/lib and usr/include, and use this line
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
    ${CSPARSE_LIBRARY}
    ${GFLAGS_LIBRARIES} 
    ${GLOG_LIBRARIES}
)

############### My Files ###############
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )

add_executable(run_vo run_vo.cpp)
target_link_libraries( run_vo
    basics
    geometry
    display
    vo
    ${GFLAGS_LIBRARIES} 
    ${GLOG_LIBRARIES}
)