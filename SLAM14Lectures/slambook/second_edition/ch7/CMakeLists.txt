cmake_minimum_required(VERSION 2.8)
project(vo1)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package( CSparse REQUIRED )
find_package(OpenCV 3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/"
        "/usr/local/include/"
)

# gflags
find_package(GFlags REQUIRED)

## glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

add_library(find_matches find_matches.cpp find_matches.h)
target_link_libraries(find_matches ${OpenCV_LIBS} ${GLOG_LIBRARIES})

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS}
   ${GFLAGS_LIBRARIES}
     find_matches ${GLOG_LIBRARIES}
)

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS}
                  ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRARIES} 
)

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
                      ${GFLAGS_LIBRARIES}
      ${GLOG_LIBRARIES}
     find_matches
        ${OpenCV_LIBS})
add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d
      ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        ${GLOG_LIBRARIES} 
        ${OpenCV_LIBS})

add_executable( feature_extraction feature_extraction.cpp  )
target_link_libraries( feature_extraction
                       ${OpenCV_LIBS}
                       ${GFLAGS_LIBRARIES} 
                       ${GLOG_LIBRARIES}
                      )