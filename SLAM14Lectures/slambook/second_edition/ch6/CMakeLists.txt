cmake_minimum_required(VERSION 2.8)
project(ch6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")

#add_executable(gaussNewton gaussNewton.cpp)
#target_link_libraries(gaussNewton ${OpenCV_LIBS})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(simple_bundle_adjuster official_code/simple_bundle_adjuster.cpp)
target_link_libraries(simple_bundle_adjuster ${OpenCV_LIBS} ${CERES_LIBRARIES}
                      ${GFLAGS_LIBRARIES}
                      )

add_executable(bundle_adjuster official_code/bundle_adjuster.cc official_code/bal_problem.cc)
target_link_libraries(bundle_adjuster ${OpenCV_LIBS} ${CERES_LIBRARIES}
                      ${GFLAGS_LIBRARIES}
                      )
add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o_core g2o_stuff)
