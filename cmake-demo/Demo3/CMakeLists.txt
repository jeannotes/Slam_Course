cmake_minimum_required(VERSION 2.8)
project(demo3)
add_subdirectory(math)
add_executable(demo3 main.cc)

target_link_libraries(demo3 MathFunctions)