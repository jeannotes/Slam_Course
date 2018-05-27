cmake_minimum_required(VERSION 2.8)
project(demo2)

aux_source_directory(. dir)

#add_executable(demo2 main.cc MathFunctions.cc)
add_executable(demo2 ${dir})