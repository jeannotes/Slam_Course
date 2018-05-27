cmake_minimum_required(VERSION 2.8)
project(demo4)

configure_file(
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

option(use_math
  "use math function provided" ON
)

if(use_math)
  include_directories(math)
  add_subdirectory(math)
  set(extra ${extra} MathFunctions) 
endif(use_math)

add_executable(demo4 main.cc)
target_link_libraries(demo4 ${extra})