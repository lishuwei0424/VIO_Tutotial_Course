cmake_minimum_required(VERSION 3.5)
project(Triangulate)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
"/usr/include/eigen3"
)

add_executable(estimate_depth triangulate.cpp)
#target_link_libraries(estimate_depth  ${Sophus_LIBRARIES})
