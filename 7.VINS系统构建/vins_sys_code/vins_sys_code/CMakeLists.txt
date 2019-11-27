cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# -o3 优化等级
# -wall  编译生成警告
# -g  生成符号表，进行调试
# https://cmake.org/documentation/
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)

#OpenCV3.4.3
set(OpenCV_DIR  /usr/local/opencv-3.4.3/share/OpenCV)
find_package(OpenCV 3.4.3   REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}  /usr/local/opencv-3.4.3/include)
link_directories(${OpenCV_LIBRARY_DIRS}  /usr/local/opencv-3.4.3/lib)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(camera_model SHARED
    src/camera_models/chessboard/Chessboard.cc
    src/camera_models/calib/CameraCalibration.cc
    src/camera_models/camera_models/Camera.cc
    src/camera_models/camera_models/CameraFactory.cc
    src/camera_models/camera_models/CostFunctionFactory.cc
    src/camera_models/camera_models/PinholeCamera.cc
    src/camera_models/camera_models/CataCamera.cc
    src/camera_models/camera_models/EquidistantCamera.cc
    src/camera_models/camera_models/ScaramuzzaCamera.cc
    src/camera_models/sparse_graph/Transform.cc
    src/camera_models/gpl/gpl.cc
    src/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


ADD_LIBRARY(MyVio SHARED
    src/System.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/feature_tracker.cpp

    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    src/backend/vertex.cpp
    src/backend/edge.cpp
    src/backend/problem.cpp
    src/backend/vertex_pose.cpp
    src/backend/edge_reprojection.cpp
    src/backend/edge_imu.cpp
    src/backend/edge_prior.cpp
    src/backend/loss_function.cpp
    src/backend/imu_integration.cpp
    )

target_link_libraries(MyVio  
  ${OpenCV_LIBS}  
  ${CERES_LIBRARIES}
	${Pangolin_LIBRARIES}
  camera_model)

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc 
  MyVio  
  -lpthread) 

add_executable(run_myslam test/run_myslam.cpp)
target_link_libraries(run_myslam 
  MyVio  
  -lpthread) 
add_executable(testCurveFitting test/CurveFitting.cpp)
target_link_libraries(testCurveFitting MyVio)

