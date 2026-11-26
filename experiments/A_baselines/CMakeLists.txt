cmake_minimum_required(VERSION 3.5)
project(a_baselines)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

if(NOT DEFINED ORB_SLAM3_ROOT)
  message(FATAL_ERROR "Set ORB_SLAM3_ROOT to the ORB-SLAM3 checkout")
endif()

include_directories(
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  "${EIGEN3_INCLUDE_DIR}"
  "${Pangolin_INCLUDE_DIRS}"
)

add_executable(run_tartanair run_tartanair.cc)
target_link_libraries(run_tartanair
  "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  boost_serialization
  crypto
)
set_target_properties(run_tartanair PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
)
