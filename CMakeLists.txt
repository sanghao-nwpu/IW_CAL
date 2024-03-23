cmake_minimum_required(VERSION 3.0.0)
project(imu_wheel_calibration VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -Werror -std=c11")
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found: ${EIGEN3_FOUND}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3 libraries: ${EIGEN3_LIBRARIES}")
include_directories(${EIGEN3_INCLUDE_DIR})

# glog
find_package(Glog REQUIRED)
include_directories(${PACKAGE_PREFIX_DIR}/include/glog)
message("GLOG_INCLUDE_DIRS: ${PACKAGE_PREFIX_DIR}/include/glog")


add_executable(imu_wheel_calibration main.cpp)
target_link_libraries(imu_wheel_calibration ${GLOG_LIBRARIES})
message(STATUS "Linking with ${GLOG_LIBRARIES}")
