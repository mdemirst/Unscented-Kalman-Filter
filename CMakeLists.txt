project(Unscented-Kalman-Filter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  include/UKF
  external_libs)

FILE(GLOB_RECURSE HEADER_FILES include *.h)


add_library(kf
  src/UKF/kalman_filter.cpp)

add_library(ukf
  src/UKF/unscented_kalman_filter.cpp)

add_library(ukf_lidar
  src/ukf_lidar.cpp
  src/tools.cpp)

add_library(ukf_radar
  src/ukf_radar.cpp
  src/tools.cpp)

add_library(ukf_fusion
  src/ukf_fusion.cpp
  src/tools.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(UnscentedKF src/main.cpp ${HEADER_FILES})

target_link_libraries(ukf kf)

target_link_libraries(ukf_lidar ukf)

target_link_libraries(ukf_radar ukf)

target_link_libraries(ukf_fusion ukf_lidar ukf_radar)

target_link_libraries(UnscentedKF ukf_fusion z ssl uv uWS)

################################
# GTest
################################
ADD_SUBDIRECTORY (external_libs/gtest-1.8.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
add_executable(kalman_filter_test test/kalman_filter.cpp)
target_link_libraries(kalman_filter_test kf gtest gtest_main pthread)
add_test(kalman_filter_test kalman_filter_test)
