cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wno-ignored-attributes")
project (ACT_TRAJ_REPLANNING_TEST)

add_subdirectory(googletest)

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ../
)

add_executable(
testmain
main.cpp
test_PointCloud.cpp
)

target_link_libraries(
testmain
gtest
)
