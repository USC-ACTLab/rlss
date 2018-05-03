cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")
project (ACT_TRAJ_REPLANNING)

find_package (Eigen3 REQUIRED)


add_definitions(-DHAVE_CSTDDEF)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  /home/act/qpOASES-3.2.1/include/
# /home/whoenig/sw/qpOASES-3.2.1/include/
#  /home/whoenig/sw/osqp/include/
)


link_directories(
  /home/act/qpOASES-3.2.1/build/libs/
# /home/whoenig/sw/qpOASES-3.2.1/bin/
# /home/whoenig/sw/osqp/build/out/
)

add_executable(main main.cpp)
target_include_directories(main PRIVATE lib)

find_package(stdc++fs)
add_library(geometry hyperplane.cpp vectoreuc.cpp edt.cpp edtv2.cpp occupancy_grid.cpp)
add_library(svm svm_seperator.cpp lib/libsvm/svm.cpp)
target_include_directories(svm PRIVATE lib)
add_library(obstacle obstacle.cpp)
add_library(traj curve.cpp trajectory.cpp bezier_2d_8pts.cpp)
add_library(csv lib/csv/CSVparser.cpp)
add_library(optimization
  optimization.cpp
  svmoptimization.cpp
  ipopt_optimize.cpp
  qp_optimize.cpp
)

target_link_libraries(
  main
  geometry
  svm
  obstacle
  optimization
  traj
  csv
  nlopt
  stdc++fs
  ipopt
  qpOASES
#  osqp
)
