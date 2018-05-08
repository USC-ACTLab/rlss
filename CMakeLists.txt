cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")
project (ACT_TRAJ_REPLANNING)

add_subdirectory(lib/qpOASES)
set(UNITTESTS OFF CACHE BOOL "disable osqp unittests")
add_subdirectory(lib/osqp)

add_subdirectory(lib/RVO2)

find_package (Eigen3 REQUIRED)
find_package(stdc++fs)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  lib/qpOASES/include
)

add_executable(main main.cpp)
target_include_directories(main PRIVATE lib)

add_library(svm2pt4obs
  cvxgen/2pt4obspt/cvxgen/solver.cpp
  cvxgen/2pt4obspt/cvxgen/ldl.cpp
  cvxgen/2pt4obspt/cvxgen/matrix_support.cpp
  cvxgen/2pt4obspt/cvxgen/util.cpp
  svmcvxwrapper_2pt_4obs.cpp
)

add_library(svm8pt4obs
  cvxgen/8pt4obspt/cvxgen/solver.cpp
  cvxgen/8pt4obspt/cvxgen/ldl.cpp
  cvxgen/8pt4obspt/cvxgen/matrix_support.cpp
  cvxgen/8pt4obspt/cvxgen/util.cpp
  svmcvxwrapper_8pt_4obs.cpp
)

add_library(geometry
  hyperplane.cpp
  vectoreuc.cpp
  occupancy_grid.cpp
)

add_library(svm
  svm_seperator.cpp
  lib/libsvm/svm.cpp
)

target_include_directories(svm PRIVATE lib)

add_library(obstacle
  obstacle.cpp
)

add_library(traj
  curve.cpp
  trajectory.cpp
  bezier_2d_8pts.cpp
)

add_library(csv
  lib/csv/CSVparser.cpp
)

add_library(optimization
  svmoptimization.cpp
  qp_optimize.cpp
)

target_link_libraries(main
  geometry
  svm
  obstacle
  optimization
  traj
  csv
  stdc++fs
  qpOASES
  svm2pt4obs
  svm8pt4obs
  osqp
)

add_executable(rvo2_main
  rvo2_main.cpp
)
target_include_directories(rvo2_main PRIVATE lib lib/RVO2/src)
target_link_libraries(rvo2_main
  obstacle
  geometry
  traj
  csv
  stdc++fs
  RVO
)
