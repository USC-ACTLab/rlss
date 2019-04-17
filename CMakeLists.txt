cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "-O3 -Wno-ignored-attributes" CACHE STRING)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -DACT_GENERATE_OUTPUT_JSON" CACHE STRING "debug flags" FORCE)
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
project (ACT_TRAJ_REPLANNING)



add_subdirectory(lib/qpOASES)

set(
SVM_CPP_FILES
cvxgen/16robot8obstacle/solver.cpp
cvxgen/16robot8obstacle/ldl.cpp
cvxgen/16robot8obstacle/util.cpp
cvxgen/16robot8obstacle/matrix_support.cpp
cvxgen/64robot8obstacle/solver.cpp
cvxgen/64robot8obstacle/ldl.cpp
cvxgen/64robot8obstacle/util.cpp
cvxgen/64robot8obstacle/matrix_support.cpp
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  lib/qpOASES/include
  lib/splx/include
)

add_executable(
main
main.cpp
${SVM_CPP_FILES}
)
target_include_directories(main PRIVATE lib solvers)

add_executable(
svmtest
svmtest.cpp
${SVM_CPP_FILES}
)
target_include_directories(svmtest PRIVATE lib)

add_executable(
pointcloudtest
pointcloudtest.cpp
)
target_include_directories(pointcloudtest PRIVATE lib)

add_library(csv
  lib/csv/CSVparser.cpp
)


target_link_libraries(
  main
  csv
  stdc++fs
  qpOASES
)

target_compile_definitions(main PRIVATE ACT_GENERATE_OUTPUT_JSON ACT_DEBUG)
