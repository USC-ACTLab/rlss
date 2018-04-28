cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)

project (qp-test)

find_package (Eigen3 REQUIRED)

add_definitions(
  -D__USE_LONG_INTEGERS__
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  /home/whoenig/sw/qpOASES-3.2.1/include/
)

link_directories(
  /home/whoenig/sw/qpOASES-3.2.1/bin/
)

add_executable(qptest
  src/main.cpp)

target_link_libraries(qptest
  qpOASES
)
  # Eigen3::Eigen)
