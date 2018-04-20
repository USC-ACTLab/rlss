cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")
project (ACT_TRAJ_REPLANNING)

add_definitions(-DHAVE_CSTDDEF)

add_executable(main main.cpp)
target_include_directories(main PRIVATE lib)

find_package(stdc++fs)
add_library(geometry hyperplane.cpp vectoreuc.cpp edt.cpp)
add_library(obstacle obstacle.cpp)
add_library(traj curve.cpp trajectory.cpp bezier_2d_8pts.cpp)
add_library(csv lib/csv/CSVparser.cpp)
add_library(optimization optimization.cpp svmoptimization.cpp)
target_link_libraries(main geometry obstacle optimization traj csv nlopt stdc++fs ipopt)
