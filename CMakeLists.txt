cmake_minimum_required(VERSION 3.1)
project(rlss)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party/splx)
find_package(Boost REQUIRED)

add_library(rlss INTERFACE)
target_include_directories(
    rlss INTERFACE
    include
    ${Boost_INCLUDE_DIRS}
    third_party/libMultiRobotPlanning/include
)

target_link_libraries(
    rlss INTERFACE
    splx
    qp_wrappers
    ${Boost_LIBRARIES}
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    include(tests/Tests.cmake)
endif()

option(BUILD_RLSS_EXAMPLES "build examples of rlss" OFF)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_RLSS_EXAMPLES)
    include(examples/Examples.cmake)
endif()
