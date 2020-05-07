cmake_minimum_required(VERSION 3.1)
project(rlss)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party/splx)

add_library(rlss INTERFACE)
target_include_directories(
    rlss INTERFACE
    include
)

target_link_libraries(
    rlss INTERFACE
    splx
    qp_wrappers
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    include(tests/Tests.cmake)
endif()