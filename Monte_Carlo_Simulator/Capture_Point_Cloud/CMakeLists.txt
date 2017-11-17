cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(capture_pc)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (capture_pc capture_pc.cpp)
target_link_libraries (capture_pc ${PCL_LIBRARIES})

set_property(TARGET capture_pc PROPERTY CXX_STANDARD 11)
set_property(TARGET capture_pc PROPERTY CXX_STANDARD_REQUIRED ON)
