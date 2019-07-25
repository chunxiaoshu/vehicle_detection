cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(trunk_measuring trunk_measuring.cpp)
target_link_libraries(trunk_measuring ${PCL_LIBRARIES})
