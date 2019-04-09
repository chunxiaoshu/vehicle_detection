cmake_minimum_required(VERSION 2.8.3)
project(lds_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lds_data
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lds_data src/lds_data.cpp)
target_link_libraries(lds_data ${catkin_LIBRARIES})
