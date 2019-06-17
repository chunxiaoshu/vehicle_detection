cmake_minimum_required( VERSION 2.8.3 )
project( vehicle_description )

add_compile_options( -std=c++11 )

find_package( gazebo REQUIRED )
find_package( catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_vehicle
#  CATKIN_DEPENDS vehicle_description
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
