cmake_minimum_required( VERSION 2.8.3 )
project( lds_controler )

add_compile_options( -std=c++11 )

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  laser_geometry
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros laser_geometry tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

link_directories( ${PCL_LIBRARY_DIRS} )

add_executable( lds_holder_control src/lds_holder_control.cpp )
target_link_libraries( lds_holder_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( lds_rail_control src/lds_rail_control.cpp )
target_link_libraries( lds_rail_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( lds_revolute_control src/lds_revolute_control.cpp )
target_link_libraries( lds_revolute_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( lds_revolute_multi_control src/lds_revolute_multi_control.cpp )
target_link_libraries( lds_revolute_multi_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
