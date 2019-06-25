cmake_minimum_required( VERSION 2.8.3 )
project( get_scan_data )

add_compile_options( -std=c++11 )

find_package( catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
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

add_executable( get_lds_data src/get_lds_data.cpp )
target_link_libraries( get_lds_data ${catkin_LIBRARIES} )

add_executable( get_lds_data_holder src/get_lds_data_holder.cpp )
target_link_libraries( get_lds_data_holder ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( get_lds_data_rail src/get_lds_data_rail.cpp )
target_link_libraries( get_lds_data_rail ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( get_lds_data_rail_slant src/get_lds_data_rail_slant.cpp )
target_link_libraries( get_lds_data_rail_slant ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( get_lds_data_revolute src/get_lds_data_revolute.cpp )
target_link_libraries( get_lds_data_revolute ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( get_lds_data_revolute_multi src/get_lds_data_revolute_multi.cpp )
target_link_libraries( get_lds_data_revolute_multi ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
