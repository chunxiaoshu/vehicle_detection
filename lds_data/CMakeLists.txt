cmake_minimum_required(VERSION 2.8.3)
project(lds_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
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
link_directories(${PCL_LIBRARY_DIRS})

# add_executable(lds_data src/lds_data.cpp)
# target_link_libraries(lds_data ${catkin_LIBRARIES})

add_executable( lds_prismatic_data src/laserscan2pointcloud_prismatic.cpp )
target_link_libraries( lds_prismatic_data ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( lds_revolute_data src/laserscan2pointcloud_revolute.cpp)
target_link_libraries( lds_revolute_data ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

