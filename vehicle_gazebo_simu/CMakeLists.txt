cmake_minimum_required(VERSION 2.8.3)
project(vehicle_gazebo_simu)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_vehicle
#  CATKIN_DEPENDS vehicle_detection
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


add_library(PriusPlugin SHARED plugins/PriusPlugin.cpp)
target_link_libraries(PriusPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusPlugin ${catkin_EXPORTED_TARGETS})


# add_library(gazebo_ros_laser SHARED plugins/gazebo_ros_laser.cpp)
# target_link_libraries(gazebo_ros_laser ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
# add_dependencies(gazebo_ros_laser ${catkin_EXPORTED_TARGETS})

install(TARGETS PriusPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

