cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project( vehicle_detection )

find_package( PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# add_executable( vh_detect_holder vh_detect_holder.cpp )
# target_link_libraries( vh_detect_holder ${PCL_LIBRARIES} )

# add_executable( vh_detect_holder_reprocess vh_detect_holder_reprocess.cpp )
# target_link_libraries( vh_detect_holder_reprocess ${PCL_LIBRARIES} )

add_executable( vh_detect_rail vh_detect_rail.cpp )
target_link_libraries( vh_detect_rail ${PCL_LIBRARIES} )

add_executable( vh_detect_rail_old vh_detect_rail_old.cpp )
target_link_libraries( vh_detect_rail_old ${PCL_LIBRARIES} )

# add_executable( vh_detect_rail_slant vh_detect_rail_slant.cpp )
# target_link_libraries( vh_detect_rail_slant ${PCL_LIBRARIES} )

# add_executable( vh_detect_revolute_v1 vh_detect_revolute_v1.cpp )
# target_link_libraries( vh_detect_revolute_v1 ${PCL_LIBRARIES} )

# add_executable( vh_detect_revolute_v2 vh_detect_revolute_v2.cpp )
# target_link_libraries( vh_detect_revolute_v2 ${PCL_LIBRARIES} )

# add_executable( vh_detect_revolute_v3 vh_detect_revolute_v3.cpp )
# target_link_libraries( vh_detect_revolute_v3 ${PCL_LIBRARIES} )

