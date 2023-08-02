cmake_minimum_required(VERSION 2.8.3)
project(ld_engineering_training_camp)

find_package(OpenCV REQUIRED)

## Compile option
if(${CMAKE_BUILD_TYPE} MATCHES "Release")
  #set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall")
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11 -Wall")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
else()
  set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_common/
${OpenCV_INCLUDE_DIRS}
)

file(GLOB  LDLIDAR_DATA_SHOW ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_show/*.cpp)
file(GLOB  LDLIDAR_DATA_NAIVE_ROTATION_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_naive_rotation_matching/*.cpp)
file(GLOB  LDLIDAR_DATA_NAIVE_TRANSLATION_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_naive_translation_matching/*.cpp)

file(GLOB  LDLIDAR_DATA_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_matching/*.cpp)
file(GLOB  LDLIDAR_DATA_MATCH_POINTS_MAP ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_match_points_map/*.cpp)
file(GLOB  LDLIDAR_GRID_MAP_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_grid_map_matching/*.cpp)
#file(GLOB  GRID_MATCHING_WITH_RTREE ${CMAKE_CURRENT_SOURCE_DIR}/src/grid_matching_with_rtree/*.cpp)
file(GLOB  PATH_PLANING_WITH_ASTAR ${CMAKE_CURRENT_SOURCE_DIR}/src/path_planing_with_astar/*.cpp)
#file(GLOB  MAZE_RUNNING_WITH_SLAM_AND_NAVIGATION ${CMAKE_CURRENT_SOURCE_DIR}/src/maze_running_with_slam_and_navigation/*.cpp)


add_executable(lidar_data_show
	${LDLIDAR_DATA_SHOW})	
target_link_libraries(lidar_data_show ${OpenCV_LIBS})

add_executable(lidar_data_naive_rotation_matching
	${LDLIDAR_DATA_NAIVE_ROTATION_MATCHING})	
target_link_libraries(lidar_data_naive_rotation_matching ${OpenCV_LIBS})

add_executable(lidar_data_naive_translation_matching
	${LDLIDAR_DATA_NAIVE_TRANSLATION_MATCHING})	
target_link_libraries(lidar_data_naive_translation_matching ${OpenCV_LIBS})

add_executable(lidar_data_matching 
	${LDLIDAR_DATA_MATCHING})
target_link_libraries(lidar_data_matching ${OpenCV_LIBS})

add_executable(lidar_data_match_points_map
	${LDLIDAR_DATA_MATCH_POINTS_MAP})
target_link_libraries(lidar_data_match_points_map ${OpenCV_LIBS})

add_executable(lidar_grid_map_matching 
	${LDLIDAR_GRID_MAP_MATCHING})
target_link_libraries(lidar_grid_map_matching ${OpenCV_LIBS})

#add_executable(grid_matching_with_rtree
#	${GRID_MATCHING_WITH_RTREE})
#target_link_libraries(grid_matching_with_rtree ${OpenCV_LIBS})

add_executable(path_planing_with_astar
	${PATH_PLANING_WITH_ASTAR})
target_link_libraries(path_planing_with_astar ${OpenCV_LIBS})

#add_executable(maze_running_with_slam_and_navigation
#	${MAZE_RUNNING_WITH_SLAM_AND_NAVIGATION})
#target_link_libraries(maze_running_with_slam_and_navigation ${OpenCV_LIBS})
