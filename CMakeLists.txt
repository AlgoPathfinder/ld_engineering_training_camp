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
${CMAKE_CURRENT_SOURCE_DIR}/src/alg_common/
${OpenCV_INCLUDE_DIRS}
)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/core/
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/dataprocess/
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/filter/
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/logger/
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/networkcom/
${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/include/serialcom/
)

file(GLOB  LDLIDAR_DRI_CORE ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/core/*.cpp)
file(GLOB  LDLIDAR_DRI_DATARPC ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/dataprocess/*.cpp)
file(GLOB  LDLIDAR_DRI_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/filter/*.cpp)
file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/logger/*.cpp)
file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/networkcom/*.cpp)
file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/src/serialcom/*.cpp)


file(GLOB  LDLIDAR_DATA_SHOW ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_show/*.cpp)
file(GLOB  LDLIDAR_DATA_NAIVE_ROTATION_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_naive_rotation_matching/*.cpp)
file(GLOB  LDLIDAR_DATA_NAIVE_TRANSLATION_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_naive_translation_matching/*.cpp)

file(GLOB  LDLIDAR_DATA_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_matching/*.cpp)
file(GLOB  LDLIDAR_DATA_MATCH_POINTS_MAP ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_match_points_map/*.cpp)
file(GLOB  LDLIDAR_GRID_MAP_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_grid_map_matching/*.cpp)
file(GLOB  GRID_MATCHING_WITH_RTREE ${CMAKE_CURRENT_SOURCE_DIR}/src/grid_matching_with_rtree/*.cpp)
file(GLOB  PATH_PLANING_WITH_ASTAR ${CMAKE_CURRENT_SOURCE_DIR}/src/path_planing_with_astar/*.cpp)
file(GLOB  PLAY_SOUND ${CMAKE_CURRENT_SOURCE_DIR}/src/play_sound/*.cpp)
file(GLOB  LD_CAMP ${CMAKE_CURRENT_SOURCE_DIR}/src/ld_camp/*.cpp)
file(GLOB  ICP_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/icp_matching/*.cpp)
file(GLOB  ALG_COMMON ${CMAKE_CURRENT_SOURCE_DIR}/src/alg_common/*.cpp)
file(GLOB  GRID_LOCALIZATION ${CMAKE_CURRENT_SOURCE_DIR}/src/grid_localization/*.cpp)


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

add_executable(grid_matching_with_rtree
	${GRID_MATCHING_WITH_RTREE})
target_link_libraries(grid_matching_with_rtree ${OpenCV_LIBS})

add_executable(path_planing_with_astar
	${PATH_PLANING_WITH_ASTAR})
target_link_libraries(path_planing_with_astar ${OpenCV_LIBS})

add_executable(play_sound
	${PLAY_SOUND})
target_link_libraries(play_sound)

add_executable(grid_localization
	${GRID_LOCALIZATION})
target_link_libraries(grid_localization ${OpenCV_LIBS})

add_executable(ld_camp
	${LD_CAMP})
target_link_libraries(ld_camp pthread ${OpenCV_LIBS})

add_executable(icp_matching
	${ICP_MATCHING}
  ${ALG_COMMON})
target_link_libraries(icp_matching  ${OpenCV_LIBS})

