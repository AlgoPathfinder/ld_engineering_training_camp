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
#file(GLOB  LDLIDAR_DATA_MATCHING ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_matching/*.cpp)
#file(GLOB  LDLIDAR_DATA_MATCH_POINTS_MAP ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_match_points_map/*.cpp)
#file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/logger/*.cpp)
#file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/networkcom/*.cpp)
#file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/serialcom/*.cpp)

#file(GLOB  MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/linux_demo/*.cpp)
#file(GLOB  PRESSURE_TEST ${CMAKE_CURRENT_SOURCE_DIR}/src/pressure_test/*.cpp)
#file(GLOB  LIDAR_SHOW_DEMO ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_show_demo/*.cpp)

add_executable(lidar_data_show
	${LDLIDAR_DATA_SHOW})	
target_link_libraries(lidar_data_show ${OpenCV_LIBS})

#add_executable(lidar_data_matching 
#	${LDLIDAR_DATA_MATCHING})
#target_link_libraries(lidar_data_matching ${OpenCV_LIBS})

#add_executable(lidar_data_match_points_map
#	${LDLIDAR_DATA_MATCH_POINTS_MAP})
#target_link_libraries(lidar_data_match_points_map ${OpenCV_LIBS})