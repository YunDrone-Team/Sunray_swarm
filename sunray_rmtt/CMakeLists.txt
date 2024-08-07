cmake_minimum_required(VERSION 2.8.3)
project(sunray_rmtt)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  sunray_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS  message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)




###############################
##      声明可执行cpp文件     ##
###############################

add_executable(rmtt_show src/rmtt_show.cpp)
add_dependencies(rmtt_show sunray_rmtt_gencpp)
target_link_libraries(rmtt_show ${catkin_LIBRARIES})

add_executable(rmtt_nokov src/rmtt_nokov.cpp)
add_dependencies(rmtt_nokov sunray_rmtt_gencpp)
target_link_libraries(rmtt_nokov ${catkin_LIBRARIES})

add_executable(video src/rmtt_video.cpp)
add_dependencies(video sunray_rmtt_gencpp)
target_link_libraries(video ${catkin_LIBRARIES})

add_executable(tello_station src/tello_station.cpp)
add_dependencies(tello_station sunray_rmtt_gencpp)
target_link_libraries(tello_station ${catkin_LIBRARIES})

add_executable(rmtt_control_node
  src/rmtt_control_node.cpp
  src/rmtt_control.cpp	
	)
target_link_libraries(rmtt_control_node
    ${catkin_LIBRARIES}
	)

add_executable(rmtt_orca_node
  src/rmtt_orca_node.cpp
  src/rmtt_orca.cpp	
  orca/Agent.cpp
  orca/KdTree.cpp
	orca/Obstacle.cpp
	orca/RVOSimulator.cpp

	)
target_link_libraries(rmtt_orca_node
    ${catkin_LIBRARIES}
	)

add_executable(rmtt_sim_node
  sim/rmtt_sim.cpp	
	sim/rmtt_sim_node.cpp
	)
target_link_libraries(rmtt_sim_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
	)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
