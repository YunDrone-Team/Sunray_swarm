cmake_minimum_required(VERSION 2.8.3)
project(sunray_swarm)

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
  include_ugv
  include_rmtt
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################

add_executable(rmtt_show src/rmtt_show.cpp)
add_dependencies(rmtt_show sunray_swarm_gencpp)
target_link_libraries(rmtt_show ${catkin_LIBRARIES})

add_executable(rmtt_nokov src/rmtt_nokov.cpp)
add_dependencies(rmtt_nokov sunray_swarm_gencpp)
target_link_libraries(rmtt_nokov ${catkin_LIBRARIES})


## 无人车控制
add_executable(ugv_control_node
  ugv_control/ugv_control_node.cpp
  ugv_control/ugv_control.cpp	
)
target_link_libraries(ugv_control_node ${catkin_LIBRARIES})

## 无人车仿真
add_executable(ugv_sim_node
  ugv_control/ugv_sim.cpp	
  ugv_control/ugv_sim_node.cpp
)
target_link_libraries(ugv_sim_node ${catkin_LIBRARIES})

## 无人车地面站
add_executable(ugv_station ugv_control/ugv_station.cpp)
add_dependencies(ugv_station sunray_swarm_gencpp)
target_link_libraries(ugv_station ${catkin_LIBRARIES})

## RMTT控制
add_executable(rmtt_control_node
  rmtt_control/rmtt_control_node.cpp
  rmtt_control/rmtt_control.cpp	
)
target_link_libraries(rmtt_control_node ${catkin_LIBRARIES})

## RMTT仿真
add_executable(rmtt_sim_node
  rmtt_control/rmtt_sim.cpp	
  rmtt_control/rmtt_sim_node.cpp
)
target_link_libraries(rmtt_sim_node ${catkin_LIBRARIES})

## RMTT地面站
add_executable(rmtt_station rmtt_control/rmtt_station.cpp)
add_dependencies(rmtt_station sunray_swarm_gencpp)
target_link_libraries(rmtt_station ${catkin_LIBRARIES})

## ORCA
add_executable(orca_node
  orca/orca_node.cpp
  orca/orca.cpp	
  orca/Agent.cpp
  orca/KdTree.cpp
	orca/Obstacle.cpp
	orca/RVOSimulator.cpp
	)
target_link_libraries(orca_node ${catkin_LIBRARIES})


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
