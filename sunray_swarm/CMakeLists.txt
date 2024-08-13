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
  sim
  agent_control
  orca
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################

## 无人车控制
add_executable(ugv_control_node
  agent_control/ugv_control_node.cpp
  agent_control/ugv_control.cpp	
)
target_link_libraries(ugv_control_node ${catkin_LIBRARIES})

## RMTT控制
add_executable(rmtt_control_node
  agent_control/rmtt_control_node.cpp
  agent_control/rmtt_control.cpp	
)
target_link_libraries(rmtt_control_node ${catkin_LIBRARIES})

## 终端地面站
add_executable(agent_station agent_control/agent_station.cpp)
add_dependencies(agent_station sunray_swarm_gencpp)
target_link_libraries(agent_station ${catkin_LIBRARIES})

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

## 仿真
add_executable(agent_sim_node
  sim/agent_sim.cpp	
  sim/agent_sim_node.cpp
)
target_link_libraries(agent_sim_node ${catkin_LIBRARIES})

## DEMO - 三智能体绕圈
add_executable(circle_trajectory demo/circle_trajectory.cpp)
add_dependencies(circle_trajectory sunray_swarm_gencpp)
target_link_libraries(circle_trajectory ${catkin_LIBRARIES})

## DEMO - 目标追踪
add_executable(track_mission demo/track_mission.cpp)
add_dependencies(track_mission sunray_swarm_gencpp)
target_link_libraries(track_mission ${catkin_LIBRARIES})

## DEMO - NOKOV队形
add_executable(formation_nokov demo/formation_nokov.cpp)
add_dependencies(formation_nokov sunray_swarm_gencpp)
target_link_libraries(formation_nokov ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
