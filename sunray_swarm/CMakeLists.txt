cmake_minimum_required(VERSION 2.8.3)
project(sunray_swarm)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
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
  agent_control
  orca
  ${catkin_INCLUDE_DIRS}
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
add_executable(agent_terminal_station agent_control/agent_terminal_station.cpp)
add_dependencies(agent_terminal_station sunray_swarm_gencpp)
target_link_libraries(agent_terminal_station ${catkin_LIBRARIES})

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

## 地图pose
add_executable(rmtt_tf
  agent_control/rmtt_tf.cpp
)
target_link_libraries(rmtt_tf ${catkin_LIBRARIES})

## RMTT_DEMO - 定点悬停
add_executable(rmtt_hover demo_rmtt/rmtt_hover.cpp)
add_dependencies(rmtt_hover sunray_swarm_gencpp)
target_link_libraries(rmtt_hover ${catkin_LIBRARIES})
## RMTT_DEMO - 无人机绕圆
add_executable(rmtt_circle demo_rmtt/rmtt_circle.cpp)
add_dependencies(rmtt_circle sunray_swarm_gencpp)
target_link_libraries(rmtt_circle ${catkin_LIBRARIES})
## RMTT_DEMO - 航点
add_executable(rmtt_waypoint demo_rmtt/rmtt_waypoint.cpp)
add_dependencies(rmtt_waypoint sunray_swarm_gencpp)
target_link_libraries(rmtt_waypoint ${catkin_LIBRARIES})
# RMTT_DEMO - 无人机跟随无人车
add_executable(rmtt_follow_ugv demo_rmtt/rmtt_follow_ugv.cpp)
add_dependencies(rmtt_follow_ugv sunray_swarm_gencpp)
target_link_libraries(rmtt_follow_ugv ${catkin_LIBRARIES})

# UGV_DEMO - 无人车绕圆
add_executable(ugv_circle demo_ugv/ugv_circle.cpp)
add_dependencies(ugv_circle sunray_swarm_gencpp)
target_link_libraries(ugv_circle ${catkin_LIBRARIES})
# UGV_DEMO - 无人车航点
add_executable(ugv_waypoint demo_ugv/ugv_waypoint.cpp)
add_dependencies(ugv_waypoint sunray_swarm_gencpp)
target_link_libraries(ugv_waypoint ${catkin_LIBRARIES})
# UGV_DEMO - 无人车跟随无人机
add_executable(ugv_follow_rmtt demo_ugv/ugv_follow_rmtt.cpp)
add_dependencies(ugv_follow_rmtt sunray_swarm_gencpp)
target_link_libraries(ugv_follow_rmtt ${catkin_LIBRARIES})

# ORCA_DEMO - 固定障碍物路径规划
add_executable(path_planning orca/path_planning.cpp)
add_dependencies(path_planning sunray_swarm_gencpp)
target_link_libraries(path_planning ${catkin_LIBRARIES})

## SWARM_DEMO - 集群绕圆
add_executable(swarm_circle demo_swarm/swarm_circle.cpp)
add_dependencies(swarm_circle sunray_swarm_gencpp)
target_link_libraries(swarm_circle ${catkin_LIBRARIES})
## SWARM_DEMO - 集群绕8
add_executable(swarm_lemniscate demo_swarm/swarm_lemniscate.cpp)
add_dependencies(swarm_lemniscate sunray_swarm_gencpp)
target_link_libraries(swarm_lemniscate ${catkin_LIBRARIES})
## SWARM_DEMO - swarm_dance
add_executable(swarm_dance demo_swarm/swarm_dance.cpp)
add_dependencies(swarm_dance sunray_swarm_gencpp)
target_link_libraries(swarm_dance ${catkin_LIBRARIES})


## SWARM_DEMO - 固定阵型变换
add_executable(swarm_formation demo_swarm/swarm_formation.cpp)
add_dependencies(swarm_formation sunray_swarm_gencpp)
target_link_libraries(swarm_formation ${catkin_LIBRARIES})
## SWARM_DEMO - 领从阵型变换
add_executable(swarm_leader_follower demo_swarm/swarm_leader_follower.cpp)
add_dependencies(swarm_leader_follower sunray_swarm_gencpp)
target_link_libraries(swarm_leader_follower ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
