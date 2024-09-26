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
## RMTT_DEMO - 路径规划
add_executable(rmtt_pathPlanning demo_rmtt/rmtt_pathplanning.cpp)
add_dependencies(rmtt_pathPlanning sunray_swarm_gencpp)
target_link_libraries(rmtt_pathPlanning ${catkin_LIBRARIES})
# RMTT_DEMO - 无人机跟随无人车
add_executable(rmtt_track_mission demo_rmtt/rmtt_track_mission.cpp)
add_dependencies(rmtt_track_mission sunray_swarm_gencpp)
target_link_libraries(rmtt_track_mission ${catkin_LIBRARIES})

# UGV_DEMO - 无人车绕圆
add_executable(ugv_circle demo_ugv/ugv_circle.cpp)
add_dependencies(ugv_circle sunray_swarm_gencpp)
target_link_libraries(ugv_circle ${catkin_LIBRARIES})
# UGV_DEMO - 无人车绕圆
add_executable(ugv_pathplanning demo_ugv/ugv_pathplanning.cpp)
add_dependencies(ugv_pathplanning sunray_swarm_gencpp)
target_link_libraries(ugv_pathplanning ${catkin_LIBRARIES})
# UGV_DEMO - 无人车绕圆
add_executable(ugv_waypoint demo_ugv/ugv_waypoint.cpp)
add_dependencies(ugv_waypoint sunray_swarm_gencpp)
target_link_libraries(ugv_waypoint ${catkin_LIBRARIES})




## SWARM_DEMO - 三智能体绕圈
add_executable(circle_trajectory demo_swarm/swarm_circle.cpp)
add_dependencies(circle_trajectory sunray_swarm_gencpp)
target_link_libraries(circle_trajectory ${catkin_LIBRARIES})
## SWARM_DEMO - 智能体跟随
add_executable(leader_follower demo_swarm/swarm_leader_follower.cpp)
add_dependencies(leader_follower sunray_swarm_gencpp)
target_link_libraries(leader_follower ${catkin_LIBRARIES})

## DEMO - 目标追踪
add_executable(track_mission demo/track_mission.cpp)
add_dependencies(track_mission sunray_swarm_gencpp)
target_link_libraries(track_mission ${catkin_LIBRARIES})

## DEMO - NOKOV队形
add_executable(formation_nokov demo/formation_nokov.cpp)
add_dependencies(formation_nokov sunray_swarm_gencpp)
target_link_libraries(formation_nokov ${catkin_LIBRARIES})

## DEMO - 有固定障碍物情况下的集群避障
add_executable(swarm_with_obstacles demo/swarm_with_obstacles.cpp)
add_dependencies(swarm_with_obstacles sunray_swarm_gencpp)
target_link_libraries(swarm_with_obstacles ${catkin_LIBRARIES})




#DEMO 编队控制
# add_executable(formation_control demo/formation_control.cpp)
# add_dependencies(formation_control sunray_swarm_gencpp)
# target_link_libraries(formation_control ${catkin_LIBRARIES})

#DEMO 编队控制，三角形
add_executable(formation_control2 demo/multi_formation_control.cpp)
add_dependencies(formation_control2 sunray_swarm_gencpp)
target_link_libraries(formation_control2 ${catkin_LIBRARIES})

#DEMO 跟随飞行
# add_executable(leader_follower demo/multi_leader_follower.cpp)
# add_dependencies(leader_follower sunray_swarm_gencpp)
# target_link_libraries(leader_follower ${catkin_LIBRARIES})

#DEMO 自主降落
add_executable(autonomous_landing demo/multi_autonomous_landing.cpp)
add_dependencies(autonomous_landing sunray_swarm_gencpp)
target_link_libraries(autonomous_landing ${catkin_LIBRARIES})


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
