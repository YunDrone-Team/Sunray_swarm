cmake_minimum_required(VERSION 2.8.3)
project(sunray_swarm_sim)

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
  sunray_swarm_msgs
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

  
)

###############################
##      声明可执行cpp文件     ##
###############################


## 仿真
add_executable(agent_sim_node
  rviz_sim/agent_sim.cpp	
  rviz_sim/agent_sim_node.cpp
)
target_link_libraries(agent_sim_node ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
