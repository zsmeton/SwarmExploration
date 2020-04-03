################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(distributed_localization)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  map_server
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs map_server nav_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(flat_world_imu_node src/flat_world_imu_node.cpp)
# add_dependencies(flat_world_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(flat_world_imu_node ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
#install(TARGETS flat_world_imu_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
