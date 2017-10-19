cmake_minimum_required(VERSION 2.8.3)

project(foot_surgical_robot)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
)

# add_message_files(
   # FILES
# )


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/footIsometricController_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation kdl_parser dynamic_reconfigure
  # DEPENDS Eigen
 )

include_directories(include ${catkin_INCLUDE_DIRS})

# Generate executable
add_executable(footIsometricController src/footIsometricController_node.cpp src/FootIsometricController.cpp)
target_link_libraries(footIsometricController ${catkin_LIBRARIES})
add_dependencies(footIsometricController ${PROJECT_NAME}_gencpp)