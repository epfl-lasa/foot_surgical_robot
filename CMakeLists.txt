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
generate_dynamic_reconfigure_options()

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

# Generate executable
# add_executable(footMouseInterface src/footMouseInterface_node.cpp src/FootMouseInterface.cpp)
# target_link_libraries(footMouseInterface ${catkin_LIBRARIES})
# add_dependencies(footMouseInterface ${PROJECT_NAME}_gencpp)

# add_executable(footMouseController src/footMouseController_node.cpp src/FootMouseController.cpp)
# target_link_libraries(footMouseController ${catkin_LIBRARIES})
# add_dependencies(footMouseController ${PROJECT_NAME}_gencpp)

# add_executable(footMouseSharedControl src/footMouseSharedControl_node.cpp src/FootMouseSharedControl.cpp src/FootMouseController.cpp)
# target_link_libraries(footMouseSharedControl ${catkin_LIBRARIES})
# add_dependencies(footMouseSharedControl ${PROJECT_NAME}_gencpp)

# add_executable(contactTaskSharedControl src/contactTaskSharedControl_node.cpp src/ContactTaskSharedControl.cpp src/FootMouseController.cpp)
# target_link_libraries(contactTaskSharedControl ${catkin_LIBRARIES})
# add_dependencies(contactTaskSharedControl ${PROJECT_NAME}_gencpp)

# add_executable(contactSurfaceController src/contactSurfaceController_node.cpp src/ContactSurfaceController.cpp)
# target_link_libraries(contactSurfaceController ${catkin_LIBRARIES})
# add_dependencies(contactSurfaceController ${PROJECT_NAME}_gencpp)