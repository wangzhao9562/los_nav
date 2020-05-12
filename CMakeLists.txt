cmake_minimum_required(VERSION 2.8.3)
project(los_nav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  los_nav_msgs
  message_generation
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_srvs
  tf
)

add_message_files(
  DIRECTORY msg
  FILES
  Mission.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
# catkin_python_setup()

# generate_dynamic_reconfigure_options(
#   cfg/LosNav.cfg	
# )

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    los_nav_msgs
    nav_msgs
    roscpp  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(los_nav
  src/common.cpp
  src/los_nav.cpp
  src/pf_los_controller.cpp
  src/ros_los_nav.cpp
)

target_link_libraries(los_nav
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(los_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(los_nav_node
  src/los_nav_node.cpp
)

target_link_libraries(los_nav_node
  los_nav
  ${catkin_LIBRARIES}
)

add_dependencies(los_nav_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

