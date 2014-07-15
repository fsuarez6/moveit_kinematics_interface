cmake_minimum_required(VERSION 2.8.3)

project(moveit_kinematics_interface)

# Names
set(KIN_INTERFACE moveit_kinematics_interface)

find_package(catkin REQUIRED COMPONENTS 
  dynamic_reconfigure
  geometry_msgs
  joint_limits_interface
  message_generation
  moveit_core
  moveit_ros_planning
  pluginlib
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_service_files(FILES 
  GetPoseMetrics.srv
  GetStateMetrics.srv
  GetJointLimits.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

generate_dynamic_reconfigure_options(
  cfg/KinematicsSolvers.cfg
)

catkin_package( 
  LIBRARIES 
    ${KIN_INTERFACE}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    joint_limits_interface
    message_runtime
    moveit_core
    moveit_ros_planning
    pluginlib
    sensor_msgs
    std_msgs
)

include_directories(include)

find_package(Eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Libraries
add_library(${KIN_INTERFACE} src/kinematics_interface.cpp)
target_link_libraries(${KIN_INTERFACE} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

# Nodes
add_executable(kinematics_services src/kinematics_services.cpp)
target_link_libraries(kinematics_services ${KIN_INTERFACE} ${catkin_LIBRARIES})
add_dependencies(kinematics_services moveit_kinematics_interface_gencpp moveit_kinematics_interface_gencfg)
