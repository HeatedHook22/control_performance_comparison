cmake_minimum_required(VERSION 3.8)
project(bare_quad_sim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

#################
# Setup targets #
#################

# add executables
add_executable(single_quad 
  #src/offboard_control.cpp
  src/offboard_controller.cpp
  src/attitude_rate_control.cpp
  src/pos_vel_acc_control.cpp
  src/test_generation.cpp
)

include_directories(include)

ament_target_dependencies(single_quad 
  rclcpp 
  px4_msgs 
  geometry_msgs 
  tf2
)

# Make sure Eigen's include path is used
target_include_directories(single_quad PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  # ${px4_ros_com_INCLUDE_DIRS}  # Add PX4 frame transforms
)

# find executables
install(TARGETS single_quad DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############
# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp px4_msgs)

# Install header files
# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
