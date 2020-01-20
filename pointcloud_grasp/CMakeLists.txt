cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_grasp)

add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cmake_modules
  geometric_shapes
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  franka_gripper
  actionlib
)

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS    roscpp
                    pcl_conversions
                    pcl_ros moveit_core
                    moveit_ros_planning_interface
                    interactive_markers
)

add_executable(recognize src/recognize.cpp)
target_link_libraries(recognize
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(test_node src/test.cpp)
target_link_libraries(test_node
        ${catkin_LIBRARIES}
)

add_executable(move_group_demo src/move_group.cpp)
target_link_libraries(move_group_demo
        ${catkin_LIBRARIES}
)
