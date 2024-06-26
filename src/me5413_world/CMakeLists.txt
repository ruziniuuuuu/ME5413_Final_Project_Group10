cmake_minimum_required(VERSION 3.0.2)
project(me5413_world)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  image_transport
  tf
  cv_bridge
)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES me5413_world
  # CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Add Nodes
add_executable(goal_publisher_node src/goal_publisher_node.cpp)
target_link_libraries(goal_publisher_node ${catkin_LIBRARIES})
add_dependencies(goal_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(object_spawner_gz_plugin SHARED src/object_spawner_gz_plugin.cpp)
target_link_libraries(object_spawner_gz_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(box_explorer_node src/box_explorer_node.cpp)
target_link_libraries(box_explorer_node ${catkin_LIBRARIES})
add_dependencies(box_explorer_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(template_matching_node src/template_matching_node.cpp)
target_link_libraries(template_matching_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(template_matching_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/template_matching_node_py.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
catkin_install_python(PROGRAMS scripts/sift_detection_node_py.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/find_2d_box_node_py.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/track_error_logger.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})