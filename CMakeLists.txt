cmake_minimum_required(VERSION 2.8.3)
project(navigation_test)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include/navigation_test
  CATKIN_DEPENDS actionlib costmap_2d geometry_msgs map_msgs move_base_msgs nav_msgs roscpp std_msgs tf visualization_msgs
)

include_directories(include/navigation_test)
include_directories(${catkin_INCLUDE_DIRS})

set(NAV_TEST_SRC
    ${PROJECT_SOURCE_DIR}/src/navigation_test.cpp
    ${PROJECT_SOURCE_DIR}/src/navigation_test_node.cpp
)

add_executable(navigation_test_node ${NAV_TEST_SRC})
target_link_libraries(navigation_test_node ${catkin_LIBRARIES})

