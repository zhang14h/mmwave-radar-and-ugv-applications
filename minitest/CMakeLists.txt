cmake_minimum_required(VERSION 3.0.2)
project(minitest)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)



catkin_package(
   INCLUDE_DIRS include
   LIBRARIES minitest
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs
   DEPENDS system_lib
)


include_directories(
    include  ${catkin_INCLUDE_DIRS}
)

add_executable (minitest_node src/minitest.cpp)
target_link_libraries (minitest_node ${catkin_LIBRARIES})
