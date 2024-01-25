cmake_minimum_required(VERSION 3.16)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  message_generation
)

# find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  idip.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS 
  # DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/swarm_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  yaml-cpp
  zmq
  zmqpp
)