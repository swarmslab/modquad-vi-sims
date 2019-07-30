cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_magnet)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  geometry_msgs 
  gazebo_ros
  )

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  Attach.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -O2")

add_library(gazebo_dipole_magnet src/dipole_magnet.cc)
target_link_libraries(gazebo_dipole_magnet ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_links src/gazebo_ros_link_attacher.cpp)
target_link_libraries(gazebo_links ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_links ${gazebo_magnet_EXPORTED_TARGETS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    geometry_msgs 
    message_runtime
)
