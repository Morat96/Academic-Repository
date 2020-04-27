cmake_minimum_required(VERSION 2.8.3)
project(hw4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  tf2_msgs
  apriltag_ros
  std_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  robotiq_3f_gripper_articulated_msgs
  ur_msgs
  actionlib
  move_base_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   msg1.msg
 )

#service
 add_service_files(
   FILES
   User.srv
   srv1.srv
   Attach.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   apriltag_ros
   std_msgs
 )

# depends
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system)

## HW1
# add esecutables
set(SRC1
    src/hw1/main.cpp
    src/hw1/ObjectDetector.h
    src/hw1/ObjectDetector.cpp
)
add_executable(apriltag ${SRC1})
target_link_libraries(apriltag ${catkin_LIBRARIES})

## HW2
set(SRC2
    src/hw2/main.cpp
    src/hw2/manipulation.h
    src/hw2/manipulation.cpp
)

add_executable(move ${SRC2})
target_link_libraries(move ${catkin_LIBRARIES})

## HW3
set(SRC3
    src/hw3/main.cpp
    src/hw3/navigation.h
    src/hw3/navigation.cpp
	src/hw3/parameters.h
	src/hw3/parameters.cpp
)

add_executable(nav ${SRC3})
target_link_libraries(nav ${catkin_LIBRARIES})

## USER SERVICE
set(SRC4
    src/user_service.cpp
)

add_executable(service ${SRC4})
target_link_libraries(service ${catkin_LIBRARIES})
