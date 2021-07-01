cmake_minimum_required(VERSION 3.0.2)
project(visual_servo_tsuchida)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visp_bridge
  actionlib_msgs
  actionlib
  geometry_msgs
  moveit_ros_planning_interface
  cv_bridge
)

add_action_files(
  DIRECTORY action
  FILES Servo_message.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visual_servo_tsuchida
  CATKIN_DEPENDS roscpp visp_bridge actionlib actionlib_msgs geometry_msgs cv_bridge
#  DEPENDS system_lib
)




include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#add_library(servo include/visual_servo_tsuchida/servo_base.h)
add_library(servo_library SHARED src/servo_base_1.cpp src/servo_base_2.cpp src/servo_base_3.cpp)
target_link_libraries(servo_library
  ${catkin_LIBRARIES}
)

add_executable(main3 src/main.cpp)
target_link_libraries(main3
  servo_library
  ${catkin_LIBRARIES}
)

add_executable(move src/moveit_try.cpp)
target_link_libraries(move
  ${catkin_LIBRARIES}
  servo_library
)

add_library(server_visual src/server_visual.cpp)
target_link_libraries(server_visual
  ${catkin_LIBRARIES}
  servo_library
)

add_library(client_visual src/client_visual.cpp)
target_link_libraries(client_visual
  ${catkin_LIBRARIES}
  servo_library
)

add_executable(server_main src/server_main.cpp)
target_link_libraries(server_main
  ${catkin_LIBRARIES}
  servo_library
  server_visual
)

add_executable(client_main_1 src/client_main.cpp)
target_link_libraries(client_main_1
   servo_library
  client_visual
  ${catkin_LIBRARIES}
)

add_executable(tutorial src/tutorial-mb-generic-tracker.cpp)
target_link_libraries(tutorial
  ${catkin_LIBRARIES}
)