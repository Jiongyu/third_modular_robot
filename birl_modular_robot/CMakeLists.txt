cmake_minimum_required(VERSION 2.8.3)
project(birl_module_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  descartes_point.msg
  joint_point.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES birl_modual_robot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 "/usr/include/eigen3"
 "./../kinematics"
 "./../kinematics/CML"
 "/src/include"

)

## Declare a C++ library
 add_library( Kine_lib
   ./../kinematics/Kine.cpp
   ./../kinematics/Matrix.cpp
   ./../kinematics/Setup.cpp
   ./../kinematics/MyPath.cpp
   ./../kinematics/CopleyCAN.cpp
   ./../kinematics/Interpolation.cpp
  ./../kinematics/SpeedProcess.cpp
  ./../kinematics/CML/
 )

link_libraries(Kine_lib)

add_executable(biped5d_descartes_command src/biped5d_descartes_command.cpp)
add_executable(biped5d_descartes_feedback src/biped5d_descartes_feedback.cpp)
add_executable(biped5d_command_test src/biped5d_command_test.cpp)


target_link_libraries(biped5d_descartes_command
  ${catkin_LIBRARIES})

target_link_libraries(biped5d_descartes_feedback
  ${catkin_LIBRARIES})

target_link_libraries(biped5d_command_test
  ${catkin_LIBRARIES})