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

# 添加Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  FILES
  pole_position.msg
)

add_service_files(
  FILES
  inverse_solution.srv
  positive_solution.srv
  robot_increTransTool.srv
  grasp_point.srv
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
 "/src/include"
 "./"

)

## Declare a C++ library
 add_library( Kine_lib
   ./../kinematics/Kine.cpp
   ./../kinematics/Matrix.cpp
   ./../kinematics/Setup.cpp
 )

link_libraries(Kine_lib)

### server node
add_executable(inverse_solution_server src/inverse_solution_server.cpp)
target_link_libraries(inverse_solution_server ${catkin_LIBRARIES})

add_executable(positive_solution_server src/positive_solution_server.cpp)
target_link_libraries(positive_solution_server ${catkin_LIBRARIES})

add_executable(robot_increTransTool_server src/robot_increTransTool_server.cpp)
target_link_libraries(robot_increTransTool_server ${catkin_LIBRARIES})


add_library( find_grasp
./src/find_grasp_point.cpp
./src/grasp_intelligent.cpp

)
add_executable(find_grasp_point_test src/test.cpp)
target_link_libraries(find_grasp_point_test find_grasp ${catkin_LIBRARIES} )

add_executable(find_grasp_point_server src/find_grasp_point_server.cpp)
target_link_libraries(find_grasp_point_server find_grasp ${catkin_LIBRARIES} )
