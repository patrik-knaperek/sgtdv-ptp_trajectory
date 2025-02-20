cmake_minimum_required(VERSION 3.0.2)
project(ptp_trajectory)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sgtdv_msgs
  std_msgs
  std_srvs
  message_generation
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


 add_service_files(
   FILES
   GoRectangle.srv
   SetTarget.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sgtdv_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp sgtdv_msgs std_msgs std_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

 add_executable(${PROJECT_NAME} src/ptp_trajectory_node.cpp src/ptp_trajectory.cpp)

  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
