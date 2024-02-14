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
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/ptp_trajectory_node.cpp src/ptp_trajectory.cpp)

  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
