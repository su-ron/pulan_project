cmake_minimum_required(VERSION 2.8.3)
project(turn_on_wheeltec_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
	custom_msgs_srvs
)

add_compile_options(-g -std=c++11)    #-g -> core dumped


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turn_on_wheeltec_robot
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)



# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)

add_executable(wheeltec_robot_node src/wheeltec_robot.cpp)
target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})

add_executable(test_chassis_node src/test_chassis_node.cpp)
target_link_libraries(test_chassis_node ${catkin_LIBRARIES})

install(
    TARGETS
    wheeltec_robot_node test_chassis_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch param_common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
