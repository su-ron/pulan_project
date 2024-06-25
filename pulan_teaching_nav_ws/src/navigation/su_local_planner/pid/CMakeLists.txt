cmake_minimum_required(VERSION 3.0.2)
project(pid_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
  custom_msgs_srvs
  #utils
)

# uncomment the following 4 lines to use the Eigen library
# find_package(cmake_modules REQUIRED)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  #CATKIN_DEPENDS utils
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
src/local_planner.cpp src/pid_planner.cpp 
)

# add_library(${PROJECT_NAME}
# src/local_planner.cpp src/pid_planner.cpp 
# )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch params
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)