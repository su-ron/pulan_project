cmake_minimum_required(VERSION 2.8.3)
project(velocity_smoother)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS 
     roscpp 
     std_msgs
     geometry_msgs 
     nav_msgs 
     dynamic_reconfigure
     custom_msgs_srvs
)
message(STATUS "HOST_PLATFORM : ${HOST_PLATFORM}")
# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
   cfg/VelocitySmooth.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs dynamic_reconfigure custom_msgs_srvs
)

include_directories(
   include 
   ${catkin_INCLUDE_DIRS}
)

# Nodelet library
add_library(${PROJECT_NAME} src/velocity_smoother.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/velocity_smoother.cpp src/state_control.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
