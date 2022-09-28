cmake_minimum_required(VERSION 3.0.2)
project(landing_commander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  nav_msgs
  std_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  eigen_conversions
  mavros
  mavros_msgs
  message_filters
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME} src/LandingCommander.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})

add_executable(landing_commander_node src/landing_commander_node.cpp)
target_link_libraries(landing_commander_node ${PROJECT_NAME} ${LINK_LIBS})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_landing_commander.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
