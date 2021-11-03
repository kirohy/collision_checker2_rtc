cmake_minimum_required(VERSION 3.0.2)
project(octomap_collision_check)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  moveit_core
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_collision_check
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/octomap_collision_check.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
