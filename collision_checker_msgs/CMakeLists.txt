cmake_minimum_required(VERSION 3.0.2)
project(collision_checker_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  geometry_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Collision.msg
  CollisionArray.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# initialize rtmbuild (generate_messages is called)
rtmbuild_init(geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES CollisionSkel CollisionStub
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS openrtm_aist
)

###########
## Build ##
###########

# generate idl
rtmbuild_genidl()

# generate bridge
# rtmbuild_genbridge()
add_custom_target(genidl ALL DEPENDS RTMBUILD_${PROJECT_NAME}_genrpc)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_collision_checker_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
