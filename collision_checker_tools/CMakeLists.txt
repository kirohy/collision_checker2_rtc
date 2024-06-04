cmake_minimum_required(VERSION 3.0.2)
project(collision_checker_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

 # rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  tf
  tf_conversions
  eigen_conversions
  collision_checker_msgs
  )

# initialize rtmbuild (generate_messages is called)
rtmbuild_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES collision_checker_tools
#  CATKIN_DEPENDS openrtm_aist
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
 )

add_executable(collision_array_frame_converter src/collision_array_frame_converter.cpp)

target_link_libraries(collision_array_frame_converter
  ${catkin_LIBRARIES}
 )

install(TARGETS collision_array_frame_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(rtc)
