cmake_minimum_required(VERSION 3.0.2)
project(collision_checker_tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

 # rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild)

# initialize rtmbuild (generate_messages is called)
rtmbuild_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES collision_checker_tools
#  CATKIN_DEPENDS openrtm_aist
#  DEPENDS system_lib
)

add_subdirectory(rtc)
