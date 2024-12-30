set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  collision_checker_msgs
  octomap_msgs
  moveit_core
  octomap_msgs_rtmros_bridge
  )
# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、親のCMakeLists.txtで1回だけ呼ぶ.
# find_package(choreonoid REQUIRED)
find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )

add_library(OctomapCollisionChecker SHARED
  OctomapCollisionChecker.cpp
  OctomapCollisionCheckerService_impl.cpp
  )
target_link_libraries(OctomapCollisionChecker
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  )
set_target_properties(OctomapCollisionChecker PROPERTIES PREFIX "") # libOctomapCollisionChecker.so -> OctomapCollisionChecker.so
add_dependencies(OctomapCollisionChecker RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(OctomapCollisionCheckerComp OctomapCollisionCheckerComp.cpp)
target_link_libraries(OctomapCollisionCheckerComp OctomapCollisionChecker)

install(TARGETS OctomapCollisionChecker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS OctomapCollisionCheckerComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
