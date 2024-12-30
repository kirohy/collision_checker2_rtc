set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  urdf
  collision_checker_msgs)
find_package(choreonoid REQUIRED)
find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS})
add_library(CollisionROSBridge SHARED CollisionROSBridge.cpp)
target_link_libraries(CollisionROSBridge ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES} ${openrtm_aist_LIBRARIES})
set_target_properties(CollisionROSBridge PROPERTIES PREFIX "") # libPrimitiveStateROSBridge.so -> PrimitiveStateROSBridge.so
add_dependencies(CollisionROSBridge RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(CollisionROSBridgeComp CollisionROSBridgeComp.cpp)
target_link_libraries(CollisionROSBridgeComp CollisionROSBridge)

install(TARGETS CollisionROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS CollisionROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
