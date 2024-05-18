add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  geometry_msgs
  tf2
  urdf
  collision_checker_msgs)
find_package(choreonoid REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
rtmbuild_add_library(CollisionROSBridge SHARED CollisionROSBridge.cpp)
target_link_libraries(CollisionROSBridge ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})
set_target_properties(CollisionROSBridge PROPERTIES PREFIX "") # libPrimitiveStateROSBridge.so -> PrimitiveStateROSBridge.so
add_dependencies(CollisionROSBridge RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(CollisionROSBridgeComp CollisionROSBridgeComp.cpp)
target_link_libraries(CollisionROSBridgeComp CollisionROSBridge)
