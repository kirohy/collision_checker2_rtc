set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  choreonoid_vclip
  choreonoid_qhull
  collision_checker_msgs
  )
# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、親のCMakeLists.txtで1回だけ呼ぶ.
# find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

rtmbuild_add_library(CollisionChecker SHARED
  CollisionChecker.cpp
  )
target_link_libraries(CollisionChecker
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )
set_target_properties(CollisionChecker PROPERTIES PREFIX "") # libCollisionChecker.so -> CollisionChecker.so
add_dependencies(CollisionChecker RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(CollisionCheckerComp CollisionCheckerComp.cpp)
target_link_libraries(CollisionCheckerComp CollisionChecker)
