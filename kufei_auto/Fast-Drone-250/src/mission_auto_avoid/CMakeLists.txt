cmake_minimum_required(VERSION 3.0.2)
project(mission_auto_avoid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odom_velocity_to_world src/odom_velocity_to_world.cpp)
target_compile_features(odom_velocity_to_world PRIVATE cxx_std_14)
target_link_libraries(odom_velocity_to_world
  ${catkin_LIBRARIES}
)
add_dependencies(odom_velocity_to_world ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/auto_avoid_manager.py
  scripts/mission_waypoint_uploader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS odom_velocity_to_world
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
