cmake_minimum_required(VERSION 3.0.2)
project(potato_mapping)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs sensor_msgs geometry_msgs tf)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/lio_planar_odom.py
  scripts/mapping_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
