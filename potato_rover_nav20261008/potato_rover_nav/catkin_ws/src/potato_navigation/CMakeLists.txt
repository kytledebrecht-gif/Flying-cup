cmake_minimum_required(VERSION 3.0.2)
project(potato_navigation)
find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
  actionlib actionlib_msgs move_base_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/nav_monitor.py
  scripts/amcl_auto_localizer.py
  scripts/record_waypoint.py
  scripts/waypoint_navigator.py
  scripts/self_scan_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
