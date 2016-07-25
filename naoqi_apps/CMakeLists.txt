cmake_minimum_required(VERSION 2.8.3)
project(naoqi_apps)

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  cfg/NaoqiTracker.cfg
  )
catkin_package()
