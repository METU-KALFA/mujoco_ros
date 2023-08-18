cmake_minimum_required(VERSION 3.0.2)
project(ur5e_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
)

#############
## Install ##
#############

install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)