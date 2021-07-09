cmake_minimum_required(VERSION 3.0.2)
project(hector_spot_description)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY 
  urdf
  launch  
  meshes  
  rviz  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
