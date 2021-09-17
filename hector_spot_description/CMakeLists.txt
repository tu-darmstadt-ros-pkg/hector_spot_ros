cmake_minimum_required(VERSION 3.0.2)
project(hector_spot_description)

find_package(catkin REQUIRED)

catkin_add_env_hooks(50.export.hector_spot_description.bash.em SHELLS bash)

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
