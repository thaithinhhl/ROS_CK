cmake_minimum_required(VERSION 2.8.3)
project(xerobotvisai2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_manager
  joint_state_controller
  effort_controllers
  gazebo_ros
  gazebo_ros_control
)

catkin_package()

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
