cmake_minimum_required(VERSION 2.8.3)
project(four_wheel_steering_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
