cmake_minimum_required(VERSION 2.8.3)
project(floor_detection)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    actionlib
    rospy
    ropod_ros_msgs
)

catkin_python_setup()

add_action_files(DIRECTORY ros/action
    FILES
    DetectFloor.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    actionlib
    rospy
    ropod_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/floor_detection
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
