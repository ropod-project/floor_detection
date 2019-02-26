cmake_minimum_required(VERSION 2.8.3)
project(floor_detection)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    ropod_ros_msgs
)

catkin_python_setup()

add_service_files(DIRECTORY ros/srv
    FILES
    DetectFloor.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
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
