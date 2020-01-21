cmake_minimum_required(VERSION 2.8.3)
project(labanotation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation trajectory_msgs actionlib_msgs)

add_message_files(
  FILES
  LabanotationDirectionSymbol.msg
  LabanotationTrajectoryPoint.msg
  LabanotationTrajectory.msg
  )


add_action_files(
  FILES
  Labanotation.action
  LabanotationFromJsonFile.action
  )

generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs)
catkin_package()
