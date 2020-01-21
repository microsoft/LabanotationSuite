cmake_minimum_required(VERSION 2.8.3)
project(aero_labanotation)

# If you do not want to compile this package,
# please "touch .CATKIN_IGNORE"

find_package(catkin QUIET COMPONENTS
  roscpp labanotation_msgs actionlib aero_std)

catkin_package(LIBRARIES aerolaban)
include_directories(include ${catkin_INCLUDE_DIRS})

# Compile options

add_definitions(-std=c++11)
if("$ENV{ROS_DISTRO}" STRGREATER "jade")
  add_definitions(-DKINETIC)
endif()

# Labanotation compiler library

add_library(aerolaban src/aerolaban.cc)

# Labanotation converter node

add_executable(aero_labanotation src/aero_labanotation_action.cc)
target_link_libraries(aero_labanotation ${catkin_LIBRARIES} aerolaban)
