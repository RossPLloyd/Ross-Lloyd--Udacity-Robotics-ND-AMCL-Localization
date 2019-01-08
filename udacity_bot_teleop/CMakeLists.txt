cmake_minimum_required(VERSION 2.8.3)
project(udacity_bot_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy
  DEPENDS
)

###########
## Build ##
###########

add_executable(udacity_bot_teleop_joy src/turtlebot_joy.cpp)
target_link_libraries(udacity_bot_teleop_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/udacity_bot_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##Mark executables and/or libraries for installation
install(TARGETS udacity_bot_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
