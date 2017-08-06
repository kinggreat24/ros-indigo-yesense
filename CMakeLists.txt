cmake_minimum_required(VERSION 2.8.3)
project(yesense)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


set(CMAKE_CXX_FLAGS
  -fPIC
 )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


## This plugin includes Qt widgets, so we must include Qt like so:
set(Qt_prefix "/home/liweixin/Qt5.6.0/5.6/gcc_64/lib/cmake")
set(Qt5Core_DIR "${Qt_prefix}/Qt5Core")
set(Qt5SerialPort_DIR "${Qt_prefix}/Qt5SerialPort")
find_package(Qt5Core 5.6 REQUIRED)
find_package(Qt5SerialPort 5.6 REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS})

set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5SerialPort_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yesense
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


qt5_wrap_cpp(MOC_FILES
	src/yesense.h
)
set(SOURCE_FILE
	src/yesense.cpp
	${MOC_FILES}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/yesense.cpp
# )
add_library(yesenselib ${SOURCE_FILE})
target_link_libraries(yesenselib ${QT_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/yesense_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} yesenselib)



