cmake_minimum_required(VERSION 2.8.3)
project(aautest)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy std_msgs image_transport dynamic_reconfigure cv_bridge sensor_msgs tf pcl_ros )


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(CoViS REQUIRED HINTS $ENV{HOME}/CoViS $ENV{HOME}/Code/CoViS $ENV{HOME}/workspace/CoViS ../../../CoViS)
include_directories( ${COVIS_INCLUDE_DIR} )
link_directories( ${COVIS_LIBRARY_DIRS} )


add_executable(example1 src/example1.cpp)
target_link_libraries(example1 ${catkin_LIBRARIES} ${COVIS_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(example1 example1_cpp)


