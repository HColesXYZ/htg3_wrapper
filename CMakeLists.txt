cmake_minimum_required(VERSION 3.0.2)
project(htg3_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Change this to your installation of HTG3_COLLECTOR_DIR.
set(HTG3_COLLECTOR_DIR
   $ENV{HOME}/htg3_ros/HT_G3_Test_rig/PC/data-collector/src
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${HTG3_COLLECTOR_DIR}
)

# Add the data_collector Library
add_library(data_collector_lib STATIC IMPORTED)
set_target_properties(data_collector_lib PROPERTIES IMPORTED_LOCATION ${HTG3_COLLECTOR_DIR}/build/libdata_collector.a)

set(LIBS
   ${catkin_LIBRARIES}
   data_collector_lib
)

# create executable node
add_executable (${PROJECT_NAME}_node
   src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${LIBS})

#############
## Install ##
#############

#############
## Testing ##
#############
