cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# Set the name of the ROS wrapper package
project(htg3_wrapper)
message (STATUS "[${PROJECT_NAME}] Processing ${CMAKE_CURRENT_LIST_FILE}")

set (CMAKE_IGNORE_PATH $ENV{HOME}/anaconda3)
set (CMAKE_IGNORE_PATH $ENV{HOME}/miniforge3)

# Set the C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set HTG3_COLLECTOR_DIR
set(HTG3_COLLECTOR_DIR $ENV{HOME}/htg3_ros/HT_G3_Test_rig/PC/data-collector/src)

#####################
# Find Packages
#####################

# Find the NatNet library.
find_library(NATNET_LIBRARY
  NAMES NatNet
  PATHS "${HTG3_COLLECTOR_DIR}/optotrack/libs"
  NO_DEFAULT_PATH) # This line ensures CMake only looks in the specified path.

# Check if the library was found.
if(NOT NATNET_LIBRARY)
  message(FATAL_ERROR "NatNet library not found")
endif()

# Find packages
if (NOT TARGET ids_peak)
    find_package (ids_peak REQUIRED
        HINTS 
            $ENV{HOME}/Documents/ids-peak-with-ueyetl_2.4.0.0-15977_amd64/lib
    )
endif ()

if (NOT TARGET ids_peak_ipl)
    find_package (ids_peak_ipl REQUIRED
        HINTS 
            $ENV{HOME}/Documents/ids-peak-with-ueyetl_2.4.0.0-15977_amd64/lib
    )
endif()

SET(LIB_DIR_ARENA $ENV{HOME}/Documents/ArenaSDK_Linux_x64/lib64)
SET(LIB_DIR_GENICAM $ENV{HOME}/Documents/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64)
SET(LIB_DIR_FFMPEG $ENV{HOME}/Documents/ArenaSDK_Linux_x64/ffmpeg)
SET(LIB_DIR_OPENVR ${HTG3_COLLECTOR_DIR}/svr/openvr/lib/linux64)

find_library(openvr_LIB NAMES openvr_api PATHS ${LIB_DIR_OPENVR})
find_library(arena_LIB NAMES arena PATHS ${LIB_DIR_ARENA})
find_library(save_LIB NAMES save PATHS ${LIB_DIR_ARENA})
find_library(gentl_LIB NAMES gentl PATHS ${LIB_DIR_ARENA})
find_library(lucidlog_LIB NAMES lucidlog PATHS ${LIB_DIR_ARENA})
find_library(GCBase_LIB NAMES GCBase_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(Log_LIB NAMES Log_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(log4cpp_LIB NAMES log4cpp_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(MathParser_LIB NAMES MathParser_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(NodeMapData_LIB NAMES NodeMapData_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(XmlParser_LIB NAMES XmlParser_gcc54_v3_3_LUCID PATHS ${LIB_DIR_GENICAM})
find_library(avcodec_LIB NAMES avcodec PATHS ${LIB_DIR_FFMPEG})
find_library(avformat_LIB NAMES avformat PATHS ${LIB_DIR_FFMPEG})
find_library(avutil_LIB NAMES avutil PATHS ${LIB_DIR_FFMPEG})
find_library(swresample_LIB NAMES swresample PATHS ${LIB_DIR_FFMPEG})

find_package (Threads REQUIRED)
find_package (Threads REQUIRED)
find_package (OpenCV REQUIRED)
find_package(Glog REQUIRED)
find_package(CURL REQUIRED) 
find_package(fmt REQUIRED) 

INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS filesystem thread) 

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost library not found")
endif()

# Find ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  # Add any other ROS dependencies you need here
)

#####################
# Create Package
#####################

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

set (HGT3_INCLUDE_DIRS
    ${HTG3_COLLECTOR_DIR}/camera
    ${HTG3_COLLECTOR_DIR}/data_collector
    ${HTG3_COLLECTOR_DIR}/totalStation
    ${HTG3_COLLECTOR_DIR}/log
    ${HTG3_COLLECTOR_DIR}/Lidar
    ${HTG3_COLLECTOR_DIR}/optotrack
    ${HTG3_COLLECTOR_DIR}/optotrack/includes
    ${HTG3_COLLECTOR_DIR}/rpi_library
    ${HTG3_COLLECTOR_DIR}/tof
    ${HTG3_COLLECTOR_DIR}/json
    $ENV{HOME}/Documents/ArenaSDK_Linux_x64/include/Arena
    $ENV{HOME}/Documents/ArenaSDK_Linux_x64/include/Save
    $ENV{HOME}/Documents/ArenaSDK_Linux_x64/include/GenTL
    $ENV{HOME}/Documents/ArenaSDK_Linux_x64/GenICam/library/CPP/include
    ${HTG3_COLLECTOR_DIR}/svr/openvr/headers
    ${HTG3_COLLECTOR_DIR}/svr
    ${HTG3_COLLECTOR_DIR}/stm32_library
)

set (HGT3_SOURCES
    ${HTG3_COLLECTOR_DIR}/data_collector/data_collector.cpp
    ${HTG3_COLLECTOR_DIR}/camera/ids_camera_collector.cpp
    ${HTG3_COLLECTOR_DIR}/camera/autofeaturesmanager.cpp
    ${HTG3_COLLECTOR_DIR}/log/log.cpp
    ${HTG3_COLLECTOR_DIR}/Lidar/livox_lidar.cpp
    ${HTG3_COLLECTOR_DIR}/optotrack/OptitrackInterface.cpp
    ${HTG3_COLLECTOR_DIR}/rpi_library/RPIComms.cpp
    ${HTG3_COLLECTOR_DIR}/rpi_library/RPIInterface.cpp
    ${HTG3_COLLECTOR_DIR}/tof/helios2.cpp
    ${HTG3_COLLECTOR_DIR}/tof/heliosDevice.cpp
    ${HTG3_COLLECTOR_DIR}/svr/svr_grabber.cpp
    ${HTG3_COLLECTOR_DIR}/totalStation/tsCollect.cpp
    ${HTG3_COLLECTOR_DIR}/totalStation/tcptsinterface.cpp
    ${HTG3_COLLECTOR_DIR}/stm32_library/BoostSerial.cpp
    ${HTG3_COLLECTOR_DIR}/stm32_library/stm32Protocol.cpp
)

include_directories(
  include
  ${HGT3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Add executable for the ROS wrapper
add_executable(${PROJECT_NAME}
  src/main.cpp
  ${HGT3_SOURCES}
)

# Link against libraries
target_link_libraries (${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ids_peak
   ids_peak_ipl
   ${CMAKE_THREAD_LIBS_INIT}
   opencv_core
   opencv_imgproc
   opencv_highgui
   glog::glog
   fmt::fmt
   livox_lidar_sdk_static
   ${NATNET_LIBRARY}
   ${arena_LIB}
   ${save_LIB}
   ${gentl_LIB}
   ${lucidlog_LIB}
   ${GCBase_LIB}
   ${Log_LIB}
   ${log4cpp_LIB}
   ${MathParser_LIB}
   ${NodeMapData_LIB}
   ${XmlParser_LIB}
   ${avcodec_LIB}
   ${avformat_LIB}
   ${avutil_LIB}
   ${swresample_LIB}
   ${openvr_LIB}
   -lffi
   -pthread
   -ltiff
   ${CURL_LIBRARIES}
   Boost::filesystem
   Boost::thread
)

# Specify dependencies of the ROS wrapper on the other packages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Call deploy functions
# These functions will add a post-build steps to your target in order to copy all needed files (e.g. DLL's) to the output directory.
ids_peak_deploy(${PROJECT_NAME})
ids_peak_ipl_deploy(${PROJECT_NAME})

# Set C++ standard to 14 (required for ids_peak)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS NO
)

# Enable multiprocessing for MSVC
if (MSVC)
    target_compile_options (${PROJECT_NAME}
        PRIVATE "/MP"
    )
endif ()