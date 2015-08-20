cmake_minimum_required(VERSION 2.8.7)

project(ros_simple_zed_cuda_driver)

###############################################################################
## Note by Myzhar                                                            ##
## ===============                                                           ##
##                                                                           ##
## You need to install the official StereoLabs ZED SDK from the website:     ##
## https://www.stereolabs.com/developers/                                    ##
##                                                                           ##
## For ARM based system you must request the SDK to StereoLabs Support       ##
##                                                                           ##
###############################################################################

###############################################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" OFF )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
###############################################################################

###############################################################################
# INCLUDES

set(zed_INC_PATH ${ros_simple_zed_cuda_driver_SOURCE_DIR}/src/)
set(zed_INC_DIRS ${zed_INC_PATH})

# Specify locations of header files.
include_directories(${zed_INC_DIRS})
###############################################################################

###############################################################################
# STEREOLABS ZED SDK
if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 OLD)
        cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


SET(LINUX_ONLY_LIBRARIES "")

SET(ZED_DIR "/usr/local/zed/")
find_package(ZED 1.0 REQUIRED)
#find_package(GLUT)
SET(LINUX_ONLY_LIBRARIES pthread npps nppi)

find_package(CUDA 6.5 EXACT REQUIRED)
#find_package(OpenGL)

include_directories(${ZED_INCLUDE_DIRS})
#include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
#link_directories(${GLUT_LIBRARY_DIRS})
#link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
###############################################################################



###############################################################################
# CATKIN
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    image_transport
)


include_directories(${catkin_INCLUDE_DIRS})
###############################################################################


###############################################################################
# SOURCES
set(zed_SRC
    ${ros_simple_zed_cuda_driver_SOURCE_DIR}/src/zed_simple_camera_node.cpp
)

set(zed_HDR
)
###############################################################################

###############################################################################
# EXECUTABLE
add_executable(
  zed_simple_camera_node
  ${zed_SRC}
  ${zed_HDR}
)

add_definitions(-std=c++0x)# -m64)

target_link_libraries(
        zed_simple_camera_node        
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${LINUX_ONLY_LIBRARIES}
        #${OPENGL_LIBRARIES}
        #${GLUT_LIBRARY}
        ${catkin_LIBRARIES}
    )

if(NOT DEBUG_ACTIVE)
    MESSAGE("Release optimizations active")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode
endif()
###############################################################################

