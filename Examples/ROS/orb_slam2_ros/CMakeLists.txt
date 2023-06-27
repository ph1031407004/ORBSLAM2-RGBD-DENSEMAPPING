cmake_minimum_required(VERSION 2.4.6)
project(orb_slam2_ros)#1
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()
SET(CMAKE_BUILD_TYPE Release)

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3 -march=native  -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 -march=native -g")
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)
find_package(catkin REQUIRED COMPONENTS#3
    roscpp
    std_msgs
    sensor_msgs

    tf
    cv_bridge
    image_transport
    pcl_ros
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.3.91 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.8 REQUIRED)


catkin_package(
    INCLUDE_DIRS
      ${PROJECT_SOURCE_DIR}/../../../include#
    #  LIBRARIES nav_goal
    CATKIN_DEPENDS
       roscpp
       std_msgs
       sensor_msgs
       tf
       cv_bridge
       image_transport
    #  DEPENDS system_lib
)
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}#
)

add_definitions( ${PCL_DEFINITIONS} )   ####### 
link_directories( ${PCL_LIBRARY_DIRS} ) ####### 
set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${PCL_LIBRARIES} ####### 
${catkin_LIBRARIES}
)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#add_library(orb_slam2_ros ${SOURCE_FILES})

# Node for monocular camera
add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono boost_system boost_filesystem
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
add_executable(MonoAR
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
)

target_link_libraries(MonoAR boost_system boost_filesystem
${LIBS}
)

# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo boost_system boost_filesystem
${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD boost_system boost_filesystem
${LIBS}
)

