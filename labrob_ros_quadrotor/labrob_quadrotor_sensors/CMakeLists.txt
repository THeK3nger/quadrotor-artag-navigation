cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

include(./buildFiles/CMakeFindROSVersion.cmake)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()
if(ROS_ELECTRIC_FOUND)
	# Camera
	rosbuild_add_library(labrob_gazebo_camera src/labrob_gazebo_camera.cpp)
	# IMU
	rosbuild_add_library(labrob_gazebo_IMU src/labrob_gazebo_IMU.cpp)
	# Sonar
	rosbuild_add_library(labrob_sonar_controller src/labrob_sonar_controller.cpp)
elseif(ROS_FUERTE_FOUND)
	# Camera
	include_directories(${ROS_INC})
	rosbuild_add_library(labrob_gazebo_camera_utilsFuerte src/labrob_gazebo_camera_utilsFuerte.cpp)
	target_link_libraries(labrob_gazebo_camera_utilsFuerte gazebo_sdf_interface)
	rosbuild_link_boost(labrob_gazebo_camera_utilsFuerte thread)
	rosbuild_add_library(labrob_gazebo_camera src/labrob_gazebo_cameraFuerte.cpp)
	target_link_libraries(labrob_gazebo_camera labrob_gazebo_camera_utilsFuerte DepthCameraPlugin)
	rosbuild_link_boost(labrob_gazebo_camera thread)
	# IMU
	rosbuild_add_library(labrob_gazebo_IMU src/labrob_gazebo_IMUFuerte.cpp)
	# Sonar
	rosbuild_add_library(labrob_gazebo_sonar src/labrob_gazebo_sonarFuerte.cpp)
rosbuild_link_boost(labrob_gazebo_IMU thread)
endif(ROS_ELECTRIC_FOUND)
rosbuild_link_boost(labrob_gazebo_camera thread)

#rosbuild_add_boost_directories()
#if(ROS_ELECTRIC_FOUND)
	#rosbuild_add_library(labrob_gazebo_camera src/labrob_gazebo_IMU.cpp)
#elseif(ROS_FUERTE_FOUND)
	#rosbuild_add_library(labrob_gazebo_camera src/labrob_gazebo_IMUFuerte.cpp)
#endif(ROS_ELECTRIC_FOUND)
#rosbuild_link_boost(labrob_gazebo_IMU thread)
