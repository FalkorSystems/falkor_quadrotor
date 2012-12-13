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

rosbuild_add_boost_directories()
rosbuild_add_library(pwm_io src/pwm_input.cpp)
rosbuild_add_library(pwm_io src/pwm_pololu.cpp)
rosbuild_add_library(pwm_io src/pwm_switch.cpp)
rosbuild_link_boost(pwm_io system)

rosbuild_add_executable(pwm_switch_node src/pwm_switch_node.cpp)
target_link_libraries(pwm_switch_node pwm_io)

rosbuild_add_executable(pwm_io_node src/pwm_io_node.cpp)
target_link_libraries(pwm_io_node pwm_io)