cmake_minimum_required(VERSION 3.10) 
project(linefit_ground_segmentation_ros)

# find_package(catkin_simple 0.1.0 REQUIRED )

# catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto)

ament_auto_find_build_dependencies()
ament_auto_add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
ament_auto_add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc)

ament_auto_package(INSTALL_TO_SHARE launch)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
