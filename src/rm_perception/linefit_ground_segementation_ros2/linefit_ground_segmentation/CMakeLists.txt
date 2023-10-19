cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation)

# find_package(catkin_simple 0.1.0 REQUIRED )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)

ament_auto_package()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})