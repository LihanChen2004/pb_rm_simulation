cmake_minimum_required(VERSION 3.10)
project(ros2_livox_simulation)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

ament_auto_add_library(ros2_livox SHARED
    src/livox_points_plugin.cpp
    src/livox_ode_multiray_shape.cpp
)

target_link_libraries(ros2_livox
    RayPlugin
    GpuRayPlugin
    libprotobuf.so.9
    libboost_chrono.so.1.71.0
)

#############
## Testing ##
#############

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  scan_mode
  urdf
)
