# livox_laser_simulation for RO2
this is a ros2 port of the original repo: https://github.com/Livox-SDK/livox_laser_simulation.

tested in ros2 foxy and humble.

### Usage 
1. clone this repo in your ros2 workspace
```
git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git
```
2. build your ros2 workspace (if there are warnings that do not allow the compilation, run the build again and you will see that the error disappears)
```
colcon build && source install/setup.bash
```
3. include the lidar sensor in your URDF file, for example:
```
<xacro:include filename="$(find ros2_livox_simulation)/urdf/mid70.xacro" />
```
4. attach the sensor to your robot in the URDF (or xacro) file, for example:
```
  <xacro:mid70 name="livox" parent="base_link" topic="mid70">
    <origin xyz="1 0 1" rpy="0 0 0"/>
  </xacro:mid70>
```
you need to specify the parent link (usually base_link)

that's it. the example that i gave you is for mid70, but you can use mid40, mid70, mid360 and so on.

thanks to the original repo, you can find more info in it.

