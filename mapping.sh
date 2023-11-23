colcon build --symlink-install
cmds=(  "ros2 launch pb_rm_simulation rm_simulation.launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch fast_lio mapping_mid360.launch.py rviz:=false"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch rm_navigation online_async_launch.py"
	"ros2 launch rm_navigation bringup_no_amcl_launch.py")

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done