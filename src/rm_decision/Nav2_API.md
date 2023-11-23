# Nav2 API (Python3)

原版地址：https://github.com/ros-planning/navigation2/tree/humble/nav2_simple_commander

## 概述

此软件包的目标是为 Python3 用户提供“导航库”功能。我们提供了一个处理所有 ROS2 相关和 Action Server 相关事项的 API，以便您可以专注于构建利用 Nav2 能力的应用程序。我们还为您提供了使用 API 构建自主移动机器人的常见基本功能的演示和示例。

该软件包由 [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) 在 [Samsung Research](https://www.sra.samsung.com/) 构建，最初为 [2021 ROS Developers Day](https://www.theconstructsim.com/ros-developers-day-2021/) 大会上的主题演讲准备了初版原型（代码可以在 [这里](https://github.com/SteveMacenski/nav2_rosdevday_2021)找到）。

![](media/readme.gif)

## API

请查看 [API 指南页面](https://navigation.ros.org/commander_api/index.html) 以获取附加参数描述。

基本导航器提供的方法如下，包括输入和预期返回值。如果服务器失败，它可能会引发异常或返回 `None` 对象，因此请务必正确包装您的导航调用以进行 try/catch 并检查结果是否为 `None` 类型。

自 2023 年 9 月起：简单导航器构造函数将接受一个 `namespace` 字段，以支持多机器人应用程序或具有命名空间的 Nav2 启动。

|机器人导航方法|描述|
|-|-|
|setInitialPose(initial_pose)|将机器人的初始姿势（`PoseStamped`）设置为定位。|
|goThroughPoses(poses, behavior_tree='')|请求机器人穿过一组姿势（`PoseStamped` 列表）。|
|goToPose(pose, behavior_tree='')|请求机器人驶向一个姿势（`PoseStamped`）。|
|followWaypoints(poses)|请求机器人跟随一组路标（`PoseStamped` 列表）。这将在每个姿势处执行特定的 `TaskExecutor`。|
|followPath(path, controller_id='', goal_checker_id='')|请求机器人从起始点到目标 `PoseStamped`、`nav_msgs/Path` 遵循路径。|
|spin(spin_dist=1.57, time_allowance=10)|请求机器人执行给定角度的原地旋转。|
|backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10)|请求机器人倒车一定距离。|
|cancelTask()|取消进行中的任务请求。|
|isTaskComplete()|检查任务是否已完成，超时为 `100ms`。如果已完成，则返回 `True`，如果仍在进行中，则返回 `False`。|
|getFeedback()|获取任务的反馈，返回动作服务器反馈对象。|
|getResult()|获取任务的最终结果，在 `isTaskComplete` 返回 `True` 后调用。返回动作服务器结果对象。|
|getPath(start, goal, planner_id='', use_start=False)|获取从起始点到目标 `PoseStamped`、`nav_msgs/Path` 的路径。|
|getPathThroughPoses(start, goals, planner_id='', use_start=False)|获取通过起始点到一组目标点的路径，一组 `PoseStamped`、`nav_msgs/Path` 的列表。|
|smoothPath(path, smoother_id='', max_duration=2.0, check_for_collision=False)|平滑给定的 `nav_msgs/msg/Path` 路径。|
|changeMap(map_filepath)|请求从当前地图更改到 `map_filepath` 的 yaml。|
|clearAllCostmaps()|清除全局和局部成本地图。|
|clearLocalCostmap()|清除局部成本地图。|
|clearGlobalCostmap()|清除全局成本地图。|
|getGlobalCostmap()|返回全局成本地图，`nav2_msgs/Costmap`|
|getLocalCostmap()|返回局部成本地图，`nav2_msgs/Costmap`|
|waitUntilNav2Active(navigator='bt_navigator, localizer='amcl')|阻塞直到 Nav2 完全在线，并且生命周期节点处于活动状态。与自动启动或外部生命周期启动一起使用。可以指定自定义导航器和定位器节点|
|lifecycleStartup()|发送请求到所有生命周期管理服务器，将它们带入活动状态，如果 `autostart` 为 `false`，并且您希望此程序控制 Nav2 的生命周期时使用。|
|lifecycleShutdown()|发送请求到所有生命周期管理服务器，关闭它们。|
|destroyNode()|释放对象使用的资源。|

构建应用程序的一般模板如下：

```python

from rm_decision.robot_navigator import BasicNavigator
import rclpy

rclpy.init()

nav = BasicNavigator()
...
nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # 如果自动启动，则使用 `lifecycleStartup()`
...
path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)
...
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	if feedback.navigation_duration > 600:
		nav.cancelTask()
...
result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('目标成功完成!')
elif result == TaskResult.CANCELED:
    print('目标已取消!')
elif result == TaskResult.FAILED:
    print('目标失败!')
```



