# fake_vel_transform

该包用于实现雷达和云台共用的 yaw 轴时的兼容性，即使云台处于旋转扫描状态，速度变换后，仍然可以实现较稳定的轨迹跟踪效果。

## 实现方式

创建 XYZ 轴基于 `base_link`, RPY 轴基于局部路径规划朝向的 `base_link_fake` 坐标系，使得 nav2 局部规划器能够将机器人的方向视为与当前路径规划方向一致。

nav2 发布的速度也是基于 `base_link_fake` 坐标系的，通过 tf2 将速度转换到 `base_link` 坐标系，使机器人能够正常运动。在仿真中表现即底盘小陀螺时仍能跟随轨迹。

电控端执行底盘控制命令时，机器人运动正方向为云台枪管朝向。

## fake_vel_transform_node

订阅：

- nav2 发布的基于 base_link_fake 坐标系的速度指令 `/cmd_vel`
- nav2 controller 发布的局部路径朝向 `/local_path`
- `odom` 到 `base_link` 的 tf 变换

发布：

- 转换到 base_link 坐标系的速度 `/cmd_vel_chassis`

静态参数：

- 底盘固定旋转速度 `spin_speed`

  搭配电控固定小陀螺速度，将 spin_speed 设为负，可实现移动时小陀螺减慢。
