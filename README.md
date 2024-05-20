# PB_RM_Simulation

深圳北理莫斯科大学 北极熊战队 哨兵导航仿真/实车包

## 一. 项目介绍

本项目使用全向移动小车，附加 Livox Mid360 雷达与 IMU，在 RMUC/RMUL 地图进行导航算法仿真，仅需要调整参数即可移植到真实机器人中导航。

早期功能演示视频：[寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)

|Gazebo 仿真|Fast_LIO/Point_LIO + Navigation2|
|:-:|:-:|
|![Gazebo 仿真](.docs/gazebo_RMUL.png)|![Fast_LIO/Point_LIO + Navigation2](.docs/FAST_LIO+Nav2.png)|

|动态避障|
|:-:|
|![Gazebo 仿真](.docs/2024.5.11RMUL动态避障2.gif)|

|一键偷家|
|:-:|
|![Gazebo 仿真](.docs/2024.5.11RMUC轨迹跟踪一键偷家.gif)|

### 1.1 rm_simulation 话题接口

| **Topic name**      | **Type**                        | **Note**                         |
|:-------------------:|:-------------------------------:|:--------------------------------:|
| /livox/lidar             | livox_ros_driver2/msg/CustomMsg | Mid360 自定义消息类型   |
| /livox/lidar/pointcloud | sensor_msgs/msg/PointCloud2     | ROS2 点云消息类型                      |
| /livox/imu                | sensor_msgs/msg/Imu             | Gazebo 插件仿真 IMU                  |
| /cmd_vel            | geometry_msgs/msg/Twist         | 麦克纳姆轮小车运动控制接口                  |

### 1.2 整体框图

![功能包流程图](.docs/功能包流程图.png)

## 二. 环境配置

当前开发环境为 Ubuntu22.04, ROS2 humble, Gazebo Classic 11.10.0

1. 克隆仓库

    ```sh
    git clone --recursive https://github.com/LihanChen2004/PB_RMSimulation.git
    ```

2. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    sudo apt install cmake
    ```

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

3. 安装依赖

    ```sh
    cd pb_rmsimulation

    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

4. 编译

    ```sh
    colcon build --symlink-install
    ```

## 三. 运行

### 3.1 可选参数

1. `world`:

    - 仿真模式
        - `RMUL` - [2024 Robomaster 3V3 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)
        - `RMUC` - [2024 Robomaster 7V7 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)

    - 真实环境
        - 自定，world 等价于 `.pcd(ICP使用的点云图)` 文件和 `.yaml(Nav使用的栅格地图)` 的名称

2. `mode`:
   - `mapping` - 边建图边导航
   - `nav` - 已知全局地图导航

3. `lio`:
   - `fastlio` - 使用 [Fast_LIO](https://github.com/LihanChen2004/FAST_LIO/tree/ROS2)，里程计约 10Hz
   - `pointlio` - 使用 [Point_LIO](https://github.com/LihanChen2004/Point-LIO/tree/RM2024_SMBU_auto_sentry)，可以输出100+Hz的Odometry，对导航更友好，但相对的，CPU占用会更高

4. `localization` (仅 `mode:=nav` 时本参数有效)
   - `slam_toolbox` - 使用 [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) localization 模式定位，动态场景中效果更好
   - `amcl` - 使用 [AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html) 经典算法定位
   - `icp` - 使用 [icp_registration](https://github.com/baiyeweiguang/CSU-RM-Sentry/tree/main/src/rm_localization/icp_registration)，仅在第一次启动或者手动设置 /initialpose 时进行点云配准。获得初始位姿后只依赖 LIO 进行定位，没有回环检测，在长时间运行后可能会出现累积误差。

    Tips:
    1. 若使用 AMCL 算法定位时，启动后需要在 rviz2 中手动给定初始位姿。
    2. 若使用 slam_toolbox 定位，需要提供 .posegraph 地图，详见 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)
    3. 若使用 ICP_Localization 定位，需要提供 .pcd 点云图

5. `lio_rviz`:
   - `True` - 可视化 FAST_LIO 或 Point_LIO 的点云图

6. `nav_rviz`:
   - `True` - 可视化 navigation2

### 3.2 仿真模式示例

- 边建图边导航

    ```sh
    ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=fastlio \
    lio_rviz:=False \
    nav_rviz:=True
    ```

- 已知全局地图导航

    ```sh
    ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
    ```

### 3.3 真实模式示例

- 边建图边导航

    ```sh
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=mapping  \
    lio:=fastlio \
    lio_rviz:=False \
    nav_rviz:=True
    ```

    Tips:

    1. 保存点云 pcd 文件：需先在 [fastlio_mid360.yaml](src/rm_nav_bringup/config/reality/fastlio_mid360_real.yaml) 中 将 `pcd_save_en` 改为 `true`，并设置 .pcd 文件的路径，运行时新开终端输入命令 `ros2 service call /map_save std_srvs/srv/Trigger`，即可保存点云文件。
    2. 保存地图：请参考 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)。地图名需要与 `YOUR_WORLD_NAME` 保持一致。

- 已知全局地图导航

    ```sh
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
    ```

    Tips: 栅格地图文件和 pcd 文件需具为相同名称，分别存放在 `src/rm_nav_bringup/map` 和 `src/rm_nav_bringup/PCD` 中，启动导航时 world 指定为文件名前缀即可。

### 3.4 小工具 - 键盘控制

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 四. 实车适配关键参数

1. 雷达 ip

    本导航包已内置 [livox_ros_driver2](https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble)，可直接修改 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `lidar_configs` - `ip`

2. 测量机器人底盘正中心到雷达的相对坐标

    x, y 距离比较重要，将影响云台旋转时解算到 base_link 的坐标准确性

    填入 [measurement_params_real.yaml](./src/rm_nav_bringup/config/reality/measurement_params_real.yaml)

    若雷达倾斜放置，无需在此处填入 rpy，而是将点云旋转角度填入 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `extrinsic_parameter`

3. 测量雷达与地面的垂直距离

    此参数影响点云分割效果

    填入 [segmentation_real.yaml](./src/rm_nav_bringup/config/reality/segmentation_real.yaml) - `sensor_height`

4. nav2_params

    参数很多，比较重要的是 robot_radius 和 速度相关参数。详见 [nav2官方文档](https://docs.nav2.org/)

## 后记

> 这个仿真包也是我学习的一个记录，也是我学习的一个起点。

很难想象一年前的五月，我连 Ubuntu 和 ROS 是都不知道。笔者大一时只是个混子，北极熊第一届视觉组成员（其实啥也没干），但误打误撞去了 2023 联盟赛广东站赛场。当时只是作为一个观众+摄影师，但赛场的氛围是无以言表的，回来后 RM 浓度就开始逐渐增高，尝试部署华师2023的视觉开源，也在这段时间逐渐学会了自学的方式，写下了第一篇 [算法组文档](https://flowus.cn/lihanchen/facb28a9-5d34-42a7-9bc8-630a182c3571) 。

暑假时参加了“快递速达”的支线任务（比赛），那是第一次初识导航，还记得举着电脑拿着2D雷达在实验楼建图的喜悦。后来被春茧里华农的“自动驾驶”哨兵震撼到了，于是乎大二上和“实验楼安家组”出去喝粥路上，队长问我 24 赛季你想做啥，我毫不犹豫的答出了“导航”。

抱着玩一玩练练手的心态创建了本仓库，第一个 [commit](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/commit/a3e475ce59c60d68462e4555a76113e4ba1295f1) 在 2023-09-27，当时借鉴的还是 [华农 2023 RMUL 哨兵导航开源包](https://github.com/SCAU-RM-NAV/rm2023_auto_sentry_ws) 和 [湖工大RMUC地图](https://github.com/HBUTHUANGPX/Hbut_LC_sentry)，在此基础上只是加了 mid360 的仿真。后来 2023.10 月 中南大学 FYT 战队开源了[RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry)，是当时少见的 ROS2 nav2 导航框架，于是乎当时就想着缝进初代导航包（直到现在也是中南 FYT 的模样）。在不断尝试新算法的过程中，对 Gazebo 和 ros2_launch 也有了更深入的了解。

如果没有开发这个仿真包，我可能也不会有机会接触到导航算法，也不会有机会接触到这么多优秀的开源项目。我还记得那是 2023.1.26，在港中深的哨兵上部署了我的仿真包（ RMUC 联队），改改 launch 文件居然就能让实车动起来了，意料之外地实现了 Sim2Real。由于学校没有机械专业，哨兵在联盟赛前不到两周才完全出生，于是乎备赛期间一直都是在赛博调车优化。

上赛场了，从观众席到检录区。但当时由于在仿真中忽略了雷达偏心放置时的 tf 问题，导致实车云台旋转时定位不准，最终也没能在 RMUL 中发挥丝滑的导航走位...挺遗憾的。

鄙人非大佬，只是个缝合怪菜鸡。我的学习路径个人觉得是有点畸形的，并不是自下而上地先学理论再实践，而是自上而下地先实践再不断补理论的坑。但这种学习方式也不断地给我正反馈，梦里都在改代码，每天醒来都有盼头。

## 致谢（不分先后）

Mid360 点云仿真：参考了 [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation/blob/main/src/livox_points_plugin.cpp)、 [livox_laser_simulation_RO2](https://github.com/stm32f303ret6/livox_laser_simulation_RO2/blob/main/src/livox_points_plugin.cpp)、 [Issue15: CustomMsg](https://github.com/Livox-SDK/livox_laser_simulation/issues/15)。

导航算法框架：基于 [中南大学 FYT 战队 RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry) 修改并适配仿真，在原有基础上添加对 base_link 的建模，提供多种可选定位方式并完善 launch 文件。

感谢深技大 Shockley，对雷达跟随云台旋转时的速度变换问题提供了很好的解决思路，从而有了 [fake_vel_transform](./src/rm_navigation/fake_vel_transform/) 功能包。

感谢上海工程技术大学、辽宁科技大学、上海电力大学对本开源包的深度使用与交流，给了我很多优化方向。

还有很多很多很多 RM 网友给了我很多鼓励和帮助，这里就不一一列举了（已达成龙王结局，QQ 里 RM 分组九十多人）。
