# ATTENTION

> 实验性功能，决策判断十分简陋，仍在开发中

# rm_decision

## 概述

`rm_decision` 由 深圳北理莫斯科大学-北极熊战队 开发的哨兵决策模块。作为 RMUC 哨兵机器人的决策逻辑，根据机器人状态、视觉识别结果、队友位置等做出决策，从而在不同的状态之间切换，执行巡逻、跟随、攻击和逃跑等动作。代码参考了 [华农3V3哨兵决策](https://github.com/SCAU-RM-NAV/rm2022_auto_infantry_ws/tree/main/src/auto_nav/scripts)

## 节点功能

- 订阅来自其他节点的机器人状态、血量、友方位置、视觉识别等消息。

- 根据接收到的消息更新状态机的状态。

- 根据当前状态执行相应的任务，包括巡逻、跟随、攻击和逃跑等。

- 使用 [Nav2_API](/src/rm_decision/Nav2_API.md) 发送导航信息，交由导航节点规划

## 主要类和方法

### AutoFSM类

- `__init__`: 初始化方法，设置节点名称、配置参数、创建订阅器、初始化等。

- `init_wait_for_messages`: 等待各种消息到达的初始化方法。

- `thread_FSM_state`: 状态更新线程，根据接收到的消息更新状态机的状态。

- `state_update`: 实际的状态更新逻辑，根据不同条件更新状态。

- `thread_FSM`: 任务执行线程，根据状态机的状态执行相应任务。

- `mission_random`: 随机巡逻任务，随机选择路径上的目标点进行巡逻。

- `mission_follow`: 跟随任务，根据当前友方机器人的状态选择一个进行跟随。

- `mission_attack`: 攻击任务，暂未实现。

- `mission_escape`: 逃跑任务，根据事先定义的逃跑点进行逃跑（一键回城）

### CallbackMsg类

- `callback_AllRobotHP`: 处理 `AllRobotHP` 消息的回调函数，更新友方和敌方机器人的血量信息。

- `callback_FriendLocation`: 处理 `FriendLocation` 消息的回调函数，更新友方机器人的位置信息。

- `callback_RobotStatus`: 处理 `RobotStatus` 消息的回调函数，更新机器人的状态信息。

- `callback_rm_vision`: 处理 `Armor` 消息（视觉识别）的回调函数，接收视觉部分识别到的装甲板信息，更新敌人距离信息。

### Utility类

- `yaml_read`: 从YAML文件中读取配置，包括目标点、路径和随机点列表。

- `wait_for_message`: 等待指定消息到达的方法。

- `decide_robot_to_follow`: 选择要跟随的机器人的方法，目前选择血量最高的友方机器人。

- `goto_point`: 导航到指定目标点的方法，使用 Nav2 导航节点执行导航任务。

## 使用方法

### 启动有限状态机决策

```Shell
ros2 launch rm_decision pb_auto_fsm_launch.py
```

### 虚拟裁判系统和视觉部分，发布假消息

- robot_status

    ```
    source install/setup.bash
    ros2 topic pub -r 5 /robot_status rm_decision_interfaces/msg/RobotStatus "{
        robot_id: 7,
        current_hp: 200,
        maximum_hp: 200,
        shooter_heat_limit: 50,
        shooter_heat: 19,
    }"
    ```

- robot_hp

    ```
    source install/setup.bash
    ros2 topic pub -r 5 /robot_hp rm_decision_interfaces/msg/AllRobotHP "{
        red_1_robot_hp: 200,
        red_2_robot_hp: 90,
        red_3_robot_hp: 100,
        red_4_robot_hp: 40,
        red_5_robot_hp: 200,
        red_7_robot_hp: 200,
        red_outpost_hp: 0,
        red_base_hp: 30,
        blue_1_robot_hp: 95,
        blue_2_robot_hp: 85,
        blue_3_robot_hp: 75,
        blue_4_robot_hp: 200,
        blue_5_robot_hp: 55,
        blue_7_robot_hp: 45,
        blue_outpost_hp: 35,
        blue_base_hp: 25
    }"
    ```

- friend_location

    ```
    source install/setup.bash
    ros2 topic pub -r 5 /friend_location rm_decision_interfaces/msg/FriendLocation "{
        hero_x: 2.75, hero_y: 0.0, 
        engineer_x: 10.36, engineer_y: -3.79, 
        standard_3_x: 7.91, standard_3_y: 3.74, 
        standard_4_x: 12.00, standard_4_y: 4.15, 
        standard_5_x: 18.75, standard_5_y: 0.0
    }" 
    ```
    `hero`: 己方哨兵出生点 <br>
    `engineer`: 能量机关（靠己方）<br>
    `standard_3`: 己方环形高地 <br>
    `standard_4`: 敌方前哨站 <br>
    `standard_5`: 敌方基地 <br>

- armors

    ```
    source install/setup.bash
    ros2 topic pub -r 5 /detector/armors rm_decision_interfaces/msg/Armor "{
        number: '1',
        type: 'small',
        distance_to_image_center: 0.0,
        pose: {
            position: {x: 0.0, y: 0.0, z: 0.0},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
    }"

    ```


