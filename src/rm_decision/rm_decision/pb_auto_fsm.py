import os
import inspect
import threading
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy import qos

from geometry_msgs.msg import Twist, PoseStamped
from rm_decision_interfaces.msg import RobotStatus, FriendLocation, AllRobotHP, Armor

from rm_decision.callback_msg import CallbackMsg
from rm_decision.robot_navigator import BasicNavigator
from rm_decision.utility import Utility

class AutoFSM(Node):
    """
    深圳北理莫斯科大学 北极熊战队 哨兵决策部分
    Attention: 开发中, 有点乱！
    """

    def __init__(self):
        super().__init__('auto_fsm_node')

        # Set Config
        self.set_unhealth_point_percent = 0.3
        self.set_enemy_max_dis = 5.0
        self.set_attack_blacklist = [1, 1, 1, 1, 1, 1, 1, 1]
        self.yaml_name = '/goal.yaml'
        self.yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '/config' + self.yaml_name
        self.goal_dict = dict()
        self.goal_path = []
        self.nav_goal_name = []
        self.random_list = []
        self.nav_timeout = 10.0
        self.interval_time = 1.0
        self.follow_robot_id = None
        self.msg_callback = CallbackMsg()
        self.FSM_state = 2

        # Create subscription
        self.robot_hp_sub = self.create_subscription(AllRobotHP,
                                                    '/robot_hp',
                                                    self.msg_callback.callback_AllRobotHP,
                                                    QoSProfile(depth=1))
        self.enemy_locate_sub = self.create_subscription(FriendLocation,
                                                        '/friend_location',
                                                        self.msg_callback.callback_FriendLocation,
                                                        QoSProfile(depth=1))
        self.robot_states_sub = self.create_subscription(RobotStatus,
                                                         '/robot_status',
                                                         self.msg_callback.callback_RobotStatus,
                                                         QoSProfile(depth=1))
        self.rm_vision_sub = self.create_subscription(Armor,
                                                        '/detector/armors',
                                                        self.msg_callback.callback_rm_vision,
                                                        qos.qos_profile_sensor_data)

        # Init function
        Utility.yaml_read(self)
        self.rate = self.create_rate(0.5)
        self.rate_nav = self.create_rate(100)
        self.init_wait_for_messages()

        # Connect to Navigation
        self.navigator = BasicNavigator()
        self.navigator.get_logger().info('等待 navigator 服务器连接...')
        self.navigator.waitUntilNav2Active()
        self.navigator.get_logger().info('成功连接导航服务')

        # Create Threads
        self.thread_FSM_ = threading.Thread(target=self.thread_FSM)
        self.thread_FSM_.start()

        self.thread_FSM_state_ = threading.Thread(target=self.thread_FSM_state)
        self.thread_FSM_state_.start()

    def init_wait_for_messages(self):
        message_topics = [
            (RobotStatus, '/robot_status'),
            (AllRobotHP, '/robot_hp'),
            (FriendLocation, '/friend_location'),
            (Armor, '/detector/armors')
        ]
        for message_type, topic in message_topics:
            self.get_logger().info(f'等待 {topic} 消息')
            Utility.wait_for_message(self, message_type, topic)

    def thread_FSM_state(self):
        while rclpy.ok():
            self.state_update()
            self.rate_nav.sleep()

    def state_update(self):
        if self.msg_callback.percent_hp < self.set_unhealth_point_percent or self.msg_callback.high_heat_flag == 1:
            '''
            血量百分比低于设定值 或 枪口热量过高，进入逃跑状态
            '''
            self.FSM_state = 4
        
        elif self.msg_callback.enemy_dis != 0 and self.msg_callback.enemy_dis <= self.set_enemy_max_dis and self.msg_callback.high_heat_flag == 0 :
            '''
            视觉识别到敌人，进入攻击状态
            '''
            self.FSM_state = 3

        elif self.msg_callback.friend_robot_HP[8] == 0 and self.msg_callback.high_heat_flag == 0:
            '''
            己方前哨站被击毁，进入跟随状态
            '''
            self.FSM_state = 2
        
        elif self.msg_callback.friend_robot_HP[8] != 0 and self.msg_callback.high_heat_flag == 0:
            '''
            己方前哨站存活，进入巡逻状态
            '''
            self.FSM_state = 1

        
        # elif self.game_state !=4:
        #     self.FSM_state =0 #休眠状态
        #     self.get_logger().info('进入休眠状态')

    def thread_FSM(self):
        self.get_logger().info('准备完成, 等待比赛开始')
        while rclpy.ok():
            if self.FSM_state == 1:
                self.mission_random() # 巡逻模式
            elif self.FSM_state == 2:
                self.mission_follow() # 跟随模式
            elif self.FSM_state == 3:
                self.mission_attack() # 攻击模式
            elif self.FSM_state == 4:
                self.mission_escape() # 逃跑模式

    def mission_random(self):
        self.get_logger().info("开始随机跑点")

        while self.FSM_state == 1 and rclpy.ok():
            random_num = random.randint(0, len(self.random_list) - 1)
            reach_result = Utility.goto_point(self, self.goal_dict[self.random_list[random_num]], self.nav_timeout)
            if reach_result == 1:
                self.get_logger().info("到达目标点 " + str(self.random_list[random_num]))
                reach_result = 0
            time.sleep(self.interval_time)
        self.get_logger().info("退出随机跑点")

    def mission_follow(self):
        self.get_logger().info("开始跟随")

        while self.FSM_state == 2 and self.msg_callback.percent_hp > 0.3 and rclpy.ok():
            follow_robot_id = Utility.decide_robot_to_follow(self)

            # 检查是否出现了不同的follow_robot_id
            if follow_robot_id != self.follow_robot_id:
                self.get_logger().info("切换到跟随" + str(follow_robot_id) + "号机器人")
                self.navigator.cancelTask()

            self.follow_robot_id = follow_robot_id

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.msg_callback.friend_position[follow_robot_id][0]
            goal_pose.pose.position.y = self.msg_callback.friend_position[follow_robot_id][1]
            goal_pose.pose.orientation.w = 0.0

            self.navigator.goToPose(goal_pose)

            self.rate.sleep()

        self.get_logger().info("退出跟随")

    def mission_attack(self):
        self.get_logger().info("开始攻击")
        while self.FSM_state == 3 and rclpy.ok():
            self.get_logger().info("正在攻击状态中...")
            self.rate_nav.sleep()
        self.get_logger().info("退出攻击")
    
    def mission_escape(self):
        self.get_logger().info("开始逃跑")
        while self.FSM_state == 4 and rclpy.ok():
            self.get_logger().info("正在逃跑状态中...")

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.goal_dict[self.random_list[0]][0]
            goal_pose.pose.position.y = self.goal_dict[self.random_list[0]][1]
            goal_pose.pose.orientation.w = 0.0

            self.navigator.goToPose(goal_pose)
            self.rate.sleep()
        self.get_logger().info("退出逃跑")

def main():
    rclpy.init()
    auto_fsm = AutoFSM()

    executor = MultiThreadedExecutor()
    executor.add_node(auto_fsm)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        auto_fsm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
