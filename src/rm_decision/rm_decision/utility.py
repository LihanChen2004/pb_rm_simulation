import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from rm_decision.robot_navigator import TaskResult
import time
import yaml


class Utility:

    def __init__(self):
        self.goal_dict = None
        self.goal_path = None
        self.nav_goal_name = None
        self.random_list = None
        self.msg_callback = None
        self.yaml_path = None

    def yaml_read(self):
        with open(self.yaml_path, 'r', encoding='utf-8') as f:
            cfg = f.read()
            self.goal_dict = yaml.safe_load(cfg)
            self.nav_goal_name = list(self.goal_dict.keys())
            self.goal_path = self.goal_dict['path']
            self.random_list = self.goal_dict['random_list']

    @staticmethod
    def wait_for_message(node, topic_type, topic):
        class _vfm(object):
            def __init__(self):
                self.msg = None

            def cb(self, msg):
                self.msg = msg

        vfm = _vfm()
        subscription = node.create_subscription(topic_type, topic, vfm.cb, 1)
        while rclpy.ok():
            if vfm.msg is not None:
                return vfm.msg
            rclpy.spin_once(node)
            time.sleep(0.001)
        # unsubscription
        subscription.destroy()

    def decide_robot_to_follow(self):
        """
        选择要跟随的机器人。
        """
        # 寻找血量最高的友方机器人
        max_hp = 0
        robot_id = 0
        for i in [1, 3, 4, 5]:  # 仅考虑跟随英雄, 三号步兵, 四号步兵, 五号步兵
            if self.msg_callback.friend_robot_HP[i] > max_hp:
                max_hp = self.msg_callback.friend_robot_HP[i]
                robot_id = i
        self.get_logger().info('跟随 ' + str(robot_id) + ' 号机器人')
        return robot_id

    def goto_point(self, goal_point, nav_timeout):
        """
        导航到给定的目标点。

        Args:
        - goal_point: 目标点的坐标
        - nav_timeout: 导航超时时间

        Returns:
        - 1-到达目标, 0-失败
        """
        # 设置 Nav 目标点
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_point[0]
        goal_pose.pose.position.y = goal_point[1]
        goal_pose.pose.orientation.w = 0.0

        # 发送 Nav 目标点
        self.navigator.goToPose(goal_pose)

        self.last_FSM_state = self.FSM_state

        while not self.navigator.isTaskComplete():

            # 处理反馈
            feedback = self.navigator.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=nav_timeout):
                self.get_logger().info('导航超时')
                self.navigator.cancelTask()
                return 0

            if self.last_FSM_state != self.FSM_state:
                self.navigator.cancelTask()
                self.get_logger().info('状态更换, 导航取消')
                return 0

        # 根据返回代码执行某些操作
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return 1
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return 0
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return 0
        else:
            print('Goal has an invalid return status!')
