from rclpy.node import Node
from rclpy import logging

class CallbackMsg(Node):
    """
    自定义消息回调类
    """

    def __init__(self):
        self.friend_color = 0 # 0-Red, 1-Blue

        # Robot HP （1-英雄 2-工程 3-步兵 4-步兵 5-步兵 7-哨兵 8-前哨站 9-基地）
        self.friend_robot_HP = [0] * 10;
        self.enemy_robot_HP = [0] * 10;
        
        # Robot Status
        self.robot_id = 0 # 机器人ID
        self.current_hp = 0 # 当前血量
        self.maximum_hp = 0 # 最大血量
        self.shooter_heat_limit = 0 # 热量上限
        self.shooter_heat = 0 # 发射机构的枪口热量 

        # Friend Position
        self.friend_position = [[]]

        # RM Vision
        self.enemy_dis = 0

        # Custom Variables
        self.percent_hp = 0
        self.high_heat_flag = 0
    
    def callback_AllRobotHP(self, msg):
        if self.friend_color == 0:
            self.friend_robot_HP[1] = msg.red_1_robot_hp
            self.friend_robot_HP[2] = msg.red_2_robot_hp
            self.friend_robot_HP[3] = msg.red_3_robot_hp
            self.friend_robot_HP[4] = msg.red_4_robot_hp
            self.friend_robot_HP[5] = msg.red_5_robot_hp
            self.friend_robot_HP[7] = msg.red_7_robot_hp # 友方哨兵血量
            self.friend_robot_HP[8] = msg.red_outpost_hp
            self.friend_robot_HP[9] = msg.red_base_hp
            self.enemy_robot_HP[1] = msg.blue_1_robot_hp
            self.enemy_robot_HP[2] = msg.blue_2_robot_hp
            self.enemy_robot_HP[3] = msg.blue_3_robot_hp
            self.enemy_robot_HP[4] = msg.blue_4_robot_hp
            self.enemy_robot_HP[5] = msg.blue_5_robot_hp
            self.enemy_robot_HP[7] = msg.blue_7_robot_hp # 敌方哨兵血量
            self.enemy_robot_HP[8] = msg.blue_outpost_hp
            self.enemy_robot_HP[9] = msg.blue_base_hp
        elif self.friend_color == 1:
            self.friend_robot_HP[1] = msg.blue_1_robot_hp
            self.friend_robot_HP[2] = msg.blue_2_robot_hp
            self.friend_robot_HP[3] = msg.blue_3_robot_hp
            self.friend_robot_HP[4] = msg.blue_4_robot_hp
            self.friend_robot_HP[5] = msg.blue_5_robot_hp
            self.friend_robot_HP[7] = msg.blue_7_robot_hp # 友方哨兵血量
            self.friend_robot_HP[8] = msg.blue_outpost_hp
            self.friend_robot_HP[9] = msg.blue_base_hp
            self.enemy_robot_HP[1] = msg.red_1_robot_hp
            self.enemy_robot_HP[2] = msg.red_2_robot_hp
            self.enemy_robot_HP[3] = msg.red_3_robot_hp
            self.enemy_robot_HP[4] = msg.red_4_robot_hp
            self.enemy_robot_HP[5] = msg.red_5_robot_hp
            self.enemy_robot_HP[7] = msg.red_7_robot_hp # 敌方哨兵血量
            self.enemy_robot_HP[8] = msg.red_outpost_hp
            self.enemy_robot_HP[9] = msg.red_base_hp
        else :
            logging.get_logger('AllRobotHP').warn('Cannot identified Enemies and friends!!!')
        
        # logging.get_logger('AllRobotHP').info('Friend HP: ' + str(self.friend_robot_HP))
        # logging.get_logger('AllRobotHP').info('Enemy HP: ' + str(self.enemy_robot_HP))

    def callback_FriendLocation(self, msg):
        self.friend_position = [[], # 下标索引从1开始
                                [msg.hero_x, msg.hero_y], 
                                [msg.engineer_x, msg.engineer_y], 
                                [msg.standard_3_x, msg.standard_3_y], 
                                [msg.standard_4_x, msg.standard_4_y], 
                                [msg.standard_5_x, msg.standard_5_y]]
    
    def callback_RobotStatus(self, msg):
        self.robot_id = msg.robot_id
        self.current_hp = msg.current_hp
        self.maximum_hp = msg.maximum_hp
        self.shooter_heat_limit = msg.shooter_heat_limit
        self.shooter_heat = msg.shooter_heat

        self.percent_hp = self.current_hp / self.maximum_hp

        if self.shooter_heat > (self.shooter_heat_limit - 5):
            self.high_heat_flag = 1
        else:
            self.high_heat_flag = 0
        '''
        敌我识别
        TODO：根据24赛季新裁判系统协议，更改敌我判断的条件
        '''
        if 1 <= self.robot_id <= 7:
            self.friend_color = 0
            # logging.get_logger('RobotStatus').info('We are: RED')
        elif 101 <= self.robot_id <= 107:
            self.friend_color = 1
            # logging.get_logger('RobotStatus').info('We are: BLUE')
        else:
            logging.get_logger('RobotStatus').warn('Cannot identified Color')

        # DEBUG
        # logging.get_logger('RobotStatus').info('self.robot_id: ' + str(self.robot_id))
        # logging.get_logger('RobotStatus').info('self.current_hp: ' + str(self.current_hp))
        # logging.get_logger('RobotStatus').info('self.maximum_hp: ' + str(self.maximum_hp))
        # logging.get_logger('RobotStatus').info('self.shooter_heat_limit: ' + str(self.shooter_heat_limit))
        # logging.get_logger('RobotStatus').info('self.shooter_heat: ' + str(self.shooter_heat))
        # logging.get_logger('RobotStatus').info(' ------------------------------------- ')

    def callback_rm_vision(self, msg):
        self.enemy_dis = msg.distance_to_image_center