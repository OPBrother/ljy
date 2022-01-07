#!/usr/bin/env python
# encoding: utf-8
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
import time
from re import S
import rospy
import numpy as np
from lib.envs.XMLReader import parse
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self, ):
        self.goal_x = 6.3
        self.goal_y = 4.3
        self.angular_speed = 0.3
        self.linear_speed = 0.1
        self.min_distance = 0.5
        self.linear_duration = self.min_distance / self.linear_speed 
        self.action_yaw = [-0.5*pi, pi,0,0.5*pi]
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # 机器人方位
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        # 重置机器人
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.filename = "turtlebot.xml"
        self.actions, self.ccontrollable,  self.ncontrollable,  self.states,  self.terminal,  self.initial_state,  self.transitions = parse(self.filename)
        self.actual_state = self.initial_state
        self.action = None
        self.state_tran = np.ones([5,7], dtype=type)
        for i in range(7):
            for j in range(5):
                self.state_tran[j][i] = [0.5+1*i, 0.5+1*j]
        self.state_tran = self.state_tran.reshape(1,35)
        self.yaw =0
        self.goal_angle =0
        self.goal_distance= 0

    def getGoalDistace(self):
        goal_distance = round(self.goal_x - self.position.x+ self.goal_y - self.position.y, 2)
        self.goal_distance =goal_distance

    # 实时获取机器人位置
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw =yaw

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        self.goal_angle =goal_angle

    def getState(self, scan):
        scan_range = []
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        # 撞墙，回合结束
        if min_range > min(scan_range) > 0:
            done = True
        # 接近奖励，赢得游戏
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True

        return done

    def setReward(self, done):
        # reward = 50/self.goal_distance-10
        reward = -self.goal_distance
        if done:
            rospy.loginfo("Collision!!")
            # reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 1000
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
    
        return reward

    def step(self, action, pre_state):
         # 得到fsm当前状态
        for i in range(35):
            if self.state_tran[0][i] == pre_state:
                self.actual_state = i
                break
        # 给定旋转速度使其旋转
        # 达到转到角度时停止转动
        vel_cmd = Twist()
        if abs(round(self.yaw - self.action_yaw[action], 2)) > 0.04: 
            vel_cmd.angular.z = self.angular_speed
            self.pub_cmd_vel.publish(vel_cmd)
        while 1:
            if abs(round(self.yaw - self.action_yaw[action], 2)) < 0.1:
                self.pub_cmd_vel.publish(Twist())
                time.sleep(1)

                vel_cmd.angular.z = 0
                vel_cmd.linear.x = self.linear_speed
                self.pub_cmd_vel.publish(vel_cmd)
                break
            
        # 在规定时间内到达下一个状态停下
        # 得到fsm执行事件后新状态
        for trans in self.transitions:
            if (trans[0] == self.actual_state and trans[2] == action):
                self.actual_state = trans[1]
                break
        state = self.state_tran[0][self.actual_state]
        start_time = time.time()
        flag = False
        while not flag and time.time()- start_time < 10:
            # 计时，5秒内没到达就先退出（之后可改为try，raise）
            if  abs(round(math.hypot(state[0] - self.position.x, state[1] - self.position.y),2)) < 0.15:
                self.pub_cmd_vel.publish(Twist())
                flag = True
        if not flag:
            self.pub_cmd_vel.publish(Twist())
            rospy.loginfo('没有到达下个状态')      
            
        # 没有到达则检查是否撞墙
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        done = self.getState(data)
        self.getGoalDistace()
        reward = self.setReward( done)

        return state, reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('scan', LaserScan, timeout=5)
        #     except:
        #         pass

        # if self.initGoal:
        self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            # self.initGoal = False

        self.getGoalDistace()
        self.get_goalbox = False
        state = [round(self.position.x, 1),round( self.position.y,1)]
        return state

