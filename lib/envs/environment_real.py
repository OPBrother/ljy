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

import rospy
import numpy as np
from lib.envs.XMLReader import parse
from math import  hypot, sqrt, pow, pi, atan2,hypot
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PoseStamped
import sys

class Env():
    def __init__(self, action_dim=2):
        self.goal_x = 4.5
        self.goal_y = 3.5
        self.angular_speed = 1
        self.linear_speed = 1
        self.min_distance = 0.5
        self.action_yaw = [-0.5*pi, pi,0,0.5*pi]
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        # self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.r = rospy.Rate(1000)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)

        self.filename = "T2.xml"
        self.actions, self.ccontrollable,  self.ncontrollable,  self.states,  self.terminal,  self.initial_state,  self.transitions = parse(self.filename)
        self.actual_state = self.initial_state
        self.action = None
        self.x , self.y = 5,5
        self.state_tran = np.ones([self.y,self.x], dtype=type)
        for i in range(self.x):
            for j in range(self.y):
                self.state_tran[j][i] = [0.5+1*i, 0.5+1*j]
        self.state_tran = self.state_tran.reshape(1,self.x*self.y)
        self.yaw =0
        self.last_goal_distance= 0
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def wait_for_goal(self):
         raw_goal_data = None
         goal_data  = None
         
         print('waiting goal...')
         while raw_goal_data is None:
            try:
                raw_goal_data = rospy.wait_for_message('move_base_simple/goal',PoseStamped, timeout=5)
            except:
                pass
         while goal_data is None:
            try:
                goal_data = self.tfBuffer.transform(raw_goal_data, 'odom', timeout=rospy.Duration(10))
            except:
                e = sys.exc_info()
                rospy.logerr(e)
                sys.exit(1) 

         self.goal_x = goal_data.pose.position.x
         self.goal_y = goal_data.pose.position.y
         return self.goal_x, self.goal_y

    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(self.goal_x - self.position.x+ self.goal_y - self.position.y, 2)
        return goal_distance

    # 实时获取机器人位置
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw =yaw

    def getState(self, scan):
        scan_range = []
        min_range = 0.13
        done = False

        for i in range(0, len(scan.ranges), 15):
            temp = max(scan.ranges[i-1], scan.ranges[i], scan.ranges[i+1])
            if temp == float('Inf') or temp == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(temp) or temp == float('nan'):
                scan_range.append(0)
            else:
                scan_range.append(temp)
        # 撞墙，回合结束
        if min_range > min(scan_range) > 0:
            done = True
        # 接近奖励，赢得游戏
        current_distance = round(hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True

        return done

    def setReward(self,goal_distance, done):
        reward = -10*(goal_distance - self.last_goal_distance)
        if done:
            rospy.loginfo("Collision!!")
            reward = -100
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            # self.goal_x, self.goal_y = self.wait_for_goal()
            self.goal_x, self.goal_y = 5.3 ,4.3
        return reward

    def step(self, action, pre_state):
         # 得到fsm当前状态
        for i in range(self.x * self.y):
            if self.state_tran[0][i] == pre_state:
                self.actual_state = i
                break
        # 在规定时间内到达下一个状态停下
        # 得到fsm执行事件后新状态
        for trans in self.transitions:
            if (trans[0] == self.actual_state and trans[2] == action):
                self.actual_state = trans[1]
                break
        state = self.state_tran[0][self.actual_state]

        move_cmd = Twist()
        # while abs(round(self.yaw - self.action_yaw[action], 2)) > 0.1:
        #     move_cmd.angular.z = 0.1
        #     self.pub_cmd_vel.publish(move_cmd)

        # self.pub_cmd_vel.publish(Twist())
        last_rotation = self.yaw
        # 计算与下个目标点的欧式距离
        distance = sqrt(pow(state[0] - self.position.x, 2) + pow(state[1] - self.position.y, 2))
        while distance > 0.05:
            path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)

            if path_angle < -pi/4 or path_angle > pi/4:
                if state[1] < 0 and  self.position.y < state[1]:
                    path_angle = -2*pi + path_angle
                elif state[1] >= 0 and  self.position.y > state[1]:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and self.yaw <= 0:
                self.yaw = 2*pi + self.yaw
            elif last_rotation < -pi+0.1 and self.yaw > 0:
                self.yaw = -2*pi + self.yaw
            move_cmd.angular.z = self.angular_speed * path_angle-self.yaw

            distance = sqrt(pow((state[0] - self.position.x), 2) + pow((state[1] -  self.position.y), 2))
            move_cmd.linear.x = min(self.linear_speed * distance, 0.2)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = self.yaw
            self.pub_cmd_vel.publish(move_cmd)
            # self.r.sleep()
        self.pub_cmd_vel.publish(Twist())
        # 没有到达则检查是否撞墙
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        done = self.getState(data)
        goal_distance = self.getGoalDistace()
        reward = self.setReward(goal_distance,done)
        self.last_goal_distance = goal_distance
        return state, reward, done

    def reset(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            # self.goal_x, self.goal_y = self.wait_for_goal()
            self.goal_x, self.goal_y = 4.5 , 3.5
            self.initGoal = False
        self.goal_distance = self.getGoalDistace()
        # state = [round(self.position.x, 1),round( self.position.y,1)]
        state = [0.5 , 0.5]
        return state
