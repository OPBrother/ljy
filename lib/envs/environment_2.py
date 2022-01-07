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
import tf
from re import S
import rospy
import numpy as np
from lib.envs.XMLReader import parse
from math import  hypot, sqrt, pow, pi, atan2,hypot
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
        self.angular_speed = 1
        self.linear_speed = 1
        self.min_distance = 0.5
        # self.linear_duration = self.min_distance / self.linear_speed 
        self.action_yaw = [-0.5*pi, pi,0,0.5*pi]
        self.initGoal = True
        self.get_goalbox = False
        # self.position = Pose()
        self.position = Point()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(1)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
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

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def getGoalDistace(self):
        goal_distance = round(self.goal_x - self.position.x+ self.goal_y - self.position.y, 2)
        self.goal_distance =goal_distance

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            # self.position=  Point(*trans)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

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
        (self.position, rotation) = self.get_odom()
        current_distance = round(hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True

        return done

    def setReward(self, done):
        reward = 50/self.goal_distance-10
        # reward = -self.goal_distance
        reward = 0
        if done:
            rospy.loginfo("Collision!!")
            reward = -200
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
        # 在规定时间内到达下一个状态停下
        # 得到fsm执行事件后新状态
        for trans in self.transitions:
            if (trans[0] == self.actual_state and trans[2] == action):
                self.actual_state = trans[1]
                break
        state = self.state_tran[0][self.actual_state]

        move_cmd = Twist()
        (self.position, rotation) = self.get_odom()
        last_rotation = 0
        # 计算与下个目标点的欧式距离
        distance = sqrt(pow(state[0] - self.position.x, 2) + pow(state[1] - self.position.y, 2))
        while distance > 0.05:
            (self.position, rotation) = self.get_odom()
            x_start = self.position.x
            y_start = self.position.y
            path_angle = atan2(state[1] - y_start, state[0]- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if state[1] < 0 and y_start < state[1]:
                    path_angle = -2*pi + path_angle
                elif state[1] >= 0 and y_start > state[1]:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = self.angular_speed * path_angle-rotation

            distance = sqrt(pow((state[0] - x_start), 2) + pow((state[1] - y_start), 2))
            move_cmd.linear.x = min(self.linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.pub_cmd_vel.publish(move_cmd)
            self.r.sleep()
        (self.position, rotation) = self.get_odom()


        self.pub_cmd_vel.publish(Twist())
            
        # 没有到达则检查是否撞墙
        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('scan', LaserScan, timeout=5)
        #     except:
        #         pass
        (self.position, rotation) = self.get_odom()
        current_distance = round(hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True
        # done = self.getState(data)
        done = False
        self.getGoalDistace()
        reward = self.setReward( done)

        return state, reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
        self.goal_x, self.goal_y = self.respawn_goal.getPosition()
        self.getGoalDistace()
        self.get_goalbox = False
        # state = [round(self.position.x, 1),round( self.position.y,1)]
        state = [0.5,0.5]
        return state

