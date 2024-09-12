#!/usr/bin/env python3.6
# -*- coding: utf-8 -*
import rospy
import string
import math
import time
import sys
import csv
import numpy as np
import os
import threading
import subprocess

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose


def load_data(path):
    goalList = []
    with open(os.path.join("points", path), "r") as f:
        reader = csv.reader(f)
        for cols in reader:
            goalList.append([float(value) for value in cols])

        goalList = np.array(goalList)
        print("read {} suc!!".format(path))
    return goalList

def to_quaternion(yaw):
    return [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]


def start_yolov():
    pass


class MultiGoals:
    def __init__(self, map_frame):
        self.load_all_data()
        # 需要一个定位坐标点(出了栏杆) -> 根据第一个红色锥桶和第二个红色锥桶的距离和角度计算出第二个红色锥桶的坐标 -> 根据第二个红色锥桶的距离和角度计算出第三个锥桶的相对坐标 -> 根据第三个锥桶计算出最后拐角的相对坐标 -> 直线行走(交给踢球)

        self.now_object_angle = 0
        self.check_start = False
        self.check_stop = False
        self.finish_run_time = 5

        self.run = False
        self.MIN_DISTANCE = 0.05
        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pose_amcl = rospy.Subscriber(
            "/odometry/filtered_map", Odometry, self.getPose_odom, queue_size=10
        )

        self.sub_command = rospy.Subscriber(
            "/control_flag", String, self.on_control_flag, queue_size=10
        )

        self.pub_status = rospy.Publisher("/dog_status", String, queue_size=1)

        self.up_command = rospy.Subscriber(
            "/goal_loop_control", String, self.on_up_command, queue_size=1
        )

        self.now_object_angle_sub = rospy.Subscriber(
            "/object_angle", String, self.on_object_angle, queue_size=1
        )

        self.goalListX = []

        self.run_finish_lambda = lambda: print("finish")

        self.init_data()

        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame

        # 启动yolo程序
        start_yolov()
        print("------------------yolo----------")
        self.start_time = rospy.get_time()

    def init_data(self):
        self.run = False
        # self.kx = 0
        # self.ky = 0
        self.gx = 0
        self.gy = 0
        self.flag = 1
        self.MIN_DISTANCE = 0.3 # min distance of the judge between the goal and odometrypose
        self.LONG = len(self.goalListX)
        self.goalId = 0
        self.count = 0
        self.start_time = 0
        self.pubfinal = False

    def load_all_data(self):
        # 原点
        self.zero_point = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.02629456914795107, 0.9996542380409956]])
        # 加载stop坐标点
        self.stop_point = load_data("stop_point.csv")
        # 加载起始点位置(第一个栏杆之前, 第一个锥桶之后的一个坐标)
        self.start_point = load_data("start_point.csv")
        # 第二个点(第二个锥桶之后)(计算得知)
        self.cone2_back_point = load_data("cone2.csv")
        # # 第三个点(第三个锥桶之后)(计算得知)
        self.cone3_back_point = load_data("cone3.csv")
        # # 最后拐角(计算得知)
        self.corner_point = load_data("corner.csv")
        # 最后踢球
        self.last_point = load_data("last.csv")

    def statusCB(self):
        self.gx = (
            self.goalListX[self.goalId - 1]
            if (self.goalId != 0)
            else self.goalListX[self.goalId]
        )
        self.gy = (
            self.goalListY[self.goalId - 1]
            if (self.goalId != 0)
            else self.goalListY[self.goalId]
        )

        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)

        # print("status-------------------", self.dist)
        if not self.dist is None and self.dist < self.MIN_DISTANCE and self.flag == 1:
            if (len(self.goalListX)) == 1:
                self.run = False
                self.goalId = 0
                print("final")
                self.run_finish_lambda()
                return

            finish_time = rospy.get_time()
            interval = finish_time - self.start_time
            print(interval)
            if self.goalId == self.LONG:
                self.goalId = 0
                self.run = False
                print("final")
                self.run_finish_lambda()
                return

            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.position.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.x = self.goalListQX[self.goalId]
            self.goalMsg.pose.orientation.y = self.goalListQY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListQZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListQW[self.goalId]
            self.pub.publish(self.goalMsg)

            rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)
            rospy.loginfo("intostatusCB")
            self.count = self.count + 1
            print(self.count)

            if self.goalId < (len(self.goalListX)):
                time.sleep(1)
                self.goalId = self.goalId + 1
            else:
                self.goalId = 0
                self.run = False
                print("final")
                self.run_finish_lambda()

    def getPose_odom(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y

        dist = self.distance(self.kx, self.ky, self.start_point[0][0], self.start_point[0][1])
        dist_stop = self.distance(self.kx, self.ky, self.stop_point[0][0], self.stop_point[0][1])

        if not dist is None and dist < 0.1 and not self.check_start:
            self.pub_status.publish("now-cone1")
            self.check_start = False

        if not dist_stop is None and dist_stop < 0.1 and not self.check_stop:
            self.pub_status.publish("now-stop")
            self.check_stop = False

        if self.run:
            self.statusCB()

    def distance(self, kx, ky, gx, gy):
        try:
            return math.sqrt((kx - gx) ** 2 + (ky - gy) ** 2)
        except:
            return None

    def change_status(self, goalList):
        self.goalListX = goalList[:, 0]
        self.goalListY = goalList[:, 1]
        self.goalListZ = goalList[:, 2]
        self.goalListQX = goalList[:, 3]
        self.goalListQY = goalList[:, 4]
        self.goalListQZ = goalList[:, 5]
        self.goalListQW = goalList[:, 6]

        self.init_data()
        self.run = True

        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalListX[self.goalId]
        self.goalMsg.pose.position.y = self.goalListY[self.goalId]
        self.goalMsg.pose.position.z = self.goalListZ[self.goalId]
        self.goalMsg.pose.orientation.x = self.goalListQX[self.goalId]
        self.goalMsg.pose.orientation.y = self.goalListQY[self.goalId]
        self.goalMsg.pose.orientation.z = self.goalListQZ[self.goalId]
        self.goalMsg.pose.orientation.w = self.goalListQW[self.goalId]
        self.pub.publish(self.goalMsg)

        rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)

    def calc_next_point(self, cone_points, angle_q = None):

        temp = cone_points.copy()

        print(np.size(temp, 0))

        for i in range(np.size(temp, 0)):
            # temp[i][0] -= dog_x
            # temp[i][1] -= dog_y

            temp[i][0] += self.kx
            temp[i][1] += self.ky

            if not angle_q is None:
                temp[i][3] += angle_q[0]
                temp[i][4] += angle_q[1]
                temp[i][5] += angle_q[2]
                temp[i][6] += angle_q[3]

        return temp

    def finish_cone2(self):
        time.sleep(self.finish_run_time)
        self.pub_status.publish("finish-cone2")

    def finish_cone3(self):
        time.sleep(self.finish_run_time)
        self.pub_status.publish("finish-cone3")

    def finish_corner(self):
        time.sleep(self.finish_run_time)
        self.pub_status.publish("finish-corner")

    def finish_last(self):
        time.sleep(self.finish_run_time)
        self.pub_status.publish("finish-last")

    def on_control_flag(self, data):
        print(data.data)

        if data.data == "cone2":
            # 前往锥桶2
            # 首先计算狗相对与锥桶2的位置
            # 让后用锥桶3相对与锥桶2的位置减去狗相对与锥桶2的位置
            # 需要根据当前根据视觉计算出来的距离重新计算点位置
            self.change_status(self.calc_next_point(self.cone2_back_point))
            self.pub_status.publish("start-cone2")
            self.run_finish_lambda = self.finish_cone2

        if data.data == "cone3":
            # 前往锥桶3
            # 需要根据当前根据视觉计算出来的距离重新计算点位置
            self.change_status(self.calc_next_point(self.cone3_back_point))
            self.pub_status.publish("start-cone3")
            self.run_finish_lambda = self.finish_cone3

        if data.data == "corner":
            # 前往最后拐角处
            # 需要根据当前根据视觉计算出来的距离重新计算点位置
            self.change_status(self.calc_next_point(self.corner_point))
            self.pub_status.publish("start-corner")
            self.run_finish_lambda = self.finish_corner

        if data.data == "last":
            # 将视觉计算出来的角度(yaw偏航角)转换成四元数
            # angle_q = to_quaternion(self.now_object_angle + math.pi / 2)
            # print(angle_q)
            self.change_status(self.calc_next_point(self.last_point))
            self.pub_status.publish("start-last")
            self.run_finish_lambda = self.finish_last


    def on_up_command(self, data):
        print("up command", data)
        self.up_command = data.data
        if data.data == "reload":
            self.load_all_data()


    def on_object_angle(self, data):
        data = data.data
        # print("data: ", data)
        # self.now_object_angle = float(data)

if __name__ == "__main__":
    try:
        # ROS Init
        rospy.init_node("multi_goals", anonymous=True)
        map_frame = rospy.get_param("frame", "odom")

        rospy.loginfo("Multi Goals Executing...")
        mg = MultiGoals(map_frame)
        # mg.run_loop()
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")
