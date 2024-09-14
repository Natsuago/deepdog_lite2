#!/usr/bin/env python3.6
# -*- coding: utf-8 -*
import rospy  # ROS（机器人操作系统）Python库
import string  # 字符串处理库
import math  # 数学库，用于数学计算
import time  # 时间相关操作库
import sys  # 系统库，处理输入输出
import csv  # 用于处理CSV文件
import numpy as np  # Numpy库，提供数组操作功能
import os  # 操作系统接口库，用于文件路径处理
import threading  # 多线程处理库
import subprocess  # 子进程管理库，用于执行系统命令

# 导入ROS中的相关消息类型
from nav_msgs.msg import Odometry  # 导入里程计消息类型
from std_msgs.msg import String, Float64  # 导入标准的字符串和浮点数消息类型
from move_base_msgs.msg import MoveBaseActionResult  # 导入MoveBase的动作结果消息
from actionlib_msgs.msg import GoalStatusArray  # 导入目标状态数组消息
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose  # 导入几何相关消息类型

# 加载CSV文件中的坐标点数据
def load_data(path):
    goalList = []
    with open(os.path.join("points", path), "r") as f:
        reader = csv.reader(f)
        for cols in reader:
            goalList.append([float(value) for value in cols])  # 将每一列的值转换为浮点数并加入列表

        goalList = np.array(goalList)  # 转换为Numpy数组
        print("读取 {} 成功!".format(path))  # 打印成功读取文件的消息
    return goalList  # 返回读取的坐标点数据

# 将偏航角（yaw）转换为四元数表示，用于表示机器人姿态
def to_quaternion(yaw):
    return [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]  # 计算四元数表示

# YOLO程序启动函数（空函数，实际功能未实现）
def start_yolov():
    pass

# 多目标导航类
class MultiGoals:
    # 初始化函数，加载数据并设置ROS发布/订阅
    def __init__(self, map_frame):
        self.load_all_data()  # 加载所有路径点数据

        # 初始化相关参数，用于导航逻辑
        self.now_object_angle = 0
        self.check_start = False
        self.check_stop = False
        self.finish_run_time = 5

        self.run = False
        self.MIN_DISTANCE = 0.05  # 设定目标点与当前位置的最小距离，用于判断是否到达目标
        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)  # 目标点发布器
        self.pose_amcl = rospy.Subscriber(
            "/odometry/filtered_map", Odometry, self.getPose_odom, queue_size=10  # 订阅里程计消息
        )

        self.sub_command = rospy.Subscriber(
            "/control_flag", String, self.on_control_flag, queue_size=10  # 订阅控制标志
        )

        self.pub_status = rospy.Publisher("/dog_status", String, queue_size=1)  # 发布机器狗状态信息

        self.up_command = rospy.Subscriber(
            "/goal_loop_control", String, self.on_up_command, queue_size=1  # 订阅上行控制命令
        )

        self.now_object_angle_sub = rospy.Subscriber(
            "/object_angle", String, self.on_object_angle, queue_size=1  # 订阅物体角度信息
        )

        self.goalListX = []  # 初始化目标点X坐标列表

        self.run_finish_lambda = lambda: print("导航完成")  # 默认导航完成后的回调函数

        self.init_data()

        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame  # 设置目标点的参考坐标系

        # 启动YOLO目标检测程序（未具体实现）
        start_yolov()
        print("------------------YOLO启动成功----------")
        self.start_time = rospy.get_time()  # 记录程序启动时间

    # 初始化导航相关数据
    def init_data(self):
        self.run = False
        self.gx = 0
        self.gy = 0
        self.flag = 1
        self.MIN_DISTANCE = 0.3  # 目标点与当前位置的最小判断距离
        self.LONG = len(self.goalListX)  # 获取目标点的总数
        self.goalId = 0
        self.count = 0
        self.start_time = 0
        self.pubfinal = False

    # 加载所有目标点的数据，包括起点、拐角点、终点等
    def load_all_data(self):
        self.zero_point = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.02629456914795107, 0.9996542380409956]])  # 原点坐标
        self.stop_point = load_data("stop_point.csv")  # 加载停止点
        self.start_point = load_data("start_point.csv")  # 加载起始点
        self.cone2_back_point = load_data("cone2.csv")  # 第二个位置
        self.cone3_back_point = load_data("cone3.csv")  # 第三个位置
        self.corner_point = load_data("corner.csv")  # 拐角点位置
        self.last_point = load_data("last.csv")  # 最终点位置

    # 状态回调函数，处理到达目标点后的逻辑
    def statusCB(self):
        # 获取当前目标点的X和Y坐标
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

        # 计算当前位置和目标点之间的距离
        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)

        # 当距离小于设定的最小距离时，表示已到达目标点
        if not self.dist is None and self.dist < self.MIN_DISTANCE and self.flag == 1:
            if len(self.goalListX) == 1:
                self.run = False
                self.goalId = 0
                print("导航结束")
                self.run_finish_lambda()
                return

            finish_time = rospy.get_time()  # 获取当前时间，计算到达目标点的时间间隔
            interval = finish_time - self.start_time
            print(interval)

            if self.goalId == self.LONG:
                self.goalId = 0
                self.run = False
                print("导航完成")
                self.run_finish_lambda()
                return

            # 发布新的目标点
            self.goalMsg.header.stamp = rospy.Time.now()  # 更新时间戳
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.position.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.x = self.goalListQX[self.goalId]
            self.goalMsg.pose.orientation.y = self.goalListQY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListQZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListQW[self.goalId]
            self.pub.publish(self.goalMsg)  # 发布目标点消息

            rospy.loginfo("发布目标点！目标ID为: %d", self.goalId)
            self.count += 1  # 计数
            print(self.count)

            if self.goalId < len(self.goalListX):
                time.sleep(1)  # 延迟1秒，模拟移动过程
                self.goalId += 1
            else:
                self.goalId = 0
                self.run = False
                print("导航完成")
                self.run_finish_lambda()

    # 里程计消息的回调函数，获取当前位置
    def getPose_odom(self, data):
        self.kx = data.pose.pose.position.x  # 获取当前位置的X坐标
        self.ky = data.pose.pose.position.y  # 获取当前位置的Y坐标

        # 计算当前位置与起始点、停止点的距离
        dist = self.distance(self.kx, self.ky, self.start_point[0][0], self.start_point[0][1])
        dist_stop = self.distance(self.kx, self.ky, self.stop_point[0][0], self.stop_point[0][1])

        # 判断是否接近起点
        if not dist is None and dist < 0.1 and not self.check_start:
            self.pub_status.publish("now-cone1")  # 发布到达第一个位置的状态
            self.check_start = False

        # 判断是否接近停止点
        if not dist_stop is None and dist_stop < 0.1 and not self.check_stop:
            self.pub_status.publish("now-stop")  # 发布到达停止点的状态
            self.check_stop = False

        # 如果导航开始，调用状态回调函数
        if self.run:
            self.statusCB()

    # 计算两个点之间的欧几里得距离
    def distance(self, kx, ky, gx, gy):
        try:
            return math.sqrt((kx - gx) ** 2 + (ky - gy) ** 2)  # 计算两点之间的直线距离
        except:
            return None  # 如果计算失败，返回None

    # 更改目标点的状态，并发布下一个目标点
    def change_status(self, goalList):
        # 提取目标点的各个坐标和四元数信息
        self.goalListX = goalList[:, 0]
        self.goalListY = goalList[:, 1]
        self.goalListZ = goalList[:, 2]
        self.goalListQX = goalList[:, 3]
        self.goalListQY = goalList[:, 4]
        self.goalListQZ = goalList[:, 5]
        self.goalListQW = goalList[:, 6]

        self.init_data()  # 初始化数据
        self.run = True  # 开始导航

        # 发布第一个目标点
        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalListX[self.goalId]
        self.goalMsg.pose.position.y = self.goalListY[self.goalId]
        self.goalMsg.pose.position.z = self.goalListZ[self.goalId]
        self.goalMsg.pose.orientation.x = self.goalListQX[self.goalId]
        self.goalMsg.pose.orientation.y = self.goalListQY[self.goalId]
        self.goalMsg.pose.orientation.z = self.goalListQZ[self.goalId]
        self.goalMsg.pose.orientation.w = self.goalListQW[self.goalId]
        self.pub.publish(self.goalMsg)  # 发布目标点消息

        rospy.loginfo("初始目标点发布！目标ID为: %d", self.goalId)

    # 计算下一个目标点的相对位置，基于当前的位置和传感器信息
    def calc_next_point(self, cone_points, angle_q=None):
        temp = cone_points.copy()  # 复制位置点坐标

        # 调整每个点的坐标，使其相对于当前位置
        for i in range(np.size(temp, 0)):
            temp[i][0] += self.kx  # 加上当前位置的X坐标
            temp[i][1] += self.ky  # 加上当前位置的Y坐标

            # 如果给定了四元数，调整姿态信息
            if not angle_q is None:
                temp[i][3] += angle_q[0]
                temp[i][4] += angle_q[1]
                temp[i][5] += angle_q[2]
                temp[i][6] += angle_q[3]

        return temp  # 返回调整后的目标点

    # 位置2任务完成后的回调函数
    def finish_cone2(self):
        time.sleep(self.finish_run_time)  # 延迟
        self.pub_status.publish("finish-cone2")  # 发布完成位置2任务的状态

    # 位置3任务完成后的回调函数
    def finish_cone3(self):
        time.sleep(self.finish_run_time)  # 延迟
        self.pub_status.publish("finish-cone3")  # 发布完成位置3任务的状态

    # 拐角任务完成后的回调函数
    def finish_corner(self):
        time.sleep(self.finish_run_time)  # 延迟
        self.pub_status.publish("finish-corner")  # 发布完成拐角任务的状态

    # 最终任务完成后的回调函数
    def finish_last(self):
        time.sleep(self.finish_run_time)  # 延迟
        self.pub_status.publish("finish-last")  # 发布完成最终任务的状态

    # 接收到控制标志后的处理逻辑
    def on_control_flag(self, data):
        print(data.data)

        # 根据收到的指令，切换至不同的导航任务
        if data.data == "cone2":
            self.change_status(self.calc_next_point(self.cone2_back_point))  # 切换到位置2任务
            self.pub_status.publish("start-cone2")
            self.run_finish_lambda = self.finish_cone2  # 任务完成后的回调设置为finish_cone2

        if data.data == "cone3":
            self.change_status(self.calc_next_point(self.cone3_back_point))  # 切换到位置3任务
            self.pub_status.publish("start-cone3")
            self.run_finish_lambda = self.finish_cone3  # 任务完成后的回调设置为finish_cone3

        if data.data == "corner":
            self.change_status(self.calc_next_point(self.corner_point))  # 切换到拐角任务
            self.pub_status.publish("start-corner")
            self.run_finish_lambda = self.finish_corner  # 任务完成后的回调设置为finish_corner

        if data.data == "last":
            self.change_status(self.calc_next_point(self.last_point))  # 切换到最终任务
            self.pub_status.publish("start-last")
            self.run_finish_lambda = self.finish_last  # 任务完成后的回调设置为finish_last

    # 上行命令处理函数，处理重新加载等操作
    def on_up_command(self, data):
        print("上行命令", data)
        self.up_command = data.data
        if data.data == "reload":  # 如果收到reload命令，重新加载所有数据
            self.load_all_data()

    # 处理接收到的物体角度信息
    def on_object_angle(self, data):
        data = data.data
        # 处理物体角度信息，具体实现未完善
        # self.now_object_angle = float(data)

if __name__ == "__main__":
    try:
        # ROS节点初始化
        rospy.init_node("multi_goals", anonymous=True)
        map_frame = rospy.get_param("frame", "odom")  # 获取坐标系参数

        rospy.loginfo("执行多目标导航...")  # 打印导航开始日志
        mg = MultiGoals(map_frame)  # 创建MultiGoals对象并执行导航
        rospy.spin()  # 保持ROS节点运行

    except KeyboardInterrupt:
        print("程序关闭")  # 捕获键盘中断，关闭程序
