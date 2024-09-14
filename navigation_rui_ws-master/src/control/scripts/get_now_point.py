#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy  # 导入ROS（机器人操作系统）Python库
import math  # 数学库（当前未使用）
import sys, select, termios, tty  # 系统输入输出相关库
import tf  # ROS中的坐标变换库（当前未使用）
import csv  # 用于将数据写入CSV文件
import codecs  # 处理文件编码的库
from nav_msgs.msg import Odometry  # 导入ROS中的里程计消息类型

# 定义全局变量，用于存储当前的里程计数据点
now_point = []

# 全局变量，用于计数和存储记录的点
count = 0
points = []

# 里程计数据回调函数，处理接收到的里程计消息
def odometryCb(msg):
    x = msg.pose.pose.position.x  # 获取位置x
    y = msg.pose.pose.position.y  # 获取位置y
    z = msg.pose.pose.position.z  # 获取位置z
    qx = msg.pose.pose.orientation.x  # 获取四元数的x
    qy = msg.pose.pose.orientation.y  # 获取四元数的y
    qz = msg.pose.pose.orientation.z  # 获取四元数的z
    qw = msg.pose.pose.orientation.w  # 获取四元数的w

    global now_point
    now_point = [x, y, z, qx, qy, qz, qw]  # 保存当前的位置信息和姿态信息

# 键盘按键获取函数
def getKey():
    tty.setraw(sys.stdin.fileno())  # 将终端设置为原始模式，用于捕获按键
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 等待键盘输入
    if rlist:
        key = sys.stdin.read(1)  # 读取按下的按键
    else:
        key = ""  # 未按键
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 恢复终端设置
    return key  # 返回按下的键

# 将数据写入CSV文件的函数
def data_write_csv(file_name, datas):
    file_csv = codecs.open(file_name, "w+", "utf-8")  # 以utf-8编码打开CSV文件
    writer = csv.writer(file_csv, delimiter=",", quoting=csv.QUOTE_MINIMAL)  # 初始化CSV写入器
    for data in datas:
        writer.writerow(data)  # 将数据写入CSV文件
        print("写入成功！")  # 提示用户写入成功

# 主程序入口
if __name__ == "__main__":
    try:
        rospy.init_node("getPoint", anonymous=True)  # 初始化ROS节点
        rospy.Subscriber("/odometry/filtered_map", Odometry, odometryCb, queue_size=10)  # 订阅里程计话题
        settings = termios.tcgetattr(sys.stdin)  # 保存终端设置，方便后续恢复
        print("初始化完成")  # 提示用户初始化完成
        while True:
            Key = getKey()  # 等待按键输入
            if Key == "f" or Key == "F":  # 如果按下“f”键，记录当前点
                points.append(now_point)  # 将当前点添加到列表中
                data_write_csv("now_points.csv", points)  # 将点保存到CSV文件
            if Key == "\x03":  # 如果按下Ctrl+C，退出循环
                break
        rospy.spin()  # 保持ROS节点运行
    except rospy.ROSInterruptException:
        pass  # 处理ROS中断异常
