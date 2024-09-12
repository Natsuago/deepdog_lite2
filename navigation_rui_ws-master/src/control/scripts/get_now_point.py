#!/usr/bin/python
# -*- coding: utf-8 -*

import rospy
import math
import sys, select, termios, tty
import tf
import csv
import codecs
from nav_msgs.msg import Odometry

now_point = []

count = 0
points = []

def odometryCb(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    global now_point
    now_point = [x, y, z, qx, qy, qz, qw]


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def data_write_csv(file_name, datas):
    file_csv = codecs.open(file_name, "w+", "utf-8")
    writer = csv.writer(file_csv, delimiter=",", quoting=csv.QUOTE_MINIMAL)
    for data in datas:
        writer.writerow(data)
        print("write succ!!")


if __name__ == "__main__":
    try:
        rospy.init_node("getPoint", anonymous=True)
        rospy.Subscriber("/odometry/filtered_map", Odometry, odometryCb, queue_size=10)
        settings = termios.tcgetattr(sys.stdin)
        print("init finish")
        while True:
            Key = getKey()
            if Key == "f" or Key == "F":
                points.append(now_point)
                data_write_csv("now_points.csv", points)
            if Key == "\x03":
                break
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
