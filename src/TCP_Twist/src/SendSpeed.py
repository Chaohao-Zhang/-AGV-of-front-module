#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import socket
import struct
from geometry_msgs.msg import Twist

# 配置 TCP 客户端连接外部设备
HOST = '192.168.1.22'  # 外部设备的 IP 地址
PORT = 5000            # 外部设备的端口号

# 创建 TCP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def callback(msg):
    """
    处理接收到的 Twist 消息并通过 TCP 发送给外部设备
    """
    # 从 Twist 消息中获取线速度和角速度
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    angular_R = msg.angular.x

    # 打包消息数据为二进制格式（这里使用 struct 来打包数据）
    # 假设我们发送两个浮动数字：线速度和角速度
    data = struct.pack('fff', linear_x, angular_z,angular_R)  # 格式：两个浮动类型（float）

    # 发送数据到外部设备
    try:
        sock.sendall(data)
        rospy.loginfo("Sent data: Linear Velocity = %f, Angular Velocity = %f, Angular Radius = %f", linear_x, angular_z, angular_R)
    except Exception as e:
        rospy.logerr("Failed to send data: %s", str(e))

def listener():
    """
    初始化 ROS 节点并订阅 cmd_vel 话题
    """
    # 初始化 ROS 节点
    rospy.init_node('tcp_twist_sender', anonymous=True)

    # 订阅 'cmd_vel' 话题，接收 Twist 消息
    rospy.Subscriber('synergy_vel', Twist, callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        # 连接到外部设备
        sock.connect((HOST, PORT))
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 关闭 TCP 套接字
        sock.close()
