#!/usr/bin/env python3
# -*- coding=utf-8 -*-
# import rospy
# import socket
# import struct
# from geometry_msgs.msg import Twist

# # 配置 TCP 客户端连接外部设备
# HOST = '192.168.1.22'  # 外部设备的 IP 地址
# PORT = 5000            # 外部设备的端口号

# # 创建 TCP 套接字
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# def callback(msg):
#     """
#     处理接收到的 Twist 消息并通过 TCP 发送给外部设备
#     """
#     # 从 Twist 消息中获取线速度和角速度
#     linear_x = msg.linear.x
#     angular_z = msg.angular.z
#     angular_R = msg.angular.x
#     rear_vice_arm = msg.liner.y
#     rear_prime_arm = msg.liner.z

#     # 打包消息数据为二进制格式（这里使用 struct 来打包数据）
#     # 假设我们发送两个浮动数字：线速度和角速度
#     data = struct.pack('fffff', linear_x, angular_z,angular_R, rear_vice_arm,rear_prime_arm)  # 格式：5个浮动类型（float）

#     # 发送数据到外部设备
#     try:
#         sock.sendall(data)
#         rospy.loginfo("Sent data: Linear Velocity = %f, VICE ARM = %f, PRIME ARM = %f, Angular Velocity = %f, Angular Radius = %f", 
#                       linear_x, rear_vice_arm, rear_prime_arm, angular_z, angular_R)
#     except Exception as e:
#         rospy.logerr("Failed to send data: %s", str(e))

# def listener():
#     """
#     初始化 ROS 节点并订阅 cmd_vel 话题
#     """
#     # 初始化 ROS 节点
#     rospy.init_node('tcp_twist_sender', anonymous=True)

#     # 订阅 'cmd_vel' 话题，接收 Twist 消息
#     rospy.Subscriber('synergy_vel', Twist, callback)

#     # 保持节点运行
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         # 连接到外部设备
#         sock.connect((HOST, PORT))
#         listener()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         # 关闭 TCP 套接字
#         sock.close()



import rospy
import socket
import struct
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# 配置 TCP 连接参数
HOST = '192.168.1.22'  # 目标设备IP
PORT = 5000            # 目标设备端口
RECONNECT_INTERVAL = 5  # 重试间隔(秒)

class TCPConnector:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.reconnect_flag = threading.Event()
        self.receive_thread = None
        self.subscriber = None

        # 初始化ROS节点
        rospy.init_node('tcp_twist_sender', anonymous=True)
        rospy.loginfo("节点初始化完成")

    def connect(self):
        """建立TCP连接并保持重连"""
        while not rospy.is_shutdown():
            try:
                # 创建新socket实例
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(1.0)
                
                rospy.loginfo("尝试连接 %s:%d...", HOST, PORT)
                self.sock.connect((HOST, PORT))
                self.connected = True
                rospy.loginfo("连接成功")

                # 启动接收线程
                self.receive_thread = threading.Thread(target=self.receive_thread_func)
                self.receive_thread.daemon = True
                self.receive_thread.start()

                # 创建订阅者（只在连接成功后）
                self.subscriber = rospy.Subscriber('synergy_vel', Twist, self.callback)
                self.touch_wheel_pub = rospy.Publisher('rear_arm_touch_wheel', Bool, queue_size=1)
                self.arm_closed_pub = rospy.Publisher('Rear_Prime_ARM_CLOSED', Bool, queue_size=1)

                # 等待连接中断信号
                self.reconnect_flag.wait()
                self.reconnect_flag.clear()

            except (ConnectionRefusedError, socket.timeout) as e:
                rospy.logwarn("连接失败: %s，%d秒后重试...", str(e), RECONNECT_INTERVAL)
            except Exception as e:
                rospy.logerr("意外错误: %s", str(e))
            
            # 清理资源
            self.cleanup()
            if not rospy.is_shutdown():
                rospy.sleep(RECONNECT_INTERVAL)

    # def receive_thread_func(self):
    #     """接收数据线程"""
    #     try:
    #         while self.connected and not rospy.is_shutdown():
    #             try:
    #                 data = self.sock.recv(1024)
    #                 if not data:
    #                     rospy.logwarn("收到空数据，连接可能已中断")
    #                     break
                    
    #                 # 布尔值解析逻辑
    #                 if len(data) >= 1:
    #                     received_bool = data[0] != 0x00
    #                     rospy.loginfo("收到状态: %s", received_bool)
    #                     self.touch_wheel_pub.publish(received_bool)
    #                 else:
    #                     rospy.logwarn("异常数据长度: %d", len(data))
                        
    #             except socket.timeout:
    #                 continue
    #             except (ConnectionResetError, BrokenPipeError) as e:
    #                 rospy.logwarn("连接中断: %s", str(e))
    #                 break
    #             except OSError as e:  # 捕获文件描述符异常
    #                 if e.errno == 9:
    #                     rospy.logwarn("套接字已关闭，停止接收线程")
    #                     break
    #     finally:
    #         self.reconnect_flag.set()
    def receive_thread_func(self):
        """支持消息类型标识和粘包处理的接收线程"""
        recv_buffer = bytes()  # 新增接收缓冲区[1,6](@ref)
        try:
            while self.connected and not rospy.is_shutdown():
                try:         
                    data = self.sock.recv(1024)
                    # 分批次读取并填充缓冲区
                    recv_buffer += data  # 追加到缓冲区
                    
                    # 粘包处理循环（关键修改）
                    while len(recv_buffer) >= 2:  # 确保能解析完整消息[7](@ref)
                        # 提取并解析单个消息
                        msg_bytes, recv_buffer = recv_buffer[:2], recv_buffer[2:]
                        msg_type, value = struct.unpack('B?', msg_bytes)
                        
                        # 根据消息类型分发数据[1](@ref)
                        if msg_type == 0x01:
                            rospy.loginfo("收到触摸轮状态: %s", value)
                            self.touch_wheel_pub.publish(value)
                        elif msg_type == 0x02:
                            rospy.loginfo("收到主机械臂闭合状态: %s", value) 
                            self.arm_closed_pub.publish(value)
                        else:
                            rospy.logwarn("未知消息类型: 0x%02X", msg_type)
                            
                except socket.timeout:
                    continue
                except (ConnectionResetError, BrokenPipeError) as e:
                    rospy.logwarn("连接中断: %s", str(e))
                    break
                except struct.error as e:  # 新增结构体解析异常捕获[7](@ref)
                    rospy.logerr("数据解析失败: %s", str(e))
                    recv_buffer = bytes()  # 清空异常缓冲区
                except OSError as e:
                    if e.errno == 9:
                        rospy.logwarn("套接字已关闭，停止接收线程")
                        break
                        # if len(data) == 2:
                    #     msg_type, value = struct.unpack('B?', data)
                    #     if msg_type == 0x01:
                    #         print(f"触摸轮状态: {value}")
                    #     elif msg_type == 0x02:
                    #         print(f"主机械臂闭合状态: {value}")
                    # else:
                    #     continue
        #             if not data:
        #                 rospy.logwarn("收到空数据，连接可能已中断")
        #                 break
        #             if len(data) != 2:
        #                 continue
        #             msg_type, value = struct.unpack('B?', data)
        #             if msg_type == 0x01:
        #                 rospy.loginfo("收到触摸轮状态: %s", value)
        #                 self.touch_wheel_pub.publish(value)
        #             elif msg_type == 0x02:
        #                 rospy.loginfo("收到主机械臂闭合状态: %s", value) 
        #                 self.arm_closed_pub.publish(value)
        #             else:
        #                 rospy.logwarn("未知消息类型: 0x%02X", msg_type)
        #         except socket.timeout:
        #             continue
        #         except (ConnectionResetError, BrokenPipeError) as e:
        #             rospy.logwarn("连接中断: %s", str(e))
        #             break
        #         except OSError as e:  # 捕获文件描述符异常
        #             if e.errno == 9:
        #                 rospy.logwarn("套接字已关闭，停止接收线程")
        #                 break

        finally:
            self.reconnect_flag.set()

    def callback(self, msg):
        """速度指令回调"""
        if not self.connected:
            return

        try:
            # 修正后的数据打包格式 (6个float)
            data = struct.pack('ffffff',
                             msg.linear.x,    # X线速度
                             msg.angular.z,   # Z角速度
                             msg.angular.x,   # 转向半径R
                             msg.linear.y,    # 副机械臂位置
                             msg.linear.z,    # 主机械臂位置
                             msg.angular.y)   # IO状态
            
            self.sock.sendall(data)
            rospy.loginfo("已发送 | X:%.2f YAW:%.4f R:%.2f 副臂:%.2f 主臂:%.2f IO:%d",
                        msg.linear.x,
                        msg.angular.z,
                        msg.angular.x,
                        msg.linear.y,
                        msg.linear.z,
                        int(msg.angular.y))
        except Exception as e:
            rospy.logerr("发送失败: %s", str(e))
            self.reconnect_flag.set()

    def cleanup(self):
        """资源清理"""
        self.connected = False
        
        # 先取消订阅
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None

        # 关闭socket
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception as e:
                rospy.logwarn("套接字关闭异常: %s", str(e))
            finally:
                self.sock = None

        rospy.loginfo("连接资源已清理")

if __name__ == '__main__':
    connector = TCPConnector()
    try:
        connector.connect()
    except rospy.ROSInterruptException:
        connector.cleanup()
        rospy.loginfo("节点关闭")