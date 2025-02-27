#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include "pubmotor/motor.h"
#include <sstream>

void doPerson(const pubmotor::motor::ConstPtr &motor_msg){
    ROS_INFO("订阅的人信息1:%s, %f %f", motor_msg->id.c_str(), motor_msg->position, motor_msg->velocity);
}
void doPerson1(const pubmotor::motor::ConstPtr &motor_msg){
    ROS_INFO("订阅的人信息2:%s, %f %f", motor_msg->id.c_str(), motor_msg->position, motor_msg->velocity);
}
int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"sub");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pubmotor::motor>("motor_message",10,doPerson);
    ros::Subscriber sub1 = nh.subscribe<pubmotor::motor>("motor_message",10,doPerson1);
    while (ros::ok())
    {
    //1.初始化 ROS 节点
    //3.创建订阅对象
    //4.回调函数中处理 person

    // ros::spin();
    ros::spinOnce();
    ROS_INFO("xunhuan");
    }
    return 0;
    
}
