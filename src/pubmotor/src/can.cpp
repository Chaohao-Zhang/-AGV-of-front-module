#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "std_msgs/String.h" 
#include "pubmotor/motor.h"
#include <time.h>
#include <omp.h>

void canCallBack(const can_msgs::Frame::ConstPtr &msg)
{
    static int times = 0;
	std::cout << times++ << "ID: " << msg->id << " data: " ;    
    for (int i = 0; i < msg->dlc; i++)
    {
        printf("%X ", msg->data[i]);
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);

    can_msgs::Frame can_frame_msg;
    can_frame_msg.id =  0x521;
    can_frame_msg.dlc = 8;
    // 分开赋值
    can_frame_msg.data[0] = 0x00;
    can_frame_msg.data[1] = 0x11;
    can_frame_msg.data[2] = 0x22;
    can_frame_msg.data[3] = 0x33;
    can_frame_msg.data[4] = 0x44;
    can_frame_msg.data[5] = 0x55;
    can_frame_msg.data[6] = 0x66;
    can_frame_msg.data[7] = 0x77;
    // can_frame_msg.data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};   

    // ros::Subscriber sub = nh.subscribe("/received_messages", 100, &canCallBack);


    ros::Rate r(1);
    while (ros::ok())
    {
        // pub.publish(can_frame_msg);
        // ROS_INFO("send can message");
        ros::Time before_time_ = ros::Time::now();
        r.sleep();
        ros::spinOnce();
        // ROS_INFO("recive can message");
       
        ros::Time now = ros::Time::now();  
        // std::cout << "Elapsed time: " << (now - before_time_).toSec() << " seconds" << std::endl;

    }

    return 0;
}

