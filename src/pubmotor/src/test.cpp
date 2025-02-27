#include "ros/ros.h"
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <boost/thread.hpp>
#include "pubmotor/motor.h"

using namespace std;
uint8 data1[8];

void chatterCallback(const can_msgs::Frame::ConstPtr &msg )//const can_msgs::Frame(数据的类型)::ConstPtr &msg
{
    
    for (int i = 0; i < msg->dlc; i++)
    {
        printf("%X",msg->data[i]);
        data1[i] = msg->data[i];
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_sub");//冒号里面写的是你想要取的结点的名字，名字不能重复
    ros::NodeHandle nh;//ros的句柄
    ros::Subscriber sub = nh.subscribe("/received_messages", 1, chatterCallback);//参数说明：话题名称   订阅的信息长度 回调函数
    ros::Publisher pub = nh.advertise<can_msgs::Frame>("/sent_messages",10);//参数说明： <数据类型>+（话题名称，队列长度）
/***
 * 
 * std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 id
bool is_rtr
bool is_extended
bool is_error
uint8 dlc
uint8[8] data

 * 
*/
    while (ros::ok())
    {
        can_msgs::Frame msg;
        msg.header.seq = 1;
        msg.header.stamp = 100000;
        msg.header.frame_id = "123";
        msg.id = 0;
        msg.dlc = 8;
        msg.data = data1;
        pub.publish(msg);//固定格式：发布者的名字.publish(自己定义的要发送的数据变量的名字)；

        ros::spinOnce();//spin的作用是重复进入回调函数，spinOnce的作用是还能够运行接下来的代码
        cout<<1<<endl;   
        
    }

    return;
}
