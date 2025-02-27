#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include "pubmotor/motor.h"
#include <sstream>

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<pubmotor::motor>("motor_message",10);
    pubmotor::motor motor_msg;
    motor_msg.id = "motor_001";
    motor_msg.velocity = 0.5;
    motor_msg.acceleration = 0.5;
   
    int count = 0; //消息计数器

    //逻辑(一秒1次)
    ros::Rate rate(1);

    //节点不死
    while (ros::ok())
    {
        //键盘实时输入话题指令
        // std::cout << "请输入话题指令" << std::endl;
        // std::cin >> motor_msg.id;
        // std::cout << "请输入速度指令" << std::endl;
        // std::cin >> motor_msg.velocity;
        // std::cout << "请输入加速度指令" << std::endl;
        // std::cin >> motor_msg.acceleration;  
        // motor_msg.id = "motor_001";
       
        //发布消息
        pub.publish(motor_msg);
        //加入调试，打印发送的消息
        ROS_INFO("Talker: motor_msg.id:%s, motor_msg.velocity:%f, motor_msg.acceleration:%f",motor_msg.id.c_str(),motor_msg.velocity,motor_msg.acceleration);

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        rate.sleep();
        //暂无应用
        ros::spinOnce();
    }


    return 0;
}

