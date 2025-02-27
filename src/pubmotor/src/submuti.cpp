#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <can_msgs/Frame.h>
#include <serial/serial.h>
#include<geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>
#include "pubmotor/motor.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <signal.h>

#define MOTOR_NUM 4

#define sBUFFERSIZE 100 // send buffer size串口发送缓存长度
#define rBUFFERSIZE 100 // receive buffer size 串口接收缓存长度
#define CLOSED 0
#define OPENED 1
#define CLOSE_ARRIVALED 2
#define OPEN_ARRIVALED 1
#define PI 3.14159
#define MOVING 0
#define Wheel_2_C_DIS 0.37
#define READ_IO false
#define WRITE_IO true
#define NOT_TOUCH false
#define TOUCH true
#define liner_vel 0.2
#define Wheel_L_2_R_DIS 0.766

using namespace std;
unsigned char s_buffer[sBUFFERSIZE]; //发送缓存
unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

typedef enum IO_STATE_ITEMS{
	READ,
	WRITE,
}IO_STATE_ITEMS;

vector<unsigned char>  ser_shell;
vector<unsigned char>  ser_RIO_shell = {0x01, 0x02, 0x00, 0x00, 0x00, 0x08, 0x79, 0xCC};

uint8_t Left_Is_Arrival = 0;
uint8_t Right_Is_Arrival = 0;
bool is_touch = NOT_TOUCH;
bool is_synergy = false;
IO_STATE_ITEMS IO_STATE = WRITE;
serial::Serial ser;

unsigned char save_data[8] = {0,0,0,0, 0, 0, 0, 0};

struct Quaternion {
    double w, x, y, z;
};
 
struct EulerAngles {
    double roll, pitch, yaw;
};
 
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}

class multiThreadListener
{
public:
	multiThreadListener()
	{
   //  pub = n.advertise<pubmotor::motor>("motor_message",10);
   //  sub = n.subscribe("motor_message", 1, &multiThreadListener::chatterCallback1,this);
    can1 = n.subscribe("received_messages", 1, &multiThreadListener::chatterCallback3,this);
    sub_agv_vel = n.subscribe("agv_vel", 1, &multiThreadListener::AgvVelotryCallback, this);
    sub_agv_nav_vel = n.subscribe("cmd_vel", 1, &multiThreadListener::NavVelotryCallback, this);
    io_state = n.subscribe("Ser_IO_state", 1, &multiThreadListener::SerIOCallback, this);
   //  sub_agv_posiiton = n.subscribe("Odometry", 1, &multiThreadListener::AgvPositionCallback, this);
    pubcan = n.advertise<can_msgs::Frame>("sent_messages", 100);
    arm_wheel_can = n.advertise<std_msgs::Bool>("arm_touch_wheel", 10);
    pub_syn = n.advertise<geometry_msgs::Twist>("synergy_vel",1);;
	}
  void chatterCallback1(const pubmotor::motor::ConstPtr& msg);
  void chatterCallback3(const can_msgs::Frame::ConstPtr &msg);
  void AgvVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void NavVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void SerIOCallback(const std_msgs::Bool::ConstPtr& msg);
  // void publish_data(string id, float velocity,float acceleration);
  void publish_can(vector<unsigned char>& data, uint8_t Len);
  void publish_arm_wheel(bool state);

  void CAN_msg_classify(const unsigned char  *data, uint32_t motor_id, uint8_t Len);
  vector<unsigned char >send_data={0x12,0x34,0x00,0x00,0x00,0x00,0x00,0x56};
  void PDO_deal_info(const int slave_id,const int pdo_id, unsigned char *pdo_data);
  void Motor_Info(int motor_id);
  void NMT_Rec_Func(const int slave_id, unsigned char *pdo_data);
  void ERROR_CODE(const int slave_id, unsigned char *err_data);
  bool SDO_rec_FUNC(const int slave_id, const unsigned int mode, unsigned char *sdo_data, unsigned char *save_data);

  void PDO_Send_func(const int slave_id,const int pdo_id,const vector<unsigned char>& pdo_data,const unsigned int dlen);
  void NMT_ShellFunc(const int slave_id,unsigned char shell);

private:
   typedef struct{
      int Actual_Pos = 0;
      float Actual_Val = 0;
      int Status_word = 0;
      unsigned int Inputs = 0;
      unsigned int Status = 0;
      unsigned char Mode = 0;
   }Motor_INFO;
   Motor_INFO testMotor_info[MOTOR_NUM];
  int slave_id;
  unsigned int w_or_r = 0;/*SDO：1为写入数据，2为读取数据*/
  ros::NodeHandle n;
//   ros::Subscriber sub;
//   ros::Subscriber sub2;
  ros::Subscriber can1;
  ros::Subscriber sub_agv_vel;
  ros::Subscriber sub_agv_nav_vel;
  ros::Subscriber io_state;
//   ros::Subscriber sub_agv_posiiton;
//   ros::Publisher pub;
  ros::Publisher pubcan;
  ros::Publisher arm_wheel_can;
  ros::Publisher pub_syn;
};

void multiThreadListener::Motor_Info(int motor_id){
  cout<<"Motor"<<motor_id<<" the speed is: "<<testMotor_info[motor_id-1].Actual_Val<<", Motor"<<motor_id<<" the position is: "<<testMotor_info[motor_id-1].Actual_Pos<<endl;
}

/*PDO发送函数*/
void multiThreadListener::PDO_Send_func(const int slave_id,const int pdo_id,const vector<unsigned char>& pdo_data,const unsigned int dlen){
   // snd_thread_arg_t *thread_arg = (snd_thread_arg_t *)param;
   can_msgs::Frame can_frame_msg; 
   int ret = 0;
   unsigned char data[8];
   memcpy(data, pdo_data.data(), 8);

   /*确定COB_ID*/
   switch (pdo_id)
   {
   case 1:
   {
      can_frame_msg.id = 0x200 + slave_id; 
      break;
   }
   case 2:
   {
      can_frame_msg.id  = 0x300 + slave_id; 
      break;
   }
   case 3:
   {
      can_frame_msg.id = 0x400 + slave_id; 
      break;
   }
   case 4:
   {
      can_frame_msg.id  = 0x500 + slave_id; 
      break;
   }
   default:
      break;
   }

   /*一些CAN的通信配置*/
   can_frame_msg.dlc = dlen;  // Length of data
   can_frame_msg.is_rtr = false;  // false-数据帧；ture-远程帧
   can_frame_msg.is_extended = false;  // fasle-标准帧；ture-扩展帧
   can_frame_msg.is_error = false;		//false-非错误帧；ture-错误帧

   for ( int i = 0; i <dlen; i++ ) 
   {
      can_frame_msg.data[i] = pdo_data[i];
      // printf("%02X  ", pdo_data[i]);
   }
   // ROS_INFO("is sending can message");
  pubcan.publish( can_frame_msg);

}

void multiThreadListener::NMT_ShellFunc(const int slave_id,unsigned char shell){
   can_msgs::Frame can_frame_msg; 

   can_frame_msg.id = 0x0;
   can_frame_msg.dlc = 2;  // Length of data
   can_frame_msg.is_rtr = false;  // false-数据帧；ture-远程帧
   can_frame_msg.is_extended = false;  // fasle-标准帧；ture-扩展帧
   can_frame_msg.is_error = false;		//false-非错误帧；ture-错误帧

   can_frame_msg.data[0] = shell;
   can_frame_msg.data[1] = slave_id;

   pubcan.publish( can_frame_msg);

}
/*以下为根CAN接受有关的函数*/
void multiThreadListener::PDO_deal_info(const int slave_id,const int pdo_id, unsigned char *pdo_data)
{
   int ret;
   // cout<<"receive can data is motor "<<slave_id<<"POD:"<<pdo_id<<"message:";
   // for(int i = 0;i<8;i++){
   //  printf("%02X  ",pdo_data[i]);
   // }
   // printf("\r\n");
   switch (pdo_id)
   {
      /*TPDO1 接受为状态字*/
      case 1:
      {
         testMotor_info[slave_id-1].Status_word = pdo_data[0]|(pdo_data[1]<<8);
         break;
      }
      /*TPDO2 接受为状态字、实际位置、控制模式*/
      case 2:
      {
         testMotor_info[slave_id-1].Status_word = pdo_data[0]|(pdo_data[1]<<8);
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[2]|(pdo_data[3]<<8)|(pdo_data[4]<<16)|(pdo_data[5]<<24);
         testMotor_info[slave_id-1].Mode = pdo_data[6];
         break;
      }
      /*TPDO3 接受为实际位置和速度*/
      case 3:
      {
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[0]|(pdo_data[1]<<8)|(pdo_data[2]<<16)|(pdo_data[3]<<24);
         testMotor_info[slave_id-1].Actual_Val = pdo_data[4]|(pdo_data[5]<<8)|(pdo_data[6]<<16)|(pdo_data[7]<<24);
      }
      /*TPDO1 接受为实际位置和输入信号*/
      case 4:
      {
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[0]|(pdo_data[1]<<8)|(pdo_data[2]<<16)|(pdo_data[3]<<24);
         testMotor_info[slave_id-1].Inputs = pdo_data[4]|(pdo_data[5]<<8)|(pdo_data[6]<<16)|(pdo_data[7]<<24);
         break;
      }
      default:
      break;
   };
   
/*to check the active arm is arriving while opening*/
   if(testMotor_info[3].Actual_Pos <= -5000000){
      Left_Is_Arrival = OPEN_ARRIVALED;
   }else if(testMotor_info[3].Actual_Pos >= -0){
      Left_Is_Arrival = CLOSE_ARRIVALED;
   }else {
      Left_Is_Arrival = MOVING;
   }
   if(testMotor_info[2].Actual_Pos <= -5000000){
      Right_Is_Arrival = OPEN_ARRIVALED;
   }else if(testMotor_info[2].Actual_Pos >= -0){
      Right_Is_Arrival = CLOSE_ARRIVALED;
   }else {
      Right_Is_Arrival = MOVING;
   }
}
/*NMT报文处理，读取到的数据存入电机状态*/

void multiThreadListener::NMT_Rec_Func(const int slave_id, unsigned char *pdo_data){
   testMotor_info[slave_id-1].Status = pdo_data[0];
   // cout<<"Motor status is:"<<testMotor_info[slave_id-1].Status<<endl;
}
// /*如果为错误代码进行显示*/
void multiThreadListener::ERROR_CODE(const int slave_id, unsigned char *err_data){
   int errcode = 0;
   errcode = err_data[0] | (err_data[1]<<8);
   printf("the Drive :%04x is WRONGING, the ERROR CODE is: %04X!!!" ,slave_id, errcode);
}

bool multiThreadListener::SDO_rec_FUNC(const int slave_id, const unsigned int mode, unsigned char *sdo_data, unsigned char *save_data)
{
   unsigned int address = 0;
   unsigned int chlid_add = 0;
   unsigned int response = 0;
   int data = 0;
   unsigned int ret = 0;

   address = sdo_data[1]|(sdo_data[2]<<8);
   chlid_add = sdo_data[3];
   switch (mode)
      {
         /*当写入数据时，返回函数中如果为0x60的代表写入成功，如果为0x80代表写入失败*/
         case 1:{
            if(sdo_data[0] == 0x60){
               printf("Write data to %04x  %02x True!!\r\n",address , chlid_add);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x80){
               printf("Write data to %04x  %02x FALSE!!!\r\n",address , chlid_add);
               ret = false;
               goto end;
            }
            break;
         }
         case 2:{
            data = sdo_data[4]|(sdo_data[5]<<8)|(sdo_data[6]<<16)|(sdo_data[7]<<24);
            if(sdo_data[0] == 0x4f){
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               memcpy(save_data,sdo_data, 1);
               goto end;
            }else if(sdo_data[0] == 0x4b){
               memcpy(save_data,sdo_data, 2);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x47){
               memcpy(save_data,sdo_data, 3);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x43){
               memcpy(save_data,sdo_data,4);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x80){
               printf("Read data from %04x  %02x FALSE!!!\r\n",address , chlid_add);
               ret =false;
               goto end;
            }
            break;
         }
         default:
         break;
      }
end:
   return ret;
}

unsigned char buff[8] = {0,0,0,0,0,0,0,0};
void multiThreadListener::CAN_msg_classify(const unsigned char  *data, uint32_t can_id, uint8_t Len){
  int COB_id = 0;
  memcpy(buff, data, Len);// 将读取到的数据存入bull中
  slave_id = can_id&0x7f;//解析相应的从站ID，根据ID进行不同的处理
  COB_id = can_id>>7;//解析出COB_ID，方便后续的处理
  COB_id = COB_id<<7;
  switch (COB_id)//根据不同的COB_ID，判断所属的报文类型
  {
    /*ERROR message*/
    case 0x080:
    {
        ERROR_CODE(slave_id, buff);
        break;
    }
    /*PDO1*/
    case 0x180:
    {
        PDO_deal_info(slave_id, 1, buff);
        break;
    }
    /*PDO2*/
    case 0x280:
    {
        PDO_deal_info(slave_id, 2, buff);
        break;
    }
    /*PDO3*/
    case 0x380:
    {
        PDO_deal_info(slave_id, 3, buff);
        break;
    }
    /*PDO4*/
    case 0x480:
    {
        PDO_deal_info(slave_id, 4, buff); 
        break;
    }
    /* SDO receive message*/
    case 0x580:
    {
        SDO_rec_FUNC(slave_id, w_or_r, buff, save_data);
        break;
    }
    /*NMT reflect drive message*/
    case 0x700:
    {
        NMT_Rec_Func(slave_id, buff);
        break;
    }
    default:
    break;
  }
}


// void multiThreadListener::publish_data(string id, float velocity,float acceleration){
//   pubmotor::motor motor_msg;
//   motor_msg.id = id;
//   motor_msg.velocity = velocity;
//   motor_msg.acceleration = acceleration;
//   for(int i=0;i++;i<10){pub.publish(motor_msg);};
//   };
void multiThreadListener:: publish_can( vector<unsigned char>& data, uint8_t Len)
{
   can_msgs::Frame can_frame_msg;
   can_frame_msg.dlc = 8;
   // 分开赋值
   can_frame_msg.data[0] = data[0];
   can_frame_msg.data[1] = data[1];
   can_frame_msg.data[2] = data[2];
   can_frame_msg.data[3] = data[3];
   can_frame_msg.data[4] = data[4];
   can_frame_msg.data[5] = data[5];
   can_frame_msg.data[6] = data[6];
   can_frame_msg.data[7] = data[7];
   ROS_INFO("send can message");
   pubcan.publish( can_frame_msg);
};

void multiThreadListener::AgvVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   static uint8_t Assistant_ARM  = CLOSED;
   int motor1_speed,motor2_speed, motor3_speed, motor4_speed  = 0;
   int slave_id1 = 1;
   int slave_id2 = 2;
   int slave_id3 = 3;
   int slave_id4 = 4;
   vector<unsigned char> shell_data1 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data2 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data3 = {0x07,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data4 = {0x07,0,0,0, 0, 0, 0x03, 0};

   float vel_x, vel_yaw, vel_pole, vel_e_pole,left_speed,right_speed= 0;
   
   vel_x = msg->linear.x/5;
   vel_yaw = msg->angular.z/300*4;
   vel_pole = msg->linear.z;
   vel_e_pole  =msg->linear.y;

   if(msg->angular.x>0){
      is_synergy = true;
   }else if(msg->angular.x<0){
      is_synergy = false;
   }
   if(is_synergy){
      float turning_R,turning_w,vel_left,vel_right = 0.0f;
      geometry_msgs::Twist v;
      vel_x = vel_x/2;
      v.linear.x = vel_x;
      
      // v.angular.z = vel_yaw;
      /*计算转弯半径和角速度*/
      if(vel_yaw != 0){
         
         vel_yaw = vel_yaw/2;
         turning_w = ((liner_vel+vel_yaw) - (liner_vel-vel_yaw)) / Wheel_L_2_R_DIS;
         turning_R = ((liner_vel+vel_yaw) + (liner_vel-vel_yaw)) /2 /turning_w;
         v.linear.x = vel_x;
         v.angular.z = turning_w;
         v.angular.x = turning_R;
      }
      pub_syn.publish(v);
      ROS_INFO("SENDING synergy speed!!!");

      if(vel_x != 0){
         left_speed = vel_x+vel_yaw;
         right_speed = vel_x-vel_yaw;
         if(vel_x < 0){
            left_speed = vel_x-vel_yaw;
            right_speed = vel_x+vel_yaw;
         }
      }else{
         left_speed = 0;
         right_speed = 0;
      }
   }else{
      // ROS_INFO("NOT SENDING synergy speed!!!");
      left_speed = vel_x+vel_yaw;
      right_speed = vel_x-vel_yaw;
      if(vel_x < 0){
         left_speed = vel_x-vel_yaw;
         right_speed = vel_x+vel_yaw;
      }
   }

   cout<<"the agv linear speed is:"<<left_speed<<"  ,the yaw speed is :"<<right_speed<<endl;
   motor1_speed = (left_speed)/(2*PI*0.06)*16*10000;
   motor1_speed = motor1_speed*-1;
   motor2_speed = (right_speed)/(2*PI*0.06)*16*10000;
   // motor2_speed = motor2_speed*-1;
   if(motor1_speed == 0 && abs(testMotor_info[slave_id1-1].Actual_Val) <= 100){
      shell_data1[0] = 0x07;
   }else{
      shell_data1[0] = 0x0f;
   }
   if(motor2_speed == 0 && abs(testMotor_info[slave_id1-1].Actual_Val) <= 100){
      shell_data2[0] = 0x07;
   }else{
      shell_data2[0] = 0x0f;
   }
   // cout<<"the motor 1 speed is:"<<motor1_speed<<"  ,the motor 2 speed is :"<<motor2_speed<<endl;
   cout<<"the feedback motor 1 speed is:"<<testMotor_info[slave_id1-1].Actual_Val<<"  ,the feedback motor 2 speed is :"<<testMotor_info[slave_id2-1].Actual_Val<<endl;
   shell_data1[2] = motor1_speed&0xff;
   shell_data1[3] = (motor1_speed>>8)&0xff;
   shell_data1[4] = (motor1_speed>>16)&0xff;
   shell_data1[5] = (motor1_speed>>24)&0xff;
   shell_data2[2] = motor2_speed&0xff;
   shell_data2[3] = (motor2_speed>>8)&0xff;
   shell_data2[4] = (motor2_speed>>16)&0xff;
   shell_data2[5] = (motor2_speed>>24)&0xff;

   PDO_Send_func(slave_id1, 3, shell_data1, 8);
   PDO_Send_func(slave_id2, 3, shell_data2, 8);


   // cout<<"the motor 3 speed Actual_Pos:"<<testMotor_info[2].Actual_Pos<<"the motor 4 speed Actual_Pos:"<<testMotor_info[3].Actual_Pos<<endl;
   // cout<<"the motor 3 status is:"<<Right_Is_Arrival<<"  ,the motor 4 status is :"<<Left_Is_Arrival<<endl;
   // printf("the motor 3 status is:%d ,the motor 4 status is :%d\r\n",Right_Is_Arrival, Left_Is_Arrival);
   /*if the active arm is arrived, the motor should be disable while the braking is closed*/
   if(Left_Is_Arrival == MOVING ){
      shell_data4[0] = 0x0f;
   }
   if(Right_Is_Arrival == MOVING ){
      shell_data3[0] = 0x0f;
   }
   if(vel_pole == 1) {
      if(Left_Is_Arrival == CLOSE_ARRIVALED ){
         shell_data4[0] = 0x07;
      }
      else{
         shell_data4[0] = 0x0f;
      }
      if(Right_Is_Arrival == CLOSE_ARRIVALED){
         shell_data3[0] = 0x07;
      }
      else{
         shell_data3[0] = 0x0f;
      }
      motor3_speed = vel_pole*200000;
      motor4_speed = vel_pole*200000;
      if(testMotor_info[2].Actual_Pos >= -300000){
         motor3_speed *= 0.1;
      }
      if(testMotor_info[3].Actual_Pos >= -300000){
         motor4_speed *= 0.1;
      }
      shell_data3[2] = motor3_speed&0xff;
      shell_data3[3] = (motor3_speed>>8)&0xff;
      shell_data3[4] = (motor3_speed>>16)&0xff;
      shell_data3[5] = (motor3_speed>>24)&0xff;
      shell_data4[2] = motor4_speed&0xff;
      shell_data4[3] = (motor4_speed>>8)&0xff;
      shell_data4[4] = (motor4_speed>>16)&0xff;
      shell_data4[5] = (motor4_speed>>24)&0xff;
   }else if( Assistant_ARM == OPENED && vel_pole == -1){
      if(Left_Is_Arrival == OPEN_ARRIVALED ){
         shell_data4[0] = 0x07;
      }
      else{
         shell_data4[0] = 0x0f;
      }
      if(Right_Is_Arrival == OPEN_ARRIVALED){
         shell_data3[0] = 0x07;
      }
      else{
         shell_data3[0] = 0x0f;
      }
      motor3_speed = vel_pole*200000;
      motor4_speed = vel_pole*200000;
      if(testMotor_info[2].Actual_Pos <= -4700000){
         motor3_speed *= 0.1;
      }
      if(testMotor_info[3].Actual_Pos <= -4700000){
         motor4_speed *= 0.1;
      }
      // cout<<"the motor 3 speed is:"<<motor3_speed<<"  ,the motor 4 speed is :"<<motor4_speed<<endl;
      shell_data3[2] = motor3_speed&0xff;
      shell_data3[3] = (motor3_speed>>8)&0xff;
      shell_data3[4] = (motor3_speed>>16)&0xff;
      shell_data3[5] = (motor3_speed>>24)&0xff;
      shell_data4[2] = motor4_speed&0xff;
      shell_data4[3] = (motor4_speed>>8)&0xff;
      shell_data4[4] = (motor4_speed>>16)&0xff;
      shell_data4[5] = (motor4_speed>>24)&0xff;
   }

   // printf(" the motor 3 shell is:%02X,the motor 4 shell is:%02X\r\n",shell_data3[0], shell_data4[0]);
   PDO_Send_func(slave_id3, 3, shell_data3, 8);
   PDO_Send_func(slave_id4, 3, shell_data4, 8);

   // cout<<"the eletricity pole speed is:"<<vel_e_pole<<endl;
   if(vel_e_pole == 1){
      Assistant_ARM = OPENED;
      ser_shell = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};
   }else if(vel_e_pole == -1){
      Assistant_ARM = CLOSED;
      ser_shell = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};
   }else{
      ser_shell = {0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00, 0xE4, 0x80};
   }
}


void multiThreadListener::NavVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   static uint8_t Assistant_ARM  = CLOSED;
   int motor1_speed,motor2_speed, motor3_speed, motor4_speed  = 0;
   int slave_id1 = 1;
   int slave_id2 = 2;
   int slave_id3 = 3;
   int slave_id4 = 4;
   vector<unsigned char> shell_data1 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data2 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data3 = {0x07,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data4 = {0x07,0,0,0, 0, 0, 0x03, 0};

   float vel_x, vel_yaw, vel_pole, vel_e_pole= 0;
   vel_x = msg->linear.x;
   vel_yaw = msg->angular.z*Wheel_2_C_DIS;
   vel_pole = msg->linear.z;
   vel_e_pole = msg->linear.y;
   cout<<"the agv linear speed is:"<<vel_x<<"  ,the yaw speed is :"<<vel_yaw<<endl;
   motor1_speed = (vel_x+vel_yaw)/(2*PI*0.06)*16*10000;
   motor1_speed = motor1_speed*-1;
   motor2_speed = (vel_x*-1+vel_yaw)/(2*PI*0.06)*16*10000;
   motor2_speed = motor2_speed*-1;
   if(motor1_speed == 0 && abs(testMotor_info[slave_id1-1].Actual_Val) <= 100){
      shell_data1[0] = 0x07;
   }else{
      shell_data1[0] = 0x0f;
   }
   if(motor2_speed == 0 && abs(testMotor_info[slave_id1-1].Actual_Val) <= 100){
      shell_data2[0] = 0x07;
   }else{
      shell_data2[0] = 0x0f;
   }
   // cout<<"the motor 1 speed is:"<<motor1_speed<<"  ,the motor 2 speed is :"<<motor2_speed<<endl;

   shell_data1[2] = motor1_speed&0xff;
   shell_data1[3] = (motor1_speed>>8)&0xff;
   shell_data1[4] = (motor1_speed>>16)&0xff;
   shell_data1[5] = (motor1_speed>>24)&0xff;
   shell_data2[2] = motor2_speed&0xff;
   shell_data2[3] = (motor2_speed>>8)&0xff;
   shell_data2[4] = (motor2_speed>>16)&0xff;
   shell_data2[5] = (motor2_speed>>24)&0xff;

   PDO_Send_func(slave_id1, 3, shell_data1, 8);
   PDO_Send_func(slave_id2, 3, shell_data2, 8);


   if(Left_Is_Arrival == MOVING ){
      shell_data4[0] = 0x0f;
   }
   if(Right_Is_Arrival == MOVING ){
      shell_data3[0] = 0x0f;
   }
   if(vel_pole == 1) {
      if(Left_Is_Arrival == CLOSE_ARRIVALED ){
         shell_data4[0] = 0x07;
      }
      else{
         shell_data4[0] = 0x0f;
      }
      if(Right_Is_Arrival == CLOSE_ARRIVALED){
         shell_data3[0] = 0x07;
      }
      else{
         shell_data3[0] = 0x0f;
      }
      motor3_speed = vel_pole*200000;
      motor4_speed = vel_pole*200000;
      if(testMotor_info[2].Actual_Pos >= -50000){
         motor3_speed *= 0.2;
      }
      if(testMotor_info[3].Actual_Pos >= -50000){
         motor4_speed *= 0.2;
      }
      shell_data3[2] = motor3_speed&0xff;
      shell_data3[3] = (motor3_speed>>8)&0xff;
      shell_data3[4] = (motor3_speed>>16)&0xff;
      shell_data3[5] = (motor3_speed>>24)&0xff;
      shell_data4[2] = motor4_speed&0xff;
      shell_data4[3] = (motor4_speed>>8)&0xff;
      shell_data4[4] = (motor4_speed>>16)&0xff;
      shell_data4[5] = (motor4_speed>>24)&0xff;
   }else if( Assistant_ARM == OPENED && vel_pole == -1){
      if(Left_Is_Arrival == OPEN_ARRIVALED ){
         shell_data4[0] = 0x07;
      }
      else{
         shell_data4[0] = 0x0f;
      }
      if(Right_Is_Arrival == OPEN_ARRIVALED){
         shell_data3[0] = 0x07;
      }
      else{
         shell_data3[0] = 0x0f;
      }
      motor3_speed = vel_pole*200000;
      motor4_speed = vel_pole*200000;
      if(testMotor_info[2].Actual_Pos <= -4950000){
         motor3_speed *= 0.2;
      }
      if(testMotor_info[3].Actual_Pos <= -4950000){
         motor4_speed *= 0.2;
      }
      // cout<<"the motor 3 speed is:"<<motor3_speed<<"  ,the motor 4 speed is :"<<motor4_speed<<endl;
      shell_data3[2] = motor3_speed&0xff;
      shell_data3[3] = (motor3_speed>>8)&0xff;
      shell_data3[4] = (motor3_speed>>16)&0xff;
      shell_data3[5] = (motor3_speed>>24)&0xff;
      shell_data4[2] = motor4_speed&0xff;
      shell_data4[3] = (motor4_speed>>8)&0xff;
      shell_data4[4] = (motor4_speed>>16)&0xff;
      shell_data4[5] = (motor4_speed>>24)&0xff;
   }

   // printf(" the motor 3 shell is:%02X,the motor 4 shell is:%02X\r\n",shell_data3[0], shell_data4[0]);
   PDO_Send_func(slave_id3, 3, shell_data3, 8);
   PDO_Send_func(slave_id4, 3, shell_data4, 8);

   // cout<<"the eletricity pole speed is:"<<vel_e_pole<<endl;
   if(vel_e_pole == 1){
      Assistant_ARM = OPENED;
      ser_shell = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};
   }else if(vel_e_pole == -1){
      Assistant_ARM = CLOSED;
      ser_shell = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};
   }else{
      ser_shell = {0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00, 0xE4, 0x80};
   }

}

void multiThreadListener::AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& Odo_msg)
{
   Quaternion quaternion;
   EulerAngles angles;
   float AGV_pos_X, AGV_pos_Y, AGV_pos_Z;
   quaternion.x = Odo_msg->pose.pose.orientation.x;
   quaternion.y = Odo_msg->pose.pose.orientation.y;
   quaternion.z = Odo_msg->pose.pose.orientation.z;
   quaternion.w = Odo_msg->pose.pose.orientation.w;
   AGV_pos_X = Odo_msg->pose.pose.position.x;
   AGV_pos_Y = Odo_msg->pose.pose.position.y;
   AGV_pos_Z = Odo_msg->pose.pose.position.z;
   angles = ToEulerAngles(quaternion);
   // cout<<"the AGV now orientation yaw is: "<<angles.yaw/PI*180<<"; orientation pitch"<<angles.pitch/PI*180<<"; orientation roll"<<angles.roll/PI*180<<";"<<endl;
   // cout<<"the AGV now postion is "<<AGV_pos_X<<"m; "<<AGV_pos_Y<<"m; "<<AGV_pos_Z<<"m"<<endl;
}

void multiThreadListener::chatterCallback1(const pubmotor::motor::ConstPtr& msg)
{
  ROS_INFO("id: [%s]", msg->id.c_str());
}

void multiThreadListener::chatterCallback3(const can_msgs::Frame::ConstPtr &msg)
{
   static int times = 0;
	// std::cout << times++ << "ID: " << msg->id << " data: " ;    
   //  for (int i = 0; i < msg->dlc; i++)
   //  {
   //      printf("%X ", msg->data[i]);
   //  }
   CAN_msg_classify(msg->data.data(), msg->id,  msg->dlc);
}

void multiThreadListener::SerIOCallback(const std_msgs::Bool::ConstPtr& msg){
   if(msg->data == WRITE_IO){
      IO_STATE = WRITE;
   }else if(msg->data == READ_IO){
      IO_STATE = READ;
   }
}

void multiThreadListener::publish_arm_wheel(bool state){
   std_msgs::Bool data;
   if(state == TOUCH){
      data.data = TOUCH;
   }else if(state == NOT_TOUCH){
      data.data = NOT_TOUCH;
   }
   arm_wheel_can.publish(data);
}


void sigintHandler(int sig){
   multiThreadListener ur_msg;
   ur_msg.NMT_ShellFunc(0x00,0x81);
   ser.close();
   ros::shutdown();
}

uint8_t serial_send(void){
   uint8_t Len = 0;
   if(IO_STATE == WRITE){
      Len = ser_shell.size();
      for(uint8_t i =0; i < Len; i++){
         s_buffer[i] = ser_shell[i];
         // printf("%02x",s_buffer[i]);
      }
      // printf("\r\n");
   }else if(IO_STATE == READ){
      Len = ser_RIO_shell.size();
      for(uint8_t i =0; i < Len; i++){
         s_buffer[i] = ser_RIO_shell[i];
         // printf("%02x",s_buffer[i]);
      }
      // printf("\r\n");
   }
   
   return Len;
}

// int main(int argc, char **argv)
// {
//    uint8_t len = 0;
//   ros::init(argc, argv, "multi_sub", ros::init_options::NoSigintHandler);

//   multiThreadListener ur_msg;
//   signal(SIGTERM, sigintHandler);
//   signal(SIGINT, sigintHandler);

//    serial::Serial ser;
//    try {
//       ser.setPort("/dev/ttyS0");
//       ser.setBaudrate(9600);
//       serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//       ser.setTimeout(to);
//       ser.open();
//    }
//    catch (serial::IOException& e) {
//       ROS_ERROR_STREAM("Unable to open port ");
//       return -1;
//    }

//    if(ser.isOpen()){
//       ROS_INFO_STREAM("Serial Port initialized for writing");
//    } else {
//       return -1;
//    }
//   vector<unsigned char> data = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};
//    ur_msg.NMT_ShellFunc(0x00,0x81);
//    std::this_thread::sleep_for(std::chrono::milliseconds(500));
//    ur_msg.NMT_ShellFunc(0x00,0x01);
//    std::this_thread::sleep_for(std::chrono::milliseconds(500));


//    ur_msg.PDO_Send_func(1,3, data,8);
//    ur_msg.PDO_Send_func(2,3, data,8);
//    ur_msg.PDO_Send_func(3,3, data,8);
//    ur_msg.PDO_Send_func(4,3, data,8);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    data[0] = 0x07;
//    ur_msg.PDO_Send_func(1,3, data,8);
//    ur_msg.PDO_Send_func(2,3, data,8);
//    ur_msg.PDO_Send_func(3,3, data,8);
//    ur_msg.PDO_Send_func(4,3, data,8);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    data[0] = 0x0f;
//    ur_msg.PDO_Send_func(1,3, data,8);
//    ur_msg.PDO_Send_func(2,3, data,8);
//    ur_msg.PDO_Send_func(3,3, data,8);
//    ur_msg.PDO_Send_func(4,3, data,8);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

// //   ros::AsyncSpinner spinner(3); // Use 2 threads
// //   spinner.start();
//   ros::Rate rate(20);
//   while (ros::ok())
//   {
//       ROS_INFO("11!!!");
//       if(ser.available()){
//          if (IO_STATE == READ){
//             ROS_INFO("Detecting the IO state!!!");
//             size_t result;
//             result = ser.read(r_buffer,ser.available());
//             cout<< result<<endl;
//             if(result < 9){
//                for (int i = 0; i < result; i++)
//                {
//                   printf("%02X ", r_buffer[i]);
//                }
//                printf("\r\n");
//                if ((r_buffer[3] & 0x01) == 0x01){
//                   ur_msg.publish_arm_wheel(TOUCH);
//                   ROS_INFO("CLOSED to the CAR!!!");
//                }else{
//                   ur_msg.publish_arm_wheel(NOT_TOUCH);
//                   ROS_INFO(" NOT CLOSED to the CAR!!!");
//                }
//             }
//          }

//       }
//       ROS_INFO("22!!!");
//       len = serial_send();
//       ser.write(s_buffer,len);
//       ROS_INFO("33!!!");
//       // ur_msg.PDO_Send_func(1,1, data,8);
//       // ros::waitForShutdown();
//       // ur_msg.Motor_Info(1);
//       // ur_msg.Motor_Info(2);
//       rate.sleep();
//       ros::spinOnce();
//   }
//   return 0;
// }


void readSerialData()
{
   while (ros::ok())
   {
      if (ser.available()) 
      {
         size_t result;
         result = ser.read(r_buffer, ser.available());
         if (result < 9 && IO_STATE == READ) 
         {
            for (int i = 0; i < result; i++) 
            {

               printf("%02X ", r_buffer[i]);
            }
            printf("\r\n");
            if ((r_buffer[3] & 0x01) == 0x01) 
            {
               is_touch = TOUCH;
               ROS_INFO("CLOSED to the CAR!!!");
            }
            else 
            {
               is_touch = NOT_TOUCH;
               ROS_INFO("NOT CLOSED to the CAR!!!");
            }
         }
      }
   }
}

int main(int argc, char **argv)
{
   uint8_t len = 0;
   ros::init(argc, argv, "multi_sub", ros::init_options::NoSigintHandler);

   signal(SIGTERM, sigintHandler);
   signal(SIGINT, sigintHandler);

   multiThreadListener ur_msg;

   try 
   {
      ser.setPort("/dev/ttyS0");
      ser.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
   } 
   catch (serial::IOException& e) 
   {
      ROS_ERROR_STREAM("Unable to open port");
      return -1;
   }

   if (ser.isOpen()) 
   {
      ROS_INFO_STREAM("Serial Port initialized for writing");
   } 
   else 
   {
      return -1;
   }

   // Start thread for serial data reading
   std::thread serial_thread(readSerialData);

   // ROS spinner for handling ROS callbacks in parallel
   ros::AsyncSpinner spinner(2); // Use 2 threads for ROS callbacks
   spinner.start();

   // Simulating some PDO functions for initial setup
   vector<unsigned char> data = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};
   ur_msg.NMT_ShellFunc(0x00, 0x81);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
   ur_msg.NMT_ShellFunc(0x00, 0x01);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));

   ur_msg.PDO_Send_func(1, 3, data, 8);
   ur_msg.PDO_Send_func(2, 3, data, 8);
   ur_msg.PDO_Send_func(3, 3, data, 8);
   ur_msg.PDO_Send_func(4, 3, data, 8);
   std::this_thread::sleep_for(std::chrono::milliseconds(100));

   data[0] = 0x07;
   ur_msg.PDO_Send_func(1, 3, data, 8);
   ur_msg.PDO_Send_func(2, 3, data, 8);
   ur_msg.PDO_Send_func(3, 3, data, 8);
   ur_msg.PDO_Send_func(4, 3, data, 8);
   std::this_thread::sleep_for(std::chrono::milliseconds(100));

   data[0] = 0x0f;
   ur_msg.PDO_Send_func(1, 3, data, 8);
   ur_msg.PDO_Send_func(2, 3, data, 8);
   ur_msg.PDO_Send_func(3, 3, data, 8);
   ur_msg.PDO_Send_func(4, 3, data, 8);
   std::this_thread::sleep_for(std::chrono::milliseconds(100));

   ros::Rate rate(20); // Loop at 20 Hz
   while (ros::ok()) 
   {
      // ROS_INFO("Running...");

      len = serial_send();
      ser.write(s_buffer, len);

      if( IO_STATE == READ){
         ur_msg.publish_arm_wheel(is_touch);
      }

      rate.sleep(); // Sleep for the duration of the rate
      ros::spinOnce(); // Call ROS callbacks
   }

   // Join the serial reading thread before exiting
   serial_thread.join();
   return 0;
}

 
