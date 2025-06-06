#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "move/detect_state.h"
#include <can_msgs/Frame.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <iomanip>
#include <signal.h>

#define PI 3.1415926
#define Speed_Max 0.8
#define UPCAR_Speed_Max Speed_Max/8*5
#define Car_L 300
#define START 1
#define STOP 0
#define Wheel_L_2_R_DIS 0.766
#define BASE2LIDAR 0.9375 // m
#define CONTION_START_LINE 1
#define CONTION_START_CLUSTER 2
#define CONTION_STOPA 3
#define CONTION_STOPB 4
#define CONTION_WAIT 5
#define SEND false
#define NOT_SEND true
#define READ false
#define WRITE true
#define NOT_TOUCH false
#define TOUCH true
#define TURN_YAW 2
#define TURN_RADIUS1 4
#define TURN_RADIUS2 3
#define BEZIER 0
#define UPCAR true
#define NOT_UPCAR false


typedef enum ROBOT_STATE_ITEMS
{
	WAITING,//0
	GOTO_P1,//1
	TURNNING_CIRA1,//2
	TURN_CIRA1,//3

	GOTO_P2,//4
	TURNNING_CIRA2,//5
	TURN_CIRA2,//6

	GOTO_P3,//7
	TURNNING_CIRA3,//8
	TURN_CIRA3,//9

	GOTO_P4,//10
	TURNNING_CIRA4,//11
	GOTO_P5,
	GOTO_P6,

	DETECTING_WHEEL,//12
	TURNNING_GOTO_CAR,//13
	GOTO_CAR,//14
	TURNNING_FACE_CAR,//15

	DETECTING_WHEEL_PASS,//16
	TURNNING_PATH,//17
	CLOSING_TO_CAR,//18
	TURNNING_CLOSE_CAR,//19

	PLAN_STARIGHT,//20
	STARIGHT_PASS, //21

	PLAN_STARIGHT_READY, //22
	STARIGHT_PASS1, //23
	REAR_TOUCH_WHEEL, // 24
	STARIGHT_PASS2, //25
	TAKING_UP_THE_CAR, //26

	FRONT_TAKING_DOWN_THE_CAR,//27
	FRONT_CLOSING_TO_REAR,//28
	REAR_TAKING_DOWN_THE_CAR,//29
	BACK_AWAY_CAR,//30

	STOPPING,//31
}ROBOT_STATE_ITEMS;
typedef enum DETECT_WHEEL_STATE_ITEMS{
	NO_CAR_POINT,
	GET_CAR_POINT,
	ARRIVIALED_CAR,
}DETECT_WHEEL_STATE_ITEMS;

typedef struct TRCK_PIONTS
{
	float X;           //������
	float Y;             
	float T;           // MS
	float theate;
	float K;           //�õ㵼��
	float V;           //�ٶ�               M/S
	//float theat;       //����Ŀ����̬      ���
}TRCK_PIONTS;
TRCK_PIONTS *NOW_PATH = NULL;
void PlanLeftPath(TRCK_PIONTS *PATH ,uint num);
/*往前走，离开开始区域*/
TRCK_PIONTS GOTO_PATH1_UPCAR[6] = 
{
	{0.0, 1000.0, 0.0f, 0.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{0.0, 1500, 0.0f, 0.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{0.0f,2000.0, 0.0f, 0.0f, 0.0f,0.5f*UPCAR_Speed_Max},
	{0.0f,2000.0, 0.0f, 0.0f, 0.0f,0.5f*UPCAR_Speed_Max},
	{0.0f,2500.0, 0.0f, 0.0f, 0.0f,0.2f*UPCAR_Speed_Max},
	{0.0f,2900.0, 0.0f, 0.0f, 0.0f,0.2f*UPCAR_Speed_Max},
};
TRCK_PIONTS GOTO_PATH1[7] = 
{
{0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
{0.0, 1000.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
{0.0, 1500, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
{0.0f,2000.0, 0.0f, 0.0f, 0.0f,0.5f*Speed_Max},
{0.0f,3000.0, 0.0f, 0.0f, 0.0f,0.4f*Speed_Max},
{0.0f,3500.0, 0.0f, 0.0f, 0.0f,0.2f*Speed_Max},
{0.0f,3900.0, 0.0f, 0.0f, 0.0f,0.2f*Speed_Max},
// {0.0, 3000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
// {0.0, 4000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
// {0.0, 4500.0, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
// /*代码测试为了方便进行的修改，实机测试需要删掉*/
// // {0.0, 5000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
// {0.0, 5000.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
};
// TRCK_PIONTS GOTO_PATH2[6] = 
// {
// // {0.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.2*Speed_Max},
// // {600.0, 5000.0, 0.0f, 90.0f, 0.0f, 0.5f*0.5f*Speed_Max},
// {1000.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
// {1500.0, 6000.0f, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {2000.0, 6000.0f, 0.0f, 90.0f, 0.0f,1.0f*Speed_Max},
// {3000.0, 6000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {4000.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
// {5000.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
// // {5000.0, 5000.0, 0.0f, 90.0f, 0.0f, 1.0f*0.5f*Speed_Max},
// };
/*行使在主干道*/
TRCK_PIONTS GOTO_PATH2_UPCAR[15] = 
{
/*radius：4m*/
	{3000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*UPCAR_Speed_Max},
	{2000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.5f*UPCAR_Speed_Max},
	{1000.0, 7000.0f, 0.0f, 90.0f, 0.0f,1.0f*UPCAR_Speed_Max},
	{-2000.0, 7000.0f, 0.0f, 90.0f, 0.0f,1.0f*UPCAR_Speed_Max},
	{-4000.0, 7000.0f, 0.0f, 90.0f, 0.0f,1.0f*UPCAR_Speed_Max},
	{-5000.0, 7000.0f, 0.0f, 90.0f, 0.0f,1.0f*UPCAR_Speed_Max},
	{-5500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-6000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-7000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-8500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-10000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-11000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-12000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
	{-12500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{-12900.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
};
TRCK_PIONTS GOTO_PATH2[11] = 
{
// {0.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.2*Speed_Max},
// {600.0, 5000.0, 0.0f, 90.0f, 0.0f, 0.5f*0.5f*Speed_Max},
/*radius：4m*/
// {-4000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
// {-5000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.4f*Speed_Max},
// {-5500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.6f*Speed_Max},
// {-6000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {-7000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {-8500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {-10000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {-11500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
// {-12500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
// {-13000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
/*raidus:3m*/
{-3000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
{-4000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.4f*Speed_Max},
{-4500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.6f*Speed_Max},
{-6000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-7000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-8500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-10000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-12000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-13000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
{-13500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
{-13900.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
};
/*进入车辆停放区域*/
TRCK_PIONTS GOTO_PATH3[6] = 
{
{-17000.0, 3000.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
{-17000.0, 2500.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
{-17000.0, 1000.0, 0.0f, 179.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
{-17000.0, -1000.0, 0.0f, 179.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
{-17000.0, -1500.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
{-17000.0, -2000.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
// {7000.0, 2000.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
// {7000.0, 3500.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
// {7000.0, 3000.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
// {7000.0, 2700.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
// {7000.0, 2300.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
// {7000.0, 2000.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
// {7000.0, 500.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
// /*代码测试为了方便进行的修改，实机测试需要删掉*/
// {5000.0, 0.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
// {5000.0, 0.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
};
/*离开车辆停放区域*/
TRCK_PIONTS GOTO_PATH4_UPCAR[6] = 
{
	{-17000.0, 0.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{-17000.0, 1000.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{-17000.0, 1500.0, 0.0f, 179.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
	{-17000.0, 2000.0, 0.0f, 179.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
	{-17000.0, 2500.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{-17000.0, 2900.0, 0.0f, 179.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
// {2000.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
// {1000.0, 0.0, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
// {500.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
};
TRCK_PIONTS GOTO_PATH4[5] = 
{
{-17000.0, 1000.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
{-17000.0, 1500.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
{-17000.0, 3000.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
{-17000.0, 3500.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
{-17000.0, 3900.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
// {2000.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
// {1000.0, 0.0, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
// {500.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
};
/*行使在主干道*/
TRCK_PIONTS GOTO_PATH5[12] = 
{
{-14000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
{-13000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
{-12000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
{-11000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.5f*Speed_Max},
{-9500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.8f*Speed_Max},
{-8500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-8000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-6500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-5000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*Speed_Max},
{-4000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
{-3600.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
{-3100.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_PATH5_UPCAR[11] = 
{
	{-13000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*UPCAR_Speed_Max},
	{-12000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*UPCAR_Speed_Max},
	{-11000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.5f*UPCAR_Speed_Max},
	{-10000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.8f*UPCAR_Speed_Max},
	{-8500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-8000.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-6500.0, 7000.0, 0.0f, 90.0f, 0.0f, 1.0f*UPCAR_Speed_Max},
	{-5500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.75f*UPCAR_Speed_Max},
	{-5000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*UPCAR_Speed_Max},
	{-4500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{-4100.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
};
/*返回开始点*/
TRCK_PIONTS GOTO_PATH6[6] = 
{
{0.0, 4000.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
{0.0, 3000.0, 0.0f, 0.0f, 0.0f, 0.4f*Speed_Max},
{0.0, 2000.0, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
{0.0, 1000.0, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
{0.0, 500, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
{0.0f,0.0, 0.0f, 0.0f, 0.0f,0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_PATH6_UPCAR[4] = 
{
	{0.0, 3000.0, 0.0f, 0.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
	{0.0, 2000.0, 0.0f, 0.0f, 0.0f, 0.4f*UPCAR_Speed_Max},
	{0.0, 1500.0, 0.0f, 0.0f, 0.0f, 0.4f*UPCAR_Speed_Max},
	{0.0, 1000.0, 0.0f, 0.0f, 0.0f, 0.2f*UPCAR_Speed_Max},
// {0.0, 700, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
// {0.0f,0.0, 0.0f, 0.0f, 0.0f,0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_CIRA1[10] = 
{
{0.0, 5000.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{30.38, 5347.29, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{120.61, 5684.04, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{267.95, 6000.00, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{467.91, 6285.57, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{714.42, 6532.09, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{1000.00, 6732.05, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{1315.96, 6879.38, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{1652.70, 6969.61, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{2000.0, 7000.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_CIRA2[10] = 
{
{5000.0, 7000.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{5347.29, 6969.61, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{5684.04, 6879.38, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6000.0, 6732.05, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6285.57, 6532.08, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6532.08, 6285.57, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6732.05, 6000.00, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6879.38, 5684.04, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6969.61, 5347.29, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{7000.0, 5000.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_CIRA3[10] = 
{
{7000.0, 2000.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{6969.61, 1652.70, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{6879.38, 1315.96, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6732.05, 1000.00, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6532.09, 714.42, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6285.57, 467.91, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{6000.00, 267.95, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{5684.04, 120.61, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{5347.29, 30.38, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{5000.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
};
TRCK_PIONTS Car_location[5] = 
{
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
};
// TRCK_PIONTS Car_location[10] = 
// {
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// };
TRCK_PIONTS Pass_location[3] = 
{
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
};
TRCK_PIONTS Straight_pass[5] = 
{
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
};

/*一些相关状态*/
typedef enum ROBOT_MOVE
{
	MOVE_STOP,
	MOVE_GOING,
	MOVE_ARRIVE,
}ROBOT_MOVE;
typedef enum TURNNING_STATE
{
	TURNNING,
	TURNNED,
	NO_TURNNING,
}TURNNING_STATE;
typedef enum TURNNING_TYPE
{
	ROTATION,
	TURN,
	NO_TURN,
}TURNNING_TYPE;
typedef enum TURNNING_ORIENTATION
{
	LEFT,
	RIGHT,
	Straight,
}TURNNING_ORIENTATION;
typedef enum PLANNING_STATE
{
	PLANNNING,
	PLANNED,
	NO_PALNNING,
}PLANNING_STATE;
typedef struct TIMES
{
	uint16_t TIME_LAST;           //�ϴε�ʱ��
	uint16_t TIME_pass;           //������ʱ��   
} TIMES;
typedef struct PATH_FOLLOW
{
	double TIME_LAST; //上次的时间
	double TIME_pass; //经过的时间
	uint8_t COUNT;	   //点数
} PATH_FOLLOW;
typedef enum IO_STATE_ITEMS{
	RECEIVE,
	NOT_RECEIVE,
}IO_STATE_ITEMS;
typedef enum STRAIGHT_STATE_ITEMS{
	MOVING,
	STOPING,
}STRAIGHT_STATE_ITEMS;
typedef enum ARM_STATE_ITEMS{
	OPEN,
	CLOSE,
	NOT,
}ARM_STATE_ITEMS;

/*四元素解算参数*/
struct Quaternion {
    double w, x, y, z;
};
struct EulerAngles {
    double roll, pitch, yaw;
};

/*PID计算参数*/
typedef struct PID_Date
{
	float error ;
	float last_error ;
	float earlier_error ;
	float K_P ;
	float K_I ;
	float K_D ;
	float I_Separate ;
	float I_Limit ;
	float Out_MAX ;
	float Out_MIN ; 
	float Dead_Size;
  float Output ;
}PID_Date;
PID_Date YAW_PID,TRACK_PID,TURN_PID;

class robot_vel{
	private:
		ros::NodeHandle n; 
		ros::Publisher pub ;
	public:
		robot_vel(){
			pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
		}
		void Cal_VEL(void);
};

using namespace std;
uint8_t IS_REC = 0;
TURNNING_STATE TURN_STATE = NO_TURNNING;
TURNNING_TYPE TURN_TYPE = NO_TURN;
TURNNING_ORIENTATION TURN_Orientation = Straight;
ROBOT_STATE_ITEMS 	ROBOT_STATE = WAITING;
ROBOT_MOVE	MOVING_STATE = MOVE_STOP;
DETECT_WHEEL_STATE_ITEMS DETECTING_STATE = NO_CAR_POINT;
PLANNING_STATE PLAN_STATE = NO_PALNNING;
IO_STATE_ITEMS Ser_IO_STATE = NOT_RECEIVE;
IO_STATE_ITEMS REAR_IO_STATE = NOT_RECEIVE;
STRAIGHT_STATE_ITEMS PASS_WHEEL = MOVING;
STRAIGHT_STATE_ITEMS CLOSING_REAR = MOVING;
STRAIGHT_STATE_ITEMS REAR_PASS_WHEEL = MOVING;
ARM_STATE_ITEMS ASSIST_ARM_STATE = NOT;
ARM_STATE_ITEMS REAR_ASSIST_ARM_STATE = NOT;
ARM_STATE_ITEMS PRIME_ARM_STATE = NOT;
ARM_STATE_ITEMS REAR_PRIME_ARM_STATE = NOT;
PATH_FOLLOW    Path_Contorl = {0,1};
uint32_t PATH_POINT_NUM = 0;
float ROBOT_REAL_X,ROBOT_REAL_Y = 0;
float ROBOT_REAL_YAW = 0.0f;
float World_VY,World_VX,World_VW = 0;
float expect_angle = 0;
float straight_start = 0;
float straight2_start = 0;
float back_start = 0;
double car_angle, face_car_angle = 0;
uint8_t get_start = STOP;
ros::Publisher IO_state_sub;
ros::Publisher pub_syn_ctrl;
move::detect_state Dect_state;
bool Is_synergy = false;
bool Is_SemiSyn = false; // Using in the synergy's model,true:only fronts moves,false:together move
bool Is_PRIME_ARM_ARRIVED = false;
bool Is_PRIME_ARM_CLOSE = false;
bool REAR_Is_PRIME_ARM_CLOSE = false;
bool IS_UP_CAR = NOT_UPCAR;

bool SEND_flag = NOT_SEND;
int CTRL_FLAG = CONTION_WAIT;
bool control_nodes(int condition_A_to_B)
{
    ros::NodeHandle nh;
    ros::ServiceClient stopA_client = nh.serviceClient<std_srvs::Empty>("stop_node_A");
	ros::ServiceClient startA_client = nh.serviceClient<std_srvs::Empty>("start_node_A");
	ros::ServiceClient stopB_client = nh.serviceClient<std_srvs::Empty>("stop_node_B");
    ros::ServiceClient startB_client = nh.serviceClient<std_srvs::Empty>("start_node_B");

    std_srvs::Empty srv;
	bool result1 = true;
	bool result2 = true;

	switch (condition_A_to_B)
	{
	case CONTION_START_LINE:
		{
			result1 = startA_client.call(srv); // 启动节点线性检测
        	ROS_INFO("Node LINE started");
		}
		break;
	case CONTION_START_CLUSTER:
		{
			// result1 = stopA_client.call(srv);  // 停止节点线性检测
			// sleep(2);  // 等待2秒，确保节点完全停止
			result2 = startB_client.call(srv); // 启动节点点云簇检测
			ROS_INFO("Node CLUSTER started");
		}
		break;
	case CONTION_STOPA:
		{
			result1 = stopA_client.call(srv);
			ROS_INFO("Node LINE stoped"); // 停止节点点云簇检测
		}
		break;
	case CONTION_STOPB:
	{
		result1 = stopB_client.call(srv);
		ROS_INFO("Node CLUSTER stoped"); // 停止节点点云簇检测
	}break;
	case CONTION_WAIT:
	{
		ROS_INFO("WAITING node!!!");
	}
	break;
	default:
		break;
	}

	ROS_INFO("The result is:%s", (result1)?"true":"false");
	if(result1 && result2){
		return true;
	}else{
		return false;
	}

}
void send_ctrl_node(int node_type){
	ROS_INFO("The CTRL_FLAG is:%d, The CTRL_FLAG is:%s", node_type, SEND_flag?"NOT_SEND":"SEND");
	if(SEND_flag == SEND){
		SEND_flag = control_nodes(node_type);
	}else{
		CTRL_FLAG = CONTION_WAIT;
	}
}



double degreeToRadian(double degree) {
    return degree * PI / 180.0;
}

double radianToDegree(double radian) {
    return radian * 180.0 / PI;
}
/*计算前后两次角度的误差值*/
double angleDifference(double angle1, double angle2) {
    double diff = atan2(sin(degreeToRadian(angle2 - angle1)), cos(degreeToRadian(angle2 - angle1)));
    return radianToDegree(diff);
}
/*四元素转为欧拉角*/
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


/*PID相关*/
void PID_Parameter_Speed_Init(PID_Date *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit)
{
	PID->K_P = Pi;
	PID->K_I = Ki;
	PID->K_D = Di;
	PID->Dead_Size = Dead_Size;
	PID->Out_MAX = Out_MAX;
	PID->Out_MIN = -Out_MAX;
	PID->I_Limit = I_Limit;
	PID->I_Separate = I_Separate;

	PID->error = PID->last_error = PID->earlier_error = 0;
	PID->Output = 0;
}
float PID_Position_Calculate(PID_Date *PID, float expect, float Encoder_Count)
{
	static float integral = 0;
	float differential = 0;
	PID->error = expect - Encoder_Count;		 //计算误差
	differential = PID->error - PID->last_error; //微分

	if (PID->I_Separate == 0)
		integral += PID->error;
	else
	{
		//判断是否积分分离
		if (abs(PID->error) < PID->I_Separate)
			integral += PID->error;
		else
			integral = 0;
	}

	//积分限幅
	if (integral > PID->I_Limit)
		integral = PID->I_Limit;
	if (integral < -PID->I_Limit)
		integral = -PID->I_Limit;

	//死区
	if (abs(PID->error) > PID->Dead_Size)
	{
		PID->Output = PID->K_P * PID->error + PID->K_I * integral + PID->K_D * differential;
	}
	else
		PID->Output = 0;

	PID->last_error = PID->error;

	if (PID->Output > PID->Out_MAX)
		PID->Output = PID->Out_MAX;
	if (PID->Output < PID->Out_MIN)
		PID->Output = PID->Out_MIN;

	// printf("PID->error: %f, integral: %f, differential: %f, PID->Output: %f\n", PID->error, integral, differential, PID->Output);
	return PID->Output;
	
}

/*路径跟踪相关*/
float Caculate_K(float dx , float dy )
{
	float k = 0;
	if(dx == 0)
		k = 10000;
  	else
		k = dy / dx ;
	return k ;
}

float k_dirction = 0;
float Vector_Unitization(float k , uint8_t num)
{
	float X_unit = 0;
	float y_unit = 0;
	//б��Ϊ0ʱ
	if(k == 0 )
	{
		X_unit = 1;
		y_unit = 0;
	}
	else if(k == 10000)
	{
		X_unit = 0;
		y_unit = 1;
	}
	else
	{
	X_unit = sqrt(1.0f /( k*k+ 1.0f) );
	y_unit = k * X_unit ;	
	}
	
	if(num == 0)
		return X_unit;
	else
		return y_unit;
}


void POINT_FOLLOW(TRCK_PIONTS *PIONTS ,TRCK_PIONTS *Last_PIONTS)
{
	printf("%d , %f , %f, %f, %f\r\n", Path_Contorl.COUNT, PIONTS->X , PIONTS->Y , ROBOT_REAL_X , ROBOT_REAL_Y);
	// cout<<"the AGV now postion is "<<ROBOT_REAL_X<<"mm; "<<ROBOT_REAL_Y<<"mm; "<<endl;
	float T_U_X = 0, T_U_Y = 0;
	float F_U_X = 0, F_U_Y = 0;
	float error = 0;
	float K_F = 0;
	float E_U_X = 0, E_U_Y = 0;
	float Proportion = 0;
	float b = 0;
	b = PIONTS->Y - PIONTS->K * PIONTS->X;
	cout<<"the K is "<<PIONTS->K<<endl;

	if (PIONTS->K != 10000) //´æÔÚ
	{
		if (PIONTS->K != 0) //Ð±ÂÊ²»Îª0
		{
			error = abs(-PIONTS->K * ROBOT_REAL_X + ROBOT_REAL_Y - b) / sqrt(1 + PIONTS->K * PIONTS->K);
		}
		else
			error = abs(ROBOT_REAL_Y - PIONTS->Y);
	}
	else
	{
		error = abs(ROBOT_REAL_X - PIONTS->X);
	}

	if (PIONTS->K == 10000)
		K_F = 0;
	else if (PIONTS->K == 0)
		K_F = 10000;
	else
		K_F = -1.0f / PIONTS->K;

	T_U_X = k_dirction * Vector_Unitization(PIONTS->K, 0);
	T_U_Y = k_dirction * Vector_Unitization(PIONTS->K, 1);

	if (K_F > 0)
	{
		if (ROBOT_REAL_Y < PIONTS->K * ROBOT_REAL_X + b)
		{
			F_U_X = Vector_Unitization(K_F, 0);
			F_U_Y = Vector_Unitization(K_F, 1);
		}
		else
		{
			F_U_X = -Vector_Unitization(K_F, 0);
			F_U_Y = -Vector_Unitization(K_F, 1);
		}
	}
	else
	{
		if (ROBOT_REAL_Y < PIONTS->K * ROBOT_REAL_X + b)
		{
			F_U_X = -Vector_Unitization(K_F, 0);
			F_U_Y = -Vector_Unitization(K_F, 1);
		}
		else
		{
			F_U_X = Vector_Unitization(K_F, 0);
			F_U_Y = Vector_Unitization(K_F, 1);
		}
	}
	Proportion = PID_Position_Calculate(&TRACK_PID, 0, -error);
	E_U_X = (Proportion * F_U_X + (1 - Proportion) * T_U_X) / sqrt((Proportion * F_U_X + (1 - Proportion) * T_U_X) * (Proportion * F_U_X + (1 - Proportion) * T_U_X) + (Proportion * F_U_Y + (1 - Proportion) * T_U_Y) * (Proportion * F_U_Y + (1 - Proportion) * T_U_Y));
	E_U_Y = (Proportion * F_U_Y + (1 - Proportion) * T_U_Y) / sqrt((Proportion * F_U_X + (1 - Proportion) * T_U_X) * (Proportion * F_U_X + (1 - Proportion) * T_U_X) + (Proportion * F_U_Y + (1 - Proportion) * T_U_Y) * (Proportion * F_U_Y + (1 - Proportion) * T_U_Y));

	World_VY = PIONTS->V * E_U_Y;
	World_VX = PIONTS->V * E_U_X;
	// cout<<"the Proportion:"<<Proportion<<", the error X:"<<error<<endl;
	// cout<<"the E_U_Y speed Y:"<<E_U_Y<<"the E_U_X speed X:"<<E_U_X<<endl;
	// cout<<"the F_U_Y speed Y:"<<F_U_Y<<"the F_U_X speed X:"<<F_U_X<<endl;
	// cout<<"the T_U_Y speed Y:"<<T_U_Y<<"the T_U_X speed X:"<<T_U_X<<endl;
// 	cout<<"the world speed Y:"<<World_VY<<"the world speed X:"<<World_VX<<endl;
}

int M = 0;
void PATH_TRACKING(TRCK_PIONTS *pionts , uint8_t num )
{
	float distence = 0;
	if (Path_Contorl.COUNT >= num )
	{
		Path_Contorl.COUNT = 0;
		World_VY = 0;
	  	World_VX= 0;
		MOVING_STATE = MOVE_ARRIVE;
		M = 0; 
	}
	else
	{
		//获取经过时间
		Path_Contorl.TIME_pass =  ros::Time::now().toNSec()/1e6 - Path_Contorl.TIME_LAST;//这里需要修改为获取系统时间
		if (Path_Contorl.TIME_pass >= pionts[Path_Contorl.COUNT].T)
		{
			//跟踪下一点
			Path_Contorl.COUNT++;
			if (Path_Contorl.COUNT < num)
			{ 
				//切换点后获取时间
				Path_Contorl.TIME_LAST = ros::Time::now().toNSec()/1e6;//这里需要修改为获取系统时间
				Path_Contorl.TIME_pass = 0;
				
				distence = sqrt((ROBOT_REAL_X - pionts[Path_Contorl.COUNT].X) * (ROBOT_REAL_X - pionts[Path_Contorl.COUNT].X) +
								(ROBOT_REAL_Y - pionts[Path_Contorl.COUNT].Y) * (ROBOT_REAL_Y - pionts[Path_Contorl.COUNT].Y));

				//计算当前矢量
				if (pionts[Path_Contorl.COUNT].X - ROBOT_REAL_X > 0)
					k_dirction = 1;
				else if (pionts[Path_Contorl.COUNT].X - ROBOT_REAL_X == 0)
				{
					if (pionts[Path_Contorl.COUNT].Y - ROBOT_REAL_Y > 0)
						k_dirction = 1;
					else
						k_dirction = -1;
				}
				else
					k_dirction = -1;
				
				pionts[Path_Contorl.COUNT].T = distence / pionts[Path_Contorl.COUNT].V;
				expect_angle = pionts[Path_Contorl.COUNT].theate;
				pionts[Path_Contorl.COUNT].K = Caculate_K(ROBOT_REAL_X - pionts[Path_Contorl.COUNT].X, ROBOT_REAL_Y - pionts[Path_Contorl.COUNT].Y);
				printf("%d,%d , %f , %f, %f, %f\r\n", num,Path_Contorl.COUNT, pionts[Path_Contorl.COUNT].X , pionts[Path_Contorl.COUNT].Y , ROBOT_REAL_X , ROBOT_REAL_Y);
			}
		}

		//计算当前速度矢量
		//校正车体位置 
		if (Path_Contorl.COUNT < num )
		{
			POINT_FOLLOW(&pionts[Path_Contorl.COUNT], &pionts[Path_Contorl.COUNT - 1]);
		}
		
	}
	
	//����ƫ����
	
}

/*在进行绕圆弧转弯时候，输入指定角速度和转弯半径*/
/*
 * @brief 计算差速轮绕固定半径圆弧运动的左右轮线速度，正半径+正角速度 → 逆时针左转； 负半径+负角速度 → 顺时针右转
 * @param angular_speed 角速度（单位：度/秒，正值表示朝车头方向）
 * @param radius 圆弧半径（单位：米，正值表示圆心在左侧）
 * @return std::tuple<double, double> 返回分解后的车辆线速度（单位：米/秒）、偏航角速度（单位：弧度/秒）
*/
std::tuple<float, float> calculateWheelSpeeds(float angular_speed, float radius){
	angular_speed = angular_speed/180.0f*M_PI;      //单位切换为弧度/秒
	const float linear_vel = angular_speed * abs(radius);
	const float yaw_vel = (radius>0?abs(angular_speed):-abs(angular_speed));
	return{linear_vel, yaw_vel};
}


void robot_vel::Cal_VEL(void){
	uint8_t SEND_VEL = 1;
	static uint8_t senconds = 0;
	float wheel_x,wheel_y,target_angle,wheel_w = 0;
	float wheel_vel = 0;
	float W_x,W_y = 0;
	double diff = 0;
	double AngleERR = 0;
	geometry_msgs::Twist robot_v;
	geometry_msgs::Twist synergy_V;
	if(MOVING_STATE == MOVE_GOING && TURN_STATE != TURNNING){ // 线段的路径跟踪
		SEND_VEL = 1;
		cout<<"GOING!!"<<endl;
		// W_y = World_VW * Car_L; //角速度解算到车轮的线速度
		wheel_y = World_VY + W_y;
		wheel_x = World_VX;

		wheel_vel = sqrt(wheel_x*wheel_x + wheel_y*wheel_y);
		cout<<"the Y axis speed is:"<<wheel_y<<" ,the X axis speed is:"<<wheel_x<<endl;
		target_angle = atan2(wheel_y, wheel_x)/PI*180.0f-90;
		if(target_angle > 180){
			target_angle -= 360.0;
		}else if(target_angle < -180){
			target_angle += 360.0;
		}
		cout<<"the ROBOT's angle of needing:"<<target_angle<<endl;

		AngleERR = angleDifference(ROBOT_REAL_YAW, target_angle);
		if(abs(AngleERR) > 90.0f){
			wheel_vel = -wheel_vel;
			target_angle = target_angle+180.0;
		}
		diff = angleDifference(ROBOT_REAL_YAW, target_angle);
		cout<<"the angle difference is :"<<diff<<". the target angle is:"<<target_angle<<endl;
		// cout<<"the angle difference is :"<<diff<<endl;
		// wheel_w = PID_Position_Calculate(&TURN_PID, target_angle, ROBOT_REAL_YAW);
		wheel_w = -PID_Position_Calculate(&YAW_PID, 0, diff);
		if(wheel_vel < 0){
			wheel_w = -wheel_w;
		}
		// cout<<"World YAW SPEED :"<<World_VW<<endl;
	}else if(MOVING_STATE != MOVE_GOING && TURN_STATE != TURNNING){
		wheel_vel = 0;
		wheel_w = 0;
	}
	else{ // 原地转弯
		SEND_VEL = 1;
		target_angle = expect_angle;
		wheel_vel = World_VY;
		wheel_w = World_VW;
	}
	
	if((ROBOT_STATE == STARIGHT_PASS || ROBOT_STATE == STARIGHT_PASS1 || ROBOT_STATE == STARIGHT_PASS2) && MOVING_STATE == MOVE_GOING)
	{
		wheel_vel = 0.3f;
		wheel_w = 0.0;
		if(ROBOT_STATE == STARIGHT_PASS1 &&  abs(ROBOT_REAL_Y-straight_start) > 2100){
			wheel_vel = 0.1f;
		}else if(ROBOT_STATE == STARIGHT_PASS2 &&  abs(ROBOT_REAL_Y-straight2_start) > 1000){
			wheel_vel = 0.1f;
		}
	}else if(ROBOT_STATE == FRONT_CLOSING_TO_REAR && MOVING_STATE == MOVE_GOING){
		wheel_vel = -0.3f;
		wheel_w = 0.0;
		if(abs(ROBOT_REAL_Y-back_start) > 1200){
			wheel_vel = -0.1f;
		}
	}else if(ROBOT_STATE == BACK_AWAY_CAR && MOVING_STATE == MOVE_GOING){
		wheel_vel = -0.3f;
		wheel_w = 0.0;
	}
	robot_v.linear.x = wheel_vel;
	robot_v.angular.z = wheel_w;
	
	// cout<<"World YAW SPEED :"<<wheel_w<<endl;
	// cout<<"World liner SPEED :"<<wheel_vel<<endl;
	if(ASSIST_ARM_STATE == OPEN){
		robot_v.linear.y = 1.0;
	}else if(ASSIST_ARM_STATE == CLOSE){
		robot_v.linear.y = -1.0;
	}else{
		robot_v.linear.y = 0.0;
	}
	if(PRIME_ARM_STATE == OPEN){
		robot_v.linear.z = -1.0;
	}else if(PRIME_ARM_STATE == CLOSE){
		robot_v.linear.z = 1.0;
	}else{
		robot_v.linear.z = 0.0;
	}
	if(MOVING_STATE == MOVE_STOP){
		SEND_VEL = 0;
	}


	/*Sending the speed to driver*/
	if (SEND_VEL){

		/*if the model of synergy is open, Sending the contorl for rear module*/
		if(Is_synergy){
			/*
			linear:
				x:liner_vel,
				y:assistant arm
				z:prime arm
			angular:
				x:the radius of turning
				y:IO's model
				z:yaw's speed
			*/
			float turning_w = 0, turning_R = 0, liner_vel = 0, vel_yaw = 0;
			/*sending for rear module speed on moving*/
			if(!Is_SemiSyn){
				ROS_INFO_STREAM(" the robot is in the Syn!!!");
				liner_vel = robot_v.linear.x;
				vel_yaw = robot_v.angular.z*Wheel_L_2_R_DIS/2;//映射到前车驱动轮的轮子线速度
				/*calculate the radius of turning*/
				ROS_INFO_STREAM("the liner's speed:"<<liner_vel<<", the yaw speed:"<<vel_yaw);
				turning_w = ((liner_vel+vel_yaw) - (liner_vel-vel_yaw)) / Wheel_L_2_R_DIS;
				if(turning_w != 0){
					turning_R = abs(((liner_vel+vel_yaw) + (liner_vel-vel_yaw)) /2 /turning_w);
				}else{
					turning_R = 0;
				}
				ROS_INFO_STREAM("the Sunergy yaw's speed:"<<turning_w<<", the yaw's radius:"<<turning_R);
				if(IS_UP_CAR == UPCAR){
					// turning_R = ((turning_R >= 0.01 && turning_R <= 3.5)?3.5:turning_R);
					if(turning_R >= 0.01 && turning_R <= 3.5){
						turning_R = 3.5f;
						float turn_radius = 0, turn_direc = 0;
						turn_direc = ((liner_vel > 0)?abs(turning_w):-abs(turning_w))*180.0/M_PI;
						turn_radius = ((turning_w > 0)?turning_R:-turning_R);
						auto [linear_speed, yaw_vel] = calculateWheelSpeeds(turn_direc, turn_radius);
						robot_v.linear.x = linear_speed;
						liner_vel = linear_speed;
						robot_v.angular.z = yaw_vel;
						ROS_INFO_STREAM("After reCAL the FRONT's linear :"<<linear_speed<<", the yaw :"<<yaw_vel);
					}
				}else{
					// turning_R = ((turning_R >= 0.01 && turning_R <= 1.3)?1.3:turning_R);
					if(turning_R >= 0.01 && turning_R <= 1.3){
						turning_R = 1.3f;
						float turn_radius = 0, turn_direc = 0;
						turn_direc = ((liner_vel > 0)?abs(turning_w):-abs(turning_w))*180.0/M_PI;
						turn_radius = ((turning_w > 0)?turning_R:-turning_R);
						auto [linear_speed, yaw_vel] = calculateWheelSpeeds(turn_direc, turn_radius);
						robot_v.linear.x = linear_speed;
						liner_vel = linear_speed;
						robot_v.angular.z = yaw_vel;
						ROS_INFO_STREAM("After reCAL the FRONT's linear :"<<linear_speed<<", the yaw :"<<yaw_vel);
					}
				}
				
			}else{
				liner_vel = 0.0f;
				turning_R = 0.0f;
				turning_w = 0.0f;
			}
			synergy_V.linear.x = liner_vel;
			synergy_V.angular.z = turning_w;
			synergy_V.angular.x = turning_R;
			
			/*control the rear assistant arm*/
			if(REAR_ASSIST_ARM_STATE == OPEN){
				synergy_V.linear.y = 1.0;
			}else if(REAR_ASSIST_ARM_STATE == CLOSE){
				synergy_V.linear.y = -1.0;
			}else{
				synergy_V.linear.y = 0.0;
			}
			/*control the rear prime arm*/
			if(REAR_PRIME_ARM_STATE == OPEN){
				synergy_V.linear.z = 1.0;
			}else if(REAR_PRIME_ARM_STATE == CLOSE){
				synergy_V.linear.z = -1.0;
			}else{
				synergy_V.linear.z = 0.0;
			}

			/*Sending IO's model*/
			if(REAR_IO_STATE == RECEIVE){
				synergy_V.angular.y = 1;
			}else{
				synergy_V.angular.y = 0;
			}
			ROS_INFO("sending rear :liner SPEED : %f, YAW SPEED : %f, Radius:%f",synergy_V.linear.x, synergy_V.angular.z, synergy_V.angular.x);
			pub_syn_ctrl.publish(synergy_V);
		}

		/*Sending the speed to driver*/
		ROS_INFO("liner SPEED : %f, YAW SPEED : %f, VICE_ARM SPEED : %f, PRIME_ARM SPEED : %f",robot_v.linear.x, robot_v.angular.z, robot_v.linear.y, robot_v.linear.z);
		pub.publish(robot_v);
	}
}


int detecting_oout = 0;
/*状态机相关*/
void Moving_FSM (void){
	// cout<<"!!!RNTRING FSM"<<endl;
	switch (ROBOT_STATE)
	{
		
		case WAITING:
			if(get_start){
				ROBOT_STATE = GOTO_P1;
				// ROBOT_STATE = TURNNING_CIRA1;
				MOVING_STATE = MOVE_GOING;
				TURN_STATE = NO_TURNNING;
				get_start = STOP;
			}
			break;
		/*转弯方式为绕固定圆弧*/
		case GOTO_P1:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CIRA1;
				TURN_STATE = TURNNING;
				TURN_TYPE = TURN;
				TURN_Orientation = LEFT;
			}
			break;
		case TURNNING_CIRA1:
		{
			// detecting_oout++;
			// if(detecting_oout > 40){
			// 	ROBOT_REAL_YAW = 90.0;
				if(TURN_STATE == TURNNED){
					ROBOT_STATE = GOTO_P2;
					MOVING_STATE = MOVE_GOING;
					// ROBOT_REAL_Y = 6000;
					// ROBOT_REAL_X = -4000;
					// detecting_oout = 0;
				}
			// }
		}break;
		case GOTO_P2:{
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CIRA2;
				TURN_STATE = TURNNING;
				TURN_TYPE = TURN;
				TURN_Orientation = LEFT;
			}
		}break;
		case TURNNING_CIRA2:
		{
			// detecting_oout++;
			// if(detecting_oout > 40){
			// 	ROBOT_REAL_YAW = 180.0;
				if(TURN_STATE == TURNNED){
					if(IS_UP_CAR){
						ROBOT_STATE = GOTO_P3;
						MOVING_STATE = MOVE_GOING;
					}else{
						ROBOT_STATE = WAITING;
						MOVING_STATE = MOVE_STOP;
					}
					/*PATH TEST*/
					// ROBOT_STATE = GOTO_P3;
					// MOVING_STATE = MOVE_GOING;
					// ROBOT_REAL_Y = 2000;
					// ROBOT_REAL_X = -17000;
					// detecting_oout = 0;
				}
			// }
		}break;
		case GOTO_P3:{
			if(MOVING_STATE == MOVE_ARRIVE){
				// detecting_oout++;
				// if(detecting_oout > 60){
					ROBOT_STATE = STOPPING;
					MOVING_STATE = MOVE_STOP;
					// ROBOT_REAL_Y = 0;
					// ROBOT_REAL_X = -17000;
				// 	detecting_oout = 0;
				// }
			}
		}break;
		case GOTO_P4:{
			
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CIRA3;
				TURN_STATE = TURNNING;
				TURN_TYPE = TURN;
				TURN_Orientation = LEFT;
			}
		}break;
		case TURNNING_CIRA3:
		{
			// detecting_oout++;
			// if(detecting_oout > 40){
			// 	ROBOT_REAL_YAW = 90.0;
				if(TURN_STATE == TURNNED){
					ROBOT_STATE = GOTO_P5;
					MOVING_STATE = MOVE_GOING;
					// ROBOT_REAL_Y = 6000;
					// ROBOT_REAL_X = -10000;
					// detecting_oout = 0;
				}
			// }
		}break;
		case GOTO_P5:{
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CIRA4;
				TURN_STATE = TURNNING;
				TURN_TYPE = TURN;
				TURN_Orientation = LEFT;
			}
		}break;
		case TURNNING_CIRA4:
		{
			// detecting_oout++;
			// if(detecting_oout > 40){
			// 	ROBOT_REAL_YAW = 0.0;
				if(TURN_STATE == TURNNED){
					ROBOT_STATE = GOTO_P6;
					MOVING_STATE = MOVE_GOING;
					// ROBOT_REAL_Y = 2000;
					// ROBOT_REAL_X = 0;
					// detecting_oout = 0;
				}
			// }
		}break;
		case GOTO_P6:{
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = STOPPING;
				MOVING_STATE = MOVE_STOP;
			}
		}break;
		/*转弯方式为跟踪路径点*/
		// case GOTO_P1:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURN_CIRA1;
		// 		TURN_STATE = NO_TURNNING;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case TURN_CIRA1:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = GOTO_P2;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case GOTO_P2:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURN_CIRA2;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case TURN_CIRA2:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = GOTO_P3;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		
		// case GOTO_P3:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURN_CIRA3;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case TURN_CIRA3:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = GOTO_P4;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case GOTO_P4:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = STOPPING;
		// 		MOVING_STATE = MOVE_STOP;
		// 	}
		// 	break;

		/*
		dectecting car postion for moving to the front of car
		*/
	/*临时关闭*/
		// case DETECTING_WHEEL:
		// 	if(DETECTING_STATE == GET_CAR_POINT){
		// 		DETECTING_STATE = NO_CAR_POINT;
		// 		MOVING_STATE = MOVE_GOING;
		// 		ROBOT_STATE = GOTO_CAR;
		// 		TURN_STATE = NO_TURNNING;
		// 	}
		// 	break;
		/*for test together module lidar in the first stage*/
		case DETECTING_WHEEL:
			if(DETECTING_STATE == GET_CAR_POINT){
				DETECTING_STATE = NO_CAR_POINT;
				MOVING_STATE = MOVE_ARRIVE;
				ROBOT_STATE = TURNNING_GOTO_CAR;
				TURN_STATE = TURNNING;
				TURN_TYPE = ROTATION;
				CTRL_FLAG = CONTION_STOPA;
				SEND_flag = SEND;
			}
			break;
		case TURNNING_GOTO_CAR:
			// TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				detecting_oout++;
				if(detecting_oout > 30){
					ROBOT_STATE = GOTO_CAR;
					MOVING_STATE = MOVE_GOING;
					detecting_oout=0;
				}
			}
			break;
		case GOTO_CAR:
			// MOVING_STATE = MOVE_ARRIVE;
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_FACE_CAR;
				TURN_STATE = TURNNING;
				TURN_TYPE = ROTATION;
			}
			break;
		case TURNNING_FACE_CAR:
			// TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				detecting_oout++;
				if(detecting_oout > 30){
					/*临时关闭*/
					IS_REC = 1;
					ROBOT_STATE = DETECTING_WHEEL_PASS;
					MOVING_STATE = MOVE_STOP;
					CTRL_FLAG = CONTION_START_CLUSTER;
					SEND_flag = SEND;
					/*for test together module lidar in the first stage*/
					// IS_REC = 0;
					// ROBOT_STATE = STOPPING;
					// MOVING_STATE = MOVE_STOP;
					// CTRL_FLAG = CONTION_STOP;
					// SEND_flag = SEND;
					detecting_oout = 0;
				}
			}
			break;

		/*
		the second time of detecting the car ,for starightly pass the car
		*/
		#if BEZIER
		case DETECTING_WHEEL_PASS:
			IS_REC = 1;
			if(DETECTING_STATE == GET_CAR_POINT){
				IS_REC = 0;
				DETECTING_STATE = NO_CAR_POINT;
				MOVING_STATE = MOVE_GOING;
				ROBOT_STATE = CLOSING_TO_CAR;
				TURN_STATE = NO_TURNNING;
			}else if(DETECTING_STATE == ARRIVIALED_CAR){
				IS_REC = 0;
				ROBOT_STATE = PLAN_STARIGHT; // PLAN_STARIGHT: single front's, PLAN_STARIGHT_READY: couple car
				MOVING_STATE = MOVE_ARRIVE;
				TURN_STATE = NO_TURNNING;
				
			}
			break;
		#else
		case DETECTING_WHEEL_PASS:{
			IS_REC = 1;
			if(DETECTING_STATE == GET_CAR_POINT){
				IS_REC = 0;
				DETECTING_STATE = NO_CAR_POINT;
				MOVING_STATE = MOVE_ARRIVE;
				ROBOT_STATE = TURNNING_PATH;
				TURN_STATE = TURNNING;
				TURN_TYPE = ROTATION;
				CTRL_FLAG = CONTION_STOPB;
				SEND_flag = SEND;
			}else if(DETECTING_STATE == ARRIVIALED_CAR){
				IS_REC = 0;
				// ROBOT_STATE = PLAN_STARIGHT; // PLAN_STARIGHT: single front's, PLAN_STARIGHT_READY: couple car
				// MOVING_STATE = MOVE_ARRIVE;
				TURN_STATE = NO_TURNNING;
				ROBOT_STATE = STOPPING; // PLAN_STARIGHT: single front's, PLAN_STARIGHT_READY: couple car
				MOVING_STATE = MOVE_STOP;
				
			}
		}break;
		case TURNNING_PATH:{
			// TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				detecting_oout++;
				if(detecting_oout > 30){
					ROBOT_STATE = CLOSING_TO_CAR;
					MOVING_STATE = MOVE_GOING;
					detecting_oout=0;
				}
			}
		}break;
		#endif
		case CLOSING_TO_CAR:{
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CLOSE_CAR;
				TURN_STATE = TURNNING;
				TURN_TYPE = ROTATION;
				DETECTING_STATE = NO_CAR_POINT;
			}
		}break;
		case TURNNING_CLOSE_CAR:
			// TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				detecting_oout++;
				if(detecting_oout > 30){
					ROBOT_STATE = DETECTING_WHEEL_PASS;
					MOVING_STATE = MOVE_ARRIVE;
					CTRL_FLAG = CONTION_START_CLUSTER;
					SEND_flag = SEND;
					detecting_oout=0;
				}
			}
			break;
		/* Has been closed the car,the agv can starightly pass through the car*/
		/*Judging by the patterns used*/
		if(!Is_synergy){
			/* the singal car status (only the front module)*/
			case PLAN_STARIGHT:
			{
				ASSIST_ARM_STATE = OPEN;
				detecting_oout++;
				if(PLAN_STATE == PLANNED){
					ROS_INFO("the agv has been planned the road,now starting to pass the car");
					ROBOT_STATE = STARIGHT_PASS;
					MOVING_STATE = MOVE_GOING;
					// Ser_IO_STATE = READ_IO;
				}
			}break;
			case STARIGHT_PASS:{
				if(PASS_WHEEL == STOPING){
					ROS_INFO("the agv has been touched the wheel of car!!!");
					// PASS_WHEEL = MOVING;
					ROBOT_STATE = STOPPING;
					MOVING_STATE = MOVE_STOP;
					// Ser_IO_STATE = WRITE_IO;
				}
			}break;
		}else{
			/*the couple status(the connected with front and rear)*/
			case PLAN_STARIGHT_READY:
			{
				REAR_ASSIST_ARM_STATE = OPEN;
				REAR_IO_STATE = NOT_RECEIVE;
				detecting_oout++;
				if(PLAN_STATE == PLANNED){
					ROS_INFO("the agv has been planned the road,now starting to pass the car");
					ROBOT_STATE = STARIGHT_PASS1;
					MOVING_STATE = MOVE_GOING;
					REAR_IO_STATE = RECEIVE;
					REAR_ASSIST_ARM_STATE = NOT;
					detecting_oout = 0;
				}
			}break;
			case STARIGHT_PASS1:
			{
				ROS_INFO("the REAR PASS WHEEL's state:%s",REAR_PASS_WHEEL?"STOPING":"MOVING");
				if(REAR_PASS_WHEEL == STOPING){
					ROS_INFO("the rear module has been touched the wheel of car!!!");
					ROBOT_STATE = REAR_TOUCH_WHEEL;
					MOVING_STATE = MOVE_ARRIVE;
					detecting_oout = 0;
				}
			}break;
			case REAR_TOUCH_WHEEL:
			{
				detecting_oout++;
				if(detecting_oout >= 20 ){
					Is_SemiSyn = true;
					REAR_IO_STATE = NOT_RECEIVE;
					REAR_PRIME_ARM_STATE = OPEN;
				}
				if(detecting_oout >= 60){
					straight2_start = ROBOT_REAL_Y;
					ROS_INFO("The straight 2 starting location is:%f", straight2_start);
					MOVING_STATE = MOVE_GOING;
					// Ser_IO_STATE = RECEIVE;
					ROBOT_STATE = STARIGHT_PASS2;
					ASSIST_ARM_STATE = OPEN;
					detecting_oout = 0;
				}
			}break;
			case STARIGHT_PASS2:
			{
				// ROS_INFO("the PASS WHEEL's state:%s",PASS_WHEEL?"STOPING":"MOVING");
				if(PASS_WHEEL == STOPING){
					ROS_INFO("the agv has been touched the wheel of car!!!");
					ASSIST_ARM_STATE = NOT;
					// Ser_IO_STATE = NOT_RECEIVE;
					ROBOT_STATE = TAKING_UP_THE_CAR;
					MOVING_STATE = MOVE_ARRIVE;
					PRIME_ARM_STATE = OPEN;
				}
			}break;
			case TAKING_UP_THE_CAR:
			{
				if(Is_PRIME_ARM_ARRIVED){
					detecting_oout++;
					// ROBOT_STATE = STOPPING;
					// MOVING_STATE = MOVE_STOP;
					PRIME_ARM_STATE = NOT;
					REAR_PRIME_ARM_STATE = NOT;
					Is_SemiSyn = false;
					IS_UP_CAR = UPCAR;
					if(detecting_oout > 60){
						if(ROBOT_REAL_X < -10000){
							PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH4_UPCAR) / sizeof(GOTO_PATH4_UPCAR[0]));
							ROS_INFO_STREAM("the out path4 of UPCAR NUM:"<<PATH_POINT_NUM);
							PlanLeftPath(GOTO_PATH4_UPCAR, PATH_POINT_NUM);
							ROBOT_STATE = GOTO_P4;
							MOVING_STATE = MOVE_GOING;
							detecting_oout = 0;
						}else{
							PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH1_UPCAR) / sizeof(GOTO_PATH1_UPCAR[0]));
							ROS_INFO_STREAM("the out path1 of UPCAR NUM:"<<PATH_POINT_NUM);
							PlanLeftPath(GOTO_PATH1_UPCAR, PATH_POINT_NUM);
							ROBOT_STATE = GOTO_P1;
							MOVING_STATE = MOVE_GOING;
							detecting_oout = 0;
						}
						
					}
				}
			}break;
		}
		case FRONT_TAKING_DOWN_THE_CAR:
		{
			PRIME_ARM_STATE = CLOSE;
			back_start = ROBOT_REAL_Y;
			if(Is_PRIME_ARM_CLOSE){
				Is_SemiSyn = true;
				PRIME_ARM_STATE = NOT;
				MOVING_STATE = MOVE_GOING;
				ROBOT_STATE = FRONT_CLOSING_TO_REAR;
			}
		}break;
		case FRONT_CLOSING_TO_REAR:
		{
			ASSIST_ARM_STATE = CLOSE;
			if(CLOSING_REAR == STOPING){
				ASSIST_ARM_STATE = NOT;
				MOVING_STATE = MOVE_ARRIVE;
				REAR_PRIME_ARM_STATE = CLOSE;
				ROBOT_STATE = REAR_TAKING_DOWN_THE_CAR;
				REAR_Is_PRIME_ARM_CLOSE = false;
				detecting_oout = 0;
			}
		}break;
		case REAR_TAKING_DOWN_THE_CAR:{
			if(REAR_Is_PRIME_ARM_CLOSE){
				REAR_PRIME_ARM_STATE = NOT;
				detecting_oout++;
				Is_SemiSyn = false;
				if(detecting_oout>=40){
					MOVING_STATE = MOVE_GOING;
					ROBOT_STATE = BACK_AWAY_CAR;
					detecting_oout = 0;
				}
				
			}
		}break;
		case BACK_AWAY_CAR:{
			detecting_oout++;
			if(detecting_oout >= 20){
				REAR_ASSIST_ARM_STATE = CLOSE;
			}
			if(detecting_oout >= 40){
				ROBOT_STATE = WAITING;
				MOVING_STATE = MOVE_STOP;
				REAR_ASSIST_ARM_STATE = NOT;
				IS_UP_CAR = NOT_UPCAR;
				detecting_oout = 0;
				if(ROBOT_REAL_X < -10000){
					PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH4) / sizeof(GOTO_PATH4[0]));
					ROS_INFO_STREAM("the out path4 NUM:"<<PATH_POINT_NUM);
					PlanLeftPath(GOTO_PATH4, PATH_POINT_NUM);
					ROBOT_STATE = GOTO_P4;
					MOVING_STATE = MOVE_GOING;
				}
			}
		}
		default:
		break;
	}
}

void Moving (void)
{
	if (MOVING_STATE == MOVE_GOING)
	{
		switch (ROBOT_STATE)
		{
						
		case GOTO_P1:
		{
			if(IS_UP_CAR == UPCAR){
				NOW_PATH = GOTO_PATH1_UPCAR;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH1_UPCAR) / sizeof(GOTO_PATH1_UPCAR[0]));
			}else{
				NOW_PATH = GOTO_PATH1;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH1) / sizeof(GOTO_PATH1[0]));
			}
			
		}
		break;
		case TURN_CIRA1:
		{
			NOW_PATH = GOTO_CIRA1;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_CIRA1) / sizeof(GOTO_CIRA1[0]));
		}
		break;
		case GOTO_P2:
		{
			if(IS_UP_CAR == UPCAR){
				NOW_PATH = GOTO_PATH2_UPCAR;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH2_UPCAR) / sizeof(GOTO_PATH2_UPCAR[0]));
			}else{
				NOW_PATH = GOTO_PATH2;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH2) / sizeof(GOTO_PATH2[0]));
			}
		}
		break;
		case TURN_CIRA2:
		{
			NOW_PATH = GOTO_CIRA2;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_CIRA2) / sizeof(GOTO_CIRA2[0]));
		}
		break;
		case GOTO_P3:
		{
			// if(IS_UP_CAR == UPCAR){
			// 	NOW_PATH = GOTO_PATH3_UPCAR;
			// 	PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH3_UPCAR) / sizeof(GOTO_PATH3_UPCAR[0]));
			// }else{
				NOW_PATH = GOTO_PATH3;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH3) / sizeof(GOTO_PATH3[0]));
			// }
		}
		break;
		case TURN_CIRA3:
		{
			NOW_PATH = GOTO_CIRA3;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_CIRA3) / sizeof(GOTO_CIRA3[0]));
		}
		break;
		case GOTO_P4:
		{
			if(IS_UP_CAR == UPCAR){
				NOW_PATH = GOTO_PATH4_UPCAR;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH4_UPCAR) / sizeof(GOTO_PATH4_UPCAR[0]));
			}else{
				NOW_PATH = GOTO_PATH4; 
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH4) / sizeof(GOTO_PATH4[0]));
			}
		}
		break;
		case GOTO_P5:
		{
			if(IS_UP_CAR == UPCAR){
				NOW_PATH = GOTO_PATH5_UPCAR;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH5_UPCAR) / sizeof(GOTO_PATH5_UPCAR[0]));
			}else{
				NOW_PATH = GOTO_PATH5; 
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH5) / sizeof(GOTO_PATH5[0]));
			}
		}
		break;
		case GOTO_P6:
		{
			if(IS_UP_CAR == UPCAR){
				NOW_PATH = GOTO_PATH6_UPCAR;
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH6_UPCAR) / sizeof(GOTO_PATH6_UPCAR[0]));
			}else{
				NOW_PATH = GOTO_PATH6; 
				PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH6) / sizeof(GOTO_PATH6[0]));
			}
		}
		break;
		case GOTO_CAR:
		{
			NOW_PATH = Car_location; 
			PATH_POINT_NUM = (uint32_t)(sizeof(Car_location) / sizeof(Car_location[0]));
		}
		break;
		case CLOSING_TO_CAR:
		{
			NOW_PATH = Pass_location; 
			PATH_POINT_NUM = (uint32_t)(sizeof(Pass_location) / sizeof(Pass_location[0]));
		}
		break;
		case STARIGHT_PASS:
		{
			// if(ROBOT_REAL_Y - straight_start > 1000){
			// 	MOVING_STATE = MOVE_ARRIVE;
			// 	ROS_INFO("The straight ending location is:%f", ROBOT_REAL_Y);
			// 	return;
			// }else{
			// 	return;
			// }
		}
		break;
					
		default:
		{
			NOW_PATH = NULL;
			return;
		}
		break;
		}
		PATH_TRACKING(NOW_PATH, PATH_POINT_NUM);		
	}

}

void Planning(void){
	if(ROBOT_STATE == PLAN_STARIGHT && PLAN_STATE == NO_PALNNING){
		Straight_pass[0].X = ROBOT_REAL_X;
		Straight_pass[0].Y = ROBOT_REAL_Y;
		for(uint8_t i = 1; i <= 4; i++){
			Car_location[i].X = ROBOT_REAL_X;
			Car_location[i].Y = ROBOT_REAL_Y + 250*i;
		}
	}
}

void TURNNING_move(void)
{
	double diff = 0;
	diff = angleDifference(ROBOT_REAL_YAW, expect_angle);
	if ( abs(diff) < 0.5f)
	{
		cout<<"ROBOT's task of turnning has been finished!!!"<<endl;
		World_VW = 0;
		TURN_STATE = TURNNED;
	}else
	{
		if(TURN_TYPE == ROTATION){
			cout<<"ROBOT is executing the task og turnning!!!"<<endl;
			cout<<"the angle difference is :"<<diff<<". the target angle is:"<<expect_angle<<endl;
			// World_VW = PID_Position_Calculate(&YAW_PID, expect_angle, ROBOT_REAL_YAW);
			World_VW = -PID_Position_Calculate(&YAW_PID, 0, diff);
			World_VY = 0;
			cout<<"World YAW SPEED :"<<World_VW<<endl;
		}else if(TURN_TYPE == TURN){
			float turn_yaw = 0,turn_radius = 0;
			if(diff < 0){
				turn_yaw = -TURN_YAW;
			}else{
				turn_yaw = TURN_YAW;
			}
			if(IS_UP_CAR == UPCAR){
				turn_radius = TURN_RADIUS1;
			}else{
				turn_yaw = turn_yaw*2;
				turn_radius = TURN_RADIUS2;
			}
			if(TURN_Orientation == LEFT){
				turn_radius = turn_radius;
			}else if(TURN_Orientation == RIGHT){
				turn_radius = -turn_radius;
			}
			auto [linear_vel, yaw_vel] = calculateWheelSpeeds(turn_yaw, turn_radius);
			World_VY = linear_vel;
			World_VW = yaw_vel;
			// ROS_INFO("the turnning disassemble is linear vel:%.2f,yaw vel:%.2f",World_VY, World_VW);
		}
		
	}
}

void Turnning (void){
	if (TURN_STATE == TURNNING){
		
		switch(ROBOT_STATE)
		{
			case TURNNING_CIRA1:
			{
				if(IS_UP_CAR){
					expect_angle = 92.0f;
				}else{
					expect_angle = 88.0f;
				}
			}break;
			case TURNNING_CIRA2:
			{
				expect_angle = 178.0f;
			}break;
			case TURNNING_CIRA3:
			{
				expect_angle = 92.0f;
			}break;
			case TURNNING_CIRA4:
			{
				// if(IS_UP_CAR){
				// 	expect_angle = 178.0f;
				// }else{
				// 	expect_angle = 2.0f;
				// }
				expect_angle = 2.0f;
			}break;
			case TURNNING_GOTO_CAR:
			{
				expect_angle = car_angle;
			}break;
			case TURNNING_FACE_CAR:
			{
				expect_angle = face_car_angle;
			}break;

			case TURNNING_PATH:
			{
				expect_angle = car_angle;
			}break;
			case TURNNING_CLOSE_CAR:
			{
				expect_angle = face_car_angle;
			}break;
		}
		TURNNING_move();

	}
}

// void Sending_IO(void){
// 	std_msgs::Bool state_pub;
// 	if(Ser_IO_STATE == NOT_RECEIVE){
// 		state_pub.data = WRITE;
// 	}else if(Ser_IO_STATE == RECEIVE){
// 		state_pub.data = READ;
// 	}

// 	IO_state_sub.publish(state_pub);
// }


/*里程计获取全场定位*/
float POS_X,POS_Y,Last_POS_X,Last_POS_Y,REAL_Y,REAL_X = 0;
bool first_rec = true;
void AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& Odo_msg)
{
	Quaternion quaternion;
	EulerAngles angles;
	float AGV_pos_X, AGV_pos_Y, AGV_pos_Z;
	float DETLA_X,DETLA_Y = 0;
	if(first_rec){
		Last_POS_X = Odo_msg->pose.pose.position.x;
		Last_POS_Y = Odo_msg->pose.pose.position.y;
		first_rec = false;
	}else{
		Last_POS_X = POS_X;
		Last_POS_Y = POS_Y;
	}

	quaternion.x = Odo_msg->pose.pose.orientation.x;
	quaternion.y = Odo_msg->pose.pose.orientation.y;
	quaternion.z = Odo_msg->pose.pose.orientation.z;
	quaternion.w = Odo_msg->pose.pose.orientation.w;
	POS_X = Odo_msg->pose.pose.position.x;
	POS_Y = Odo_msg->pose.pose.position.y;
	AGV_pos_Z = Odo_msg->pose.pose.position.z;
	angles = ToEulerAngles(quaternion);
	ROBOT_REAL_YAW = angles.yaw/PI*180;

	DETLA_X = POS_X - Last_POS_X;
	DETLA_Y = POS_Y - Last_POS_Y;
	REAL_X += DETLA_X;
	REAL_Y += DETLA_Y;

	// REAL_X = REAL_X*1000;
	// REAL_Y = REAL_Y*1000;
	cout<<"the AGV now orientation yaw is: "<<ROBOT_REAL_YAW<<endl;
	// cout<<"the AGV now orientation yaw is: "<<angles.yaw/PI*180<<"; orientation pitch"<<angles.pitch/PI*180<<"; orientation roll"<<angles.roll/PI*180<<";"<<endl;
	
	/*这里需要进一步转化为底盘两轮中间*/
	ROBOT_REAL_X = (REAL_X + BASE2LIDAR * sin(ROBOT_REAL_YAW/180*PI))*1000;
	ROBOT_REAL_Y = (REAL_Y - BASE2LIDAR * cos(ROBOT_REAL_YAW/180*PI)+BASE2LIDAR)*1000;
	// cout<<"the AGV now postion is "<<REAL_X<<"mm; "<<REAL_Y<<"mm; "<<endl;
	cout<<"the AGV now postion X is "<<ROBOT_REAL_X<<"mm; Y is"<<ROBOT_REAL_Y<<"mm; "<<AGV_pos_Z<<"mm"<<endl;
}

/*修改前，版本1.0*/
void RecCarPointCallback(const geometry_msgs::Point::ConstPtr& car_position){

	if(IS_REC){
		/*因为car_position是相对于雷达的位置信息，因此需要转换到相对于车体中央的位置，所以选择使用从雷达位置转换到车体位置的转换公式*/
		face_car_angle = car_position->z + ROBOT_REAL_YAW;
		double car_X1 = (car_position->x*cos(ROBOT_REAL_YAW/180.0*M_PI)) + (car_position->y*-sin(ROBOT_REAL_YAW/180.0*M_PI));
		double car_Y1 = (car_position->x*sin(ROBOT_REAL_YAW/180.0*M_PI)) + (car_position->y*cos(ROBOT_REAL_YAW/180.0*M_PI));
		car_X1 = (car_X1- sin((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR);
		car_Y1 = (car_Y1+ cos((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR);
		double dx = (car_X1 + sin((face_car_angle)/180.0*M_PI)*BASE2LIDAR)/4*1000;
		double dy = (car_Y1 - cos((face_car_angle)/180.0*M_PI)*BASE2LIDAR)/4*1000;
		Car_location[0].X = ROBOT_REAL_X;
		Car_location[0].Y = ROBOT_REAL_Y;
		for(uint8_t i = 1; i <= 4; i++){
			Car_location[i].X = ROBOT_REAL_X + dx*i;
			Car_location[i].Y = ROBOT_REAL_Y + dy*i;
			ROS_INFO_STREAM("the X:"<<Car_location[i].X<<", the Y:"<<Car_location[i].Y);
		}
		double theta = atan2(dy, dx)/PI*180.0 - 90;
		car_angle = theta;

		DETECTING_STATE = GET_CAR_POINT;
		ROS_INFO("Received the wheel info, x:%f, y:%f, angle:%f,!!!",car_position->x, car_position->y, car_position->z);
		ROS_INFO("Tranformed TF1 position info, x:%f, y:%f,!!!",dx*4, dy*4);
		ROS_INFO("the car current postion X:%f, Y: %f, the angle:%f", ROBOT_REAL_X,ROBOT_REAL_Y, face_car_angle);
		ROS_INFO("the car target postion X:%f, Y: %f", Car_location[4].X,Car_location[4].Y);
		IS_REC = 0;
	}
}

/*修改后，版本2.0，使用贝塞尔曲线生成路径*/
// 定义贝塞尔曲线的插值计算函数
void bezier_interpolation(const std::tuple<double, double, double>& p0,
                          const std::tuple<double, double, double>& p1,
                          const std::tuple<double, double, double>& p2,
                          const std::tuple<double, double, double>& p3,
                          double t, double& x, double& y, double& theta) {
    double u = 1.0 - t;
    x = u*u*u * std::get<0>(p0) + 3 * u*u * t * std::get<0>(p1) + 3 * u * t*t * std::get<0>(p2) + t*t*t * std::get<0>(p3);
    y = u*u*u * std::get<1>(p0) + 3 * u*u * t * std::get<1>(p1) + 3 * u * t*t * std::get<1>(p2) + t*t*t * std::get<1>(p3);
    theta = u*u*u * std::get<2>(p0) + 3 * u*u * t * std::get<2>(p1) + 3 * u * t*t * std::get<2>(p2) + t*t*t * std::get<2>(p3);
}
// 计算控制点的函数
void calculate_control_points(const std::tuple<double, double, double>& start,
                              const std::tuple<double, double, double>& end,
                              double control_point_factor, 
                              std::tuple<double, double, double>& p1,
                              std::tuple<double, double, double>& p2) {
    // 获取起点和终点的角度 (theta)
    double theta0 = std::get<2>(start);
    double theta3 = std::get<2>(end);
    
    // 计算起点到终点的欧几里得距离
    double dist = std::sqrt(std::pow(std::get<0>(end) - std::get<0>(start), 2) + std::pow(std::get<1>(end) - std::get<1>(start), 2));
    
    // 计算控制点1：沿起点的切线方向
    p1 = std::make_tuple(std::get<0>(start) + control_point_factor * dist * std::cos(theta0),
                        std::get<1>(start) + control_point_factor * dist * std::sin(theta0),
                        theta0);
    
    // 计算控制点2：沿终点的切线方向
    p2 = std::make_tuple(std::get<0>(end) - control_point_factor * dist * std::cos(theta3),
                        std::get<1>(end) - control_point_factor * dist * std::sin(theta3),
                        theta3);
}

int times = 0;
void RecPassPointCallback(const geometry_msgs::Point::ConstPtr& car_position){

	if(IS_REC){
		ROS_INFO("the entry function's times is:%d",times);
		if(times > 1){
			ROS_INFO("Received the wheel info, x:%f, y:%f, angle:%f,!!!",car_position->x, car_position->y, car_position->z);
			if(car_position->x == 0.0 && car_position->y == 0.0 && car_position->z ==0.0){
				CTRL_FLAG = CONTION_STOPB;
				SEND_flag = SEND;
				ROS_INFO("The ROBOT has been close LIDAR_DECT!!!START STRAIGHT PASSING!!!");
				DETECTING_STATE = ARRIVIALED_CAR;
				IS_REC = 0;
				times = 0;
			}else{
				Pass_location[0].X = ROBOT_REAL_X;
				Pass_location[0].Y = ROBOT_REAL_Y;
				/*以下为采用贝塞尔曲线规划的方法*/
				#if BEZIER
				// 定义起点和终点
				double start_theta = (ROBOT_REAL_YAW+90)/180.0*M_PI;
				double end_theta = (car_position->z+90)/180.0*M_PI;
				/*因为car_position是相对于雷达的位置信息，因此需要转换到相对于车体中央的位置，所以选择使用从雷达位置转换到车体位置的转换公式*/
				double end_X = (car_position->x + sin((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR)*1000 + ROBOT_REAL_X;
				double end_Y = (car_position->y - cos((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR + BASE2LIDAR)*1000 + ROBOT_REAL_Y;
				cout<<" the init target position X:"<<car_position->x*1000<<" ,Y:"<<car_position->y*1000<<endl;
				cout<<" the calculation target position X:"<<end_X<<" ,Y:"<<end_Y<<endl;
				std::tuple<double, double, double> start = std::make_tuple(ROBOT_REAL_X, ROBOT_REAL_Y, start_theta);  // 起点 (x, y, theta)
				std::tuple<double, double, double> end = std::make_tuple(end_X, end_Y, end_theta);  // 终点 (x, y, theta)

				// 计算控制点
				std::tuple<double, double, double> p1, p2;
				calculate_control_points(start, end, 0.4, p1, p2);

				// 生成轨迹
				int num_points = 5;
				std::vector<std::tuple<double, double, double>> path;
				
				for (int i = 0; i < num_points; ++i) {
					double t = static_cast<double>(i) / (num_points - 1);
					double x, y, theta;
					bezier_interpolation(start, p1, p2, end, t, x, y, theta);
					path.push_back(std::make_tuple(x, y, theta/M_PI*180.0));
				}

				// 输出路径
				uint8_t i = 0;
				for (const auto& point : path) {
					Pass_location[i].X = std::get<0>(point);
					Pass_location[i].Y = std::get<1>(point);
					// std::cout << "x: " << std::fixed << std::setprecision(2) << std::get<0>(point)
					// 		<< ", y: " << std::fixed << std::setprecision(2) << std::get<1>(point)
					// 		<< ", theta: " << std::fixed << std::setprecision(2) << std::get<2>(point) << std::endl;
					std::cout << "x: " << Pass_location[i].X
							<< ", y: " << Pass_location[i].Y << std::endl;
					i++;
				}
				#else
				/*以下是直接规划直线*/
				face_car_angle = car_position->z + ROBOT_REAL_YAW;
				double car_X1 = (car_position->x*cos(ROBOT_REAL_YAW/180.0*M_PI)) + (car_position->y*-sin(ROBOT_REAL_YAW/180.0*M_PI));
				double car_Y1 = (car_position->x*sin(ROBOT_REAL_YAW/180.0*M_PI)) + (car_position->y*cos(ROBOT_REAL_YAW/180.0*M_PI));
				car_X1 = (car_X1- sin((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR);
				car_Y1 = (car_Y1+ cos((ROBOT_REAL_YAW)/180.0*M_PI)*BASE2LIDAR);
				double dx = (car_X1 + sin((face_car_angle)/180.0*M_PI)*BASE2LIDAR)/2*1000;
				double dy = (car_Y1 - cos((face_car_angle)/180.0*M_PI)*BASE2LIDAR)/2*1000;
				double theta = atan2(dy, dx)/PI*180.0 - 90;
				car_angle = theta;
				Pass_location[0].X = ROBOT_REAL_X;
				Pass_location[0].Y = ROBOT_REAL_Y;
				for(uint8_t i = 1; i <= 2; i++){
					Pass_location[i].X = ROBOT_REAL_X + dx*i;
					Pass_location[i].Y = ROBOT_REAL_Y + dy*i;
					ROS_INFO_STREAM("the X:"<<Pass_location[i].X<<", the Y:"<<Pass_location[i].Y);
				}
				ROS_INFO("Tranformed TF1 position info, x:%f, y:%f,!!!",dx*2, dy*2);
				ROS_INFO("the car current postion X:%f, Y: %f, the angle:%f", ROBOT_REAL_X,ROBOT_REAL_Y, face_car_angle);
				ROS_INFO("the car target postion X:%f, Y: %f", Pass_location[2].X,Pass_location[2].Y);
				#endif

				DETECTING_STATE = GET_CAR_POINT;
				IS_REC = 0;
				times = 0;
			}
		}else{
			times++;
		}
		
	}
	
}

void PlanLeftPath(TRCK_PIONTS *PATH ,uint num){
	
	float dy = (2000 - ROBOT_REAL_Y)/(num-3);
	for(int i=0; i<num; i++){
		PATH[i].X = ROBOT_REAL_X;
		if(i < (num-3)){
			PATH[i].Y = ROBOT_REAL_Y + dy*i;
		}
		ROS_INFO_STREAM("the out path X:"<<PATH[i].X<<"the out path Y:"<<PATH[i].Y);
	}
}

void ArmTouchCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == TOUCH){
		ROS_INFO("The ROBOT has been touched the car!!!");
		PASS_WHEEL = STOPING;
	}else if(msg->data == NOT_TOUCH){
		ROS_INFO("The ROBOT has not been touched the car,countiuing close!!!");
		PASS_WHEEL = MOVING;
	}
}
void RearArmTouchCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == TOUCH){
		ROS_INFO("The REAR_ROBOT has been touched the car!!!");
		REAR_PASS_WHEEL = STOPING;
	}else if(msg->data == NOT_TOUCH){
		ROS_INFO("The REAR_ROBOT has not been touched the car,countiuing close!!!");
		REAR_PASS_WHEEL = MOVING;
	}
}

void FrontTouchRearCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == TOUCH){
		ROS_INFO("The FRONT has been touched the REAR!!!");
		CLOSING_REAR = STOPING;
	}else if(msg->data == NOT_TOUCH){
		ROS_INFO("The FRONT has not been touched the REAR,countiuing closing!!!");
		CLOSING_REAR = MOVING;
	}
}
bool key_state = false; // true:One_Key_UPCAR, false:One_Key_LIDAR_DECT
void StartCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		if(key_state){
			ROS_INFO("Has been recived the commend of One_Key_UPCAR!!!");
			/*for testing the One_Key_Start*/
			Is_synergy = true;
			ROBOT_STATE = PLAN_STARIGHT_READY;
			MOVING_STATE = MOVE_ARRIVE;
			TURN_STATE = NO_TURNNING;
			detecting_oout = 0;
			PLAN_STATE = NO_PALNNING;
			REAR_PASS_WHEEL = MOVING;
			CTRL_FLAG = CONTION_WAIT;
			SEND_flag = NOT_SEND;
			REAR_PRIME_ARM_STATE = NOT;
			PASS_WHEEL = MOVING;
			Is_PRIME_ARM_ARRIVED = false;
			IS_REC = 0;
			key_state = false;
		}else{
			ROS_INFO("Has been recived the commend of One_Key_LIDAR_DECT!!!");
			/*以下测试雷达对中*/
			Is_synergy = true;
			ROBOT_STATE = DETECTING_WHEEL;
			CTRL_FLAG = CONTION_START_LINE;
			SEND_flag = SEND;
			IS_REC = 1;
			key_state = true;
		}
		
	}
}

void DownCarCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		ROS_INFO("Has been recived the commend of One_Key_Down_Car!!!");
		/*for testing the One_Key_Down_Car*/
		Is_synergy = true;
		ROBOT_STATE = FRONT_TAKING_DOWN_THE_CAR;
		MOVING_STATE = MOVE_ARRIVE;
		TURN_STATE = NO_TURNNING;
		detecting_oout = 0;
		CLOSING_REAR = MOVING;
		CTRL_FLAG = CONTION_WAIT;
		SEND_flag = NOT_SEND;
		Is_PRIME_ARM_CLOSE = false;
		REAR_Is_PRIME_ARM_CLOSE = false;
		IS_REC = 0;
	}
}

void SynergyCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		ROS_INFO("Has been recived the commend of STARTING Synergy!!!");
		Is_synergy = true;
	}else if(msg->data == false){
		ROS_INFO("Has been recived the commend of CLOSEING Synergy!!!");
		Is_synergy = false;
	}
}

void PrimeArmeOpenCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		ROS_INFO("The Front_ROBOT has been open the Prime_ARM!!!");
		Is_PRIME_ARM_ARRIVED = true;
	}
}

void PrimeArmeCloseCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		ROS_INFO("The Front_ROBOT has been close the Prime_ARM!!!");
		Is_PRIME_ARM_CLOSE = true;
	}
}
void RearPrimeArmeCloseCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data == true){
		ROS_INFO("The Rear_ROBOT has been close the Prime_ARM!!!");
		REAR_Is_PRIME_ARM_CLOSE = true;
	}
}
// void Dect_state_send(void){
// 	DectState_sub.publish(Dect_state);
// 	ROS_INFO("SENDING THE dectection's state !!!");
// }

int main(int argc, char **argv)
{
	uint16_t count = 0;
	bool first = true;
	setlocale(LC_ALL,"");
	ros::init(argc, argv, "moving");
	ros::NodeHandle nh; 
	ros::Subscriber sub;
	ros::Subscriber carPoint_sub;
	ros::Subscriber passPoint_sub;
	ros::Subscriber arm_touch_wheel_sub;
	ros::Subscriber rear_arm_touch_wheel_sub;
	ros::Subscriber front_touch_rear_sub;
	ros::Subscriber prime_arm_opened_sub;
	ros::Subscriber prime_arm_closed_sub;
	ros::Subscriber rear_prime_arm_closed_sub;
	ros::Subscriber start_sub;
	ros::Subscriber down_sub;
	ros::Subscriber synergy_sub;
	// IO_state_sub = nh.advertise<std_msgs::Bool>("Ser_IO_state", 1);
	pub_syn_ctrl = nh.advertise<geometry_msgs::Twist>("synergy_vel",1);;
	sub = nh.subscribe<nav_msgs::Odometry>("Odometry", 100, &AgvPositionCallback);
	carPoint_sub = nh.subscribe<geometry_msgs::Point>("move_carPoint", 10, &RecCarPointCallback);
	passPoint_sub = nh.subscribe<geometry_msgs::Point>("move_passPoint", 10, &RecPassPointCallback);
	arm_touch_wheel_sub = nh.subscribe<std_msgs::Bool>("arm_touch_wheel", 1, &ArmTouchCallback);
	rear_arm_touch_wheel_sub = nh.subscribe<std_msgs::Bool>("rear_arm_touch_wheel", 1, &RearArmTouchCallback);
	front_touch_rear_sub = nh.subscribe<std_msgs::Bool>("front_touch_rear", 1, &FrontTouchRearCallback);
	prime_arm_opened_sub = nh.subscribe<std_msgs::Bool>("Prime_ARM_OPENED", 5, &PrimeArmeOpenCallback);
	prime_arm_closed_sub = nh.subscribe<std_msgs::Bool>("Prime_ARM_CLOSED", 5, &PrimeArmeCloseCallback);
	rear_prime_arm_closed_sub = nh.subscribe<std_msgs::Bool>("Rear_Prime_ARM_CLOSED", 5, &RearPrimeArmeCloseCallback);
	start_sub = nh.subscribe<std_msgs::Bool>("Is_start", 5, &StartCallback);
	down_sub = nh.subscribe<std_msgs::Bool>("Is_Down", 5, &DownCarCallback);
	synergy_sub = nh.subscribe<std_msgs::Bool>("Is_synergy", 5, &SynergyCallback);
	robot_vel robot_v;

	PID_Parameter_Speed_Init(&TRACK_PID,0.00015,0, 0, 0.1, 0, 0, 1 );
	PID_Parameter_Speed_Init(&YAW_PID,0.02,0.0 , 0.001, 0.15, 0, 0, 1 );
	PID_Parameter_Speed_Init(&TURN_PID,0.02,0.0 , 0.001, 2.0, 0, 0, 1 );
	// Dect_state.detect_state = false;
	// DectState_sub.publish(Dect_state);

	ros::Rate rate(20);
	while (ros::ok())
	{
		count++;
		
		if(count%10 == 0){
			send_ctrl_node(CTRL_FLAG);
			// Sending_IO();
			cout<<"MOVING_STATE:"<<MOVING_STATE<<endl;
			cout<<"ROBOT STATE:"<<ROBOT_STATE<<endl;
			cout<<"TURN STATE:"<<TURN_STATE<<endl;
		}
		if(ROBOT_STATE == PLAN_STARIGHT_READY && detecting_oout > 60){
			PLAN_STATE = PLANNED;
			straight_start = ROBOT_REAL_Y;
			ROS_INFO("The straight starting location is:%f", straight_start);
		}
		Moving_FSM();
		Moving();
		Turnning();
		robot_v.Cal_VEL();

		if(count == 120){
			if(first){
				/*以下测试路径*/
				get_start = START;
				Is_synergy = true;
				/*以下测试雷达对中*/
				// get_start = START;
				// Is_synergy = true;
				// ROBOT_STATE = DETECTING_WHEEL;
				// CTRL_FLAG = CONTION_START_LINE;
				// SEND_flag = SEND;
				// IS_REC = 1;
				first = false;
			}
			cout<<"MOVING_STATE:"<<MOVING_STATE<<endl;
			cout<<"ROBOT STATE:"<<ROBOT_STATE<<endl;
			cout<<"TURN STATE:"<<TURN_STATE<<endl;
			// cout<<"TASK starting!!!"<<endl;

			
			// ROBOT_STATE = PLAN_STARIGHT;

			// Dect_state.detect_state = false;
			/*以下两行为测试偏航PID的测试*/
			// ROBOT_STATE = TURNNING_CIRA1;
			// TURN_STATE = TURNNING;
		}else if (count == 10000){
			count = 0;
		}

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
