#include "ros/ros.h"
#include "std_msgs/String.h"
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
#include <iomanip>
#include <signal.h>

#define PI 3.14159
#define Speed_Max 0.5
#define Car_L 300
#define START 1
#define STOP 0
#define BASE2LIDAR 0.9375 // m


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

	DETECTING_WHEEL,//12
	TURNNING_GOTO_CAR,//13
	GOTO_CAR,//14
	TURNNING_FACE_CAR,//15

	DETECTING_WHEEL_PASS,//16
	CLOSING_TO_CAR,//17
	TURNNING_CLOSE_CAR,//18

	STOPPING,//19
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
TRCK_PIONTS GOTO_PATH1[9] = 
{
{0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
{0.0, 500.0, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
{0.0, 1000, 0.0f, 0.0f, 0.0f, 0.8f*Speed_Max},
{0.0f,1500.0, 0.0f, 0.0f, 0.0f,1.0f*Speed_Max},
{0.0f, 2000.0, 0.0f, 0.0f, 0.0f,1.0f*Speed_Max},
{0.0, 3000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
{0.0, 4000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
{0.0, 4500.0, 0.0f, 0.0f, 0.0f, 0.5f*Speed_Max},
/*代码测试为了方便进行的修改，实机测试需要删掉*/
// {0.0, 5000.0, 0.0f, 0.0f, 0.0f, 1.0f*Speed_Max},
{0.0, 5000.0, 0.0f, 0.0f, 0.0f, 0.2f*Speed_Max},
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
TRCK_PIONTS GOTO_PATH2[5] = 
{
// {0.0, 6000.0, 0.0f, 90.0f, 0.0f, 0.2*Speed_Max},
// {600.0, 5000.0, 0.0f, 90.0f, 0.0f, 0.5f*0.5f*Speed_Max},
{2000.0, 7000.0f, 0.0f, 90.0f, 0.0f,0.2f*Speed_Max},
{3000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
{4000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
{4500.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.5f*Speed_Max},
{5000.0, 7000.0, 0.0f, 90.0f, 0.0f, 0.2f*Speed_Max},
};
TRCK_PIONTS GOTO_PATH3[10] = 
{
{7000.0, 5000.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
{7000.0, 4800.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
{7000.0, 4500.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
{7000.0, 4200.0, 0.0f, 179.0f, 0.0f, 0.8f*Speed_Max},
{7000.0, 4000.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
{7000.0, 3500.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
{7000.0, 3000.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
{7000.0, 2700.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
{7000.0, 2300.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
{7000.0, 2000.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
// {7000.0, 500.0, 0.0f, 179.0f, 0.0f, 0.5f*Speed_Max},
// /*代码测试为了方便进行的修改，实机测试需要删掉*/
// {5000.0, 0.0, 0.0f, 179.0f, 0.0f, 0.2f*Speed_Max},
// {5000.0, 0.0, 0.0f, 179.0f, 0.0f, 1.0f*Speed_Max},
};
TRCK_PIONTS GOTO_PATH4[7] = 
{
{5000.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
{4500.0, 0.0, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{4000.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
{3000.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
{2000.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
{1000.0, 0.0, 0.0f, -90.0f, 0.0f, 0.5f*Speed_Max},
{500.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f*Speed_Max},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 1.0f*Speed_Max},
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
// TRCK_PIONTS Car_location[5] = 
// {
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.2f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// {0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
// };
TRCK_PIONTS Car_location[10] = 
{
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.1f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
};
TRCK_PIONTS Pass_location[10] = 
{
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.05f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
{0.0, 0.0, 0.0f, -90.0f, 0.0f, 0.02f},
};

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

struct Quaternion {
    double w, x, y, z;
};
struct EulerAngles {
    double roll, pitch, yaw;
};

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
ROBOT_STATE_ITEMS 	ROBOT_STATE = WAITING;
ROBOT_MOVE	MOVING_STATE = MOVE_STOP;
DETECT_WHEEL_STATE_ITEMS DETECTING_STATE = NO_CAR_POINT;
PATH_FOLLOW    Path_Contorl = {0,1};
uint32_t PATH_POINT_NUM = 0;
float ROBOT_REAL_X,ROBOT_REAL_Y = 500;
float ROBOT_REAL_YAW = 0.0f;
float World_VY,World_VX,World_VW = 0;
float expect_angle = 0;
double car_angle, face_car_angle = 0;
uint8_t get_start = STOP;
ros::Publisher DectState_sub;
move::detect_state Dect_state;

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


void robot_vel::Cal_VEL(void){
	uint8_t SEND_VEL = 1;
	static uint8_t senconds = 0;
	float wheel_x,wheel_y,target_angle,wheel_w = 0;
	float wheel_vel = 0;
	float W_x,W_y = 0;
	double diff = 0;
	geometry_msgs::Twist robot_v;
	if(MOVING_STATE == MOVE_GOING){
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

		diff = angleDifference(ROBOT_REAL_YAW, target_angle);
		cout<<"the angle difference is :"<<diff<<". the target angle is:"<<target_angle<<endl;
		// cout<<"the angle difference is :"<<diff<<endl;
		// wheel_w = PID_Position_Calculate(&TURN_PID, target_angle, ROBOT_REAL_YAW);
		wheel_w = -PID_Position_Calculate(&YAW_PID, 0, diff);
		// cout<<"World YAW SPEED :"<<World_VW<<endl;
	}else if(MOVING_STATE != MOVE_GOING && TURN_STATE != TURNNING){
		if(senconds > 2){
			SEND_VEL = 0;
		}
		senconds++;
		wheel_vel = 0;
		wheel_w = 0;
	}
	else{
		SEND_VEL = 1;
		target_angle = expect_angle;
		wheel_w = World_VW;
	}
	
	robot_v.linear.x = wheel_vel;
	robot_v.angular.z = wheel_w;
	
	// cout<<"World YAW SPEED :"<<wheel_w<<endl;
	// cout<<"World liner SPEED :"<<wheel_vel<<endl;
	
	if (SEND_VEL){
		printf("World liner SPEED : %.4f, World YAW SPEED : %.4f\r\n",wheel_vel,wheel_w);
		pub.publish(robot_v);
	}
}
uint8_t detecting_oout = 2;
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
			}
			break;
		// case GOTO_P1:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURNNING_CIRA1;
		// 		TURN_STATE = TURNNING;
		// 	}
		// 	break;
		// case TURNNING_CIRA1:
		// 	if(TURN_STATE == TURNNED){
		// 		ROBOT_STATE = GOTO_P2;
		// 		// ROBOT_REAL_Y = 5000.0f;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case GOTO_P2:
		// 	// ROBOT_REAL_Y = 5000.0f;
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURNNING_CIRA2;
		// 		TURN_STATE = TURNNING;
		// 	}
		// 	break;
		case GOTO_P1:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURN_CIRA1;
				TURN_STATE = NO_TURNNING;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case TURN_CIRA1:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = GOTO_P2;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case GOTO_P2:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURN_CIRA2;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case TURN_CIRA2:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = GOTO_P3;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		
		case GOTO_P3:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURN_CIRA3;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case TURN_CIRA3:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = GOTO_P4;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case GOTO_P4:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = STOPPING;
				MOVING_STATE = MOVE_STOP;
			}
			break;
		// case TURNNING_CIRA2:
		// 	if(TURN_STATE == TURNNED){
		// 		ROBOT_STATE = GOTO_P3;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case GOTO_P3:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURNNING_CIRA3;
		// 		TURN_STATE = TURNNING;
		// 	}
		// 	break;
		// case TURNNING_CIRA3:
		// 	if(TURN_STATE == TURNNED){
		// 		ROBOT_STATE = GOTO_P4;
		// 		MOVING_STATE = MOVE_GOING;
		// 	}
		// 	break;
		// case GOTO_P4:
		// 	if(MOVING_STATE == MOVE_ARRIVE){
		// 		ROBOT_STATE = TURNNING_CIRA4;
		// 		TURN_STATE = TURNNING;
		// 	}
		// 	break;
		// case TURNNING_CIRA4:
		// 	if(TURN_STATE == TURNNED){
		// 		ROBOT_STATE = STOPPING;
		// 		MOVING_STATE = MOVE_STOP;
		// 	}
		// 	break;
		case DETECTING_WHEEL:
			if(DETECTING_STATE == GET_CAR_POINT){
				DETECTING_STATE = NO_CAR_POINT;
				MOVING_STATE = MOVE_ARRIVE;
				ROBOT_STATE = TURNNING_GOTO_CAR;
				TURN_STATE = TURNNING;
			}
			break;
		case TURNNING_GOTO_CAR:
			// cout<<"-------TURN STATE:"<<TURN_STATE<<endl;
			TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				ROBOT_STATE = GOTO_CAR;
				MOVING_STATE = MOVE_GOING;
			}
			break;
		case GOTO_CAR:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_FACE_CAR;
				TURN_STATE = TURNNING;
				DETECTING_STATE = NO_CAR_POINT;
			}
			break;
		case TURNNING_FACE_CAR:
			// TURN_STATE = TURNNED;
			if(TURN_STATE == TURNNED){
				IS_REC = 1;
				if(detecting_oout >= 2){
					ROBOT_STATE = STOPPING;
					MOVING_STATE = MOVE_STOP;
				}else{
					ROBOT_STATE = DETECTING_WHEEL;
					MOVING_STATE = MOVE_STOP;
					Dect_state.detect_state = false;
					DectState_sub.publish(Dect_state);
				}
				detecting_oout++;
				
			}
			break;

		
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
				ROBOT_STATE = STOPPING;
				MOVING_STATE = MOVE_STOP;
			}
			break;
		case CLOSING_TO_CAR:
			if(MOVING_STATE == MOVE_ARRIVE){
				ROBOT_STATE = TURNNING_CLOSE_CAR;
				TURN_STATE = TURNNING;
				DETECTING_STATE = NO_CAR_POINT;
			}
			break;
		case TURNNING_CLOSE_CAR:
			if(TURN_STATE == TURNNED){
				ROBOT_STATE = DETECTING_WHEEL_PASS;
				MOVING_STATE = MOVE_STOP;
			}
			break;
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
			NOW_PATH = GOTO_PATH1;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH1) / sizeof(GOTO_PATH1[0]));  
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
			NOW_PATH = GOTO_PATH2;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH2) / sizeof(GOTO_PATH2[0]));
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
			NOW_PATH = GOTO_PATH3;
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH3) / sizeof(GOTO_PATH3[0]));
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
			NOW_PATH = GOTO_PATH4; 
			PATH_POINT_NUM = (uint32_t)(sizeof(GOTO_PATH4) / sizeof(GOTO_PATH4[0]));
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
					
		default:
		{
			NOW_PATH = NULL;
		}
		break;
		}
		PATH_TRACKING(NOW_PATH, PATH_POINT_NUM);

					
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
		cout<<"ROBOT is executing the task og turnning!!!"<<endl;
		cout<<"the angle difference is :"<<diff<<". the target angle is:"<<expect_angle<<endl;
		// World_VW = PID_Position_Calculate(&YAW_PID, expect_angle, ROBOT_REAL_YAW);
		World_VW = -PID_Position_Calculate(&YAW_PID, 0, diff);
		// cout<<"World YAW SPEED :"<<World_VW<<endl;
	}
}
void Turnning (void){
	if (TURN_STATE == TURNNING){
		
		switch(ROBOT_STATE)
		{
			case TURNNING_CIRA1:
			{
				expect_angle = -90.0f;
			}break;
			case TURNNING_CIRA2:
			{
				expect_angle = -180.0f;
			}break;
			case TURNNING_CIRA3:
			{
				expect_angle = 90.0f;
			}break;
			case TURNNING_CIRA4:
			{
				expect_angle = 0.0f;
			}break;
			case TURNNING_GOTO_CAR:
			{
				expect_angle = car_angle;
			}break;
			case TURNNING_FACE_CAR:
			{
				expect_angle = face_car_angle;
			}break;
			case TURNNING_CLOSE_CAR:
			{
				expect_angle = face_car_angle;
			}break;
		}
		TURNNING_move();

	}
}
/*里程计获取全场定位*/
float POS_X,POS_Y,Last_POS_X,Last_POS_Y,REAL_Y,REAL_X = 0;
void AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& Odo_msg)
{
	Quaternion quaternion;
	EulerAngles angles;
	float AGV_pos_X, AGV_pos_Y, AGV_pos_Z;
	float DETLA_X,DETLA_Y = 0;

	Last_POS_X = POS_X;
	Last_POS_Y = POS_Y;
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
// void RecCarPointCallback(const geometry_msgs::Point::ConstPtr& car_position){

// 	if(IS_REC){
// 		double dx = car_position->x/4*1000;
// 		double dy = car_position->y/4*1000;
// 		Car_location[0].X = ROBOT_REAL_X;
// 		Car_location[0].Y = ROBOT_REAL_Y;
// 		for(uint8_t i = 1; i <= 4; i++){
// 			Car_location[i].X = ROBOT_REAL_X + dx*i;
// 			Car_location[i].Y = ROBOT_REAL_Y + dy*i;
// 		}
// 		face_car_angle = car_position->z + ROBOT_REAL_YAW;

// 		double theta = atan2(car_position->y, car_position->x)/PI*180.0 - 90;
// 		car_angle = theta;

// 		DETECTING_STATE = GET_CAR_POINT;
// 		ROS_INFO("Received the wheel info, x:%f, y:%f, angle:%f,!!!",car_position->x, car_position->y, face_car_angle);
// 		ROS_INFO("the car current postion X:%f, Y: %f, the angle:%f", ROBOT_REAL_X,ROBOT_REAL_Y, car_angle);
// 		ROS_INFO("the car target postion X:%f, Y: %f", Car_location[4].X,Car_location[4].Y);
// 		IS_REC = 0;
// 	}
	
// }

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
void RecCarPointCallback(const geometry_msgs::Point::ConstPtr& car_position){

	if(IS_REC){
		ROS_INFO("Received the wheel info, x:%f, y:%f, angle:%f,!!!",car_position->x, car_position->y, car_position->z);
		Car_location[0].X = ROBOT_REAL_X;
		Car_location[0].Y = ROBOT_REAL_Y;
		// 定义起点和终点
		double start_theta = (ROBOT_REAL_YAW+90)/180.0*M_PI;
		double end_theta = (car_position->z+90)/180.0*M_PI;
		double end_X = (car_position->x + sin((car_position->z)/180.0*M_PI)*BASE2LIDAR)*1000 + ROBOT_REAL_X;
		double end_Y = (car_position->y - cos((car_position->z)/180.0*M_PI)*BASE2LIDAR + BASE2LIDAR)*1000 + ROBOT_REAL_Y;
		cout<<" the init target position X:"<<car_position->x*1000<<" ,Y:"<<car_position->y*1000<<endl;
		cout<<" the calculation target position X:"<<end_X<<" ,Y:"<<end_Y<<endl;
		std::tuple<double, double, double> start = std::make_tuple(ROBOT_REAL_X, ROBOT_REAL_Y, start_theta);  // 起点 (x, y, theta)
		std::tuple<double, double, double> end = std::make_tuple(end_X, end_Y, end_theta);  // 终点 (x, y, theta)

		// 计算控制点
		std::tuple<double, double, double> p1, p2;
		calculate_control_points(start, end, 0.4, p1, p2);

		// 生成轨迹
		int num_points = 10;
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
			Car_location[i].X = std::get<0>(point);
			Car_location[i].Y = std::get<1>(point);
			// std::cout << "x: " << std::fixed << std::setprecision(2) << std::get<0>(point)
			// 		<< ", y: " << std::fixed << std::setprecision(2) << std::get<1>(point)
			// 		<< ", theta: " << std::fixed << std::setprecision(2) << std::get<2>(point) << std::endl;
			std::cout << "x: " << Car_location[i].X
					<< ", y: " << Car_location[i].Y << std::endl;
			i++;
		}
		face_car_angle = car_position->z;
		DETECTING_STATE = GET_CAR_POINT;
		IS_REC = 0;
		
	}
	
}

int times = 0;
void RecPassPointCallback(const geometry_msgs::Point::ConstPtr& car_position){

	if(IS_REC){
		ROS_INFO("the entry function's times is:%d",times);
		if(times > 1){
			ROS_INFO("Received the wheel info, x:%f, y:%f, angle:%f,!!!",car_position->x, car_position->y, car_position->z);
			if(car_position->x == 0.0 && car_position->y == 0.0 && car_position->z ==0.0){
				ROS_INFO("The ROBOT has been close!!!START STRAIGHT PASSING!!!");
				DETECTING_STATE = ARRIVIALED_CAR;
				IS_REC = 0;
				times = 0;
			}else{
				Pass_location[0].X = ROBOT_REAL_X;
				Pass_location[0].Y = ROBOT_REAL_Y;
				// 定义起点和终点
				double start_theta = (ROBOT_REAL_YAW+90)/180.0*M_PI;
				double end_theta = (car_position->z+90)/180.0*M_PI;
				double end_X = (car_position->x + sin((car_position->z)/180.0*M_PI)*BASE2LIDAR)*1000 + ROBOT_REAL_X;
				double end_Y = (car_position->y - cos((car_position->z)/180.0*M_PI)*BASE2LIDAR + BASE2LIDAR)*1000 + ROBOT_REAL_Y;
				cout<<" the init target position X:"<<car_position->x*1000<<" ,Y:"<<car_position->y*1000<<endl;
				cout<<" the calculation target position X:"<<end_X<<" ,Y:"<<end_Y<<endl;
				std::tuple<double, double, double> start = std::make_tuple(ROBOT_REAL_X, ROBOT_REAL_Y, start_theta);  // 起点 (x, y, theta)
				std::tuple<double, double, double> end = std::make_tuple(end_X, end_Y, end_theta);  // 终点 (x, y, theta)

				// 计算控制点
				std::tuple<double, double, double> p1, p2;
				calculate_control_points(start, end, 0.4, p1, p2);

				// 生成轨迹
				int num_points = 10;
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
				face_car_angle = car_position->z;
				DETECTING_STATE = GET_CAR_POINT;
				IS_REC = 0;
				times = 0;
			}
		}else{
			times++;
		}
		
	}
	
}

void Dect_state_send(void){
	DectState_sub.publish(Dect_state);
	ROS_INFO("SENDING THE dectection's state !!!");
}

int main(int argc, char **argv)
{
	uint16_t count = 0;
	setlocale(LC_ALL,"");
	ros::init(argc, argv, "moving");
	ros::NodeHandle nh; 
	ros::Subscriber sub;
	ros::Subscriber carPoint_sub;
	ros::Subscriber passPoint_sub;
	DectState_sub = nh.advertise<move::detect_state>("dectecting_state", 1);
	sub = nh.subscribe<nav_msgs::Odometry>("Odometry", 100, &AgvPositionCallback);
	carPoint_sub = nh.subscribe<geometry_msgs::Point>("move_carPoint", 10, &RecCarPointCallback);
	passPoint_sub = nh.subscribe<geometry_msgs::Point>("move_passPoint", 10, &RecPassPointCallback);
	robot_vel robot_v;

	PID_Parameter_Speed_Init(&TRACK_PID,0.0007,0, 0, 1, 0, 0, 1 );
	PID_Parameter_Speed_Init(&YAW_PID,0.02,0.0 , 0.001, 2.0, 0, 0, 1 );
	PID_Parameter_Speed_Init(&TURN_PID,0.02,0.0 , 0.001, 2.0, 0, 0, 1 );
	Dect_state.detect_state = false;
	DectState_sub.publish(Dect_state);

	ros::Rate rate(20);
	while (ros::ok())
	{
		count++;
		if(count == 120){
			cout<<"MOVING_STATE:"<<MOVING_STATE<<endl;
			cout<<"ROBOT STATE:"<<ROBOT_STATE<<endl;
			cout<<"TURN STATE:"<<TURN_STATE<<endl;
			// cout<<"TASK starting!!!"<<endl;
			get_start = START;
			ROBOT_STATE = DETECTING_WHEEL_PASS;
			IS_REC = 1;
			// Dect_state.detect_state = false;
			/*以下两行为测试偏航PID的测试*/
			// ROBOT_STATE = TURNNING_CIRA1;
			// TURN_STATE = TURNNING;
		}
		if(count%10 == 0){
			Dect_state_send();
			cout<<"MOVING_STATE:"<<MOVING_STATE<<endl;
			cout<<"ROBOT STATE:"<<ROBOT_STATE<<endl;
			cout<<"TURN STATE:"<<TURN_STATE<<endl;
		}
		Moving_FSM();
		Moving();
		Turnning();
		
		robot_v.Cal_VEL();

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
