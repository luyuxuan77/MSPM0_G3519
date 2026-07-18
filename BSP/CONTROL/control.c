#include "control.h"
#include "SPI0_LCD/lcd.h"

//����Ҷȴ�������pid�����ṹ�������p��i��d��ֵ�ṹ���������
PID_TypeDef Grayscale_Pid;
p_i_d_Value_TypeDef PID_Value_Grayscale[3] = {{  0.0, 0, 0 },	//Ԥ������pid���������ڲ�ͬ��Ѳ���ٶȣ�
											  { 0.0, 0.0, 0.0 },
											  { 55.0, 0.0, 10.0 }};
//p_i_d_Value_TypeDef PID_Value_Grayscale[3] = {
//    { 8.0f,  0.0f,  2.0f },   // ���٣��ȣ�
//    { 6.0f,  0.0f,  1.5f },   // ����
//    { 5.0f,  0.0f,  1.0f }    // ����
//};
											  
//����Ҷ� ����������pid�����ṹ�������p��i��d��ֵ�ṹ���������
PID_TypeDef Grayscale_Direction_Pid;
p_i_d_Value_TypeDef PID_Value_Grayscale_Direction[3] = {{ 15, 0, 4 },	//Ԥ������pid���������ڲ�ͬ��Ѳ���ٶȣ�
														{ 0.0, 0.0, 0.0 },
														{ 0.0, 0.0, 0.0 }};

static Track_State_t track_state = TRACK_LINE;


static uint8_t left_corner_cnt = 0;
static uint8_t right_corner_cnt = 0;
static uint8_t turn_cnt = 0;											
int gw_offset=0;
void Track_Direction_Control(int speed)  
{
    gw_offset = get_gray_refresh_data();
	
	Pid_control(&Grayscale_Direction_Pid,gw_offset, 0);
	Pid_OutLimit(&Grayscale_Direction_Pid, 100);
	float left  = (speed + Grayscale_Direction_Pid.ActualValue * 0.4 );
	float right = (speed - Grayscale_Direction_Pid.ActualValue * 0.4 );

    Speed_Pid[0].SetPoint=left;
    Speed_Pid[1].SetPoint=right;
//	set_motor_speed(-left,-right,0,0);
//	
//	LCD_ShowString(20,90,"LEFT:",BLACK,LIGHTBLUE,16,1);
//	LCD_ShowString(120,90,"RIGHT:",BLACK,LIGHTBLUE,16,1);
//	LCD_ShowString(20,100,"ActualValue:",BLACK,WHITE,16,1);
//	
//	LCD_ShowIntNum(50, 90, left, 4, BLACK, WHITE, 16);
//    LCD_ShowIntNum(160,90,right, 4, BLACK, WHITE, 16);
//    LCD_ShowFloatNum1(50,120,Grayscale_Direction_Pid.ActualValue, 6, BLACK, WHITE, 16);
	
//	printf("Grayscale_Direction_Pid.ActualValue:   %f\r\n",Grayscale_Direction_Pid.ActualValue);
//	printf("left:%f  right:  %f\r\n",left,right);
//   printf("offset:   %d\t\n",gw_offset);
}



float turn_start_yaw;
void Turn_Start()
{
    turn_start_yaw=Yaw;
}


int Turn_Left(float speed)
{
    float x;

    x=turn_start_yaw-Yaw;

    if(x>180)x-=360;
    if(x<-180)x+=360;

    if(x>72)
    {
        Speed_Pid[0].SetPoint=0;
        Speed_Pid[1].SetPoint=0;
        return 1;
    }
    else if(x>30)
    {
        Speed_Pid[0].SetPoint=0;
        Speed_Pid[1].SetPoint=30;
    }
    else
    {
        Speed_Pid[0].SetPoint=0;
        Speed_Pid[1].SetPoint=80;
    }

    return 0;
}




int Turn_Right(float speed)
{
    float x;

    x=Yaw-turn_start_yaw;

    if(x>180)x-=360;
    if(x<-180)x+=360;

    if(x>74)
    {
        Speed_Pid[0].SetPoint=0;
        Speed_Pid[1].SetPoint=0;
        return 1;
    }
    else if(x>30)
    {
        Speed_Pid[0].SetPoint=30;
        Speed_Pid[1].SetPoint=0;
    }
    else
    {
        Speed_Pid[0].SetPoint=80;
        Speed_Pid[1].SetPoint=0;
    }

    return 0;
}




int road_tell()
{
   get_gray_refresh_data();
    u8 data = get_offset_s();

    if (data == 0) // �հ�
    {
        return 1;
    }

    // ===== ͳ�����ҵ����� =====
    int left_count = ((data >> 4) & 1) + ((data >> 5) & 1) + ((data >> 6) & 1) + ((data >> 7) & 1);
    int right_count = ((data >> 0) & 1) + ((data >> 1) & 1) + ((data >> 2) & 1) + ((data >> 3) & 1);

    // ===== T���жϣ���3��������=====
    if (left_count >= 3 && right_count < 3)
    {
        return 2; // ��T 
    }

    if (right_count >= 3 && left_count < 3)
    {
        return 3; // ��T
    }

    // ===== ʮ�֣����߶��ࣩ=====
    if (left_count >= 3 && right_count >= 3)
    {
        return 4;
    }

    return 0;
}