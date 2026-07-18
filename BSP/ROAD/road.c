#include "bsp.h"

int route_finish = 0;
int target_round=1;

void around(void)
{
    if(!run_flag)
    {
        Speed_Pid[0].SetPoint=0;
        Speed_Pid[1].SetPoint=0;
        return;
    }
    test(90);
    if(route_finish)
    {
        run_flag=0;
        key1_flag=0;
        route_finish=0;
        target_round=1;
    }
} 

void test(float speed)
{
    int tell = road_tell();

    static int step = 0;
    static int turn_cnt = 0;
	static int last_tell = 0;
    switch(step)
    {
        //正常循迹
        case 0:
            Track_Direction_Control(speed);

            if(tell == 2 && last_tell!=2)
            {
                Turn_Start();
                step = 1;
            }
            break;


        //转弯
        case 1:
            if(Turn_Left(speed))
            {
                turn_cnt++;

                if(turn_cnt >= target_round * 4)
                {
                    step = 2;
                }
                else
                {
                    step = 0;
                }
            }
            break;


        //结束
        case 2:
            Speed_Pid[0].SetPoint = 0;
            Speed_Pid[1].SetPoint = 0;

            turn_cnt = 0;
			route_finish = 1;	
            step = 0;

    }
	last_tell = tell;

}
  