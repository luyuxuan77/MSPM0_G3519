#ifndef MOTOR_H
#define MOTOR_H
#include "bsp.h"
void set_motor_speed(int m1,int m2,int m3,int m4);

void motor_init(void);

void get_motor_qei_cnt(uint32_t *cnt);
void motor_control_update(void);				
void motor_speed_pid_init(void);

void print_date();
void Pid_Speed();

#endif