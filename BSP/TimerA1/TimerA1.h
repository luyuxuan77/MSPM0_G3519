#ifndef TIMERA1_H
#define TIMERA1_H
#include "bsp.h"

extern volatile uint32_t nowtime; // 1msʱ��
extern volatile uint8_t g_i2c_read_flag;  /* 10ms I2C读取标志 */
void TimerA1_init(void);

void system_time_init(void);



/*
 * 10 ms ѭ�������жϷ����� line_following.c ʵ�֣���
 * �� TIMA1 1 ms ������ÿ 10 �δ���һ�Ρ�
 */
void line_following_control_isr(void);

/*
 * 20 ms VOFA ����֡���ȣ��� line_following.c ʵ�֣���
 * �� TIMA1 1 ms ������ÿ 20 ms ����һ�Σ�����ѭ�������Ա�֤�̶����������
 */
void line_following_vofa_20ms_tick(void);

#define SYSTEM_TIME_TICK_HZ        (1000U)
#define SYSTEM_TIME_TICKS_PER_MS   (1U)

extern volatile uint32_t nowtime;

/*
 * ��ʼ�� 1ms ϵͳʱ����
 *
 * ����ʱ����
 * - ������ SYSCFG_DL_init() ֮����ã���Ϊ��ʱ��ʵ����ʱ�Ӻ��жϺ����� SysConfig��
 *
 * Ӳ��/�ж�Լ����
 * - �ú��������� nowtime����� TIMA1 LOAD �жϱ�־���� NVIC�������� TIMA1��
 * - TIMA1_IRQHandler ��ÿ 1ms �ۼ� nowtime����ѭ������������Դ���Ϊ����ʱ���׼��
 */


/*
 * ��ȡ��ǰϵͳ���������
 *
 * ����ֵ��
 * - �� system_time_init() ���� TIMA1 ���ۼƵĺ���������λΪ ms��
 *
 * ʹ��˵����
 * - ��ֵ���жϸ��£�32 λ��������Ȼ���ƣ��ϲ��ж�ʱ���ʱ����ʹ��
 *   system_time_elapsed_ms()��������д�����߼���
 */
uint32_t system_time_get_tick_ms(void);

/*
 * �ȴ�ϵͳʱ������ǰ��һ�� tick��
 *
 * ������
 * - timeout_loop_count����ѯ�ȴ�ѭ�����ޣ����ڷ�ֹ��ʱ�����ж�δ����ʱ����������
 *
 * ����ֵ��
 * - true���۲쵽 nowtime �仯��˵�� 1ms �ж��������С�
 * - false�������ȴ�ѭ����δ�仯��ͨ����ʾ TIMA1 �� NVIC �����쳣��
 */
bool system_time_wait_for_tick(uint32_t timeout_loop_count);

/*
 * �ж�ָ�������Ƿ��Ѿ����
 *
 * ������
 * - last_tick_ms������Ϊ�ϴδ���ʱ�䣻�����ɹ�ʱ�ᱻ����Ϊ��ǰʱ�䡣
 * - interval_ms���������ڣ���λ ms��
 *
 * ����ֵ��
 * - true�������ϴδ����Ѵﵽ interval_ms��
 * - false��ʱ����δ����
 *
 * ˵����
 * - �ڲ����޷��ż������� 32 λ���ƣ��ʺ���ѭ����ʵ�� 20Hz��1Hz ����������
 */
bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms);

#endif
