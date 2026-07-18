#include "bsp.h"
#include "MOTOR/motor.h"
volatile uint32_t nowtime = 0U;

void system_time_init(void)
{
    nowtime = 0U;
    DL_TimerA_clearInterruptStatus(TimerA1_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    DL_TimerA_enableInterrupt(TimerA1_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(TimerA1_INST_INT_IRQN);
    NVIC_EnableIRQ(TimerA1_INST_INT_IRQN);
    DL_TimerA_startCounter(TimerA1_INST);
}

uint32_t system_time_get_tick_ms(void)
{
    return nowtime;
}

bool system_time_wait_for_tick(uint32_t timeout_loop_count)
{
    uint32_t start_tick = nowtime;

    while (timeout_loop_count > 0U) {
        if (nowtime != start_tick) {
            return true;
        }
        timeout_loop_count--;
    }

    return false;
}

bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms)
{
    uint32_t current_ms;

    if (last_tick_ms == NULL) {
        return false;
    }

    current_ms = system_time_get_tick_ms();
    if (interval_ms == 0U) {
        *last_tick_ms = current_ms;
        return true;
    }

    if ((uint32_t) (current_ms - *last_tick_ms) >= interval_ms) {
        *last_tick_ms = current_ms;
        return true;
    }

    return false;
}

void TimerA1_INST_IRQHandler(void)
{

	switch( DL_TimerA_getPendingInterrupt(TimerA1_INST))
	{
		case DL_TIMERA_IIDX_LOAD:
			nowtime++;
//		if((nowtime%10U)==0U)
// 			motor_control_update();
		break;
		default:
			break;
	}
}

void TimerA1_init(void)
{

	NVIC_ClearPendingIRQ(TimerA1_INST_INT_IRQN);
	NVIC_EnableIRQ(TimerA1_INST_INT_IRQN);
}



