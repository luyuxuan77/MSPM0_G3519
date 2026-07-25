#include "TimerA1/timera1.h"
#include "bsp.h"

uint32_t nowtime = 0;

void TimerA1_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TimerA1_INST))
    {
    case DL_TIMERA_IIDX_LOAD:
        nowtime++;

        /* LED1 heartbeat: toggle every 500 ms (5000 * 100 us) */
        if ((nowtime % 5000) == 0)
            LED1_toggle;

        /* ---- Motor Control Scheduling ---- */
        {
            static uint16_t tick_10ms  = 0;   /* PWM output counter */
            static uint16_t tick_128ms = 0;   /* PID control counter */

            /* Pid_Speed(): write PWM to hardware every 10 ms (100 * 100 us) */
            if (++tick_10ms >= 100)
            {
                tick_10ms = 0;
                Pid_Speed();
            }

            /* motor_control_update(): read encoder, run PID every 128 ms (1280 * 100 us) */
            if (++tick_128ms >= 1280)
            {
                tick_128ms = 0;
                motor_control_update();
            }
        }
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
