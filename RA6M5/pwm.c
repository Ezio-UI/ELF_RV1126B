/*
 * pwm.c
 *
 *  Created on: 2026年3月13日
 *      Author: Riloptop
 */
#include "hal_data.h"
#include "debug_pwm/pwm.h"


void Gpt3_Init(void)
{
    R_GPT_Open(&g_timer3_ctrl, &g_timer3_cfg);
    R_GPT_Start(&g_timer3_ctrl);
    Gpt_Pwm_Setduty(95);

}
void Gpt_Pwm_Setduty(uint8_t duty)//占空比为(100-duty)%
{
    timer_info_t info;
    uint32_t duty_count;
    if(duty>100)
    {
        duty =100;
    }
    R_GPT_InfoGet(&g_timer3_ctrl,&info);
    duty_count = info.period_counts*duty / 100;
    R_GPT_DutyCycleSet(&g_timer3_ctrl,duty_count,GPT_IO_PIN_GTIOCA);

}

