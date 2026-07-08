#include "bsp_dac.h"

void DAC_Init(void)
{

    R_DAC_Open(&g_dac1_ctrl, &g_dac1_cfg);
    R_DAC_Start(&g_dac1_ctrl);

}
uint16_t var[] = {620,1241,1861,2476};//一档：0.5v,二档：1v,三档：1.5v,四档：2v

void DAC_SinWave_Cycle(uint32_t voltagee)
{

        R_DAC_Write(&g_dac1_ctrl, var[voltagee]);
}
