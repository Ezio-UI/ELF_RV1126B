/*
 * electrical_stimulation.c
 *
 *  Created on: 2026年7月4日
 *      Author: Riloptop
 */

#include "electrical_stimulation.h"
#include "hal_data.h"
#include "stdio.h"
#include "stdint.h"

#define  delay_ms(x)        R_BSP_SoftwareDelay(x,BSP_DELAY_UNITS_MILLISECONDS)
#define  PIN_HIGH(pin_num)  R_IOPORT_PinWrite(&g_ioport_ctrl,pin_num, BSP_IO_LEVEL_HIGH)
#define  PIN_LOW(pin_num)   R_IOPORT_PinWrite(&g_ioport_ctrl,pin_num, BSP_IO_LEVEL_LOW)



typedef struct DIR_IO{
    bsp_io_port_pin_t pin_LH;
    bsp_io_port_pin_t pin_RL;
    bsp_io_port_pin_t pin_RH;
    bsp_io_port_pin_t pin_LL;
}obj_dir;

obj_dir left = {
    _01_LH,
    _01_RL,
    _01_RH,
    _01_LL

};

obj_dir center = {
    _02_LH,
    _02_RL,
    _02_RH,
    _02_LL

};

obj_dir right = {
    _03_LH,
    _03_RL,
    _03_RH,
    _03_LL

};

static void IO_SET_HIGH(bsp_io_port_pin_t pin1,bsp_io_port_pin_t pin2)
{
    PIN_HIGH(pin1);
    PIN_HIGH(pin2);
}


static void IO_SET_LOW(bsp_io_port_pin_t pin1,bsp_io_port_pin_t pin2)
{
    PIN_LOW(pin1);
    PIN_LOW(pin2);
}

void electrical_stimulation_freCtrl(Direction dir,uint16_t distance)
{
    Frequence_level level;
    bsp_io_port_pin_t pin_num1;
    bsp_io_port_pin_t pin_num2;
    bsp_io_port_pin_t pin_num3;
    bsp_io_port_pin_t pin_num4;
    switch(dir)
        {
            case 01:
                pin_num1 = left.pin_LH;
                pin_num2 = left.pin_RL;
                pin_num3 = left.pin_RH;
                pin_num4 = left.pin_LL;
                break;
            case 02:
                pin_num1 = center.pin_LH;
                pin_num2 = center.pin_RL;
                pin_num3 = center.pin_RH;
                pin_num4 = center.pin_LL;
                break;
            case 03:
                pin_num1 = right.pin_LH;
                pin_num2 = right.pin_RL;
                pin_num3 = right.pin_RH;
                pin_num4 = right.pin_LL;
                break;
            default:
                break;
        }

    if(distance <=200)
    {
        level = FAST;
    }
    if(200 < distance && distance <= 500)
    {
        level = MODER;
    }
    if(distance > 500)
    {
        level = SLOW;
    }

    for(uint8_t i = 1 ; i<10 ; i++)
    {
        IO_SET_HIGH(pin_num1,pin_num2);
        IO_SET_LOW(pin_num3,pin_num4);
        delay_ms(level);
        IO_SET_HIGH(pin_num3,pin_num4);
        IO_SET_LOW(pin_num1,pin_num2);
        delay_ms(level);
    }

}


