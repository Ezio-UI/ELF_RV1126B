/*
 * electrical_stimulation.h
 *
 *  Created on: 2026年7月4日
 *      Author: Riloptop
 */

#ifndef DEBUG_ELECTRICAL_STIMULATION_ELECTRICAL_STIMULATION_H_
#define DEBUG_ELECTRICAL_STIMULATION_ELECTRICAL_STIMULATION_H_

#include "stdint.h"

typedef enum{
     LEFT   =  1,
     CENTER =  2,
     RIGHT  =  3
}Direction;

typedef enum{
     FAST  = 20,
     MODER = 50,
     SLOW  = 100
}Frequence_level;

void electrical_stimulation_freCtrl(Direction dir,uint16_t distance);

#endif /* DEBUG_ELECTRICAL_STIMULATION_ELECTRICAL_STIMULATION_H_ */
