/*
 * beep.h
 *
 *  Created on: 9 апр. 2020 г.
 *      Author: we
 */

#ifndef BEEP_H_
#define BEEP_H_

#include "main.h"



enum BeepTypes {
	BeepType_Default = 0,
	BeepType_Success = 1,
	BeepType_Fail    = 2,
	BeepType_Bonus   = 3
};

void Beep(void);
extern void BeepCustom (uint16_t beepType);

void vBeepTask( void *argument);

#endif /* BEEP_H_ */
