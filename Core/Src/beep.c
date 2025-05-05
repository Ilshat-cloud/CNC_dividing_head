/*
 * beep.c
 *
 *  Created on: 9 апр. 2020 г.
 *      Author: we
 */

#include "beep.h"
#include "cmsis_os.h"
#include "tim.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern osMessageQueueId_t BeepQueueHandle;
extern void MX_TIM2_Init_tone(uint16_t freq);

static void Sound (uint32_t tone, uint32_t delay);
extern osThreadId BeepTaskHandle;
uint8_t bBeepDisable = 0; //Не издавать сигналов при случайном прикосновении к экрану во время ожидания карты и транзакции.

//-----------------------------------------------------------------------
void vBeepTask( void *argument)
{
	uint16_t beepType = 0;

	for(;;)
	{               
		if (osMessageQueueGet(BeepQueueHandle, &beepType, NULL, osWaitForever) == osOK)
		{
			if (beepType == BeepType_Success)
			{
				Sound (100, 100);
                                Sound (200, 100);
                                Sound (400, 100);
                                Sound (800, 100);
                                Sound (1200, 100);
                                Sound (1400, 100);
                                Sound (1600, 100);
                                Sound (1900, 100);
                                Sound (2300, 100);
                                Sound (2600, 100);
                                Sound (2900, 100);
                                Sound (3200, 100);
                                Sound (3500, 100);
			}
			else if (beepType == BeepType_Fail)
			{
				Sound (3500, 100);
                                Sound (3200, 100);
                                Sound (2900, 100);
                                Sound (2600, 100);
                                Sound (2300, 100);
                                Sound (1900, 100);
                                Sound (1600, 100);
                                Sound (1400, 100);
                                Sound (1200, 100);
                                Sound (800, 100);
                                Sound (400, 100);
                                Sound (200, 100);
                                Sound (100, 100);
			}
			else if (beepType == BeepType_Bonus)
			{
				Sound (2200, 250);
				osDelay(250);
				Sound (1600, 250);
				osDelay(250);
				Sound (2200, 250);
				osDelay(500);

				Sound (2200, 250);
				osDelay(250);
				Sound (1600, 250);
				osDelay(250);
				Sound (2200, 250);
				osDelay(500);

				Sound (1770, 125);
				osDelay(125);
				Sound (1650, 125);
				osDelay(125);
				Sound (1450, 125);
				osDelay(124);
				Sound (2400, 125);
				osDelay(125);
				Sound (2200, 125);
				osDelay(125);
				Sound (2400, 125);
				osDelay(125);
				Sound (1450, 125);
				osDelay(125);
				Sound (1650, 125);
				osDelay(125);

				Sound (1770, 250);
				osDelay(250);
				Sound (1600, 250);
				osDelay(250);
				Sound (2200, 250);
			}
			else
			{
				Sound (2100, 100);
			}
		}
	}
}

//--------------------------------------------------------------------
void Beep(void)
{
	if(bBeepDisable == 1)
		return;

	uint16_t beep_sgnl = BeepType_Default;

	osMessageQueuePut(BeepQueueHandle, (void*) &beep_sgnl, 0,0);
}

//------------------------------------------------------------------------
void BeepCustom (uint16_t beepType)
{
        osMessageQueuePut(BeepQueueHandle, (void*) &beepType, 0,0);
}

//------------------------------------------------------------------------
static void Sound (uint32_t tone, uint32_t delay)
{

	MX_TIM2_Init_tone (tone);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	osDelay(delay);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_DeInit (&htim2);

}


