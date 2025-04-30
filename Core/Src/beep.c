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

extern TIM_HandleTypeDef htim1;
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
				Sound (1396, 173);
				osDelay (192 + 96);
				Sound (1396, 86);
			}
			else if (beepType == BeepType_Fail)
			{
				Sound (200, 500);
			}
			else if (beepType == BeepType_Bonus)
			{
				Sound (440, 250);
				osDelay(250);
				Sound (165, 250);
				osDelay(250);
				Sound (440, 250);
				osDelay(500);

				Sound (440, 250);
				osDelay(250);
				Sound (165, 250);
				osDelay(250);
				Sound (440, 250);
				osDelay(500);

				Sound (330, 125);
				osDelay(125);
				Sound (294, 125);
				osDelay(125);
				Sound (262, 125);
				osDelay(124);
				Sound (494, 125);
				osDelay(125);
				Sound (440, 125);
				osDelay(125);
				Sound (494, 125);
				osDelay(125);
				Sound (262, 125);
				osDelay(125);
				Sound (294, 125);
				osDelay(125);

				Sound (330, 250);
				osDelay(250);
				Sound (165, 250);
				osDelay(250);
				Sound (440, 250);
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
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	osDelay(delay);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_DeInit (&htim1);

}


