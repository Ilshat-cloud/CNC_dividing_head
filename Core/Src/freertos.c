/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wh1602.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  struct button_without_fix {
     GPIO_PinState pos_current;
     GPIO_PinState pos_previous;
     GPIO_PinState pos_out;
     GPIO_PinState pos_normal;
  };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void Flash_read();
uint32_t Flash_write();
FLASH_EraseInitTypeDef Erase;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define flash_read(address)  (*(uint32_t*) address)
void buttin_proc(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern IWDG_HandleTypeDef hiwdg;
uint8_t flag=0;
static struct button_without_fix UP_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},DOWN_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},
PLUS_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},MINUS_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},
ENTER_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},SW1_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET},
SW2_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET};
/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ButtonProcessin */
osThreadId_t ButtonProcessinHandle;
const osThreadAttr_t ButtonProcessin_attributes = {
  .name = "ButtonProcessin",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for LEDProcessing */
osThreadId_t LEDProcessingHandle;
const osThreadAttr_t LEDProcessing_attributes = {
  .name = "LEDProcessing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartButtonProcessing(void *argument);
void StartLedProcessing(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* creation of ButtonProcessin */
  ButtonProcessinHandle = osThreadNew(StartButtonProcessing, NULL, &ButtonProcessin_attributes);

  /* creation of LEDProcessing */
  LEDProcessingHandle = osThreadNew(StartLedProcessing, NULL, &LEDProcessing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(1);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartButtonProcessing */
/**
* @brief Function implementing the ButtonProcessin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonProcessing */
void StartButtonProcessing(void *argument)
{
  /* USER CODE BEGIN StartButtonProcessing */
  /* Infinite loop */
  for(;;)
  {
    buttin_proc(&UP_btn,Btn_UP_GPIO_Port,Btn_UP_Pin);
    buttin_proc(&DOWN_btn,Btn_Down_GPIO_Port,Btn_Down_Pin);
    buttin_proc(&PLUS_btn,Btn_Plus_GPIO_Port,Btn_Plus_Pin);
    buttin_proc(&MINUS_btn,Btn_Minus_GPIO_Port,Btn_Minus_Pin);
    buttin_proc(&ENTER_btn,Btn_Enter_GPIO_Port,Btn_Enter_Pin);
    buttin_proc(&SW1_btn,EndSW1_GPIO_Port,EndSW1_Pin);
    buttin_proc(&SW2_btn,EndSW2_GPIO_Port,EndSW2_Pin);

    
    osDelay(20);
  }
  /* USER CODE END StartButtonProcessing */
}

/* USER CODE BEGIN Header_StartLedProcessing */
/**
* @brief Function implementing the LEDProcessing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedProcessing */
void StartLedProcessing(void *argument)
{
  /* USER CODE BEGIN StartLedProcessing */
  /* Infinite loop */
    char R[16];
    osDelay(500);
    uint8_t choise=0,clear_lcd;
    
    
  /* Infinite loop */
  for(;;)
  {
    switch(flag)
    {
      //------------------------------startyem--------------------------------//
      case 0:
        Cursor(0,0);
        PrintStr("C");
        SendByte(0xBF,1);  //t
        PrintStr("ap");
        SendByte(0xBF,1);
        PrintStr("ye");
        SendByte(0xBC,1);   //m            
        PrintStr("? Kop-");
        SendByte(0xE0,1);   //D
        PrintStr("a");

        Cursor(1,0);
        SendByte(0xA9,1);  //Y
        SendByte(0xE3,1);  //d
        PrintStr("ep");
        SendByte(0xB6,1);  //j
        PrintStr(".-Hac");
        SendByte(0xBF,1);  //t
        PrintStr("po");
        SendByte(0xB9,1);  //i'
        SendByte(0xBA,1);  //k
        SendByte(0xB8,1);  //i
        
//        if (button>1000) 
//          {
//            ClearLCDScreen();
//            flag=2;
//            __HAL_TIM_SET_COUNTER(&htim3,P1);
//            choise=0;
//          } else if (button>100)
//          {
//            ClearLCDScreen();
//            __HAL_TIM_SET_COUNTER(&htim3,point[curPoint].target);
//            flag=1;
//            start=1;
//            worktime=0;
//            worktime_old=0;
//          }
      break; 
      //===========================================================//
      
      
      //-------------------------------main-----------------------------//
      case 1:
        if (clear_lcd){ClearLCDScreen();clear_lcd=0;}
        Cursor(0,0);
        SendByte(0xA4,1);  //Z
        PrintStr("a");
        SendByte(0xE3,1);  //d
        PrintStr("a");
        SendByte(0xBD,1);  //n
        SendByte(0xB8,1);  //i
        PrintStr("e: ");
//        sprintf(R,"%04d",target1);
        PrintStr(R);        
        SendByte(0xEF,1);  //0
        PrintStr("C");
         

      break; 
      //===========================================================//
      
      
      //-------------------------------set1-----------------------------// 
      case 2:
        Cursor(0,0);
        PrintStr("P");
//        sprintf(R,"%03d",P1);
        PrintStr(R);
        PrintStr(" I");
//        sprintf(R,"%03d",I1);
        PrintStr(R);
        PrintStr(" D");
//        sprintf(R,"%03d",D1);
        PrintStr(R);
        Cursor(1,0);
        SendByte(0xA1,1);  //G          
        SendByte(0xB8,1);  //i  
        PrintStr("c");
        SendByte(0xBF,1);  //t        
        PrintStr(":");
//        sprintf(R,"%03d",hysteresys);
        PrintStr(R);        
        SendByte(0xEF,1);  //0
        PrintStr("C Dir-");  
//        sprintf(R,"%01d",direct);
        PrintStr(R); 
      
      break;     
      //===========================================================//
      
      
      //-------------------------------set2-----------------------------//       
      
      case 3:
        Cursor(0,0);
        SendByte(0xE0,1);   //D
        PrintStr("a");        
        SendByte(0xBF,1);  //t      
        SendByte(0xC0,1);  //4  
        SendByte(0xB8,1);  //i 
        SendByte(0xBA,1);  //k
        PrintStr("-");
 
      break; 
      //===========================================================//
      
      
      //-------------------------------set3-----------------------------//       
      case 4:
        Cursor(0,0);
        SendByte(0xB1,1);  //9I
        PrintStr("p");
        SendByte(0xBA,1);  //k
        PrintStr("-");
        //sprintf(R,"%03d",brightness);
        PrintStr(R);
        PrintStr("% To");
        SendByte(0xC0,1);  //4
        PrintStr("e");
        SendByte(0xBA,1);  //k
        //sprintf(R,"%02d",point_num);
        PrintStr(R);
        Cursor(1,0);
        PrintStr("A");
        SendByte(0xB3,1);  //v
        SendByte(0xBF,1);  //t
        PrintStr("o");
        SendByte(0xB7,1);  //z
        PrintStr("a");
        SendByte(0xBE,1);   //n-p
        PrintStr("yc");
        SendByte(0xBA,1);  //k
        PrintStr("-");
      break; 
      //===========================================================//
      
      
      //-------------------------------set4-----------------------------// 
      case 5:
      
        Cursor(0,0);
        PrintStr("To");
        SendByte(0xC0,1);  //4
        SendByte(0xBA,1);  //k
        PrintStr("a");
//        sprintf(R,"%02d",point_n);
        PrintStr(R);
        PrintStr(" Te");
        SendByte(0xBC,1);   //m
        SendByte(0xBE,1);   //n-p
        PrintStr(R);
        Cursor(1,0);
        PrintStr("M");
        SendByte(0xB8,1);  //i
        SendByte(0xBD,1);  //n
        PrintStr(":");
        //sprintf(R,"%03d",point[point_n].Minuts);
        PrintStr(R);
        PrintStr(" Ce");
        SendByte(0xBA,1);  //k
        PrintStr(":");
        //sprintf(R,"%03d",point[point_n].Sec);
        PrintStr(R);   
      
      break; 
      //===========================================================//
      
      
      //-------------------------------set5-----------------------------// 
      case 6:
        Cursor(0,0);
        PrintStr("Coxpa");
        SendByte(0xBD,1);  //n
        SendByte(0xB8,1);  //i
        SendByte(0xBF,1);  //t
        SendByte(0xC4,1);  //b
        PrintStr(" ");
        SendByte(0xBD,1);  //n
        PrintStr("ac");
        SendByte(0xBF,1);  //t
        PrintStr("p.");
        Cursor(1,0);
        PrintStr("  ");
        SendByte(0xE0,1);  //D
        PrintStr("a      He");
        SendByte(0xBF,1);  //t
        switch (choise)
        {
          case 0:
            Cursor(1,4);
            break;
          case 1:
            Cursor(1,11);
            break;
        }
//        if (button>1000) 
//          {
//            if (!choise){
//              flag=0;
//              start=0;
//              curPoint=0;
//              choise=0;
//              NVIC_DisableIRQ(EXTI3_IRQn);
//              osThreadResume(Flash_taskHandle);
//            }else{
//              flag=0;
//              start=0;
//              curPoint=0;
//              choise=0;
//              
//
//            }
//          } else if (button>100) 
//          {
//            choise=(choise>=1)?0:choise+1;
//          } 
      
      break; 
    }
     //button=0;
    osDelay(350);    
  
  }
  /* USER CODE END StartLedProcessing */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void buttin_proc(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    button->pos_previous=button->pos_current;
    button->pos_current=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
    if ((button->pos_previous==button->pos_current)&&(button->pos_current!=button->pos_normal)){
      button->pos_out=GPIO_PIN_SET;
    }else{
      button->pos_out=GPIO_PIN_RESET;
    }
}


uint32_t Flash_write(){
  taskENTER_CRITICAL();
  uint32_t flash_ret;
  HAL_FLASH_Unlock();
  Erase.TypeErase=FLASH_TYPEERASE_PAGES;
  Erase.PageAddress=User_Page_Adress[0];
  Erase.NbPages=1;  //1kBytes
  HAL_FLASHEx_Erase(&Erase,&flash_ret);
  if (flash_ret==0xFFFFFFFF)
  {
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[0],(MAX_EU&0x0000FFFF)|((MIN_EU<<16)&0xFFFF0000));
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[1],(direct&0x000000FF)|((sensor<<8)&0x0000FF00)|((brightness<<16)&0xFFFF0000));
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[2],(hysteresys&0x000000FF)|((P1<<8)&0x0000FF00)|((I1<<16)&0x00FF0000)|((D1<<24)&0xFF000000));
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress[3],(autostart&0x000000FF)|((point_num<<8)&0x0000FF00));
//
//    
    
    HAL_FLASH_Lock();
  }
  taskEXIT_CRITICAL();
  return flash_ret;
  
}

void Flash_read(){
  //-------------------------Flash---------------------//
  if (flash_read(User_Page_Adress[0])!=0xFFFFFFFF)
  {
   
//    MAX_EU=flash_read(User_Page_Adress[0]);
//    MIN_EU=flash_read(User_Page_Adress[0])>>16;
//    direct=flash_read(User_Page_Adress[1]);
//    sensor=flash_read(User_Page_Adress[1])>>8;
//    brightness=flash_read(User_Page_Adress[1])>>16;
//    hysteresys=flash_read(User_Page_Adress[2]);
//    P1=flash_read(User_Page_Adress[2])>>8;  
//    I1=flash_read(User_Page_Adress[2])>>16;
//    D1=flash_read(User_Page_Adress[2])>>24; 
//    autostart=flash_read(User_Page_Adress[3]);
//    point_num=flash_read(User_Page_Adress[3])>>8;  
//    for (flash_i=0;flash_i<20;flash_i++)
//    {
//      point[flash_i].Minuts=flash_read(User_Page_Adress[flash_i+4]);
//      point[flash_i].Sec=flash_read(User_Page_Adress[flash_i+4])>>8;
//      point[flash_i].target=flash_read(User_Page_Adress[flash_i+4])>>16;  
//    }
  }
  
  //====================================================//
}



/* USER CODE END Application */

