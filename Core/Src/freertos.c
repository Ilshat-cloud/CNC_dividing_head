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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct button_without_fix {
  GPIO_PinState pos_current;
  GPIO_PinState pos_previous;
  GPIO_PinState pos_out;
  GPIO_PinState pos_normal;
  uint16_t hold_counter;
};
struct Step_DIR_EN_M_inv{
  uint8_t Step:1;               //step inverse 0-no
  uint8_t DIR:1;                //dir inverse 1-yes
  uint8_t EN:1;                 //En inverse 0-no
  uint8_t Mot_Left:1;           //mot inverse 0-left CW or CCW
};
struct Motor{
  GPIO_PinState ReachCtrlPoint;         //feedback frrom me module
  uint32_t      Pulses_per_rev;         //pulses per revolution
  uint16_t      Max_Speed;              //max speed of motor pulses/sec
  uint16_t      output_sp;              //how many impulses we have to send
  uint16_t      out_frequency;          //how fast we will send our impulses //todo double check 
  struct        Step_DIR_EN_M_inv;      //magic number for inversion of output pins
  uint8_t       Speed_sp;               //0-100% speed SP from settings
  uint8_t       ReachCtrlPoint_avalible; //0 not used, 1 wait for set, 2 waiting for reset
};

struct Motor M1 = {
  GPIO_PIN_RESET,    // ReachCtrlPoint
  72000,             // Pulses_per_rev
  54000,             // Max_Speed
  0,                 // output_sp
  222,               // out_frequency
  {0, 0, 0, 0},      // Step_DIR_EN_M_inv (Step=0, DIR=0, EN=0, Mot_Left=0)
  0,                 //Speed Setpoint 0-100 direct
  0                  //Speed Setpoint 0-100 reverse   
};
struct Motor M2 = {
  GPIO_PIN_RESET,    // ReachCtrlPoint
  72000,             // Pulses_per_rev
  54000,             // Max_Speed
  0,                 // output_sp
  222,               // out_frequency
  {0, 0, 0, 0},      // Step_DIR_EN_M_inv (Step=0, DIR=0, EN=0, Mot_Left=0)
  0,                 //Speed Setpoint 0-100 direct
  0                  //Speed Setpoint 0-100 reverse  
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
void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */  
extern IWDG_HandleTypeDef hiwdg;
uint8_t flag=0, backlight_on=1,motor_in_use=0,startyem=0;  //screenchoise flag
uint8_t tooth_sp=0, current_tooth=0;
int16_t Deept_of_cut_mm=0, Deept_of_cut_pulses=0; 
uint16_t Delay_switching=0;
static struct button_without_fix  UP_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  DOWN_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  PLUS_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  MINUS_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  ENTER_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  SW1_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  SW2_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  RCP1_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0},
                                  RCP2_btn={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET,0};
/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
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
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(100);
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
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
    buttin_proc(&UP_btn,Btn_UP_GPIO_Port,Btn_UP_Pin);
    buttin_proc(&DOWN_btn,Btn_Down_GPIO_Port,Btn_Down_Pin);
    buttin_proc(&PLUS_btn,Btn_Plus_GPIO_Port,Btn_Plus_Pin);
    buttin_proc(&MINUS_btn,Btn_Minus_GPIO_Port,Btn_Minus_Pin);
    buttin_proc(&ENTER_btn,Btn_Enter_GPIO_Port,Btn_Enter_Pin);
    buttin_proc_without_tim(&SW1_btn,EndSW1_GPIO_Port,EndSW1_Pin);
    buttin_proc_without_tim(&SW2_btn,EndSW2_GPIO_Port,EndSW2_Pin);
    buttin_proc_without_tim(&RCP1_btn,ReachCtrlPnt1_GPIO_Port,ReachCtrlPnt1_Pin);
    buttin_proc_without_tim(&RCP2_btn,ReachCtrlPnt2_GPIO_Port,ReachCtrlPnt2_Pin);
    
    
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
  InitializeLCD();
  osDelay(500);
  uint8_t screen_substrate=1, screen_cursor=0, screen_enter_set=0;
  
  /* Infinite loop */
  for(;;)
  {
    switch(flag){
      //------------------------------screen0--------------------------------------//
    case 0:  //default screen
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("SP:XXXt   M2:00%");
        Cursor(1,0);
        PrintStr("P:+1234mm  Start");
      }
      sprintf(R,"%03d",tooth_sp);  
      PrintByCoordinats(0,3,R);
      
      sprintf(R,"%02d",M2.Speed_sp);  
      PrintByCoordinats(0,13,R);
      
      sprintf(R,"%05d",Deept_of_cut_mm);
      PrintByCoordinats(1,2,R);
      //todo Deept_of_cut_pulses=Deept_of_cut_mm*M2.pulses per mm
      switch(screen_cursor){
      case 0:
        Cursor(0,0);          //default state for each screen
        screen_enter_set=0;
        break;
      case 1:
        Cursor(0,3);          //set setpoint 1-255 plus and minus
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            tooth_sp++;  //todo flash save
          }
          if (MINUS_btn.pos_out){
            tooth_sp--;
          }
        }
        break;
      case 2:
        Cursor(0,13);          //enter to increase or decrease M2 speed SP 0-99%
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Speed_sp++;  //todo flash save
            if (M2.Speed_sp >= 100) {
              M2.Speed_sp = 99;
            }
          }
          if (MINUS_btn.pos_out){
            M2.Speed_sp--;
            if (M2.Speed_sp >= 100) {
              M2.Speed_sp = 99;
            }
          }
        }
        break;
      case 3:
        Cursor(1,2);          //enter to increase or decrease M2 speed SP 0-99%
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            Deept_of_cut_mm+=PLUS_btn.hold_counter;  //todo flash save
            if (Deept_of_cut_mm >= 1000) {
              Deept_of_cut_mm = 999;
            }
          }
          if (MINUS_btn.pos_out){
            Deept_of_cut_mm-=PLUS_btn.hold_counter;
            if (Deept_of_cut_mm <= -1000) {
              Deept_of_cut_mm = -999;
            }
          }
        }
        break;
      case 4:
        Cursor(0,11);          //enter to start or stop process
        if (ENTER_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
          flag=8;  //startyem!!!
          startyem=1;
          break;  //this break will set us to next scan;
        }
        break;
      }
      
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<4)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=1;  //go to endswitches indication  
          screen_cursor=0;
        }
      }
      
      break; 
      //=========================screen0==============================================//
      
      //------------------------------screen1--------------------------------------//
    case 1:  //settings indication screen
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("RCP1:X Set Sw1:X");
        Cursor(1,0);
        PrintStr("RCP2:X Bl1 Sw2:X");
      }
      PrintByCoordinats(0,5,(M1.ReachCtrlPoint?"0":"1")); 
      PrintByCoordinats(0,15,(SW1_btn.pos_out?"0":"1")); 
      PrintByCoordinats(1,5,(M2.ReachCtrlPoint?"0":"1"));      
      PrintByCoordinats(1,9,(backlight_on?"0":"1"));
      PrintByCoordinats(1,15,(SW1_btn.pos_out?"0":"1"));
      
      
      
      if(screen_cursor==1){
        Cursor(0,7);          //enter to settings menu go to next screen
        if (ENTER_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
          flag=2;
          break;  //this break will set us to next scan;
        } 
      }else if (screen_cursor==2){
        Cursor(1,9);
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            backlight_on=1;  //todo flash save
          }
          if (MINUS_btn.pos_out){
            backlight_on=0;
          }
        }
      }else{
        Cursor(0,0);
      }
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
        
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<2)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (UP_btn.pos_out){
          flag=0;  //go to endswitches indication  
          screen_cursor=0;
        }
      }
      break; 
            //=========================screen1==============================================//
      
      //------------------------------screen2--------------------------------------//
    case 2:  //settings reach contrl piont check and endswitches
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("Sw1_i:x Del:XXXX");  //del-delay betwen switching motors (in case if you dont use RCP)
        Cursor(1,0);
        PrintStr("Sw2_i:x F1:1F2:1");
      }
      PrintByCoordinats(0,6,(SW1_btn.pos_normal?"0":"1"));
      sprintf(R,"%04d",Delay_switching);  
      PrintByCoordinats(0,12,R);
      PrintByCoordinats(1,6,(SW2_btn.pos_normal?"0":"1"));
      PrintByCoordinats(1,11,((M1.ReachCtrlPoint_avalible>0)?((M1.ReachCtrlPoint_avalible==1)?"1":"2"):"0"));
      PrintByCoordinats(1,15,((M2.ReachCtrlPoint_avalible>0)?((M2.ReachCtrlPoint_avalible==1)?"1":"2"):"0"));
      switch(screen_cursor){
      case 0:
        Cursor(0,0);          //default state for each screen
        screen_enter_set=0;
        break;
      case 1:
        Cursor(0,6);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            SW1_btn.pos_normal=GPIO_PIN_SET;
          }
          if (MINUS_btn.pos_out){
            SW1_btn.pos_normal=GPIO_PIN_RESET;
          }
        }
        break;
       case 2:
        Cursor(0,12);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            Delay_switching+=PLUS_btn.hold_counter;
            if (Delay_switching>5000){
              Delay_switching=5000;
            }
          }
          if (MINUS_btn.pos_out){
            Delay_switching-=PLUS_btn.hold_counter;
            if (Delay_switching>5000){
              Delay_switching=0;
            }
          }
        }
        break;
      case 3:
        Cursor(1,6);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            SW2_btn.pos_normal=GPIO_PIN_SET;
          }
          if (MINUS_btn.pos_out){
            SW2_btn.pos_normal=GPIO_PIN_RESET;
          }
        }
        break;
      case 4:
        Cursor(1,11);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.ReachCtrlPoint_avalible++;
            if (M1.ReachCtrlPoint_avalible>2){
              M1.ReachCtrlPoint_avalible=2;
            }
          }
          if (MINUS_btn.pos_out){
            M1.ReachCtrlPoint_avalible--;
            if (M1.ReachCtrlPoint_avalible>2){
              M1.ReachCtrlPoint_avalible=0;
            }
          }
        }
        break;  
      case 5:
        Cursor(1,15);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.ReachCtrlPoint_avalible++;
            if (M2.ReachCtrlPoint_avalible>2){
              M2.ReachCtrlPoint_avalible=2;
            }
          }
          if (MINUS_btn.pos_out){
            M2.ReachCtrlPoint_avalible--;
            if (M2.ReachCtrlPoint_avalible>2){
              M2.ReachCtrlPoint_avalible=0;
            }
          }
        }
        break;
      }
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<4)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=3;  //go to next menu  
          screen_cursor=0;
        }
      }
      break;        
      //=========================screen2==============================================//
      
      //------------------------------screen3--------------------------------------//
    case 3:   //motor out signal inversions
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("M1:L StDirEn:111");
        Cursor(1,0);
        PrintStr("M2:L StDirEn:111");
      }
      
      PrintByCoordinats(0,3,(M1.Mot_Left?"L":"R"));
      PrintByCoordinats(0,13,(M1.Step?"1":"0"));
      PrintByCoordinats(0,14,(M1.DIR?"1":"0"));
      PrintByCoordinats(0,15,(M1.EN?"1":"0"));
      PrintByCoordinats(1,3,(M2.Mot_Left?"L":"R"));
      PrintByCoordinats(1,13,(M2.Step?"1":"0"));
      PrintByCoordinats(1,14,(M2.DIR?"1":"0"));
      PrintByCoordinats(1,15,(M2.EN?"1":"0"));
      
      switch(screen_cursor){
      case 0:
        Cursor(0,0);          //default state for each screen
        screen_enter_set=0;
        break;
      case 1:
        Cursor(0,3);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Mot_Left=1;
          }
          if (MINUS_btn.pos_out){
            M1.Mot_Left=0;
          }
        }
        break;
      case 2:
        Cursor(0,13);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Step=1;
          }
          if (MINUS_btn.pos_out){
            M1.Step=0;
          }
        }
        break;
      case 3:
        Cursor(0,14);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.DIR=1;
          }
          if (MINUS_btn.pos_out){
            M1.DIR=0;
          }
        }
        break;        
      case 4:
        Cursor(0,15);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.EN=1;
          }
          if (MINUS_btn.pos_out){
            M1.EN=0;
          }
        }
        break;
      case 5:
        Cursor(1,3);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Mot_Left=1;
          }
          if (MINUS_btn.pos_out){
            M2.Mot_Left=0;
          }
        }        
        break;
      case 6:
        Cursor(1,13);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Step=1;
          }
          if (MINUS_btn.pos_out){
            M2.Step=0;
          }
        }            
        break; 
      case 7:
        Cursor(1,14);         
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.DIR =1;
          }
          if (MINUS_btn.pos_out){
            M2.DIR =0;
          }
        }            
        break;        
      case 8:
        Cursor(1,15);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.EN =1;
          }
          if (MINUS_btn.pos_out){
            M2.EN =0;
          }
        }            
        break;        
      }
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<7)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=4;  //go to next menu  
          screen_cursor=0;
        }
      }
      break; 
      
      //=========================screen3==============================================//
      
      //------------------------------screen4--------------------------------------//
    case 4:   //motor puleses per revolution
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("M1_P:XXXXX  Test");
        Cursor(1,0);
        PrintStr("M2_P:XXXXX  Test");
      }
      sprintf(R,"%05d",M1.Pulses_per_rev);  
      PrintByCoordinats(0,5,R);
      sprintf(R,"%05d",M2.Pulses_per_rev);  
      PrintByCoordinats(1,5,R);
      
      switch(screen_cursor){
      case 0:
        Cursor(0,0);          //default state for each screen
        screen_enter_set=0;
        break;
      case 1:
        Cursor(0,5);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Pulses_per_rev+=PLUS_btn.hold_counter;
            if (M1.Pulses_per_rev>50000){
              M1.Pulses_per_rev=50000;
            }
          }
          if (MINUS_btn.pos_out){
            M1.Pulses_per_rev-=PLUS_btn.hold_counter;
            if (M1.Pulses_per_rev>50000){
              M1.Pulses_per_rev=0;
            }
          }
        }
        break;
      case 2:    
         Cursor(0,12); 
        break;      
      case 3:   
        Cursor(1,5);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Pulses_per_rev+=PLUS_btn.hold_counter;
            if (M2.Pulses_per_rev>20000){
              M2.Pulses_per_rev=20000;
            }
          }
          if (MINUS_btn.pos_out){
            M2.Pulses_per_rev-=PLUS_btn.hold_counter;
            if (M2.Pulses_per_rev>20000){
              M2.Pulses_per_rev=0;
            }
          }
        }
        break;
      case 4:     
          Cursor(1,12); 
        break;
      }        
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<4)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=5;  //go to next menu  
          screen_cursor=0;
        }
      }
      
      break; 
      //=========================screen4==============================================//
      
      //------------------------------screen5--------------------------------------//
    case 5:   //motor max speed, may be in future here will be autodjust according to RCP pin
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("M1_M:XXXXX  Test");
        Cursor(1,0);
        PrintStr("M2_M:XXXXX  Test"); 
      }
      sprintf(R,"%05d",M1.Max_Speed);  
      PrintByCoordinats(0,5,R);
      sprintf(R,"%05d",M2.Max_Speed);  
      PrintByCoordinats(1,5,R);  
           switch(screen_cursor){
      case 0:
        Cursor(0,0);          //default state for each screen
        screen_enter_set=0;
        break;
      case 1:
        Cursor(0,5);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Max_Speed+=PLUS_btn.hold_counter;
            if (M1.Max_Speed>10000){
              M1.Max_Speed=10000;
            }
          }
          if (MINUS_btn.pos_out){
            M1.Max_Speed-=PLUS_btn.hold_counter;
            if (M1.Max_Speed>10000){
              M1.Max_Speed=0;
            }
          }
        }
        break;
      case 2:    
         Cursor(0,12); 
        break;      
      case 3:   
        Cursor(1,5);          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Max_Speed+=PLUS_btn.hold_counter;
            if (M2.Max_Speed>10000){
              M2.Max_Speed=10000;
            }
          }
          if (MINUS_btn.pos_out){
            M2.Max_Speed-=PLUS_btn.hold_counter;
            if (M2.Max_Speed>10000){
              M2.Max_Speed=0;
            }
          }
        }
        break;
      case 4:     
          Cursor(1,12); 
        break;
      }
      
      if (screen_enter_set){
        if (UP_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
        }
      }else{
        if (PLUS_btn.pos_out){
          
          (screen_cursor<4)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=6;  //go to next menu  
          screen_cursor=0;
        }
      }
      break; 
      //=========================screen5==============================================//
      
      //------------------------------screen6--------------------------------------//
    case 6:  //screen sucsess
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("     Sucsess    ");
        Cursor(1,0);
        PrintStr("****************"); 
        osDelay(delay_for_sucsess_screen);
        flag=0;
      }
      break; 
      //=========================screen6==============================================//
      
      //------------------------------screen7--------------------------------------//
    case 7:
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("  Error:        ");
        Cursor(1,0);
        PrintStr("****************"); 
        osDelay(delay_for_sucsess_screen);
      }
      break;    
      //=========================screen7==============================================//
      
      //------------------------------screen8--------------------------------------//
    case 8:  //in work
      if(flag!=screen_substrate){
        screen_substrate=flag;
        Cursor(0,0);
        PrintStr("SP:XXX  Cur:XXX ");
        Cursor(1,0);
        PrintStr("MX R1:0 R2:0  11"); 
      }
      sprintf(R,"%03d",tooth_sp);  
      PrintByCoordinats(0,3,R);
      sprintf(R,"%03d",tooth_sp);  
      PrintByCoordinats(0,12,R);
      PrintByCoordinats(1,1,(motor_in_use?"2":"1"));
      PrintByCoordinats(1,6,(M1.ReachCtrlPoint?"1":"0"));
      PrintByCoordinats(1,11,(M2.ReachCtrlPoint?"1":"0"));
      PrintByCoordinats(1,14,(SW1_btn.pos_out?"1":"0"));
      PrintByCoordinats(1,15,(SW2_btn.pos_out?"1":"0"));
      break;
      //=========================screen8==============================================//
    }
    osDelay(50);    
    
  }
  /* USER CODE END StartLedProcessing */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint32_t Flash_write(){
  taskENTER_CRITICAL();
  uint32_t flash_ret;
  HAL_FLASH_Unlock();
  Erase.TypeErase=FLASH_TYPEERASE_PAGES;
  Erase.PageAddress=User_Page_Adress[0];
  Erase.NbPages=1;
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



void buttin_proc(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  button->pos_previous=button->pos_current;
  button->pos_current=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
  if ((button->pos_previous==button->pos_current)&&(button->pos_current!=button->pos_normal)){
    button->pos_out=GPIO_PIN_SET;
    if (button->hold_counter<6){
      button->hold_counter++;
    }else if(button->hold_counter<50){
      button->hold_counter+=5;
    }else if(button->hold_counter<250){
      button->hold_counter+=20;
    }else if(button->hold_counter>1250){
      return;
    }
  }else{
    button->pos_out=GPIO_PIN_RESET;
    button->hold_counter=0;
  }
}


void buttin_proc_without_tim(struct button_without_fix *button,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  button->pos_previous=button->pos_current;
  button->pos_current=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
  if ((button->pos_previous==button->pos_current)&&(button->pos_current!=button->pos_normal)){
    button->pos_out=GPIO_PIN_SET;
  }else{
    button->pos_out=GPIO_PIN_RESET;
    button->hold_counter=0;
  }
}


uint32_t Flash_write(){
  taskENTER_CRITICAL();
  uint32_t flash_ret;
  HAL_FLASH_Unlock();
  Erase.TypeErase=FLASH_TYPEERASE_PAGES;
  Erase.PageAddress=User_Page_Adress[0];
  Erase.NbPages=1;  //1kBytes
  //  Delay_switching backlight_on tooth_sp Deept_of_cut_mm Deept_of_cut_pulses M1
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

