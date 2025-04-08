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
  int32_t       output_sp;              //how many impulses we have to send
  uint16_t      out_frequency;          //how fast we will send our impulses //todo double check 
  struct        Step_DIR_EN_M_inv Step_DIR_EN_M_inv;      //magic number for inversion of output pins
  uint8_t       Speed_sp;               //0-100% speed SP from settings
  uint8_t       ReachCtrlPoint_avalible; //0 not used, 1 wait for set, 2 waiting for reset
};

struct Motor M1 = {  //мотор для поворота
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

extern TIM_HandleTypeDef htim1;
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
void Set_period_and_start_TIM(TIM_HandleTypeDef *htim,uint16_t period);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */  
extern IWDG_HandleTypeDef hiwdg;
uint8_t flag=0, backlight_on=1,motor_in_use=0,startyem=0,error=0;  //screenchoise flag
uint8_t tooth_sp=0, current_tooth=0,screen_cursor=0, screen_enter_set=0;
uint8_t screen_substrate=1;
int16_t Deept_of_cut_mm=0; 
uint16_t Delay_switching=0;
uint32_t Pulses_for_tooth=0;
int32_t Pulses_for_deptofcut=0; //may be in bouth directions
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
  Flash_read();
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
  static uint8_t motor_in_use_old=0;
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    
    osDelay(10);
    if(flag==SCREEN_STARTUEM){
      if(SW1_btn.pos_out)
      {
        error=ERROR_SW1;
      }
      if(SW2_btn.pos_out)
      {
        error=ERROR_SW2;
      }
    }
    if(motor_in_use_old!=motor_in_use){
      motor_in_use_old=motor_in_use;
      if(motor_in_use==1){
        uint16_t period=M1.Max_Speed/(100-M1.Speed_sp);
        Set_period_and_start_TIM(&htim1,period); //1000000/period
      }else if(motor_in_use==2) {
        uint16_t period2=M2.Max_Speed/(100-M2.Speed_sp);
        Set_period_and_start_TIM(&htim1,period2); //1000000/period
      }
    }
    
    
    
    
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
    osDelay(75);
    
    //-------------------navigation--------------------//
    switch(flag){
      //------------------------------screen0--------------------------------------//
    case SCREEN_MAIN:  //default screen
      switch(screen_cursor){
      case 0:
        screen_enter_set=0;
        break;
      case 1:
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
            Deept_of_cut_mm-=MINUS_btn.hold_counter;
            if (Deept_of_cut_mm <= -1000) {
              Deept_of_cut_mm = -999;
            }
          }
        }
        
        break;
      case 4:
        if (ENTER_btn.pos_out){
          if (tooth_sp){
            Pulses_for_tooth=M1.Pulses_per_rev/tooth_sp;
          }
          Pulses_for_deptofcut=M2.Pulses_per_rev*Deept_of_cut_mm;
          screen_enter_set=0;
          screen_cursor=0;
          flag=SCREEN_STARTUEM;  //startyem!!!
          startyem=1;
          break;  //this break will set us to next scan;
        }
        
        break;
      }
      
      if (screen_enter_set){
        if(UP_btn.pos_out){
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
          flag=SCREEN_MAIN2;  //go to endswitches indication  
          screen_cursor=0;
        }
      }
      break;
      //------------------------------screen1--------------------------------------//
    case SCREEN_MAIN2:  //settings indication screen
      if(screen_cursor==1){
        if (ENTER_btn.pos_out){
          screen_enter_set=0;
          screen_cursor=0;
          flag=SCREEN_SETTINGS1;
          break;  //this break will set us to next scan;
        } 
      }else if (screen_cursor==2){
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            backlight_on=1;  //todo flash save
          }
          if (MINUS_btn.pos_out){
            backlight_on=0;
          }
        }
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
          flag=SCREEN_MAIN;  //go to endswitches indication  
          screen_cursor=0;
        }
      }
      break;
      //------------------------------screen2--------------------------------------//
    case SCREEN_SETTINGS1:  //settings reach contrl piont check and endswitches     
      switch(screen_cursor){
      case 0:
        screen_enter_set=0;
        break;
      case 1:     
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
          
          (screen_cursor<5)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=SCREEN_SETTINGS2;  //go to next menu  
          screen_cursor=0;
        }
      }
      break;        
      //=========================screen2==============================================//
      
      //------------------------------screen3--------------------------------------//
    case SCREEN_SETTINGS2:   //motor out signal inversions
      switch(screen_cursor){
      case 0:
        screen_enter_set=0;
        break;
      case 1:       
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.Mot_Left=1;
          }
          if (MINUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.Mot_Left=0;
          }
        }
        break;
      case 2:          
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.Step=1;
          }
          if (MINUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.Step=0;
          }
        }
        break;
      case 3:       
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.DIR=1;
          }
          if (MINUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.DIR=0;
          }
        }
        break;        
      case 4:        
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.EN=1;
          }
          if (MINUS_btn.pos_out){
            M1.Step_DIR_EN_M_inv.EN=0;
          }
        }
        break;
      case 5:         
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.Mot_Left=1;
          }
          if (MINUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.Mot_Left=0;
          }
        }        
        break;
      case 6:         
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.Step=1;
          }
          if (MINUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.Step=0;
          }
        }            
        break; 
      case 7:     
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.DIR =1;
          }
          if (MINUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.DIR =0;
          }
        }            
        break;        
      case 8:       
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.EN =1;
          }
          if (MINUS_btn.pos_out){
            M2.Step_DIR_EN_M_inv.EN =0;
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
          
          (screen_cursor<8)?screen_cursor++:0;
        }
        if (MINUS_btn.pos_out){
          
          (screen_cursor>0)?screen_cursor--:0;
        }
        if (DOWN_btn.pos_out){
          flag=SCREEN_SETTINGS3;  //go to next menu  
          screen_cursor=0;
        }
      }
      break; 
      
      //=========================screen3==============================================//
      
      //------------------------------screen4--------------------------------------//
    case SCREEN_SETTINGS3:   //motor puleses per revolution
      switch(screen_cursor){
      case 0:
        screen_enter_set=0;
        break;
      case 1:      
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
            M1.Pulses_per_rev-=MINUS_btn.hold_counter;
            if (M1.Pulses_per_rev>50000){
              M1.Pulses_per_rev=0;
            }
          }
        }
        break;
      case 2:    
        break;      
      case 3:           
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
            M2.Pulses_per_rev-=MINUS_btn.hold_counter;
            if (M2.Pulses_per_rev>20000){
              M2.Pulses_per_rev=0;
            }
          }
        }
        break;
      case 4:     
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
          flag=SCREEN_SETTINGS4;  //go to next menu  
          screen_cursor=0;
        }
      }
      
      break; 
      //=========================screen4==============================================//
      
      //------------------------------screen5--------------------------------------//
    case SCREEN_SETTINGS4:   //motor max speed, may be in future here will be autodjust according to RCP pin
      switch(screen_cursor){
      case 0:
        screen_enter_set=0;
        break;
      case 1:        
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M1.Max_Speed+=PLUS_btn.hold_counter;
            
          }
          if (MINUS_btn.pos_out){
            M1.Max_Speed-=MINUS_btn.hold_counter;
          }
        }
        break;
      case 2:    
        
        break;      
      case 3:   
        
        if (ENTER_btn.pos_out){
          screen_enter_set=1;
        }
        if(screen_enter_set){
          if (PLUS_btn.pos_out){
            M2.Max_Speed+=PLUS_btn.hold_counter;
          }
          if (MINUS_btn.pos_out){
            M2.Max_Speed-=MINUS_btn.hold_counter;
          }
        }
        break;
      case 4:     
        
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
          flag=SCREEN_MAIN;  //go to next menu  
          screen_cursor=0;
        }
      }
      break; 
      //=========================screen5==============================================//    
      
      //------------------------------screen6--------------------------------------//
    case SCREEN_SUCSESS:  //screen sucsess
      osDelay(delay_for_sucsess_screen);
      flag=SCREEN_MAIN;
      break; 
      
      //------------------------------screen7--------------------------------------//
    case SCREEN_ERROR:
      if (UP_btn.pos_out){
        error=ERROR_NONE;
        flag=SCREEN_MAIN;
      }
      break;    
      //=========================screen7==============================================//
      //------------------------------screen8--------------------------------------//
    case SCREEN_STARTUEM:  //in work
      
      break;
      //=========================screen8==============================================//
    }    
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
  char R[17];
  InitializeLCD();
  osDelay(500);
  
  uint8_t toggle=0;
  /* Infinite loop */
  for(;;)
  {
    toggle++;
    if (screen_enter_set){
      toggle++;}
    switch(flag){
      //------------------------------screen0--------------------------------------//
    case SCREEN_MAIN:  //default screen
      
      //todo Deept_of_cut_pulses=Deept_of_cut_mm*M2.pulses per mm
      
      if (toggle&0x02){
        sprintf(R,"SP:%03d t  M2:%03d",tooth_sp,M2.Speed_sp); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"P:%05dmm  Start",Deept_of_cut_mm); 
        PrintByCoordinats(1,0,R);
      }else{
        
        switch(screen_cursor){
        case 0:
          sprintf(R,"SP:%03d t  M2:%03d",tooth_sp,M2.Speed_sp); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"P:%05dmm  Start",Deept_of_cut_mm); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"SP:    t  M2:%03d",M2.Speed_sp); 
          PrintByCoordinats(0,0,R);
          break;
        case 2:
          sprintf(R,"SP:%03d t  M2:   ",tooth_sp); 
          PrintByCoordinats(0,0,R);
          break;
        case 3:
          sprintf(R,"P:     mm  Start"); 
          PrintByCoordinats(1,0,R);
          break;
        case 4:
          sprintf(R,"P:%05dmm       ",Deept_of_cut_mm); 
          PrintByCoordinats(1,0,R);
          break;
        }
        
      }    
      break; 
      //=========================screen0==============================================//
      
      //------------------------------screen1--------------------------------------//
    case SCREEN_MAIN2:  //settings indication screen
      
      
      
      if (toggle&0x02){
        sprintf(R,"RCP1:%01d Set Sw1:%01d",M1.ReachCtrlPoint,SW1_btn.pos_out); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"RCP2:%01d Bl%01d Sw2:%01d",M2.ReachCtrlPoint,backlight_on,SW1_btn.pos_out); 
        PrintByCoordinats(1,0,R);
      }else{
        
        switch(screen_cursor){
        case 0:
          sprintf(R,"RCP1:%01d Set Sw1:%01d",M1.ReachCtrlPoint,SW1_btn.pos_out); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"RCP2:%01d Bl%01d Sw2:%01d",M2.ReachCtrlPoint,backlight_on,SW1_btn.pos_out); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"RCP1:%01d     Sw1:%01d",M1.ReachCtrlPoint,SW1_btn.pos_out); 
          PrintByCoordinats(0,0,R);
          break;
        case 2:
          sprintf(R,"RCP2:%01d Bl  Sw2:%01d",M2.ReachCtrlPoint,SW1_btn.pos_out); 
          PrintByCoordinats(1,0,R);
          break;
        }
        
      }         
      
      
      break; 
      //=========================screen1==============================================//
      
      //------------------------------screen2--------------------------------------//
    case SCREEN_SETTINGS1:  //settings reach contrl piont check and endswitches
      if (toggle&0x02){
        sprintf(R,"Sw1_i:%01d Del:%04d",SW1_btn.pos_normal,Delay_switching); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"Sw2_i%01d F1:%01d F2:%01d",SW2_btn.pos_normal,M1.ReachCtrlPoint_avalible,M2.ReachCtrlPoint_avalible); 
        PrintByCoordinats(1,0,R);
      }else{
        switch(screen_cursor){
        case 0:
          sprintf(R,"Sw1_i:%01d Del:%04d",SW1_btn.pos_normal,Delay_switching); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i%01d F1:%01d F2:%01d",SW2_btn.pos_normal,M1.ReachCtrlPoint_avalible,M2.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"Sw1_i:  Del:%04d",Delay_switching); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i%01d F1:%01d F2:%01d",SW2_btn.pos_normal,M1.ReachCtrlPoint_avalible,M2.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;
        case 2:
          sprintf(R,"Sw1_i:%01d Del:    ",SW1_btn.pos_normal); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i%01d F1:%01d F2:%01d",SW2_btn.pos_normal,M1.ReachCtrlPoint_avalible,M2.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;
        case 3:
          sprintf(R,"Sw1_i:%01d Del:%04d",SW1_btn.pos_normal,Delay_switching); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i  F1:%01d F2:%01d",M1.ReachCtrlPoint_avalible,M2.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;
        case 4:
          sprintf(R,"Sw1_i:%01d Del:%04d",SW1_btn.pos_normal,Delay_switching); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i%01d F1:  F2:%01d",SW2_btn.pos_normal,M2.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;  
        case 5:
          sprintf(R,"Sw1_i:%01d Del:%04d",SW1_btn.pos_normal,Delay_switching); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"Sw2_i%01d F1:%01d F2: ",SW2_btn.pos_normal,M1.ReachCtrlPoint_avalible); 
          PrintByCoordinats(1,0,R);
          break;
        }
      }
      break;        
      //=========================screen2==============================================//
      
      //------------------------------screen3--------------------------------------//
    case SCREEN_SETTINGS2:   //motor out signal inversions
      
      if (toggle&0x02){
        sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
        PrintByCoordinats(1,0,R);
      }else{
        switch(screen_cursor){
        case 0:
          sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"M1:  StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);
          break;
        case 2:
          sprintf(R,"M1:%s StDirEn: %01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);
          break;
        case 3:
          sprintf(R,"M1:%s StDirEn:%01d %01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);
          break;        
        case 4:
          sprintf(R,"M1:%s StDirEn:%01d%01d ",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);
          break;
        case 5:
          sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:  StDirEn:%01d%01d%01d",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);     
          break;
        case 6:
          sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn: %01d%01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.DIR,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);            
          break; 
        case 7:
          sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d %01d",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(1,0,R);        
          break;        
        case 8:
          sprintf(R,"M1:%s StDirEn:%01d%01d%01d",M1.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M1.Step_DIR_EN_M_inv.Step,M1.Step_DIR_EN_M_inv.DIR,M1.Step_DIR_EN_M_inv.EN); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2:%s StDirEn:%01d%01d ",M2.Step_DIR_EN_M_inv.Mot_Left?"L":"R",M2.Step_DIR_EN_M_inv.Step,M2.Step_DIR_EN_M_inv.DIR); 
          PrintByCoordinats(1,0,R);          
          break;        
        }
      }
      break; 
      
      //=========================screen3==============================================//
      
      //------------------------------screen4--------------------------------------//
    case SCREEN_SETTINGS3:   //motor puleses per revolution
      if (toggle&0x02){
        sprintf(R,"M1_P:%05d  Test",M1.Pulses_per_rev); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"M2_P:%05d  Test",M2.Pulses_per_rev); 
        PrintByCoordinats(1,0,R);
      }else{    
        switch(screen_cursor){
        case 0:
          sprintf(R,"M1_P:%05d  Test",M1.Pulses_per_rev); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_P:%05d  Test",M2.Pulses_per_rev); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"M1_P:       Test"); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_P:%05d  Test",M2.Pulses_per_rev); 
          PrintByCoordinats(1,0,R);
          break;
        case 2:    
          sprintf(R,"M1_P:%05d      ",M1.Pulses_per_rev); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_P:%05d  Test",M2.Pulses_per_rev); 
          PrintByCoordinats(1,0,R);
          break;      
        case 3:   
          sprintf(R,"M1_P:%05d  Test",M1.Pulses_per_rev); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_P:       Test"); 
          PrintByCoordinats(1,0,R);
          break;
        case 4:     
          sprintf(R,"M1_P:%05d  Test",M1.Pulses_per_rev); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_P:%05d      ",M2.Pulses_per_rev); 
          PrintByCoordinats(1,0,R);
          break;
        }        
      }
      break; 
      //=========================screen4==============================================//
      
      //------------------------------screen5--------------------------------------//
    case SCREEN_SETTINGS4:   //motor max speed, may be in future here will be autodjust according to RCP pin
      if (toggle&0x02){
        sprintf(R,"M1_M:%05d  Test",M1.Max_Speed); 
        PrintByCoordinats(0,0,R);
        sprintf(R,"M2_M:%05d  Test",M2.Max_Speed); 
        PrintByCoordinats(1,0,R);
      }else{          
        switch(screen_cursor){
        case 0:
          sprintf(R,"M1_M:%05d  Test",M1.Max_Speed); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_M:%05d  Test",M2.Max_Speed); 
          PrintByCoordinats(1,0,R);
          break;
        case 1:
          sprintf(R,"M1_M:       Test"); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_M:%05d  Test",M2.Max_Speed); 
          PrintByCoordinats(1,0,R);
          break;
        case 2:    
          sprintf(R,"M1_M:%05d      ",M1.Max_Speed); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_M:%05d  Test",M2.Max_Speed); 
          PrintByCoordinats(1,0,R);
          break;      
        case 3:   
          sprintf(R,"M1_M:%05d  Test",M1.Max_Speed); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_M:       Test"); 
          PrintByCoordinats(1,0,R);
          break;
        case 4:     
          sprintf(R,"M1_M:%05d  Test",M1.Max_Speed); 
          PrintByCoordinats(0,0,R);
          sprintf(R,"M2_M:%05d      ",M2.Max_Speed); 
          PrintByCoordinats(1,0,R);
          break;
        }
      }
      break; 
      //=========================screen5==============================================//
      
      //------------------------------screen6--------------------------------------//
    case SCREEN_SUCSESS:  //screen sucsess
      Cursor(0,0);
      PrintStr("     Sucsess    ");
      Cursor(1,0);
      PrintStr("****************"); 
      break; 
      //=========================screen6==============================================//
      
      //------------------------------screen7--------------------------------------//
    case SCREEN_ERROR:
      Cursor(0,0);
      PrintStr("  Error:        ");
      Cursor(1,0);
      switch (error){
      case ERROR_NONE:
        PrintStr("****************"); 
        break;
      case ERROR_SW1:
        PrintStr("END Switch 1    "); 
        break;
      case ERROR_SW2:
        PrintStr("END Switch 2    "); 
        break;
      case ERROR_RCP1_TIMEOUT:
        PrintStr("timeout CTRLP M1"); 
        break;
      case ERROR_RCP2_TIMEOUT:
        PrintStr("timeout CTRLP M2"); 
        break;
      case ERROR_SYSTEM:
        PrintStr("zvonit_4et_ne_to"); 
        break;    
      case ERROR_FLASH_W:
        PrintStr("Flash err write "); 
        break;           
      case ERROR_FLASH_R:
        PrintStr("Flash error read"); 
        break;      
      }
      break;    
      //=========================screen7==============================================//
      
      //------------------------------screen8--------------------------------------//
    case SCREEN_STARTUEM:  //in work
      sprintf(R,"SP:%03d  Cur:%03d ",tooth_sp,tooth_sp); //TODO tooth_sp изменить на текущий зуб
      PrintByCoordinats(0,0,R);
      sprintf(R,"M%01d R1:%01d R2:%01d  %01d%01d",motor_in_use,M1.ReachCtrlPoint, M2.ReachCtrlPoint,SW1_btn.pos_out,SW2_btn.pos_out); 
      PrintByCoordinats(1,0,R);   
      
      break;
      //=========================screen8==============================================//
    }
    osDelay(250);    
    if (backlight_on){
      HAL_GPIO_WritePin(backlight_GPIO_Port,backlight_Pin,GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(backlight_GPIO_Port,backlight_Pin,GPIO_PIN_RESET);
    }
    if(error){
      flag=SCREEN_ERROR;
    }
  }
  /* USER CODE END StartLedProcessing */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*
проверяем только pos_out, он зависит от нормального положения кнопки и выставляется через 2 скана нажатия (фильтр дребезга + конденсаторы на плате еще)
так же есть счетчик для удержания, в общем все нормалды
*/
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
  if (HAL_FLASHEx_Erase(&Erase, &flash_ret) != HAL_OK) {
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
    error=ERROR_FLASH_W;
    return flash_ret;
  }
  // Упаковка данных кнопок (9 бит)
  uint32_t buttons_data = 
    (UP_btn.pos_normal    << 0) |
      (DOWN_btn.pos_normal  << 1) |
        (PLUS_btn.pos_normal  << 2) |
          (MINUS_btn.pos_normal << 3) |
            (ENTER_btn.pos_normal << 4) |
              (SW1_btn.pos_normal   << 5) |
                (SW2_btn.pos_normal   << 6) |
                  (RCP1_btn.pos_normal  << 7) |
                    (RCP2_btn.pos_normal  << 8);
  
  // Запись данных
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[0], buttons_data);
  
  // M1 данные
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[1], M1.Pulses_per_rev);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[2], (M1.Max_Speed << 16) | M1.out_frequency);
  uint8_t m1_inv = M1.Step_DIR_EN_M_inv.Step | (M1.Step_DIR_EN_M_inv.DIR << 1) 
    | (M1.Step_DIR_EN_M_inv.EN << 2) | (M1.Step_DIR_EN_M_inv.Mot_Left << 3);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[3], 
                    m1_inv | (M1.Speed_sp << 8) | (M1.ReachCtrlPoint_avalible << 16));
  
  // M2 данные
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[4], M2.Pulses_per_rev);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[5], (M2.Max_Speed << 16) | M2.out_frequency);
  uint8_t m2_inv = M2.Step_DIR_EN_M_inv.Step | (M2.Step_DIR_EN_M_inv.DIR << 1) 
    | (M2.Step_DIR_EN_M_inv.EN << 2) | (M2.Step_DIR_EN_M_inv.Mot_Left << 3);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[6], 
                    m2_inv | (M2.Speed_sp << 8) | (M2.ReachCtrlPoint_avalible << 16));
  
  // Прочие переменные
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[7], 
                    (Deept_of_cut_mm << 16) | (tooth_sp << 8) | backlight_on);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, User_Page_Adress[8], Delay_switching);
  
  HAL_FLASH_Lock();
  taskEXIT_CRITICAL();
  return 0xFFFFFFFF; // Успешная запись
  
}

void Flash_read() {
  // Чтение данных кнопок
  uint32_t buttons_data = flash_read(User_Page_Adress[0]);
  if (buttons_data==0xFF){
    error=ERROR_FLASH_R;
    return; //в первый раз? 
  }
  UP_btn.pos_normal = (GPIO_PinState)((buttons_data >> 0) & 0x01);
  DOWN_btn.pos_normal = (GPIO_PinState)((buttons_data >> 1) & 0x01);
  PLUS_btn.pos_normal = (GPIO_PinState)((buttons_data >> 2) & 0x01);
  MINUS_btn.pos_normal = (GPIO_PinState)((buttons_data >> 3) & 0x01);
  ENTER_btn.pos_normal = (GPIO_PinState)((buttons_data >> 4) & 0x01);
  SW1_btn.pos_normal = (GPIO_PinState)((buttons_data >> 5) & 0x01);
  SW2_btn.pos_normal = (GPIO_PinState)((buttons_data >> 6) & 0x01);
  RCP1_btn.pos_normal = (GPIO_PinState)((buttons_data >> 7) & 0x01);
  RCP2_btn.pos_normal = (GPIO_PinState)((buttons_data >> 8) & 0x01);
  
  // M1 данные
  M1.Pulses_per_rev = flash_read(User_Page_Adress[1]);
  uint32_t m1_speed = flash_read(User_Page_Adress[2]);
  M1.Max_Speed = (m1_speed >> 16) & 0xFFFF;
  M1.out_frequency = m1_speed & 0xFFFF;
  uint32_t m1_set = flash_read(User_Page_Adress[3]);
  M1.Step_DIR_EN_M_inv.Step = m1_set & 0x01;
  M1.Step_DIR_EN_M_inv.DIR = (m1_set >> 1) & 0x01;
  M1.Step_DIR_EN_M_inv.EN = (m1_set >> 2) & 0x01;
  M1.Step_DIR_EN_M_inv.Mot_Left = (m1_set >> 3) & 0x01;
  M1.Speed_sp = (m1_set >> 8) & 0xFF;
  M1.ReachCtrlPoint_avalible = (m1_set >> 16) & 0xFF;
  
  // M2 данные (аналогично M1)
  M2.Pulses_per_rev = flash_read(User_Page_Adress[4]);
  uint32_t m2_speed = flash_read(User_Page_Adress[5]);
  M2.Max_Speed = (m2_speed >> 16) & 0xFFFF;
  M2.out_frequency = m2_speed & 0xFFFF;
  uint32_t m2_set = flash_read(User_Page_Adress[6]);
  M2.Step_DIR_EN_M_inv.Step = m2_set & 0x01;
  M2.Step_DIR_EN_M_inv.DIR = (m2_set >> 1) & 0x01;
  M2.Step_DIR_EN_M_inv.EN = (m2_set >> 2) & 0x01;
  M2.Step_DIR_EN_M_inv.Mot_Left = (m2_set >> 3) & 0x01;
  M2.Speed_sp = (m2_set >> 8) & 0xFF;
  M2.ReachCtrlPoint_avalible = (m2_set >> 16) & 0xFF;
  
  // Прочие переменные
  uint32_t vars1 = flash_read(User_Page_Adress[7]);
  backlight_on = vars1 & 0xFF;
  tooth_sp = (vars1 >> 8) & 0xFF;
  Deept_of_cut_mm = (vars1 >> 16) & 0xFFFF;
  Delay_switching = flash_read(User_Page_Adress[8]) & 0xFFFF;
}

void Set_period_and_start_TIM(TIM_HandleTypeDef *htim, uint16_t period){
  __HAL_TIM_SET_AUTORELOAD(htim, period);                //f==1000000/period 
  // Сбрасываем счётчик таймера
  __HAL_TIM_SET_COUNTER(htim, 0);
  // Если таймер остановлен, его можно перезапустить
  if (__HAL_TIM_GET_COUNTER(htim) == 0) {
    HAL_TIM_Base_Start(htim); // Запускаем таймер
  }
}
void Process_morots_from_IRQ(void){
  static uint8_t toggle=1;
  if(motor_in_use==1){
    if(M1.output_sp>0)          //SP--
    {
      HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,M1.Step_DIR_EN_M_inv.EN?GPIO_PIN_RESET:GPIO_PIN_SET); 
      HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,M1.Step_DIR_EN_M_inv.DIR?GPIO_PIN_RESET:GPIO_PIN_SET);  
      if(toggle){
        toggle=0;
        HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin,M1.Step_DIR_EN_M_inv.Step?GPIO_PIN_SET:GPIO_PIN_RESET);
      }else {
        HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, M1.Step_DIR_EN_M_inv.Step ? GPIO_PIN_RESET : GPIO_PIN_SET);
        toggle = 1;
        M1.output_sp--;
      }
    }else if (M1.output_sp==0){
      motor_in_use=0;
      toggle=1;
      HAL_GPIO_WritePin(STEP1_GPIO_Port,STEP1_Pin,M1.Step_DIR_EN_M_inv.Step?GPIO_PIN_SET:GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,M1.Step_DIR_EN_M_inv.EN?GPIO_PIN_SET:GPIO_PIN_RESET);
      HAL_TIM_Base_Stop(&htim1);
    }else{                      //SP++
      HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,M1.Step_DIR_EN_M_inv.DIR?GPIO_PIN_SET:GPIO_PIN_RESET);  
      HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,M1.Step_DIR_EN_M_inv.EN?GPIO_PIN_RESET:GPIO_PIN_SET);
      if (toggle) {
        HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, M1.Step_DIR_EN_M_inv.Step ? GPIO_PIN_SET : GPIO_PIN_RESET);
        toggle = 0;
      } else {
        HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, M1.Step_DIR_EN_M_inv.Step ? GPIO_PIN_RESET : GPIO_PIN_SET);
        toggle = 1;
        M1.output_sp++;
      }
    }
    
  }else if(motor_in_use==2){
    if (M2.output_sp > 0) {
      // Генерация шагов для M2 (прямое направление)
      HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, M2.Step_DIR_EN_M_inv.EN ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, M2.Step_DIR_EN_M_inv.DIR ? GPIO_PIN_RESET : GPIO_PIN_SET);
      
      if (toggle) {
        HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, M2.Step_DIR_EN_M_inv.Step ? GPIO_PIN_SET : GPIO_PIN_RESET);
        toggle = 0;
      } else {
        HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, M2.Step_DIR_EN_M_inv.Step ? GPIO_PIN_RESET : GPIO_PIN_SET);
        toggle = 1;
        M2.output_sp--;
      }
    } 
    else if (M2.output_sp == 0) {
      // Остановка M2
      motor_in_use=0;
      toggle = 1;
      HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, M2.Step_DIR_EN_M_inv.Step ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, M2.Step_DIR_EN_M_inv.EN ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_TIM_Base_Stop(&htim1); // Или другой таймер, если используется отдельный
    } 
    else {
      // Генерация шагов для M2 (обратное направление)
      HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, M2.Step_DIR_EN_M_inv.EN ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, M2.Step_DIR_EN_M_inv.DIR ? GPIO_PIN_SET : GPIO_PIN_RESET);
      
      if (toggle) {
        HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, M2.Step_DIR_EN_M_inv.Step ? GPIO_PIN_SET : GPIO_PIN_RESET);
        toggle = 0;
      } else {
        HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, M2.Step_DIR_EN_M_inv.Step ? GPIO_PIN_RESET : GPIO_PIN_SET);
        toggle = 1;
        M2.output_sp++;
      }
    }
  }
}
/* USER CODE END Application */

