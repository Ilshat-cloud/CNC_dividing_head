
#include "stm32f1xx_hal.h"
#include "main.h"
#ifndef wh1602
#define wh1602

#define     PIN_RS            RS_Pin          // PB12
#define     PIN_RS_PORT       RS_GPIO_Port               
#define     PIN_EN            EN_Pin          // PA10
#define     PIN_EN_PORT       EN_GPIO_Port               
#define     PIN_D7            DB7_Pin          // PA11
#define     PIN_D7_PORT       DB7_GPIO_Port               
#define     PIN_D6            DB6_Pin          // PA12
#define     PIN_D6_PORT       DB6_GPIO_Port               
#define     PIN_D5            DB5_Pin          // PA8
#define     PIN_D5_PORT       DB5_GPIO_Port               
#define     PIN_D4            DB4_Pin          // PA9
#define     PIN_D4_PORT       DB4_GPIO_Port               


void PulseLCD(void);
void SendByte(char ByteToSend, int IsData);
void Cursor(char Row, char Col);
void ClearLCDScreen(void);
void InitializeLCD(void);
void PrintStr(char *Text);
void PrintByCoordinats(char Row, char Col,char *Text);
#endif
