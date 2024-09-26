
#include "wh1602.h"
#include "FreeRTOS.h"

//---Нужная функция для работы с дисплеем, по сути "дергаем ножкой" EN---//
void PulseLCD()
{
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,(GPIO_PinState)RESET);
    osDelay(1);
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,(GPIO_PinState)SET);
    osDelay(1);
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,(GPIO_PinState)RESET);
    osDelay(1);
}

//---Отсылка байта в дисплей---//
void SendByte(char ByteToSend, int IsData)
{
   HAL_GPIO_WritePin(PIN_D7_PORT,PIN_D7,(GPIO_PinState)(ByteToSend & 0x80));
   HAL_GPIO_WritePin(PIN_D6_PORT,PIN_D6,(GPIO_PinState)(ByteToSend & 0x40));
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,(GPIO_PinState)(ByteToSend & 0x20));
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,(GPIO_PinState)(ByteToSend & 0x10));
   
   HAL_GPIO_WritePin(PIN_RS_PORT,PIN_RS,(GPIO_PinState)IsData);
   PulseLCD();
   HAL_GPIO_WritePin(PIN_D7_PORT,PIN_D7,(GPIO_PinState)(ByteToSend & 0x08));
   HAL_GPIO_WritePin(PIN_D6_PORT,PIN_D6,(GPIO_PinState)(ByteToSend & 0x04));
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,(GPIO_PinState)(ByteToSend & 0x02));
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,(GPIO_PinState)(ByteToSend & 0x01));   
   HAL_GPIO_WritePin(PIN_RS_PORT,PIN_RS,(GPIO_PinState)IsData);
   PulseLCD();         
   
}
 
//---Установка позиции курсора---//
void Cursor(char Row, char Col)
{
   char address;
   if (Row == 0)
   address = 0;
   else
   address = 0x40;
   address |= Col;
   SendByte(0x80 | address, 0);
}
 
//---Очистка дисплея---//
void ClearLCDScreen()
{
    SendByte(0x01, 0);
    SendByte(0x02, 0);    
}
 
//---Инициализация дисплея---//
void InitializeLCD(void)
{
   HAL_GPIO_WritePin(PIN_RS_PORT,PIN_RS,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(PIN_D7_PORT,PIN_D7,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(PIN_D6_PORT,PIN_D6,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,GPIO_PIN_RESET);   
   osDelay(100);
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,GPIO_PIN_SET);
   PulseLCD();
   SendByte(0x28, 0);
   SendByte(0x0E, 0);
   SendByte(0x06, 0);
}
 
//---Печать строки---//
void PrintStr(char *Text)
{
    char *c;
    c = Text;
    while ((c != 0) && (*c != 0))
    {
        SendByte(*c, 1);
        c++;
    }
}

void PrintByCoordinats(char Row, char Col,char *Text){
  Cursor(Row,Col);
  PrintStr(Text);
}