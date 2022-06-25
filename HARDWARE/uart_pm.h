#ifndef __UART_PM_H__
#define __UART_PM_H__

#include "stdint.h"
#include "lpuart.h"
#include "ddl.h"
#include "gpio.h"
#include "stdlib.h"
#include "airbox.h"
#include "lcd.h"



void App_PM_PortInit(void);                   //PM串口IO配置函数
void App_Uart_PMCfg(void);                    //PM串口配置
uint8_t Uart_Crc(void);                       //CRC校验
void Uart_Receive_Data_Handler(void);         //串口数据处理




#endif
