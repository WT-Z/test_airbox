#ifndef __UART_PM_H__
#define __UART_PM_H__

#include "stdint.h"
#include "lpuart.h"
#include "ddl.h"
#include "gpio.h"
#include "stdlib.h"
#include "airbox.h"
#include "lcd.h"



void App_PM_PortInit(void);                   //PM����IO���ú���
void App_Uart_PMCfg(void);                    //PM��������
uint8_t Uart_Crc(void);                       //CRCУ��
void Uart_Receive_Data_Handler(void);         //�������ݴ���




#endif
