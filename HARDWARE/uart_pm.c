

#include "uart_pm.h"



uint8_t u8RxData[32] = {0};
uint8_t uart_flag = 0;
uint8_t RX_PM_buffer[32]={0};
uint8_t u8TxCnt=0,u8RxCnt=0,crc = 0;

//串口引脚配置
void App_PM_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcGpioCfg);
	  

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///<使能GPIO外设时钟门控开关

    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortB,GpioPin12,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortB,GpioPin12,GpioAf3);             ///<配置PB12 为LPUART0 TX
    
	  stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortB,GpioPin11,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortB,GpioPin11,GpioAf3);             ///<配置PB11 为LPUART0 RX	
	
}

//串口配置函数
void App_Uart_PMCfg(void)
{
    stc_lpuart_cfg_t  stcCfg;
    //stc_lpuart_baud_t stcBaud;

    DDL_ZERO_STRUCT(stcCfg);
    //DDL_ZERO_STRUCT(stcBaud);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,TRUE); ///<使能LPUART0外设时钟门控开关

    ///<UART Init
    stcCfg.enRunMode        = LPUartMskMode1;                 ///<模式1
    stcCfg.enStopBit        = LPUart1bit;                  ///<1bit停止位
    //stcCfg.enMmdorCk        = UartMskDataOrAddr;            ///<多机模式时
    stcCfg.stcBaud.u32Baud  = 9600;                         ///<波特率9600
    stcCfg.stcBaud.enSclkDiv = LPUartMsk4Or8Div;              ///<通道采样分频配置
    stcCfg.stcBaud.u32Sclk  = Sysctrl_GetPClkFreq();        ///</<获得外设时钟（PCLK）频率值
    LPUart_Init(M0P_LPUART0, &stcCfg);                          ///<串口初始化

    ///<UART中断使能
    LPUart_ClrStatus(M0P_LPUART0,LPUartRC);                       ///<清接收请求
    LPUart_ClrStatus(M0P_LPUART0,LPUartTC);                       ///<清接收请求
    //Uart_EnableIrq(M0P_UART0,UartTxIrq);                    ///<使能串口发送中断
		
		LPUart_EnableIrq(M0P_LPUART0,LPUartRxIrq);
    EnableNvic(LPUART0_IRQn, IrqLevel3, TRUE);              ///<系统中断使能
		
}

//CRC校验
uint8_t Uart_Crc(void)
{
  uint8_t i;
	uint16_t crc,sum=0;
	
	crc = u8RxData[30]<<8;
	crc = crc+u8RxData[31];
  for(i=0;i<30;i++)
	{
		sum = sum + u8RxData[i];                                  //校验和
	}
  if(sum != crc){
		return 1;
	}else{
		return 0;
	}
}
//串口数据处理函数

void Uart_Receive_Data_Handler(void)
{
	uint8_t i = 0;
  if (uart_flag) {
		RX_PM_buffer[0] = u8RxData[2] << 8;
    RX_PM_buffer[0] = RX_PM_buffer[0]+u8RxData[3]; 
			
		RX_PM_buffer[1] = u8RxData[4] << 8;
    RX_PM_buffer[1] = RX_PM_buffer[1]+u8RxData[5];
		
		RX_PM_buffer[2] = u8RxData[6] << 8;
    RX_PM_buffer[2] = RX_PM_buffer[2]+u8RxData[7];
		
		RX_PM_buffer[3] = u8RxData[8] << 8;
    RX_PM_buffer[3] = RX_PM_buffer[3]+u8RxData[9];
		
		RX_PM_buffer[4] = u8RxData[10] << 8;
    RX_PM_buffer[4] = RX_PM_buffer[4]+u8RxData[11];
		
		RX_PM_buffer[5] = u8RxData[12] << 8;
    RX_PM_buffer[5] = RX_PM_buffer[5]+u8RxData[13];              //大气下PM2.5
		
		RX_PM_buffer[6] = u8RxData[14] << 8;
    RX_PM_buffer[6] = RX_PM_buffer[6]+u8RxData[15];
		
		RX_PM_buffer[7] = u8RxData[16] << 8;
    RX_PM_buffer[7] = RX_PM_buffer[7]+u8RxData[17];
		
		RX_PM_buffer[8] = u8RxData[18] << 8;
    RX_PM_buffer[8] = RX_PM_buffer[8]+u8RxData[19];
		
		RX_PM_buffer[9] = u8RxData[20] << 8;
    RX_PM_buffer[9] = RX_PM_buffer[9]+u8RxData[21];	
//		printf("RX_PM0 IS %4x\r\n",RX_PM_buffer[0]);
//		printf("RX_PM1 IS %4x\r\n",RX_PM_buffer[1]);
//		printf("RX_PM2 IS %4x\r\n",RX_PM_buffer[2]);
//		printf("RX_PM3 IS %4x\r\n",RX_PM_buffer[3]);
//		printf("RX_PM4 IS %4x\r\n",RX_PM_buffer[4]);
		printf("PM2.5 da qi  IS %d\r\n",RX_PM_buffer[5]);
		
		
//		
//		printf("RX_PM6 IS %4x\r\n",RX_PM_buffer[6]);
//		printf("RX_PM7 IS %4x\r\n",RX_PM_buffer[7]);
//		printf("RX_PM8 IS %4x\r\n",RX_PM_buffer[8]);
//		printf("RX_PM9 IS %4x\r\n",RX_PM_buffer[9]);

      
			
//		  PM_Show.PM = RX_PM_buffer[5];
//			hundreds = RX_PM_buffer[5]/100%10;
//      tens = RX_PM_buffer[5]/10%10;
//		  lows = RX_PM_buffer[5]%10;			 
				PM_Set_data(RX_PM_buffer[5]);		
		for (i=0;i<32;i++) {
		  //printf("RX_PM_buffer[%d] is %2x\r\n",i,RX_PM_buffer[i]);
			u8RxData[i] = 0;
		}	
   uart_flag=0;
	 LPUart_EnableIrq(M0P_LPUART0,LPUartRxIrq);
 }

}

void LpUart0_IRQHandler(void)
{
	if(LPUart_GetStatus(M0P_LPUART0, LPUartRC))
    {
        LPUart_ClrStatus(M0P_LPUART0, LPUartRC);              //清除中断状态位
        u8RxData[u8RxCnt]=LPUart_ReceiveData(M0P_LPUART0);  //接收数据
				if(u8RxData[0] == 0x42  && u8RxCnt<32) {
				  u8RxCnt++;				  
				}

        if (u8RxCnt>31) {                                  //如果已接收32个字节
					if(u8RxData[1] == 0x4d){
            LPUart_DisableIrq(M0P_LPUART0,LPUartRxIrq);       //禁止接收中断
						uart_flag=1;
					}	
					u8RxCnt=0;
					//uart_flag=1;
        }			 					   
    }

    /*if(Uart_GetStatus(M0P_UART0, UartTC))
    {
        Uart_ClrStatus(M0P_UART0, UartTC);              //清除中断状态位
        Uart_SendDataIt(M0P_UART0, u8TxData[u8TxCnt++]);//发送数据
        if(u8TxCnt>1)                                   //如果已发送两个字节
        {
            u8TxCnt = 0;
            u8RxCnt = 0;
            Uart_DisableIrq(M0P_UART0,UartTxIrq);       //禁止发送中断
            Uart_EnableIrq(M0P_UART0,UartRxIrq);        //使能接收中断
        }
    }*/

}















