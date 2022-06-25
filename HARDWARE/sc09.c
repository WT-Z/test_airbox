#include "sc09.h"
#include "gpio.h"
#include "lcd.h"
#include "airbox.h"


//#define SPECIAL_APP //需要特殊配置的运用

// SDA PB07
// SCL PB06
// INT PB05

#define SDA_OUT_OR_IN M0P_GPIO->PCDIR_f.PC05 	//定义 SDA 输入输出方向       PC05    0输出 1输入
#define SCL_OUT_OR_IN M0P_GPIO->PBDIR_f.PB00 	//定义 SCL 输入输出方向       PB0
#define SDA 					M0P_GPIO->PCOUT_f.PC05	//定义 SDA 数据线输出
#define SDA_IN 				M0P_GPIO->PCIN_f.PC05		//定义 SDA 数据线读取
#define SCL 					M0P_GPIO->PBOUT_f.PB00	//定义 SCL 时钟线输出


//#define CO2_SDA_OUT_OR_IN M0P_GPIO->PCDIR_f.PC05 	//定义 SDA 输入输出方向       PC05    0输出 1输入
//#define CO2_SCL_OUT_OR_IN M0P_GPIO->PBDIR_f.PB00 	//定义 SCL 输入输出方向       PB0
//#define CO2_SDA 					M0P_GPIO->PCOUT_f.PC05	//定义 SDA 数据线输出
//#define CO2_SDA_IN 				M0P_GPIO->PCIN_f.PC05		//定义 SDA 数据线读取
//#define CO2_SCL 					M0P_GPIO->PBOUT_f.PB00	//定义 SCL 时钟线输出


#define CO2_SDA_OUT_OR_IN M0P_GPIO->PDDIR_f.PD04 	//定义 CO2_SDA 输入输出方向       PD04    0输出 1输入
#define CO2_SCL_OUT_OR_IN M0P_GPIO->PDDIR_f.PD05 	//定义 CO2_SCL 输入输出方向       PD05
#define CO2_SDA 					M0P_GPIO->PDOUT_f.PD04	//定义 CO2_SDA 数据线输出
#define CO2_SDA_IN 				M0P_GPIO->PDIN_f.PD04		//定义 CO2_SDA 数据线读取
#define CO2_SCL 					M0P_GPIO->PDOUT_f.PD05	//定义 CO2_SCL 时钟线输出


uint8_t sss = 0;





//////////////////////////////////////////////////////////////////////////////////////////Register ADDR////////////////////////////////////////////////////////////////////////////
#define SenSet0_REG 0x00		  //CIN4 通道灵敏度的设置地址
#define SenSetCOM_REG 0x01		  //其他通道灵敏度的设置地址
#define CTRL0_REG 0x02			  //CTRL0 控制寄存器设置地址
#define CTRL1_REG 0x03			  //CTRL1 控制寄存器设置地址
#define Output_REG 0x08			  //触摸状态寄存器输出地址
#define SAMP_REG 0x0A			  //触摸数据值存器输出地址
#define RTM0 0					  //3 个采样周期有效， 1 个采样周期判断无效
#define RTM1 1					  //4 个采样周期有效， 2 个采样周期判断无效
#define RTM2 2					  //5 个采样周期有效， 3 个采样周期判断无效
#define RTM3 3					  //6 个采样周期有效， 4 个采样周期判断无效
#define KVF_STOP_CORREC (1u << 2) // 按键有效，触摸不校准
#define KVF_50S_CORREC (0u << 2)  // 按下有效后， 50S 开始校准
#define HOLD (1u << 3)			  //基线保持不校准
#define NOTHOLD (0u << 3)		  //基线持续校准
#define SLPCYC_LGT (0u << 5)	  //无穷大
#define SLPCYC_0R5T (1u << 5)	  //休眠后采样间隔 60MS
#define SLPCYC_1R5T (2u << 5)	  //休眠后采样间隔 180MS
#define SLPCYC_2R5T (3u << 5)	  //休眠后采样间隔 300MS
#define SLPCYC_3R5T (4u << 5)	  //休眠后采样间隔 420MS
#define SLPCYC_4R5T (5u << 5)	  //休眠后采样间隔 540MS
#define SLPCYC_5R5T (6u << 5)	  //休眠后采样间隔 660MS
#define SLPCYC_6R5T (7u << 5)	  //休眠后采样间隔 780MS
#define FAST_TO_SLEEP (1u << 4)	  //快速进入休眠
#define SLOW_TO_SLEEP (0u << 4)	  // 75S 进入休眠

void sc09_io_init(void)
{
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	stc_gpio_cfg_t stcGpioCfg;
  stcGpioCfg.enDir = GpioDirOut;    ///< 端口方向配置->
  stcGpioCfg.enDrv = GpioDrvH;     ///< 端口驱动能力配置->高驱动能力
  stcGpioCfg.enOD = GpioOdDisable; ///< 端口开漏输出配置->开漏输出关闭
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enPu = GpioPuEnable;
  stcGpioCfg.enCtrlMode = GpioAHB; ///< 端口输入/输出值寄存器总线控制模式配置->AHB

  Gpio_Init(GpioPortB, GpioPin0, &stcGpioCfg); ///< GPIO IO SCL
	Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg); ///< GPIO IO SCL
	
	
  stcGpioCfg.enDir = GpioDirOut;    ///< 端口方向配置->
  stcGpioCfg.enDrv = GpioDrvH;     ///< 端口驱动能力配置->高驱动能力
  stcGpioCfg.enOD = GpioOdDisable; ///< 端口开漏输出配置->开漏输出关闭
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enPu = GpioPuEnable;
  stcGpioCfg.enCtrlMode = GpioAHB; ///< 端口输入/输出值寄存器总线控制模式配置->AHB	
	Gpio_Init(GpioPortC, GpioPin5, &stcGpioCfg); ///< GPIO IO SDA
	Gpio_Init(GpioPortD, GpioPin4, &stcGpioCfg); ///< GPIO IO SDA	
	
//	Gpio_SetIO(GpioPortB,GpioPin0);
//	Gpio_SetIO(GpioPortC,GpioPin5);
//	Gpio_SetIO(GpioPortD,GpioPin4);
//	Gpio_SetIO(GpioPortD,GpioPin5);
	
}

/*****************************************************************************
* I2C 时钟延时函数
******************************************************************************/
void Delay(unsigned char time)
{
	unsigned char a;
	for (a = (time*30); a > 0; a--)
	{
		//for(int i=1000;i>0;i--)
		__NOP();
	}
//		;
	//delay10us(1);
	
}
/*****************************************************************************
* I2C 启动信号函数
******************************************************************************/
void I2C_Start(void)
{
	SDA_OUT_OR_IN = 0;

	SCL_OUT_OR_IN = 0;
	SDA = 1;
	SCL = 1;sss=1;
	Delay(1);
	SDA = 0;
	Delay(1);
	SCL = 0;sss=0;
	Delay(1);
}
// CO2 I2C 启动信号函数
void CO2_I2C_Start(void)
{
	CO2_SDA_OUT_OR_IN = 0;

	CO2_SCL_OUT_OR_IN = 0;
	CO2_SDA = 1;
	CO2_SCL = 1;
	Delay(1);
	CO2_SDA = 0;
	Delay(1);
	CO2_SCL = 0;
	Delay(1);
}


/*****************************************************************************
* 发送一个字节数据，并获取应答
******************************************************************************/
unsigned char SendByteAndGetNACK(unsigned char dataToSend)
{
	unsigned char i;
//	unsigned char ack;
	SDA_OUT_OR_IN = 0;
	for (i = 0; i < 8; i++)
	{
		SCL = 0;sss=0;
		Delay(1);
		SDA = (dataToSend >> 7) & 0x01;
		Delay(1);
		SCL = 1;sss=1;
		Delay(1);
		dataToSend <<= 1;
	}
	SCL = 0;sss=0;
	Delay(3);
	SDA_OUT_OR_IN = 1;
	Delay(3);
	SCL = 1;sss=1;
	Delay(1);
	i = 250;
	while (i--)
	{
		if (!SDA_IN)
		{
			SCL = 0;sss=0;
			return 0;
		}
	}
	SCL = 0;sss=0;
	return (1);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_OUT_OR_IN = 1;      //SDA设置为输入  
	SDA=1;Delay(1);	   
	SCL=1;Delay(1);	 
	while(SDA_IN)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}
	SCL=0;//时钟输出0 	   
	return 0;  
} 

//等待应答信号到来  CO2
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t CO2_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	CO2_SDA_OUT_OR_IN = 1;      //SDA设置为输入  
	CO2_SDA=1;Delay(1);	   
	CO2_SCL=1;Delay(1);	 
	while(CO2_SDA_IN)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CO2_I2C_Stop();
			return 1;
		}
	}
	CO2_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	SCL=0;
	SDA_OUT_OR_IN = 0;
	SDA=0;
	Delay(2);
	SCL=1;
	Delay(2);
	SCL=0;
}

//产生ACK应答   CO2
void CO2_IIC_Ack(void)
{
	CO2_SCL=0;
	CO2_SDA_OUT_OR_IN = 0;
	CO2_SDA=0;
	Delay(2);
	CO2_SCL=1;
	Delay(2);
	CO2_SCL=0;
}

//不产生ACK应答		    
void IIC_NAck(void)
{
	SCL=0;
	SDA_OUT_OR_IN = 0;
	SDA=1;
	Delay(2);
	SCL=1;
	Delay(2);
	SCL=0;
}	
//不产生ACK应答	   CO2
void CO2_IIC_NAck(void)
{
	CO2_SCL=0;
	CO2_SDA_OUT_OR_IN = 0;
	CO2_SDA=1;
	Delay(2);
	CO2_SCL=1;
	Delay(2);
	CO2_SCL=0;
}	
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答	
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  SDA_OUT_OR_IN = 0; 	    
    SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        SDA=(txd >> 7) & 0x01;
        txd<<=1; 	  
		Delay(2);   //对TEA5767这三个延时都是必须的
		SCL=1;
		Delay(2); 
		SCL=0;	
    }
}

//CO2 IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答	
void CO2_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  CO2_SDA_OUT_OR_IN = 0; 	    
    CO2_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        CO2_SDA=(txd >> 7) & 0x01;
        txd<<=1; 	  
		Delay(2);   //对TEA5767这三个延时都是必须的
		CO2_SCL=1;
		Delay(2); 
		CO2_SCL=0;	
    }
}


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_OUT_OR_IN = 1;//SDA设置为输入
	for(i=0;i<8;i++ )
	{
    SCL=0; 
    Delay(2);
		SCL=1;
    receive<<=1;
    if(SDA_IN)receive++;   
		Delay(2); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//CO2 读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t CO2_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	CO2_SDA_OUT_OR_IN = 1;//SDA设置为输入
	for(i=0;i<8;i++ )
	{
    CO2_SCL=0; 
    Delay(2);
		CO2_SCL=1;
    receive<<=1;
    if(CO2_SDA_IN)receive++;   
		Delay(2); 
    }					 
    if (!ack)
        CO2_IIC_NAck();//发送nACK
    else
        CO2_IIC_Ack(); //发送ACK   
    return receive;
}

/*****************************************************************************
* 读取一个字节信号，并下发应答命令.
******************************************************************************/
void I2C_Respond(unsigned char ACKSignal)
{
	SDA_OUT_OR_IN = 0;
	SDA = 0;
	SCL = 0;sss=0;
	SDA = ACKSignal;
	Delay(1);
	SCL = 1;sss=1;
	Delay(1);
	SCL = 0;sss=0;
}
/*****************************************************************************
* 停止信号
******************************************************************************/
void I2C_Stop()
{
	SCL = 0;sss=0;
	SDA_OUT_OR_IN = 0;
	SDA = 0;
	Delay(1);
	SCL = 1;sss=1;
	Delay(1);
	SDA = 1;
}
// CO2 IIC 停止
void CO2_I2C_Stop()
{
	CO2_SCL = 0;
	CO2_SDA_OUT_OR_IN = 0;
	CO2_SDA = 0;
	Delay(1);
	CO2_SCL = 1;
	Delay(1);
	CO2_SDA = 1;
}


/*****************************************************************************
* 读取一个字节函数
******************************************************************************/
unsigned char I2C_Receive8Bit(void)
{
	unsigned char i, buffer;
	SDA_OUT_OR_IN = 1;
	SCL = 0;sss=0;
	for (i = 0; i < 8; i++)
	{
		Delay(1);
		SCL = 1;sss=1;
		buffer = (buffer << 1) | SDA_IN;
		Delay(1);
		SCL = 0;sss=0;
	}
	return (buffer);
}
/*****************************************************************************
* SC09B 初始化功能函数，如无特殊运用，无需初始化
******************************************************************************/
void SC09B_Init_Function(void)
{
	//unsigned char databuf;
#ifdef SPECIAL_APP
	databuf = 0x79;
	I2C_Write_To_Device(SC05B_ADDR, SenSet0_REG, &databuf);
	databuf = 0x79;
	I2C_Write_To_Device(SC05B_ADDR, SenSetCOM_REG, &databuf);
	databuf = SLPCYC_3R5T | SLOW_TO_SLEEP | HOLD | KVF_50S_CORREC | RTM3;
	I2C_Write_To_Device(SC05B_ADDR, CTRL0_REG, &databuf);
	databuf = 0b1000;
	I2C_Write_To_Device(SC05B_ADDR, CTRL1_REG, &databuf);
#endif
}
/*************************************************************************************************************************
* SC09B 写寄存器参数运用函数
deviceAddr 设置器件地址 REG 设置寄存器地址 DAT8 写入数据内容的地址
**************************************************************************************************************************/
Complete_Status I2C_Write_To_Device(unsigned char deviceAddr, unsigned char REG, unsigned char *DAT8)
{
	I2C_Start();
	if (SendByteAndGetNACK((deviceAddr << 1) & ~0x01))
	{
		I2C_Stop();
		return UNDONE;
	}
	if (SendByteAndGetNACK(REG))
	{
		I2C_Stop();
		return UNDONE;
	}
	if (SendByteAndGetNACK(*DAT8))
	{
		I2C_Stop();
		return UNDONE;
	}
	I2C_Stop();
	return DONE;
}
/***************************************************************************************************************************
* SC09B 简易读取按键值函数（默认直接读取）
此函数只有初始化配置默认的情况下，直接调用，如果在操作前有写入或者其他读取不能调用默认
**********************************************************************************************************************************/
Complete_Status I2C_Simple_Read_From_Device(unsigned char deviceAddr, unsigned int *DAT16)
{
	unsigned char buf1, buf2;
	I2C_Start();
	if (SendByteAndGetNACK((deviceAddr << 1) | 0x01))
	{
		I2C_Stop();
		return UNDONE;
	}
	buf1 = I2C_Receive8Bit();
	I2C_Respond(0);
	buf1 = I2C_Receive8Bit();
	I2C_Respond(1);
		buf1 = I2C_Receive8Bit();
	I2C_Respond(1);
		buf1 = I2C_Receive8Bit();
	I2C_Respond(1);
		buf1 = I2C_Receive8Bit();
	I2C_Respond(1);
	I2C_Stop();
	*DAT16 = ((unsigned int)buf1 << 8) | buf2;
	return DONE;
}
/**************************************************************************************************************************
* SC09B 读取寄存器数值函数 （此函数主要是用来读取 SAMP 和 OutREG 值）
deviceAddr 设置器件地址 REG 设置寄存器地址 DAT16 读取地址对应数据内容
**************************************************************************************************************************/
Complete_Status I2C_Read_From_Device(unsigned char deviceAddr, unsigned char REG, unsigned int *DAT16)
{
	unsigned char buf1, buf2;
	I2C_Start();
	if (SendByteAndGetNACK((deviceAddr << 1) & ~0x01))
	{
		I2C_Stop();
		return UNDONE;
	}
	if (SendByteAndGetNACK(REG))
	{
		I2C_Stop();
		return UNDONE;
	}
	I2C_Stop();
	I2C_Start();
	if (SendByteAndGetNACK((deviceAddr << 1) | 0x01))
	{
		I2C_Stop();
		return UNDONE;
	}
	buf1 = I2C_Receive8Bit();
	I2C_Respond(0);
	buf2 = I2C_Receive8Bit();
	I2C_Respond(1);
	I2C_Stop();
	*DAT16 = ((unsigned int)buf1 << 8) | buf2;
	return DONE;
}
uint8_t AT24CXX_ReadOneByte(uint8_t ReadAddr)
{				  
	uint8_t temp=0;
 // uint8_t rxbuffer[12]={0};	
  I2C_Start();  
	IIC_Send_Byte(ReadAddr); 
	if(IIC_Wait_Ack())
		return 0;
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	temp=IIC_Read_Byte(1);
	I2C_Stop();    
	return temp;
}
uint16_t CO2_ReadBytes(uint8_t ReadAddr)
{
  uint8_t temp=0;
	uint8_t number=0;
	uint16_t CO2_Data=0;
  uint8_t rxbuffer[12]={0};	
  CO2_I2C_Start();  
	CO2_IIC_Send_Byte(ReadAddr); 
	if(CO2_IIC_Wait_Ack())
		return 0;
	for(number =0;number<12;number++){
	  temp=CO2_IIC_Read_Byte(1);
		rxbuffer[number] =temp;
		printf("rxbuffer[%d] is %4x\r\n",number,rxbuffer[number]);		
	}
	CO2_Data = rxbuffer[4]<<8;
	CO2_Data = CO2_Data+rxbuffer[5];
	CO2_I2C_Stop();
	if(!CO2_CRC(rxbuffer))
	{
	 //CO2_Show_Handler(CO2_Data,rxbuffer);	
		CO2_Set_data(CO2_Data);
	}
	return temp;

}
//温湿度发送命令
void Humiture_send_Bytes(uint8_t Address,uint16_t Cmd,uint8_t stop)
{
  I2C_Start();
  IIC_Send_Byte(Address<<1 | 0x00);                                                     //改成直接为地址，然后进行移位操作。0x00写 0x01 读  写成宏定义
  IIC_Wait_Ack();
	IIC_Send_Byte(Cmd>>8);
	IIC_Wait_Ack();
  IIC_Send_Byte(Cmd&0xff);
  IIC_Wait_Ack();
  if (stop) {
	I2C_Stop();
	}

}
//单次读取温湿度数据
void Humiture_Single_Shot(uint8_t *buffer)
{
  uint8_t try_time=100;
  Humiture_send_Bytes(0x44,0x2C06,1);
  delay1ms(20);
  I2C_Start();
  IIC_Send_Byte(0x44<<1 | 0x01);                                   //读命令
  while(IIC_Wait_Ack())
    {
        try_time--;
        delay10us(5);
        if(try_time==0)
            return;
    }
    buffer[0]=IIC_Read_Byte(1);
    buffer[1]=IIC_Read_Byte(1);
    buffer[2]=IIC_Read_Byte(1);
    buffer[3]=IIC_Read_Byte(1);
    buffer[4]=IIC_Read_Byte(1);
    buffer[5]=IIC_Read_Byte(0);
    I2C_Stop();		
}

//温湿度校验
static uint8_t Hum_CRC_Check(uint8_t *check_data, uint8_t num, uint8_t check_crc)
{
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter
    
 // calculates 8-Bit checksum with given polynomial x8+x5+x4+1
    for(byteCtr = 0; byteCtr < num; byteCtr++)
    {
        crc ^= (*(check_data+byteCtr));
        //crc校验，最高位是1就^0x31
        for(bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    if(crc==check_crc)
        return 1;
    else 
        return 0;
}

//获取温湿度
void SHT30_Read(void)
{    
    uint8_t buff[6];//获取raw数据
    uint16_t tem,hum;//拼接温湿度数据
    uint8_t crcT,crcH;//温度和湿度的CRC校验

//    float Temperature=0;//转换后的温湿度
//    float Humidity=0;
	  int16_t Temperature=0;
    uint16_t Humidity=0;
	
    Humiture_Single_Shot(buff);
    //SHT30_Periodic(buff);
    
    tem = ((buff[0]<<8) | buff[1]);//温度拼接
    hum = ((buff[3]<<8) | buff[4]);//湿度拼接

    //计算温度和湿度CRC校验码
    crcT = Hum_CRC_Check(buff,2,buff[2]);   //温度
    crcH = Hum_CRC_Check(buff+3,2,buff[5]); //湿度
    
    if(crcT&&crcH)//判断CRC校验是否对
    {
        //根据手册计算公式计算
//        Temperature= (175.0*(float)tem/65535.0-45.0) ;  // T = -45 + 175 * tem / (2^16-1)
//        Humidity= (100.0*(float)hum/65535.0);           // RH = hum*100 / (2^16-1)
			
			  Temperature= (175*tem/65535-45) ;  // T = -45 + 175 * tem / (2^16-1)
        Humidity= (100*hum/65535);           // RH = hum*100 / (2^16-1)
			
        if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=1)&&(Humidity<=100))//过滤超出测量范围的错误数据
        {
            TEM_Set_data(Temperature);
            HUM_Set_data(Humidity);
        }
    }
}



//CO2 校验
uint8_t  CO2_CRC(uint8_t *CO2_buffer)
{
  uint8_t i;
	uint16_t crc,sum=0;
	
	crc = CO2_buffer[10]<<8;
	crc = crc+CO2_buffer[11];
  for(i=0;i<10;i++)
	{
		sum = sum + CO2_buffer[i];                                  //校验和
	}
	printf("CRC is %4x\r\n",crc);
  if(sum != crc){
		return 1;
	}else{
		return 0;
	}


}	






