#include "sc09.h"
#include "gpio.h"
#include "lcd.h"
#include "airbox.h"


//#define SPECIAL_APP //��Ҫ�������õ�����

// SDA PB07
// SCL PB06
// INT PB05

#define SDA_OUT_OR_IN M0P_GPIO->PCDIR_f.PC05 	//���� SDA �����������       PC05    0��� 1����
#define SCL_OUT_OR_IN M0P_GPIO->PBDIR_f.PB00 	//���� SCL �����������       PB0
#define SDA 					M0P_GPIO->PCOUT_f.PC05	//���� SDA ���������
#define SDA_IN 				M0P_GPIO->PCIN_f.PC05		//���� SDA �����߶�ȡ
#define SCL 					M0P_GPIO->PBOUT_f.PB00	//���� SCL ʱ�������


//#define CO2_SDA_OUT_OR_IN M0P_GPIO->PCDIR_f.PC05 	//���� SDA �����������       PC05    0��� 1����
//#define CO2_SCL_OUT_OR_IN M0P_GPIO->PBDIR_f.PB00 	//���� SCL �����������       PB0
//#define CO2_SDA 					M0P_GPIO->PCOUT_f.PC05	//���� SDA ���������
//#define CO2_SDA_IN 				M0P_GPIO->PCIN_f.PC05		//���� SDA �����߶�ȡ
//#define CO2_SCL 					M0P_GPIO->PBOUT_f.PB00	//���� SCL ʱ�������


#define CO2_SDA_OUT_OR_IN M0P_GPIO->PDDIR_f.PD04 	//���� CO2_SDA �����������       PD04    0��� 1����
#define CO2_SCL_OUT_OR_IN M0P_GPIO->PDDIR_f.PD05 	//���� CO2_SCL �����������       PD05
#define CO2_SDA 					M0P_GPIO->PDOUT_f.PD04	//���� CO2_SDA ���������
#define CO2_SDA_IN 				M0P_GPIO->PDIN_f.PD04		//���� CO2_SDA �����߶�ȡ
#define CO2_SCL 					M0P_GPIO->PDOUT_f.PD05	//���� CO2_SCL ʱ�������


uint8_t sss = 0;





//////////////////////////////////////////////////////////////////////////////////////////Register ADDR////////////////////////////////////////////////////////////////////////////
#define SenSet0_REG 0x00		  //CIN4 ͨ�������ȵ����õ�ַ
#define SenSetCOM_REG 0x01		  //����ͨ�������ȵ����õ�ַ
#define CTRL0_REG 0x02			  //CTRL0 ���ƼĴ������õ�ַ
#define CTRL1_REG 0x03			  //CTRL1 ���ƼĴ������õ�ַ
#define Output_REG 0x08			  //����״̬�Ĵ��������ַ
#define SAMP_REG 0x0A			  //��������ֵ���������ַ
#define RTM0 0					  //3 ������������Ч�� 1 �����������ж���Ч
#define RTM1 1					  //4 ������������Ч�� 2 �����������ж���Ч
#define RTM2 2					  //5 ������������Ч�� 3 �����������ж���Ч
#define RTM3 3					  //6 ������������Ч�� 4 �����������ж���Ч
#define KVF_STOP_CORREC (1u << 2) // ������Ч��������У׼
#define KVF_50S_CORREC (0u << 2)  // ������Ч�� 50S ��ʼУ׼
#define HOLD (1u << 3)			  //���߱��ֲ�У׼
#define NOTHOLD (0u << 3)		  //���߳���У׼
#define SLPCYC_LGT (0u << 5)	  //�����
#define SLPCYC_0R5T (1u << 5)	  //���ߺ������� 60MS
#define SLPCYC_1R5T (2u << 5)	  //���ߺ������� 180MS
#define SLPCYC_2R5T (3u << 5)	  //���ߺ������� 300MS
#define SLPCYC_3R5T (4u << 5)	  //���ߺ������� 420MS
#define SLPCYC_4R5T (5u << 5)	  //���ߺ������� 540MS
#define SLPCYC_5R5T (6u << 5)	  //���ߺ������� 660MS
#define SLPCYC_6R5T (7u << 5)	  //���ߺ������� 780MS
#define FAST_TO_SLEEP (1u << 4)	  //���ٽ�������
#define SLOW_TO_SLEEP (0u << 4)	  // 75S ��������

void sc09_io_init(void)
{
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	stc_gpio_cfg_t stcGpioCfg;
  stcGpioCfg.enDir = GpioDirOut;    ///< �˿ڷ�������->
  stcGpioCfg.enDrv = GpioDrvH;     ///< �˿�������������->����������
  stcGpioCfg.enOD = GpioOdDisable; ///< �˿ڿ�©�������->��©����ر�
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enPu = GpioPuEnable;
  stcGpioCfg.enCtrlMode = GpioAHB; ///< �˿�����/���ֵ�Ĵ������߿���ģʽ����->AHB

  Gpio_Init(GpioPortB, GpioPin0, &stcGpioCfg); ///< GPIO IO SCL
	Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg); ///< GPIO IO SCL
	
	
  stcGpioCfg.enDir = GpioDirOut;    ///< �˿ڷ�������->
  stcGpioCfg.enDrv = GpioDrvH;     ///< �˿�������������->����������
  stcGpioCfg.enOD = GpioOdDisable; ///< �˿ڿ�©�������->��©����ر�
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enPu = GpioPuEnable;
  stcGpioCfg.enCtrlMode = GpioAHB; ///< �˿�����/���ֵ�Ĵ������߿���ģʽ����->AHB	
	Gpio_Init(GpioPortC, GpioPin5, &stcGpioCfg); ///< GPIO IO SDA
	Gpio_Init(GpioPortD, GpioPin4, &stcGpioCfg); ///< GPIO IO SDA	
	
//	Gpio_SetIO(GpioPortB,GpioPin0);
//	Gpio_SetIO(GpioPortC,GpioPin5);
//	Gpio_SetIO(GpioPortD,GpioPin4);
//	Gpio_SetIO(GpioPortD,GpioPin5);
	
}

/*****************************************************************************
* I2C ʱ����ʱ����
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
* I2C �����źź���
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
// CO2 I2C �����źź���
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
* ����һ���ֽ����ݣ�����ȡӦ��
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
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_OUT_OR_IN = 1;      //SDA����Ϊ����  
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
	SCL=0;//ʱ�����0 	   
	return 0;  
} 

//�ȴ�Ӧ���źŵ���  CO2
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t CO2_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	CO2_SDA_OUT_OR_IN = 1;      //SDA����Ϊ����  
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
	CO2_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//����ACKӦ��   CO2
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

//������ACKӦ��		    
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
//������ACKӦ��	   CO2
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��	
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  SDA_OUT_OR_IN = 0; 	    
    SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        SDA=(txd >> 7) & 0x01;
        txd<<=1; 	  
		Delay(2);   //��TEA5767��������ʱ���Ǳ����
		SCL=1;
		Delay(2); 
		SCL=0;	
    }
}

//CO2 IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��	
void CO2_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  CO2_SDA_OUT_OR_IN = 0; 	    
    CO2_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        CO2_SDA=(txd >> 7) & 0x01;
        txd<<=1; 	  
		Delay(2);   //��TEA5767��������ʱ���Ǳ����
		CO2_SCL=1;
		Delay(2); 
		CO2_SCL=0;	
    }
}


//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_OUT_OR_IN = 1;//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

//CO2 ��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t CO2_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	CO2_SDA_OUT_OR_IN = 1;//SDA����Ϊ����
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
        CO2_IIC_NAck();//����nACK
    else
        CO2_IIC_Ack(); //����ACK   
    return receive;
}

/*****************************************************************************
* ��ȡһ���ֽ��źţ����·�Ӧ������.
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
* ֹͣ�ź�
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
// CO2 IIC ֹͣ
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
* ��ȡһ���ֽں���
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
* SC09B ��ʼ�����ܺ����������������ã������ʼ��
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
* SC09B д�Ĵ����������ú���
deviceAddr ����������ַ REG ���üĴ�����ַ DAT8 д���������ݵĵ�ַ
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
* SC09B ���׶�ȡ����ֵ������Ĭ��ֱ�Ӷ�ȡ��
�˺���ֻ�г�ʼ������Ĭ�ϵ�����£�ֱ�ӵ��ã�����ڲ���ǰ��д�����������ȡ���ܵ���Ĭ��
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
* SC09B ��ȡ�Ĵ�����ֵ���� ���˺�����Ҫ��������ȡ SAMP �� OutREG ֵ��
deviceAddr ����������ַ REG ���üĴ�����ַ DAT16 ��ȡ��ַ��Ӧ��������
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
//��ʪ�ȷ�������
void Humiture_send_Bytes(uint8_t Address,uint16_t Cmd,uint8_t stop)
{
  I2C_Start();
  IIC_Send_Byte(Address<<1 | 0x00);                                                     //�ĳ�ֱ��Ϊ��ַ��Ȼ�������λ������0x00д 0x01 ��  д�ɺ궨��
  IIC_Wait_Ack();
	IIC_Send_Byte(Cmd>>8);
	IIC_Wait_Ack();
  IIC_Send_Byte(Cmd&0xff);
  IIC_Wait_Ack();
  if (stop) {
	I2C_Stop();
	}

}
//���ζ�ȡ��ʪ������
void Humiture_Single_Shot(uint8_t *buffer)
{
  uint8_t try_time=100;
  Humiture_send_Bytes(0x44,0x2C06,1);
  delay1ms(20);
  I2C_Start();
  IIC_Send_Byte(0x44<<1 | 0x01);                                   //������
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

//��ʪ��У��
static uint8_t Hum_CRC_Check(uint8_t *check_data, uint8_t num, uint8_t check_crc)
{
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter
    
 // calculates 8-Bit checksum with given polynomial x8+x5+x4+1
    for(byteCtr = 0; byteCtr < num; byteCtr++)
    {
        crc ^= (*(check_data+byteCtr));
        //crcУ�飬���λ��1��^0x31
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

//��ȡ��ʪ��
void SHT30_Read(void)
{    
    uint8_t buff[6];//��ȡraw����
    uint16_t tem,hum;//ƴ����ʪ������
    uint8_t crcT,crcH;//�¶Ⱥ�ʪ�ȵ�CRCУ��

//    float Temperature=0;//ת�������ʪ��
//    float Humidity=0;
	  int16_t Temperature=0;
    uint16_t Humidity=0;
	
    Humiture_Single_Shot(buff);
    //SHT30_Periodic(buff);
    
    tem = ((buff[0]<<8) | buff[1]);//�¶�ƴ��
    hum = ((buff[3]<<8) | buff[4]);//ʪ��ƴ��

    //�����¶Ⱥ�ʪ��CRCУ����
    crcT = Hum_CRC_Check(buff,2,buff[2]);   //�¶�
    crcH = Hum_CRC_Check(buff+3,2,buff[5]); //ʪ��
    
    if(crcT&&crcH)//�ж�CRCУ���Ƿ��
    {
        //�����ֲ���㹫ʽ����
//        Temperature= (175.0*(float)tem/65535.0-45.0) ;  // T = -45 + 175 * tem / (2^16-1)
//        Humidity= (100.0*(float)hum/65535.0);           // RH = hum*100 / (2^16-1)
			
			  Temperature= (175*tem/65535-45) ;  // T = -45 + 175 * tem / (2^16-1)
        Humidity= (100*hum/65535);           // RH = hum*100 / (2^16-1)
			
        if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=1)&&(Humidity<=100))//���˳���������Χ�Ĵ�������
        {
            TEM_Set_data(Temperature);
            HUM_Set_data(Humidity);
        }
    }
}



//CO2 У��
uint8_t  CO2_CRC(uint8_t *CO2_buffer)
{
  uint8_t i;
	uint16_t crc,sum=0;
	
	crc = CO2_buffer[10]<<8;
	crc = crc+CO2_buffer[11];
  for(i=0;i<10;i++)
	{
		sum = sum + CO2_buffer[i];                                  //У���
	}
	printf("CRC is %4x\r\n",crc);
  if(sum != crc){
		return 1;
	}else{
		return 0;
	}


}	






