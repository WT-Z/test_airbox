#ifndef __SC09_H__
#define __SC09_H__
#include "stdint.h"

//#define SDA_SC09	PB07
//#define SCL_SC09	PB06
//#define INT_SC09 	PB05
typedef enum //定义数据返回类型
{
	UNDONE = 0x00,
	DONE = 0x01
} Complete_Status;
#define SC09B_ADDR 0x40				   //SC09B 只有一个固定地址
void sc09_io_init(void);

/////// sc09 driver 
void I2C_Start(void);
unsigned char SendByteAndGetNACK(unsigned char dataToSend);
void I2C_Respond(unsigned char ACKSignal);
void I2C_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
void IIC_NAck(void);
void IIC_Ack(void);
uint8_t IIC_Wait_Ack(void);
uint8_t AT24CXX_ReadOneByte(uint8_t ReadAddr);
unsigned char I2C_Receive8Bit(void);
void SC09B_Init_Function(void);
Complete_Status I2C_Write_To_Device(unsigned char deviceAddr, unsigned char REG, unsigned char *DAT8);
Complete_Status I2C_Simple_Read_From_Device(unsigned char deviceAddr, unsigned int *DAT16);
Complete_Status I2C_Read_From_Device(unsigned char deviceAddr, unsigned char REG, unsigned int *DAT16);
void Humiture_send_Bytes(uint8_t Address,uint16_t Cmd,uint8_t stop);                                 //温湿度发送命令
void Humiture_Single_Shot(uint8_t *buffer);                                                          //单次读取温湿度数据

void CO2_IIC_NAck(void);
void CO2_IIC_Ack(void);
uint8_t CO2_IIC_Wait_Ack(void);
void CO2_I2C_Stop(void);
void CO2_IIC_Send_Byte(uint8_t txd);
uint8_t CO2_IIC_Read_Byte(unsigned char ack);
uint16_t CO2_ReadBytes(uint8_t ReadAddr);


uint8_t  CO2_CRC(uint8_t *CO2_buffer);
static uint8_t Hum_CRC_Check(uint8_t *check_data, uint8_t num, uint8_t check_crc);                //温湿度校验
void SHT30_Read(void);



#endif
