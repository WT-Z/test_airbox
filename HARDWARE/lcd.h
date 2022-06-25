#ifndef __LCD1_H
#define __LCD1_H


#include "stdint.h"
#include "gpio.h"
#include "airbox.h"
#include "zigbee.h"

//LCD��Ҫ������
typedef struct  
{										    
	uint16_t width;			//LCD ���
	uint16_t height;			//LCD �߶�
	uint16_t id;				//LCD ID
	uint8_t  dir;			//���������������ƣ�0��������1��������	
	uint16_t	wramcmd;		//��ʼдgramָ��
	uint16_t setxcmd;		//����x����ָ��
	uint16_t  setycmd;		//����y����ָ��	 
}_lcd_dev; 	  

#define  PM2_LOGO             0
#define  CO2_LOGO             1
#define  PM_NORMAL_LOGO       2                   
#define  PM_LIGHT_POLLUTION   3
#define  PM_HEAVY_POLLUTION   4
#define  AIR_FRESH            5
#define  AIR_POLLUTION        6
#define  TEM_LOGO             7
#define  HUM_LOGO             8
#define  DEG_LOGO             9
#define  PERC_LOGO            10

typedef struct {
	const unsigned char * image_p;
	uint8_t height;
	//uint8_t weight;
	uint16_t size;
}image_t;



//LCD����
//extern _lcd_dev lcddev;	//����LCD��Ҫ����
//LCD�Ļ�����ɫ�ͱ���ɫ	   
//extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
//extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ
////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���---------------- 
//#define LCD_LED_ON      Gpio_SetIO(GpioPortB,GpioPin5)          //LCD���⿪
//#define LCD_LED_OFF     Gpio_ClrIO(GpioPortB,GpioPin5)          //LCD�����

//#define	LCD_CS_SET  Gpio_SetIO(GpioPortB,GpioPin15)          //Ƭѡ�˿���1   PB15
//#define	LCD_RS_SET	Gpio_SetIO(GpioPortC,GpioPin6)          //����/���� 		PC6	   
//#define	LCD_WR_SET	Gpio_SetIO(GpioPortC,GpioPin7)          //д�ø�			  PC7
//#define	LCD_RD_SET	Gpio_SetIO(GpioPortC,GpioPin8)          //���ø�			  PC8

//#define	LCD_CS_CLR  Gpio_ClrIO(GpioPortB,GpioPin15)          //Ƭѡ�˿���0   PB15
//#define	LCD_RS_CLR	Gpio_ClrIO(GpioPortC,GpioPin6)          //����/���� 		PC6	   
//#define	LCD_WR_CLR	Gpio_ClrIO(GpioPortC,GpioPin7)          //д�õ�			  PC7
//#define	LCD_RD_CLR	Gpio_ClrIO(GpioPortC,GpioPin8)          //���õ�			  PC8

//#define LCD_RSEST_SET Gpio_SetIO(GpioPortB,GpioPin13)       // RESET
//#define LCD_RSEST_CLR Gpio_ClrIO(GpioPortB,GpioPin13)       // RESET


#define LCD_LED_ON   M0P_GPIO->PBOUT_f.PB05 = 1              //LCD���⿪
#define LCD_LED_OFF  M0P_GPIO->PBOUT_f.PB05 = 0              //LCD�����

#define	LCD_CS_SET  M0P_GPIO->PBOUT_f.PB15 = 1          //Ƭѡ�˿���1   PB15
#define	LCD_RS_SET	M0P_GPIO->PCOUT_f.PC06 = 1          //����/���� 		PC6	   
#define	LCD_WR_SET	M0P_GPIO->PCOUT_f.PC07 = 1          //д�ø�			  PC7
#define	LCD_RD_SET	M0P_GPIO->PCOUT_f.PC08 = 1          //���ø�			  PC8

#define	LCD_CS_CLR  M0P_GPIO->PBOUT_f.PB15 = 0          //Ƭѡ�˿���0   PB15
#define	LCD_RS_CLR	M0P_GPIO->PCOUT_f.PC06 = 0          //����/���� 		PC6	   
#define	LCD_WR_CLR	M0P_GPIO->PCOUT_f.PC07 = 0          //д�õ�			  PC7
#define	LCD_RD_CLR	M0P_GPIO->PCOUT_f.PC08 = 0          //���õ�			  PC8

#define LCD_RSEST_SET M0P_GPIO->PBOUT_f.PB13 = 1      // RESET
#define LCD_RSEST_CLR M0P_GPIO->PBOUT_f.PB13 = 0       // RESET



//PC0~7,��Ϊ������
/*#define DATAOUT(x)\
        {y = y&0x00000000;\
				z = x;\
        y = y|(x<<16);\
				y = y|((~z)&0xff);\
				Gpio_SetClrPort(GpioPortC,y);}*/          // �������
				
//#define DATAIN     Gpio_GetInputData(GpioPortC)   //��������    ֻ��Ҫ�Ͱ�λ ��lcd.C����ȥ����





//////////////////////////////////////////////////////////////////////
//ɨ�跽����
#define L2R_U2D  0 //������,���ϵ���
#define L2R_D2U  1 //������,���µ���
#define R2L_U2D  2 //���ҵ���,���ϵ���
#define R2L_D2U  3 //���ҵ���,���µ���

#define U2D_L2R  4 //���ϵ���,������
#define U2D_R2L  5 //���ϵ���,���ҵ���
#define D2U_L2R  6 //���µ���,������
#define D2U_R2L  7 //���µ���,���ҵ���

#define DFT_SCAN_DIR  L2R_U2D  //Ĭ�ϵ�ɨ�跽��
	 
//ɨ�跽����
#define L2R_U2D  0 //������,���ϵ���
#define L2R_D2U  1 //������,���µ���
#define R2L_U2D  2 //���ҵ���,���ϵ���
#define R2L_D2U  3 //���ҵ���,���µ���

#define U2D_L2R  4 //���ϵ���,������
#define U2D_R2L  5 //���ϵ���,���ҵ���
#define D2U_L2R  6 //���µ���,������
#define D2U_R2L  7 //���µ���,���ҵ���	 

#define DFT_SCAN_DIR  L2R_U2D  //Ĭ�ϵ�ɨ�跽��

//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ 
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#define WORDCOLOR       0xFFFF     //������ɫ
#define BACKCOLOR       0x0000     //������ɫ

void LCD_Init(void);													   	//��ʼ��
void LCD_DisplayOn(void);													//����ʾ
void LCD_DisplayOff(void);													//����ʾ
void LCD_Clear(uint16_t Color);	 												//����
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);										//���ù��
void LCD_DrawPoint(uint16_t x,uint16_t y);											//����
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);								//���ٻ���
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y); 											//���� 
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);					    			//��Բ
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);							//����
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);		   				//������
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);		   				//��䵥ɫ
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);				//���ָ����ɫ
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);						//��ʾһ���ַ�
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size);  						//��ʾһ������
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);				//��ʾ ����
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t p);		//��ʾһ���ַ���,12/16����

void LCD_WriteReg(uint8_t LCD_Reg, uint8_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint8_t RGB_Code);		  
void LCD_Scan_Dir(uint8_t dir);									//������ɨ�跽��
void LCD_Display_Dir(uint8_t dir);								//������Ļ��ʾ����
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);	//���ô���
void ST7789V_Init(void);
void LCD_Draw_Point(uint16_t x,uint16_t y,uint16_t color);
void LCD_WriteOneDot(uint16_t color);                   //дһ����
void Lcd_SetBox(uint16_t xStart,uint16_t yStart,uint16_t xlong,uint16_t ylong); //ѡ��Lcd��ָ���ľ�������
void LCD_Clear(uint16_t Color);    //����Ļ
uint16_t LCD_RD_DATA(void);
void DATAOUT(uint8_t x);
void getLCDID(void);
static void LCD_write_english(uint8_t data,uint16_t color,uint16_t xcolor ,uint8_t mode);    //д�ַ�
void LCD_Picture(uint16_t x,uint16_t y,uint16_t width,uint16_t height);        //дͼƬ
void LCD_write_english_string(uint16_t x, uint16_t y, char *str,uint8_t mode);  //Ӣ���ַ���
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);

void LCD_TEST(uint8_t x,uint8_t y,uint8_t mode);
void LCD_ShowChar_CH(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);        //дһ������
void LCD_rectangle(uint16_t xStart,uint16_t yStart,uint16_t xlong,uint16_t ylong);        //дһ������
void LCD_ShowPM(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode,uint16_t csize);             //��ʾPM2.5
void LCD_ShowNUM(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode,uint16_t csize);
void Clear_Part(uint16_t x,uint8_t y,uint16_t weight,uint8_t height,uint16_t Color);      //���ĳһ������


void LCD_ShowPicture(uint16_t x,uint16_t y,uint8_t mode,image_t *image);       


void CO2_Show_Handler(void);
void PM_Show_Handler(void);
void TemAndHum_Show_Handler(void);
void RTC_Show_Handler(void);                                            //������

void Net_Handler(void);

//LCD�ֱ�������
#define SSD_HOR_RESOLUTION		800		//LCDˮƽ�ֱ���
#define SSD_VER_RESOLUTION		480		//LCD��ֱ�ֱ���
//LCD������������
#define SSD_HOR_PULSE_WIDTH		1		//ˮƽ����
#define SSD_HOR_BACK_PORCH		46		//ˮƽǰ��
#define SSD_HOR_FRONT_PORCH		210		//ˮƽ����

#define SSD_VER_PULSE_WIDTH		1		//��ֱ����
#define SSD_VER_BACK_PORCH		23		//��ֱǰ��
#define SSD_VER_FRONT_PORCH		22		//��ֱǰ��
//���¼����������Զ�����
#define SSD_HT	(SSD_HOR_RESOLUTION+SSD_HOR_BACK_PORCH+SSD_HOR_FRONT_PORCH)
#define SSD_HPS	(SSD_HOR_BACK_PORCH)
#define SSD_VT 	(SSD_VER_RESOLUTION+SSD_VER_BACK_PORCH+SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)

 

#endif
