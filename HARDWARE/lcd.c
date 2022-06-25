#include "lcd.h"
#include "stdlib.h"
#include "gpio.h"
#include "logo.h"


image_t imageArray[] = 
{
	{&gImage_pm[0][0],25,392},      // PM  image 
	{&gImage_CO2[0][0],24,174},     // CO2 image
  {&gImage_ZC[0][0],28,216},      // ��������image
	{&gImage_LP[0][0],30,480},      //�����Ⱦ image
	{&gImage_HP[0][0],30,480},      //�ض���Ⱦ image
	{&gImage_AF[0][0],30,480},      //�������� image
	{&gImage_AP[0][0],30,480},      //�������� image
	{&gImage_Tem[0][0],27,232},     //�¶�     image
	{&gImage_Hum[0][0],27,232},     //ʪ��     image
	{&gImage_Deg[0][0],27,124},     //���϶�   image
	{&gImage_Perc[0][0],27,120},     //�ٷֺ�   image
	{&gImage_zigbee[0][0],20,60},     //zigbee   image
	{&gImage_gantan[0][0],20,60},     //zigbee   image
};
image_t image_Num[10] =
{
  {&NUM[0][0],82,506},           //����0
  {&NUM[1][0],82,506},
	{&NUM[2][0],82,506},
	{&NUM[3][0],82,506},
	{&NUM[4][0],82,506},
	{&NUM[5][0],82,506},
	{&NUM[6][0],82,506},
	{&NUM[7][0],82,506},
	{&NUM[8][0],82,506},
	{&NUM[9][0],82,506},
	
	//0,
};

#define  PM2_CLEAR   {Clear_Part(103,12,100,12);} 







//LCD�Ļ�����ɫ�ͱ���ɫ	   
uint16_t POINT_COLOR=0xFFFF;	//������ɫ
uint16_t BACK_COLOR=0x0000;  //����ɫ 


#define GPIO_DIR_OUT      {M0P_GPIO->PCDIR_f.PC09 = 1;\
	                         M0P_GPIO->PADIR_f.PA08 = 1;\
	                         M0P_GPIO->PADIR_f.PA09 = 1;\
	                         M0P_GPIO->PADIR_f.PA10 = 1;\
                           M0P_GPIO->PADIR_f.PA11 = 1;\
                           M0P_GPIO->PADIR_f.PA12 = 1;\
	                         M0P_GPIO->PDDIR_f.PD06 = 1;\
	                         M0P_GPIO->PDDIR_f.PD07 = 1;}	

#define GPIO_DIR_IN       {M0P_GPIO->PCDIR_f.PC09 = 0;\
	                         M0P_GPIO->PADIR_f.PA08 = 0;\
	                         M0P_GPIO->PADIR_f.PA09 = 0;\
	                         M0P_GPIO->PADIR_f.PA10 = 0;\
                           M0P_GPIO->PADIR_f.PA11 = 0;\
                           M0P_GPIO->PADIR_f.PA12 = 0;\
	                         M0P_GPIO->PDDIR_f.PD06 = 0;\
	                         M0P_GPIO->PDDIR_f.PD07 = 0;}
											 
													 
													  					    
//д�Ĵ�������
//data:�Ĵ���ֵ
void LCD_WR_REG(uint8_t cmd)
{ 
	uint16_t num;
	LCD_RS_CLR;           //����  
 	LCD_CS_CLR;           //Ƭѡ
  LCD_RD_SET;	          //������
	
	M0P_GPIO->PCOUT_f.PC09 =  cmd & 0x01;		
   num = cmd << 7;
	 num |= ((M0P_GPIO->PAOUT)&0x00FF);
   M0P_GPIO->PAOUT = num;
	M0P_GPIO->PDOUT_f.PD06 = (cmd >> 6) &0x01;		
	M0P_GPIO->PDOUT_f.PD07 = (cmd >> 7) &0x01;		
	
	LCD_WR_CLR;           //д����
	LCD_WR_SET;           //д����
 	LCD_CS_SET;           //Ƭѡ
	LCD_RS_SET;           //��Ϊ����
   	
}
//д���ݺ���
//data:�Ĵ���ֵ
void LCD_WR_DATAX(uint8_t cmd)
{
	//uint8_t num_0,num_1,num_2,num_3,num_4,num_5,num_6,num_7;
	uint16_t num;
	LCD_RS_SET;           //д����
	LCD_CS_CLR;           //Ƭѡ
	LCD_RD_SET;	          //������
	//DATAOUT(data);        //��������
	 M0P_GPIO->PCOUT_f.PC09 =  cmd & 0x01;		
   num = cmd << 7;
	 num |= ((M0P_GPIO->PAOUT)&0x00FF);
   M0P_GPIO->PAOUT = num;
	 
	M0P_GPIO->PDOUT_f.PD06 = (cmd >> 6) &0x01;		
	M0P_GPIO->PDOUT_f.PD07 = (cmd >> 7) &0x01;
	
	LCD_WR_CLR;           //д����
	LCD_WR_SET;           //д����
	LCD_CS_SET;           //Ƭѡ
}
//��LCD����
//����ֵ:������ֵ
uint16_t LCD_RD_DATA(void)
{										   
	uint16_t t = 0;
	 GPIO_DIR_IN;
	 //Gpio_ClrPort(GpioPortC,0xff);
	Gpio_ClrIO(GpioPortC,GpioPin9);
	Gpio_ClrIO(GpioPortA,GpioPin8);
	Gpio_ClrIO(GpioPortA,GpioPin9);
	Gpio_ClrIO(GpioPortA,GpioPin10);
	Gpio_ClrIO(GpioPortA,GpioPin11);
	Gpio_ClrIO(GpioPortA,GpioPin12);
	Gpio_ClrIO(GpioPortD,GpioPin6);
	Gpio_ClrIO(GpioPortD,GpioPin7);

	LCD_RS_SET;
	LCD_CS_CLR;
	LCD_WR_SET;           //д����
	LCD_RD_CLR;
	//if(lcddev.id==0X7789)delay10us(2);//FOR 7789,��ʱ2us					   
	//t = DATAIN & 0xff;
  //t = M0P_GPIO->PCIN; 
  t |= Gpio_GetInputIO(GpioPortC,GpioPin9);
	t |= Gpio_GetInputIO(GpioPortA,GpioPin8)<<1;
	t |= Gpio_GetInputIO(GpioPortA,GpioPin9)<<2;
	t |= Gpio_GetInputIO(GpioPortA,GpioPin10)<<3;
	t |= Gpio_GetInputIO(GpioPortA,GpioPin11)<<4;
	t |= Gpio_GetInputIO(GpioPortA,GpioPin12)<<5;
	t |= Gpio_GetInputIO(GpioPortD,GpioPin6)<<6;
	t |= Gpio_GetInputIO(GpioPortD,GpioPin7)<<7;

	LCD_RD_SET;
	LCD_CS_SET; 
  

  GPIO_DIR_OUT;
  Gpio_SetIO(GpioPortC,GpioPin9);
	Gpio_SetIO(GpioPortA,GpioPin8);
	Gpio_SetIO(GpioPortA,GpioPin9);
	Gpio_SetIO(GpioPortA,GpioPin10);
	Gpio_SetIO(GpioPortA,GpioPin11);
	Gpio_SetIO(GpioPortA,GpioPin12);
	Gpio_SetIO(GpioPortD,GpioPin6);
	Gpio_SetIO(GpioPortD,GpioPin7);

	return t;  
}
//д�Ĵ���
//LCD_Reg:�Ĵ������
//LCD_RegValue:Ҫд���ֵ
void LCD_WriteReg(uint8_t LCD_Reg,uint8_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATAX(LCD_RegValue);	    		 
}   
//���Ĵ���
//LCD_Reg:�Ĵ������
//����ֵ:������ֵ
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{										   
 	LCD_WR_REG(LCD_Reg);  //д��Ҫ���ļĴ�����  
	return LCD_RD_DATA(); 
} 
//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
	//LCD_WR_REG(lcddev.wramcmd);
} 
//LCDдGRAM
//RGB_Code:��ɫֵ
void LCD_WriteRAM(uint8_t RGB_Code)
{							    
	LCD_WR_DATAX(RGB_Code);//д��λGRAM
}
//��ILI93xx����������ΪGBR��ʽ��������д���ʱ��ΪRGB��ʽ��
//ͨ���ú���ת��
//c:GBR��ʽ����ɫֵ
//����ֵ��RGB��ʽ����ɫֵ
uint16_t LCD_BGR2RGB(uint16_t c)
{
	uint16_t  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
}	
//��mdk -O1ʱ���Ż�ʱ��Ҫ����
//��ʱi
void opt_delay(uint8_t i)
{
	while(i--);
}

void LCD_DisplayOn(void)
{					   
	//if(lcddev.id==0X77)LCD_WR_REG(0X29,);	//������ʾ
	LCD_WR_REG(0X29);	//������ʾ
}	 
//LCD�ر���ʾ
void LCD_DisplayOff(void)
{	   
	//if(lcddev.id==0X9341||lcddev.id==0X6804||lcddev.id==0X5310||lcddev.id==0X1963)LCD_WR_REG(0X28);	//�ر���ʾ
	LCD_WR_REG(0X28);
}   

void ST7789V_Init(void)
{
  Gpio_SetIO(GpioPortB,GpioPin13);
	delay1ms(120);
	Gpio_ClrIO(GpioPortB,GpioPin13);
	delay1ms(120);
	Gpio_SetIO(GpioPortB,GpioPin13);
	delay1ms(120);
	
    LCD_WR_REG(0x11);     //Sleep out

		delay1ms(120);         //Delay 120ms

	  LCD_WR_REG(0x36);    //memory data access control 
		LCD_WR_DATAX(0xA0);   	//page address order:top to bottom;column address order:left to right;
							                  //page/column order:normal mode;line address order:LCD refresh top to bottom,
							                  //RGB;display data latch data order:LCD refresh left to right
    LCD_WR_REG(0x21);   //display inversion on
	
	  LCD_WR_REG(0xB2);    	//Porch setting    
		LCD_WR_DATAX(0x05);
		LCD_WR_DATAX(0x05);
		LCD_WR_DATAX(0x00);  	//disable separate proch control
		LCD_WR_DATAX(0x33);  	//back porch setting in idle mode.front porch setting in idle mode.
		LCD_WR_DATAX(0x33);  	//back porch setting in partial mode.front porch setting in partial mode.
	
	  LCD_WR_REG(0xB7);  	//Gate control
		LCD_WR_DATAX(0x75); 	  //VGH=14.5V,VGL=-9.6V	
		
	  LCD_WR_REG(0xBB);  	//VCOMS Setting 
		LCD_WR_DATAX(0x22);	  //0x2A,VCOMS=1.15V; 0X26,0X1C  		
		
	  LCD_WR_REG(0xC0);  	//LCM Control
		LCD_WR_DATAX(0x2C); 	  //
		
	  LCD_WR_REG(0xC2);  	//VDV and VRH command enable  
		LCD_WR_DATAX(0x01); 	  //VDV and VRH register value comes from command write.
		
	  LCD_WR_REG(0xC3);  	//VRH set
		LCD_WR_DATAX(0x13); 	   //GVDD=4.8V		
		
	  LCD_WR_REG(0xC4);   	//VDV set 
		LCD_WR_DATAX(0x20);  	//VDV=0V	

	  LCD_WR_REG(0xC6);	 	//frame rate control in normal mode
		LCD_WR_DATAX(0x0F);

	  LCD_WR_REG(0xD0);  	//power control 1 
		LCD_WR_DATAX(0xA4);   
		LCD_WR_DATAX(0xA1);   	//AVDD=6.8V,AVCL=-4.8V,VDDS=2.3V

	  LCD_WR_REG(0xD6);   //
		LCD_WR_DATAX(0xA1);   

	  LCD_WR_REG(0xE0);    	//positive voltage gamma control
		LCD_WR_DATAX(0xD0);
		LCD_WR_DATAX(0x04);
		LCD_WR_DATAX(0x09);
		LCD_WR_DATAX(0x0A);
		LCD_WR_DATAX(0x09);
		LCD_WR_DATAX(0x26);
		LCD_WR_DATAX(0x2E);
		LCD_WR_DATAX(0x54);
		LCD_WR_DATAX(0x45);
		LCD_WR_DATAX(0x2B);
		LCD_WR_DATAX(0x18);
		LCD_WR_DATAX(0x16);
		LCD_WR_DATAX(0x29);
		LCD_WR_DATAX(0x2D);
		
	  LCD_WR_REG(0xE1);   	//negative voltage gamma control
		LCD_WR_DATAX(0xD0);
		LCD_WR_DATAX(0x06);
		LCD_WR_DATAX(0x0A);
		LCD_WR_DATAX(0x07);
		LCD_WR_DATAX(0x07);
		LCD_WR_DATAX(0x04);
		LCD_WR_DATAX(0x2C);
		LCD_WR_DATAX(0x22);
		LCD_WR_DATAX(0x46);
		LCD_WR_DATAX(0x38);
		LCD_WR_DATAX(0x16);
		LCD_WR_DATAX(0x17);
		LCD_WR_DATAX(0x2B);
		LCD_WR_DATAX(0x2E);	

	  LCD_WR_REG(0x3A);	 	//interface pixel format 
		LCD_WR_DATAX(0x05);  	//0x06,18bit/pixel; 0x05,16bit/pixel, 
	
	  //LCD_WR_REG(0x29);	
}


//Lcd�����㶨λ����
void LCD_WriteOneDot(uint16_t color)
{ 
  
    LCD_WR_DATAX(color>>8);
    LCD_WR_DATAX(color);
}

//ѡ��Lcd��ָ���ľ�������
void Lcd_SetBox(uint16_t xStart,uint16_t yStart,uint16_t xlong,uint16_t ylong)
{
  uint16_t xEnd=0, yEnd=0;
  xEnd=xStart+xlong-1;
  yEnd=yStart+ylong-1;
        
  LCD_WR_REG(0x2a);   
	LCD_WR_DATAX(xStart>>8);
	LCD_WR_DATAX(xStart);
	LCD_WR_DATAX(xEnd>>8);
	LCD_WR_DATAX(xEnd);

	LCD_WR_REG(0x2b);   
	LCD_WR_DATAX(yStart>>8);
	LCD_WR_DATAX(yStart);
	LCD_WR_DATAX(yEnd>>8);
	LCD_WR_DATAX(yEnd);

	LCD_WR_REG(0x2c);                                    
 	
}
void LCD_Draw_Point(uint16_t x,uint16_t y,uint16_t color)
{
  Lcd_SetBox(x,y,1,1);
  LCD_WriteOneDot(color);
  
}
void LCD_Clear(uint16_t Color)
{
   uint32_t i;  
   Lcd_SetBox(0,0,320,240);  
   for(i=0;i<76800;i++){       
     LCD_WriteOneDot(Color); 
  }
}

void DATAOUT(uint8_t x)
{
   uint32_t a = 0;
   uint32_t b = 0;
   a = (x<<16) & 0xFFFF0000;
   b = (~x) & 0xFF;
   a = a | b;
   Gpio_SetClrPort(GpioPortC,a);
}

void getLCDID(void)
{
    uint16_t p1,p2,p3,p4;
    LCD_WR_REG(0x04);
		p1 = LCD_RD_DATA();
		p2 = LCD_RD_DATA();
		p3 = LCD_RD_DATA();
		p4 = LCD_RD_DATA();
}
static void LCD_write_english(uint8_t data,uint16_t color,uint16_t xcolor ,uint8_t mode)//д�ַ�
{
  uint8_t j=0,n=0;
//	uint16_t i;
  uint8_t avl=0;
  data -=32;                      
  //for (i=0;i<sizeof(asc2_1206);i++) //Ϊ 20x40�ֿ�       
  {     
    //avl=asc2_1206[data][i];         
    for (j=0;j<8;j++)           
    {
      n++;
      if(avl&0x80)LCD_WriteOneDot(color);        
      else if(mode==0) LCD_WriteOneDot(xcolor);
      avl<<=1;
      if(n>5) {
        n=0;
        break;
      }//����������Ӣ��20*40���γɵ��ֿ�3��8λһ�飬ÿһ�����4λ����ʾ���ø��������ж��м�λ����Ҫ��ʾ
    }     
   }
}

void LCD_write_english_string(uint16_t x, uint16_t y, char *str,uint8_t mode)//Ӣ���ַ�����ʾ
{
 uint16_t k = 0;
  while ((*str<='~')&&(*str>=' ')) 
  {
     Lcd_SetBox(x+k,y,20,40);
     LCD_write_english( *str,WORDCOLOR,BACKCOLOR, mode);
     k+=20;
     str++;      
  }
}

void LCD_Picture(uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{  
	uint16_t i,j;
  j=width*height;
  Lcd_SetBox(x,y,width,height);
     	
 	for(i=0;i<sizeof(gImage_pm);i++)
	{ 
		LCD_WR_DATAX(gImage_pm[0][i]);
//    temp=gImage_pm[i*2]<<8&0xFF00;
//    temp+=gImage_pm[i*2+1];
		//temp=gImage_pm[i];
    //LCD_WriteOneDot(temp);
	}	  
}

//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  							  
    uint8_t temp,t1,t;
	uint16_t y0=y;
	uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
 	//num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		//if(size==32)temp=AS_test[num][t]; 	 	//����1206����
		//if(size==32)temp=AQI[num][t];
		//if(size==24)temp=PM[num][t];
		
		//else if(size==95)temp=PM1[num][t];
		//else if(size==16)temp=asc2_1608[num][t];	//����1608����
		//else if(size==24)temp=asc2_2412[num][t];	//����2412����
		//else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Draw_Point(x,y,POINT_COLOR);
			else if(mode==0)LCD_Draw_Point(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=320)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=240)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}   

void LCD_TEST(uint8_t x,uint8_t y,uint8_t mode)
{
	uint8_t t1,tempt,t;
	
	for (t=0;t<8;t++) {
	//tempt = cha3[t];
for(t1=0;t1<8;t1++)
		{			    
			if(tempt&0x80)LCD_Draw_Point(x+t,y+t1,RED);
			else if(mode==0)LCD_Draw_Point(x+t,y+t1,0XFFE0);
			tempt<<=1;
	}

  }
}

//��ָ��λ����ʾһ������
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)

void LCD_ShowChar_CH(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  							  
    uint8_t temp,t1,t;
	uint16_t y0=y;
	//uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
  uint8_t csize=(size/8+((size%8)?1:0))*(size);
 	//num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   		
//		if(size==24)temp=AQI[num][t];//�����ֿ�
		//else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80) {
				switch (num) {
					case 4: LCD_Draw_Point(x,y,GREEN); break;
				  case 5: LCD_Draw_Point(x,y,YELLOW); break;
				  case 6: LCD_Draw_Point(x,y,RED); break;
					default: LCD_Draw_Point(x,y,POINT_COLOR);break;
				}			
			}
			else if(mode==0)LCD_Draw_Point(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=320)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=240)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}   

//��ʾ�ַ���
//x,y:�������
//width,height:�����С  
//size:�����С
//*p:�ַ�����ʼ��ַ		  
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t p)
{         
	uint8_t x0=x;
	width+=x;
	height+=y;
//    while(p<sizeof(PM))//�ж��ǲ��ǷǷ��ַ�!
//    {       
//        if(x>=width){x=x0;y+=size;}
//        if(y>=height)break;//�˳�
//        LCD_ShowChar(x,y,p,size,0);
//        x+=size/2;
//        p++;
//    }  
}
//��һ��С����
//x,y:�������
//xlong,ylong:�����С
//clr
void LCD_rectangle(uint16_t xStart,uint16_t yStart,uint16_t xlong,uint16_t ylong)
{
   uint32_t i;
   uint16_t xEnd=0, yEnd=0;
  xEnd=xStart+xlong-1;
  yEnd=yStart+ylong-1;  
   Lcd_SetBox( xStart,yStart,xlong,ylong); 
  for(i=0;i<(xEnd*yEnd);i=i+40){       
     LCD_WriteOneDot(0xFFFF); 
  }

}
//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//csize:ʵ�ʿ�ȴ�С
void LCD_ShowPM(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode,uint16_t csize)
{  							  
    uint8_t temp,t1;
	uint16_t t;
	uint16_t y0=y;
	//uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	 //uint16_t csize=588;		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
 	//num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==27)temp=gImage_pm[num][t];                              //��ʾPM2.5ͼ��
		else if(size ==46)temp=gImage_ZC[num][t];                        //��ʾ����ͼ��
		else if(size ==42)temp=gImage_CO2[num][t];
		else if(size ==29)temp =gImage_AirFresh[num][t];
		else if(size ==30)temp =gImage_Tem[num][t];
		//else if(size ==96)temp=NUM[num][t];
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Draw_Point(x,y,0x0000);
			else if(mode==0)LCD_Draw_Point(x,y,0xFFFF);
			temp<<=1;
			y++;
			if(y>=240)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=320)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}

//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//csize:ʵ�ʿ�ȴ�С
void LCD_ShowNUM(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode,uint16_t csize)
{  							  
  uint8_t temp,t1;
	uint16_t t;
	uint16_t y0=y;
	//uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
	 //uint16_t csize=588;		//�õ�����һ���ַ���Ӧ������ռ���ֽ���
 	//num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==27)temp=gImage_pm[num][t];
		else if(size ==46)temp=gImage_ZC[num][t];
//		else if(size ==82)temp=NUM[num][t];
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Draw_Point(x,y,0x0000);
			else if(mode==0)LCD_Draw_Point(x,y,0xffff);
			temp<<=1;
			y++;
			if(y>=240)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=320)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}


//���ĳ������
//x:��ʼ������
//y:��ʼ������
//height:�������߶�
//weight�����������
void Clear_Part(uint16_t x,uint8_t y,uint16_t weight,uint8_t height,uint16_t Color)
{
  uint32_t i;
  Lcd_SetBox(x,y,weight,height);
  for (i=0;i<(weight*height);i++) {
	   LCD_WriteOneDot(Color);
	}
}	

void CO2_Show_Handler(void)
{
   uint8_t thousand=0 ,hundreds=0 ,tens=0,lows=0;
	 uint16_t CO2_data ; 
	 CO2_data = CO2_Get_data();

	 printf("CO2_data is %d\r\n",CO2_data);
	 thousand = CO2_data/1000; 
   hundreds = CO2_data/100%10;
	 tens = CO2_data/10%10;
	 lows = CO2_data%10;
	
	 LCD_Clear(0x0000);
	                     
	 LCD_ShowPicture(130,14,0,&imageArray[CO2_LOGO]);  //��CO2
	
	
	LCD_ShowPicture(41,72,0,&image_Num[thousand]);
	LCD_ShowPicture(102,72,0,&image_Num[hundreds]);
	LCD_ShowPicture(161,72,0,&image_Num[tens]);
	LCD_ShowPicture(221,72,0,&image_Num[lows]);
	

  if (CO2_data >1000 ) {
	  LCD_ShowPicture(89,197,0,&imageArray[AIR_POLLUTION]); //��������
		CO2_Set_Status(CO2_ARARM);
	} else {	
	  LCD_ShowPicture(89,197,0,&imageArray[AIR_FRESH]);     //��������
		CO2_Set_Status(CO2_NORMAL);
	}	
}

void PM_Show_Handler(void)
{
  uint8_t hundreds=0 ,tens=0,lows=0;
  uint8_t PM_data ;
  PM_data = PM_Get_data();
	
  hundreds = PM_data/100%10;
  tens = PM_data/10%10;
  lows = PM_data%10;


  LCD_Clear(0x0000);
  //Clear_Part(0,5,300,40,0x0000);
	//Clear_Part(0,70,300,88,0x0000);
	
	LCD_ShowPicture(111,13,0,&imageArray[PM2_LOGO]);                    //��ʾPM2.5ͼ�� 
	//Clear_Part(44,45,230,124,0x0000);                                    
			 
 
	LCD_ShowPicture(73,73,0,&image_Num[hundreds]);
	LCD_ShowPicture(135,73,0,&image_Num[tens]);
	LCD_ShowPicture(190,73,0,&image_Num[lows]);
	//LCD_ShowNUM(190,73,lows,82,0,506);
			
	Clear_Part(103,180,200,29,0x0000);
	


  if (PM_data>115) {
  	LCD_ShowPicture(89,197,0,&imageArray[PM_HEAVY_POLLUTION]);               //��ʾ�ض���Ⱦ	
    PM_Set_Status(PM_ARARM);		
	} else if(PM_data<115 && PM_data>75){
	 LCD_ShowPicture(89,197,0,&imageArray[PM_LIGHT_POLLUTION]);               //��ʾ�����Ⱦ
   PM_Set_Status(PM_ARARM);		
	} else if(PM_data<75) { 
    LCD_ShowPicture(129,197,0,&imageArray[PM_NORMAL_LOGO]);               //��ʾ����
		PM_Set_Status(PM_NORMAL);
	}	
}

void TemAndHum_Show_Handler(void)
{
  uint8_t Hum_tens=0,Hum_lows=0,Tem_tens,Tem_lows=0;
	uint16_t HUM_Data =0, Tem_Data=0;
  HUM_Data = HUM_Get_data();
	Tem_Data = TEM_Get_data();
	
	
	Hum_tens = HUM_Data/10%10;
	Hum_lows = HUM_Data%10;
	
	Tem_tens = Tem_Data/10%10;
	Tem_lows = Tem_Data%10;
  LCD_Clear(0x0000);
	LCD_ShowPicture(48,9,0,&imageArray[TEM_LOGO]);              //��ʾ�¶�ͼ��
	LCD_ShowPicture(213,9,0,&imageArray[HUM_LOGO]);             //��ʾʪ��ͼ��
	
//	LCD_ShowNUM(4,72,Tem_tens,82,0,506);
//	LCD_ShowNUM(64,72,Tem_lows,82,0,506);
	
	LCD_ShowPicture(4,72,0,&image_Num[Tem_tens]);
	LCD_ShowPicture(64,72,0,&image_Num[Tem_lows]);
	LCD_ShowPicture(121,138,0,&imageArray[DEG_LOGO]);
	
//	LCD_ShowNUM(175,72,Hum_tens,82,0,506);
//	LCD_ShowNUM(237,72,Hum_lows,82,0,506);
  LCD_ShowPicture(175,72,0,&image_Num[Hum_tens]);
	LCD_ShowPicture(237,72,0,&image_Num[Hum_lows]);
	
	LCD_ShowPicture(292,135,0,&imageArray[PERC_LOGO]);
	
}

//��ָ��λ����ʾһ��ͼ��
//x,y:��ʼ���� 
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//csize:ʵ�ʿ�ȴ�С
//image_t:ͼƬ
void LCD_ShowPicture(uint16_t x,uint16_t y,uint8_t mode,image_t *image)
{  							  
  uint8_t temp,t1;
	uint16_t t;
	uint16_t y0=y;
	uint16_t csize = 0;
	csize = image->size;
	for(t=0;t<csize;t++)
	{   
		temp=(image->image_p)[t];   //*((image->image)+t)                           //��ʾPMͼ��
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Draw_Point(x,y,0x0000);
			else if(mode==0)LCD_Draw_Point(x,y,0xFFFF);
			temp<<=1;
			y++;
			if(y>=240)return;		//��������
			if((y-y0)==image->height)
			{
				y=y0;
				x++;
				if(x>=320)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}

void RTC_Show_Handler(void)
{
  unsigned char hour =0;
	unsigned char min =0;
	unsigned char sec =0;
	unsigned short w_year =0;
	unsigned char  w_month=0;
	unsigned char  w_date=0; 
	uint8_t thousand =0,hundreds=0 ,tens=0,lows=0;
 
  hour = _time.hour+8;
	min = _time.min;
	sec = _time.sec;
	w_year = _time.w_year;
	w_month = _time.w_month;
	w_date = _time.w_date;
	
	thousand = w_year/1000;
	hundreds = w_year/100%10;
	tens = w_year/10%10;
	lows = w_year%10;
	
	LCD_Clear(0x0000);
	LCD_ShowPicture(41,72,0,&image_Num[thousand]);
	LCD_ShowPicture(102,72,0,&image_Num[hundreds]);
	LCD_ShowPicture(161,72,0,&image_Num[tens]);
	LCD_ShowPicture(221,72,0,&image_Num[lows]);
	
	thousand = w_month/10%10;                            //��
	hundreds = w_month%10;
	tens = w_date/10%10;                                 //��
	lows = w_date%10;
	
	LCD_Clear(0x0000);
	LCD_ShowPicture(41,72,0,&image_Num[thousand]);
	LCD_ShowPicture(102,72,0,&image_Num[hundreds]);
	LCD_ShowPicture(161,72,0,&image_Num[tens]);
	LCD_ShowPicture(221,72,0,&image_Num[lows]);
	
	
	thousand = hour/10%10;                            //hour
	hundreds = hour%10;
	tens = min/10%10;                                 //min
	lows = min%10;
	
	LCD_Clear(0x0000);
	LCD_ShowPicture(2,72,0,&image_Num[thousand]);
	LCD_ShowPicture(61,72,0,&image_Num[hundreds]);
	LCD_ShowPicture(110,72,0,&image_Num[tens]);
	LCD_ShowPicture(160,72,0,&image_Num[lows]);
	
	
	tens = sec/10%10;                                 //min
	lows = sec%10;
	//LCD_Clear(0x0000);
	
	LCD_ShowPicture(210,72,0,&image_Num[tens]);
	LCD_ShowPicture(260,72,0,&image_Num[lows]);
}

void Net_Handler(void)
{
	uint8_t netstate =0;
	netstate=Net_Get_State();
	if(!netstate)
	{
	Clear_Part(0,0,20,20,0x0000);
	//delay1ms(500);
	LCD_ShowPicture(0,0,0,&imageArray[12]);
	
	}else {
	//Clear_Part(0,0,20,20,0x0000);
		LCD_ShowPicture(0,0,0,&imageArray[11]);
	}	
  //LCD_ShowPicture(0,0,0,&imageArray[11]);

}	









