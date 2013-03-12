#include <ioCC2430.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "adxl345.h"
//#include "LCD3310.h"
#define SCL     P1_2   
#define SDA     P1_3

#define SDADirOut P1DIR|=0x08;      //xxxx1M01
#define SDADirIn  P1DIR&=~0x08;

#define TRUE 1
#define FALSE 0
#define	SlaveAddress 0xA6 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

INT8U BUFFER[14];
int  dis_data;

char wan,qian,bai,shi,ge;

void initUARTtest(void);
void UartTX_Send_String(char *Data,int len);
void conversion(void);

void Sendack(unsigned char h);
unsigned char I2C_Check_ack();

void I2C_Start_1(void);
void I2C_Stop_1(void);

void WriteI2CByte_1(INT8U b);
unsigned char ReadI2CByte_1(void);

unsigned char Single_Read_ADXL345(INT8U REG_Address);
void Single_Write_ADXL345(INT8U REG_Address,INT8U REG_data);

void Init_ADXL345();

void Delay_1u(unsigned int microSecs);
void Delay(INT16U n);

INT8U I2C_Check_ack(void);


void Multiple_Read_ADXL345(void);

void Delay(INT16U n)      //(5/32)*n   us
{
	INT16U d;
	for(d=0;d<n;d++);
	for(d=0;d<n;d++);
	for(d=0;d<n;d++);
	for(d=0;d<n;d++);
	for(d=0;d<n;d++);
}

void Delay_1u(unsigned int microSecs) {
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

void conversion(void)   
{  
  BUFFER[13] = 0xff;
  INT16U temp_data;
  if ((BUFFER[8] & 0x80) == 0x80)
  {
    BUFFER[13] &= ~0x01;    
  }
  temp_data = ((BUFFER[8] & 0x0f) << 8) + BUFFER[7];    
  temp_data = (INT16U)(((float)temp_data /4096 ) * 100);
  BUFFER[7] = (INT8U)((temp_data) % 10 + '0');
  BUFFER[8] = (INT8U)((temp_data) / 10 + '0');
    if ((BUFFER[10] & 0x80) == 0x80)
  {
    BUFFER[13] &= ~0x02;    
  }
  temp_data = ((BUFFER[10] & 0x0f) << 8) + BUFFER[9];    
  temp_data = (INT16U)(((float)temp_data / 4096) * 100);
  BUFFER[9] = (INT8U)((temp_data) % 10 + '0');
  BUFFER[10] = (INT8U)((temp_data) / 10 + '0');
    if ((BUFFER[12] & 0x80) == 0x80)
  {
    BUFFER[13] &= ~0x04;    
  }
  temp_data = ((BUFFER[12] & 0x0f) << 8) + BUFFER[11];    
  temp_data = (INT16U)(((float)temp_data / 4096) * 100);
  BUFFER[11] = (INT8U)((temp_data) % 10 + '0');
  BUFFER[12] = (INT8U)((temp_data) / 10 + '0');
} 

void I2C_Start_1(void)
    {
        /*启动I2C总线的函数，当SCL为高电平时使SDA产生一个负跳变*/
        SDADirOut;
        Delay_1u(1); 
        SDA=1;
        SCL=1;
        Delay_1u(5);
        SDA=0;
        Delay_1u(5);
        SCL=0;
        Delay_1u(5);
    }

    void I2C_Stop_1(void)
    {
        /*终止I2C总线，当SCL为高电平时使SDA产生一个正跳变*/
        SCL=0;
        Delay_1u(1);
        SDADirOut;
        Delay_1u(1);
        SDA=0;
        Delay_1u(5);
        SCL=1;
        Delay_1u(5);
        SDA=1;
        Delay_1u(5);
    }

void Sendack(unsigned char h){
        SCL=0;
        Delay_1u(5);
        SDADirOut;
        Delay(5);
        SDA=h&0x01;
        Delay_1u(5);
        SCL=1;
        Delay_1u(5);
        SCL=0;
        Delay_1u(5);
    }

INT8U I2C_Check_ack(void){
    SCL=0;
    Delay_1u(5);
    SDADirOut;
    SDA=1;
    SDADirIn;
    Delay_1u(5);          //此处是否有必要使SDA先拉高？！？！
    SCL=1;
    Delay_1u(5);
    if(SDA==1)
    {
      SCL=0;
      return 0;  //er
    }
    SCL=0;
    return 1;
}

    


    void WriteI2CByte_1(unsigned char b)
    {
        /*向I2C总线写一个字节*/
        unsigned char e=8;
        SDADirOut;
        while(e--){
          SCL=0;
          Delay_1u(5);
          if(b&0x80)SDA=1;
          else SDA=0;
          b<<=1;
          Delay_1u(3);
          SCL=1;
          Delay_1u(5);
        }
        SCL=0;
        I2C_Check_ack();
    }

unsigned char ReadI2CByte_1(void)
    {
        /*从I2C总线读一个字节*/
    unsigned char i=8;
    unsigned char c=0;
    SCL=0;
    SDADirOut;
    SDA=1;
    Delay_1u(3);
    SDADirIn;
    while(i--){
      c<<=1;
      SCL=0;
      Delay_1u(5);
      SCL=1;
      Delay_1u(5);
      if(SDA==1)c|=0x01;
      else c&=0xfe;
    }
    SCL=0;
    SDADirOut;
    return c; 
    }
//******单字节写入******************************************* 

void Single_Write_ADXL345(INT8U REG_Address,INT8U REG_data) 
{ 
    I2C_Start_1();                  //起始信号 
    WriteI2CByte_1(SlaveAddress);   //发送设备地址+写信号 
    WriteI2CByte_1(REG_Address);    //内部寄存器地址，请参考中文pdf22页  
    WriteI2CByte_1(REG_data);       //内部寄存器数据，请参考中文pdf22页  
    I2C_Stop_1();                   //发送停止信号 
} 

//********单字节读取***************************************** 
unsigned char Single_Read_ADXL345(INT8U REG_Address) 
{  INT8U REG_data;
    I2C_Start_1();                          //起始信号 
    WriteI2CByte_1(SlaveAddress);           //发送设备地址+写信号 
    WriteI2CByte_1(REG_Address);                   //发送存储单元地址，从0开始	
    I2C_Start_1();                          //起始信号 
    WriteI2CByte_1(SlaveAddress+1);         //发送设备地址+读信号 
    REG_data=ReadI2CByte_1();              //读出寄存器数据 
    Sendack(1);   
    I2C_Stop_1();                           //停止信号 
    return REG_data;  
} 

void Init_ADXL345() 
{ 
  P1DIR = 0x0C;
   Single_Write_ADXL345(0x31,0x0B);   //测量范围,正负16g，13位模式 
   Single_Write_ADXL345(0x2C,0x0F);   //速率设定为12.5 参考pdf13页 
   Single_Write_ADXL345(0x2D,0x08);   //选择电源模式   参考pdf24页 
   Single_Write_ADXL345(0x2E,0x80);   //使能 DATA_READY 中断 
   Single_Write_ADXL345(0x1E,0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页 
   Single_Write_ADXL345(0x1F,0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页 
   Single_Write_ADXL345(0x20,0x05);   //Z 偏移量 根据测试传感器的状态写入pdf29页 
} 

//********************************************************* 
// 
//连续读出ADXL345内部加速度数据，地址范围0x32~0x37 
// 
//********************************************************* 
void Multiple_Read_ADXL345(void) 
{   INT8U f; 
    I2C_Start_1();                          //起始信号 
    WriteI2CByte_1(SlaveAddress);           //发送设备地址+写信号 
    WriteI2CByte_1(0x32);                   //发送存储单元地址，从0x32开始	
    I2C_Start_1();                          //起始信号 
    WriteI2CByte_1(SlaveAddress+1);         //发送设备地址+读信号 
 for (f=0; f<6; f++)                      //连续读取6个地址数据，存储中BUF 
    { 
        BUFFER[7+f] = ReadI2CByte_1();          //BUF[0]存储0x32地址中的数据 
      if (f == 5) 
        { 
          Sendack(1);                //最后一个数据需要回NOACK 
        } 
        else 
        { 
          Sendack(0);                //回应ACK 
       } 
   } 
    I2C_Stop_1();                          //停止信号 
//    Delay(500);
//    Delay(500);
//    Delay(500);
//    Delay(500);
} 

void displayXYZ(INT8U *pData){
  int  dis_data;
  uint16 temp;
  dis_data=(pData[7]<<8)+pData[6];
    temp=(uint16)((float)dis_data*3.9);  //计算数据和显示,查考ADXL345快速入门第4页
	if(dis_data<0){
	dis_data=-dis_data;
        HalLcdWriteStringValue("X: -",temp,10,0);
	}
	else
    {
        HalLcdWriteStringValue("X:  ",temp,10,0);
    }
    dis_data=(pData[9]<<8)+pData[8];  //合成数据   
	    temp=(uint16)((float)dis_data*3.9);  //计算数据和显示,查考ADXL345快速入门第4页
	if(dis_data<0){
	dis_data=-dis_data;
        HalLcdWriteStringValue("Y: -",temp,10,1);
	}
	else
    {
        HalLcdWriteStringValue("Y:  ",temp,10,1);
    }
    dis_data=(pData[11]<<8)+pData[10];    //合成数据  
    temp=(uint16)((float)dis_data*3.9);  //计算数据和显示,查考ADXL345快速入门第4页
	if(dis_data<0){
	dis_data=-dis_data;
        HalLcdWriteStringValue("Z: -",temp,10,2);
	}
	else
    {
        HalLcdWriteStringValue("Z:  ",temp,10,2);
    }
}
