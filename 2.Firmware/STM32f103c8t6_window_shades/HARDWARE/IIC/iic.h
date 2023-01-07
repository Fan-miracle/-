/*********************************************************************************************
* 文件：iic.h
* 作者：zonesion
* 说明：iic头文件
* 修改：
* 注释：
*********************************************************************************************/
#ifndef _IIC_H_
#define _IIC_H_
#include "sys.h"

//IO方向设置
#define SDA_IN()  {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=8<<(4*(5-0));}
#define SDA_OUT() {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=3<<(4*(5-0));}

//IO操作函数	 
//#define IIC_SCL    PAout(4)                                                   //SCL
//#define IIC_SDA    PAout(5)                                                   //SDA	 
//#define READ_SDA   PAin(5)                                                    //输入SDA 

#define IIC_SCL    PBout(9)                                                   //SCL
#define IIC_SDA    PBout(8)                                                   //SDA	 
#define READ_SDA   PBin(8)                                                    //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                                                           //初始化IIC的IO口				 
void IIC_Start(void);				                                           //发送IIC开始信号
void IIC_Stop(void);	  			                                           //发送IIC停止信号
void IIC_Send_Byte(u8 txd);			                                           //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);                                           //IIC读取一个字节
u8 IIC_Wait_Ack(void); 				                                           //IIC等待ACK信号
void IIC_Ack(void);					                                           //IIC发送ACK信号
void IIC_NAck(void);				                                           //IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	
			 
u8 iicDevReadByte(u8 devaddr,u8 addr);	                                       /*读一字节*/
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data);	                           /*写一字节*/
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);                           /*连续读取多个字节*/
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);        

/*********************************************************************************************
* 外部原型函数
*********************************************************************************************/
//void iic_delay_us(unsigned int i);
void iic_init(void);
void iic_start(void);
void iic_stop(void);
void iic_send_ack(int ack);
int iic_recv_ack(void);
unsigned char iic_write_byte(unsigned char data);
unsigned char iic_read_byte(unsigned char ack);
//void delay(unsigned int t);

#endif 

