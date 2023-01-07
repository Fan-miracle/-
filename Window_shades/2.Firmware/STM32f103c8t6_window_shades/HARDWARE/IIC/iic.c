/*********************************************************************************************
* 文件：iic.c
* 作者：zonesion
* 说明：iic驱动程序
* 修改：
* 注释：
*********************************************************************************************/

/*********************************************************************************************
* 头文件
*********************************************************************************************/
#include "stm32f10x.h"
#include "iic.h"
#include "delay.h"

/*********************************************************************************************
* 宏定义
*********************************************************************************************/
#define I2C_GPIO                GPIOB //GPIOB  不重复 GPIOD
#define I2C_CLK                 RCC_APB2Periph_GPIOB  //RCC_APB2Periph_GPIOD  //RCC_AHB1Periph_GPIOB  不重复
#define PIN_SCL                 GPIO_Pin_9  //不重复 GPIO_Pin_6
#define PIN_SDA                 GPIO_Pin_8  //不重复 GPIO_Pin_7

#define SDA_R                   GPIO_ReadInputDataBit(I2C_GPIO,PIN_SDA) //不重复

/*********************************************************************************************
* 名称：iic_init()
* 功能：I2C初始化函数
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
//void iic_init(void)
//{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  //RCC_AHB1PeriphClockCmd(I2C_CLK, ENABLE);
//  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
//	
//  GPIO_InitStructure.GPIO_Pin = PIN_SCL | PIN_SDA;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ; //GPIO_Mode_OUT;  //推挽输出
//  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_2MHz;
//	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
//	
//  GPIO_SetBits(GPIOB,PIN_SCL | PIN_SDA);//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  
//    
//  
//}
void iic_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  //RCC_AHB1PeriphClockCmd(I2C_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(	I2C_CLK, ENABLE );
	
  GPIO_InitStructure.GPIO_Pin = PIN_SCL | PIN_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ; //GPIO_Mode_OUT;  //推挽输出
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_2MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
	
  GPIO_SetBits(I2C_GPIO,PIN_SCL | PIN_SDA);//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
    
  
}
/*********************************************************************************************
* 名称：sda_out()
* 功能：设置SDA为输出
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sda_out(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;//GPIO_Mode_OUT;
 // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_SetBits(I2C_GPIO,PIN_SDA);
    
}

/*********************************************************************************************
* 名称：sda_in()
* 功能：设置SDA为输入
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sda_in(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure); 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_SetBits(I2C_GPIO,PIN_SDA);
   
}

/*********************************************************************************************
* 名称：iic_start()
* 功能：I2C起始信号
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void iic_start(void)
{
  sda_out();
  GPIO_SetBits(I2C_GPIO,PIN_SDA);                               //拉高数据线
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                               //拉高时钟线
  delay_us(5);                                                  //延时
  GPIO_ResetBits(I2C_GPIO,PIN_SDA);                             //产生下降沿
  delay_us(5);                                                  //延时
  GPIO_ResetBits(I2C_GPIO,PIN_SCL);                             //拉低时钟线
}

/*********************************************************************************************
* 名称：iic_stop()
* 功能：I2C停止信号
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void iic_stop(void)
{
  sda_out();
  GPIO_ResetBits(I2C_GPIO,PIN_SDA);                             //拉低数据线
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                               //拉高时钟线
  delay_us(5);                                                  //延时5us
  GPIO_SetBits(I2C_GPIO,PIN_SDA);                               //产生上升沿
  delay_us(5);                                                  //延时5us
}

/*********************************************************************************************
* 名称：iic_send_ack()
* 功能：I2C发送应答
* 参数：ack -- 应答信号
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void iic_send_ack(int ack)
{
  sda_out();
  if(ack)
    GPIO_SetBits(I2C_GPIO,PIN_SDA);                             //写应答信号
  else
    GPIO_ResetBits(I2C_GPIO,PIN_SCL); 
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                               //拉高时钟线
  delay_us(5);                                                  //延时
  GPIO_ResetBits(I2C_GPIO,PIN_SCL);                             //拉低时钟线
  delay_us(5);                                                  //延时
}

/*********************************************************************************************
* 名称：iic_recv_ack()
* 功能：I2C接收应答
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
int iic_recv_ack(void)
{
  int CY = 0;
  sda_in();
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                               //拉高时钟线
  delay_us(5);                                                  //延时
  CY = SDA_R;                                                   //读应答信号
  GPIO_ResetBits(I2C_GPIO,PIN_SDA);                             //拉低时钟线
  delay_us(5);                                                  //延时
  return CY;
}

/*********************************************************************************************
* 名称：iic_write_byte()
* 功能：I2C写一个字节数据，返回ACK或者NACK，从高到低，依次发送
* 参数：data -- 要写的数据
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
unsigned char iic_write_byte(unsigned char data)
{
  unsigned char i;
  sda_out();
  GPIO_ResetBits(I2C_GPIO,PIN_SCL);                             //拉低时钟线
  for(i = 0;i < 8;i++){                                         
    if(data & 0x80){                                            //判断数据最高位是否为1
      GPIO_SetBits(I2C_GPIO,PIN_SDA);  	                                                
    }
    else
      GPIO_ResetBits(I2C_GPIO,PIN_SDA); 
    delay_us(5);                                                //延时5us
    GPIO_SetBits(I2C_GPIO,PIN_SCL); 	                        //输出SDA稳定后，拉高SCL给出上升沿，从机检测到后进行数据采样
    delay_us(5);                                                //延时5us
    GPIO_ResetBits(I2C_GPIO,PIN_SCL);                           //拉低时钟线
    delay_us(5);                                                //延时5us
    data <<= 1;                                                 //数组左移一位
  } 
  delay_us(5);                                                  //延时2us
  sda_in();
  GPIO_SetBits(I2C_GPIO,PIN_SDA);                               //拉高数据线
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                               //拉高时钟线
  delay_us(5);                                                  //延时2us，等待从机应答
  if(SDA_R){			                                //SDA为高，收到NACK
    return 1;	
  }else{ 				                        //SDA为低，收到ACK
    GPIO_ResetBits(I2C_GPIO,PIN_SCL);                           //释放总线                  
    delay_us(5);                                                //延时2us，等待从机应答
    return 0;
  }
}

/*********************************************************************************************
* 名称：iic_read_byte()
* 功能：I2C写一个字节数据，返回ACK或者NACK，从高到低，依次发送
* 参数：data -- 要写的数据
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
unsigned char iic_read_byte(unsigned char ack)
{
  unsigned char i,data = 0;
  sda_in();
  GPIO_ResetBits(I2C_GPIO,PIN_SCL); 
  GPIO_SetBits(I2C_GPIO,PIN_SDA);  			        //释放总线	
  for(i = 0;i < 8;i++){
    GPIO_SetBits(I2C_GPIO,PIN_SCL);  		                //给出上升沿
    delay_us(30);	                                        //延时等待信号稳定
    data <<= 1;	
    if(SDA_R){ 		                                        //采样获取数据
      data |= 0x01;
    }else{
      data &= 0xfe;
    }
    delay_us(10);
    GPIO_ResetBits(I2C_GPIO,PIN_SCL);    		        //下降沿，从机给出下一位值
    delay_us(20);
  }
  sda_out();
  if(ack)
    GPIO_SetBits(I2C_GPIO,PIN_SDA);	                        //应答状态
  else
    GPIO_ResetBits(I2C_GPIO,PIN_SDA);  
  delay_us(10);
  GPIO_SetBits(I2C_GPIO,PIN_SCL);                        
  delay_us(50);          
  GPIO_ResetBits(I2C_GPIO,PIN_SCL); 
  delay_us(50);
  return data;
}

//////////////////BMP280//////////////////////////////////////////////////////////////////////////////////////
//void IIC_Init(void)
//{					     
//	GPIO_InitTypeDef GPIO_InitStructure;
//	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	
//	   
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;                            //推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
// 
//	IIC_SCL=1;
//	IIC_SDA=1;

//}
//产生IIC起始信号
void IIC_Start(void)
{
	sda_out();                                                                 //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;                                                                 //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;                                                                 //钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	sda_out();                                                                 //sda线输出
	IIC_SCL=0;
	IIC_SDA=0;                                                                 //STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;                                                                 //发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	sda_in();                                                                  //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;                                                                 //时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	sda_out();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	sda_out();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	sda_out(); 	    
    IIC_SCL=0;                                                                 //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);                                                           //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	sda_in();                                                                  //SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();                                                            //发送nACK
    else
        IIC_Ack();                                                             //发送ACK   
    return receive;
}

//从指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 iicDevReadByte(u8 devaddr,u8 addr)
{				  
	u8 temp=0;		  	    																 
	iic_start();  
	IIC_Send_Byte(devaddr);                                                    //发送器件写命令 	   
	IIC_Wait_Ack(); 
	IIC_Send_Byte(addr);                                                       //发送低地址
	IIC_Wait_Ack();	

	iic_start();  	 	   
	IIC_Send_Byte(devaddr|1);                                                  //发送器件读命令			   
	IIC_Wait_Ack();	 
	temp=IIC_Read_Byte(0);			   
	IIC_Stop();                                                                //产生一个停止条件	    
	return temp;
}

//连续读多个字节
//addr:起始地址
//rbuf:读数据缓存
//len:数据长度
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	IIC_Start();  
	IIC_Send_Byte(devaddr);  
	IIC_Wait_Ack();	
	IIC_Send_Byte(addr);                                                       //地址自增  
	IIC_Wait_Ack();	

	IIC_Start();  	
	IIC_Send_Byte(devaddr|1);  	
	IIC_Wait_Ack();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i]=IIC_Read_Byte(0);                                          //最后一个字节不应答
		}
		else
			rbuf[i]=IIC_Read_Byte(1);
	}
	IIC_Stop( );	
}

//从指定地址写入一个数据
//WriteAddr :写入数据的目的地址    
//DataToWrite:要写入的数据
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	IIC_Start();  
	IIC_Send_Byte(devaddr);                                                    //发送器件写命令 	 
	IIC_Wait_Ack();	   
	IIC_Send_Byte(addr);                                                       //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(data);                                                       //发送字节							   
	IIC_Wait_Ack();  		    	   
	IIC_Stop();		                                                           //产生一个停止条件 	 
}

//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	IIC_Start();  
	IIC_Send_Byte(devaddr);  	
	IIC_Wait_Ack();	
	IIC_Send_Byte(addr);  //地址自增
	IIC_Wait_Ack();	
	for(i=0; i<len; i++)
	{
		IIC_Send_Byte(wbuf[i]);  
		IIC_Wait_Ack();		
	}
	IIC_Stop( );	
}
