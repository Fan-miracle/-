/*********************************************************************************************
* �ļ���BH1750.h
* ���ߣ�zonesion
* ˵����BH1750ͷ�ļ�
* �޸ģ�Chenkm 2017.01.10 �޸Ĵ����ʽ�����Ӵ���ע�ͺ��ļ�˵��
* ע�ͣ�
*********************************************************************************************/
#ifndef __BH1750_H__
#define __BH1750_H__

#include "stm32f10x.h"
/*********************************************************************************************
* �궨��
*********************************************************************************************/
//#define uint                    unsigned int 
//#define uchar                   unsigned char
#define DPOWR                   0X00                            //�ϵ�  ���ظ�
#define POWERR                   0X01                            //�ϵ�        POWER�ظ�
#define RESETT                   0X07                            //����     RESET�ظ�
#define CHMODE                  0X10                            //����H�ֱ���   ���ظ�
#define CHMODE2                 0X11                            //����H�ֱ���2  ���ظ�
#define CLMODE                  0X13                            //�����ͷֱ�    ���ظ�
#define H1MODE                  0X20                            //һ��H�ֱ���   ���ظ�
#define H1MODE2                 0X21                            //һ��H�ֱ���2  ���ظ�
#define L1MODE                  0X23                            //һ��L�ֱ���ģʽ  ���ظ�
#define	SlaveAddress            0xA6 //0x46                            //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�       ���ظ�
//ALT  ADDRESS���Žӵ�ʱ��ַΪ0xA6���ӵ�Դʱ��ַΪ0x3A

/*********************************************************************************************
* �ڲ�ԭ�ͺ���
*********************************************************************************************/
u8 bh1750_send_byte(u8 sla,u8 c);
u8 bh1750_read_nbyte(u8 sla,u8 *s,u8 no);
void bh1750_init(void);                                         //��ʼ
float bh1750_get_data(void);

#endif //__BH1750_H__
