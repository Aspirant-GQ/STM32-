#ifndef __MPU_IIC_H
#define __MPU_IIC_H

#include "stm32f10x.h"
#include "delay.h"

/* �궨�����ŵ�ƽ�������� */
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}  


#define MPU_IIC_SDA_1() GPIO_SetBits( GPIOB, GPIO_Pin_11 )
#define MPU_IIC_SDA_0() GPIO_ResetBits( GPIOB, GPIO_Pin_11 )

#define MPU_IIC_SCL_1() GPIO_SetBits( GPIOB, GPIO_Pin_10 )
#define MPU_IIC_SCL_0() GPIO_ResetBits( GPIOB, GPIO_Pin_10 )

#define MPU_IIC_AD0_1() GPIO_SetBits( GPIOA, GPIO_Pin_15 )
#define MPU_IIC_AD0_0() GPIO_ResetBits( GPIOA, GPIO_Pin_15 )

#define MPU_IIC_SDA_READ() GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_11 )

#define MPU_IIC_Delay() delay_us(2)


void MPU_IIC_Init( void );
void MPU_IIC_Start( void );
void MPU_IIC_Stop( void );
uint8_t MPU_IIC_Wait_Ack( void );
void MPU_IIC_Ack( void );
void MPU_IIC_NAck( void );
void MPU_IIC_Send_Byte( uint8_t data );
uint8_t MPU_IIC_Read_Byte( uint8_t ack );









#endif


/* ����ԭ�Ӵ��� */
#if 0

//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA_1();	  	  
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
 	MPU_IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL_0();
	MPU_IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_1(); 
	MPU_IIC_SDA_1();//����I2C���߽����ź�
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA_1();MPU_IIC_Delay();	   
	MPU_IIC_SCL_1();MPU_IIC_Delay();	 
	while(MPU_IIC_SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_0();//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_0();
	MPU_IIC_Delay();
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_0();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {   
			  if( ((txd&0x80)>>7)==1 )
					MPU_IIC_SDA_1();
				else
					MPU_IIC_SDA_0();
        txd<<=1; 	  
		    MPU_IIC_SCL_1();
		    MPU_IIC_Delay(); 
		    MPU_IIC_SCL_0();	
		    MPU_IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL_0(); 
        MPU_IIC_Delay();
		MPU_IIC_SCL_1();
        receive<<=1;
        if(MPU_IIC_SDA_READ())receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}

#endif
