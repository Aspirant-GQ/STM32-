#ifndef __MPU_IIC_H
#define __MPU_IIC_H

#include "stm32f10x.h"
#include "delay.h"

/* 宏定义引脚电平操作函数 */
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


/* 正点原子代码 */
#if 0

//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA_1();	  	  
	MPU_IIC_SCL_1();
	MPU_IIC_Delay();
 	MPU_IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_0();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL_0();
	MPU_IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_1(); 
	MPU_IIC_SDA_1();//发送I2C总线结束信号
	MPU_IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
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
	MPU_IIC_SCL_0();//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_0();//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
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
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}

#endif
