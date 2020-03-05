#include "mpu_iic.h"
#include "usart.h"

/*
IIC接口引脚配置
SDA:PB11
SCL:PB10
AD0:PB2
*/
void MPU_IIC_Init( void )
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init( GPIOB, &GPIO_InitStruct );
	
	MPU_IIC_SDA_1();
	MPU_IIC_SCL_1();

}

////产生IIC起始信号
//void MPU_IIC_Start(void)
//{
//	MPU_SDA_OUT();     //sda线输出
//	MPU_IIC_SDA_1();	  	  
//	MPU_IIC_SCL_1();
//	MPU_IIC_Delay();
// 	MPU_IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_0();//钳住I2C总线，准备发送或接收数据 
//}	  
////产生IIC停止信号
//void MPU_IIC_Stop(void)
//{
//	MPU_SDA_OUT();//sda线输出
//	MPU_IIC_SCL_0();
//	MPU_IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
// 	MPU_IIC_Delay();
//	MPU_IIC_SCL_1(); 
//	MPU_IIC_SDA_1();//发送I2C总线结束信号
//	MPU_IIC_Delay();							   	
//}
////等待应答信号到来
////返回值：1，接收应答失败
////        0，接收应答成功
//u8 MPU_IIC_Wait_Ack(void)
//{
//	u8 ucErrTime=0;
//	MPU_SDA_IN();      //SDA设置为输入  
//	MPU_IIC_SDA_1();MPU_IIC_Delay();	   
//	MPU_IIC_SCL_1();MPU_IIC_Delay();	 
//	while(MPU_IIC_SDA_READ())
//	{
//		ucErrTime++;
//		if(ucErrTime>250)
//		{
//			MPU_IIC_Stop();
//			return 1;
//		}
//	}
//	MPU_IIC_SCL_0();//时钟输出0 	   
//	return 0;  
//} 
////产生ACK应答
//void MPU_IIC_Ack(void)
//{
//	MPU_IIC_SCL_0();
//	MPU_SDA_OUT();
//	MPU_IIC_SDA_0();
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_1();
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_0();
//}
////不产生ACK应答		    
//void MPU_IIC_NAck(void)
//{
//	MPU_IIC_SCL_0();
//	MPU_SDA_OUT();
//	MPU_IIC_SDA_1();
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_1();
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_0();
//}					 				     
////IIC发送一个字节
////返回从机有无应答
////1，有应答
////0，无应答			  
//void MPU_IIC_Send_Byte(u8 txd)
//{                        
//    u8 t;   
//	MPU_SDA_OUT(); 	    
//    MPU_IIC_SCL_0();//拉低时钟开始数据传输
//    for(t=0;t<8;t++)
//    {   
//			  if( ((txd&0x80)>>7)==1 )
//					MPU_IIC_SDA_1();
//				else
//					MPU_IIC_SDA_0();
//        txd<<=1; 	  
//		    MPU_IIC_SCL_1();
//		    MPU_IIC_Delay(); 
//		    MPU_IIC_SCL_0();	
//		    MPU_IIC_Delay();
//    }	 
//} 	    
////读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
//u8 MPU_IIC_Read_Byte(unsigned char ack)
//{
//	unsigned char i,receive=0;
//	MPU_SDA_IN();//SDA设置为输入
//    for(i=0;i<8;i++ )
//	{
//        MPU_IIC_SCL_0(); 
//        MPU_IIC_Delay();
//		MPU_IIC_SCL_1();
//        receive<<=1;
//        if(MPU_IIC_SDA_READ())receive++;   
//		MPU_IIC_Delay(); 
//    }					 
//    if (!ack)
//        MPU_IIC_NAck();//发送nACK
//    else
//        MPU_IIC_Ack(); //发送ACK   
//    return receive;
//}



/********************************************************************************************************/
void MPU_IIC_Start( void )
{
	MPU_SDA_OUT();
	
	MPU_IIC_SDA_1();
	MPU_IIC_SCL_1();
	delay_us(2);
	MPU_IIC_SDA_0();
	delay_us(2);
	MPU_IIC_SCL_0();
}

void MPU_IIC_Stop( void )
{
	MPU_SDA_OUT();
	
	MPU_IIC_SDA_0();
	MPU_IIC_SCL_1();
	delay_us(2);
	MPU_IIC_SDA_1();
	MPU_IIC_SCL_1();
	delay_us(2);
	
}

/* 由从设备在SCL为高电平的时候拉低SDA作为应答 
返回值：1：未应答
        0：已应答
*/
uint8_t MPU_IIC_Wait_Ack( void )
{
	uint8_t count;
	MPU_SDA_IN();
	
	MPU_IIC_SCL_1();
	delay_us(2);
	MPU_IIC_SDA_1();
	delay_us(2);
	
	while( MPU_IIC_SDA_READ()==1 )
	{
		count++;
		if( count>250 )
		{
			MPU_IIC_Stop();
			return 1;
		}
	}	
	MPU_IIC_SCL_0();
	return 0;
}


void MPU_IIC_Ack( void )
{
	
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_0();
	delay_us(2);
	MPU_IIC_SCL_1();
	delay_us(2);
	MPU_IIC_SCL_0();
	
}


void MPU_IIC_NAck( void )
{
	
	
	MPU_IIC_SCL_0();
	MPU_SDA_OUT();
	MPU_IIC_SDA_1();
	delay_us(2);
	MPU_IIC_SCL_1();
	delay_us(2);
	MPU_IIC_SCL_0();
}


/* 发送一个字节数据，高位先行 */
void MPU_IIC_Send_Byte( uint8_t data )
{
	uint8_t t;
	MPU_SDA_OUT();
	
	MPU_IIC_SCL_0();
	for( t=0;t<8;t++ )
	{
		if( ((data&0x80)>>7)==1 )
			MPU_IIC_SDA_1();
		else
			MPU_IIC_SDA_0();
		data<<=1;
		MPU_IIC_SCL_1();
		delay_us(2);
		MPU_IIC_SCL_0();
		delay_us(2);
	}
}


/* 读取一个字节，ack=1时，读取完成后主机发送应答 */
uint8_t MPU_IIC_Read_Byte( uint8_t ack )
{
	uint8_t t,data=0;
	MPU_SDA_IN();
	for( t=0;t<8;t++ )
	{
		MPU_IIC_SCL_0();
		delay_us(2);//等待SDA的变化
		MPU_IIC_SCL_1();
		
		data<<=1;//必须在读取前面，因为之后一位读取后就不再移位
		if( MPU_IIC_SDA_READ()==1 )
			data++;
		
		delay_us(2);//等待SDA的变化
		
	}

	if( !ack )
		MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK 
	return data;
}












