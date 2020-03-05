#include "mpu_iic.h"
#include "usart.h"

/*
IIC�ӿ���������
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

////����IIC��ʼ�ź�
//void MPU_IIC_Start(void)
//{
//	MPU_SDA_OUT();     //sda�����
//	MPU_IIC_SDA_1();	  	  
//	MPU_IIC_SCL_1();
//	MPU_IIC_Delay();
// 	MPU_IIC_SDA_0();//START:when CLK is high,DATA change form high to low 
//	MPU_IIC_Delay();
//	MPU_IIC_SCL_0();//ǯסI2C���ߣ�׼�����ͻ�������� 
//}	  
////����IICֹͣ�ź�
//void MPU_IIC_Stop(void)
//{
//	MPU_SDA_OUT();//sda�����
//	MPU_IIC_SCL_0();
//	MPU_IIC_SDA_0();//STOP:when CLK is high DATA change form low to high
// 	MPU_IIC_Delay();
//	MPU_IIC_SCL_1(); 
//	MPU_IIC_SDA_1();//����I2C���߽����ź�
//	MPU_IIC_Delay();							   	
//}
////�ȴ�Ӧ���źŵ���
////����ֵ��1������Ӧ��ʧ��
////        0������Ӧ��ɹ�
//u8 MPU_IIC_Wait_Ack(void)
//{
//	u8 ucErrTime=0;
//	MPU_SDA_IN();      //SDA����Ϊ����  
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
//	MPU_IIC_SCL_0();//ʱ�����0 	   
//	return 0;  
//} 
////����ACKӦ��
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
////������ACKӦ��		    
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
////IIC����һ���ֽ�
////���شӻ�����Ӧ��
////1����Ӧ��
////0����Ӧ��			  
//void MPU_IIC_Send_Byte(u8 txd)
//{                        
//    u8 t;   
//	MPU_SDA_OUT(); 	    
//    MPU_IIC_SCL_0();//����ʱ�ӿ�ʼ���ݴ���
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
////��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
//u8 MPU_IIC_Read_Byte(unsigned char ack)
//{
//	unsigned char i,receive=0;
//	MPU_SDA_IN();//SDA����Ϊ����
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
//        MPU_IIC_NAck();//����nACK
//    else
//        MPU_IIC_Ack(); //����ACK   
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

/* �ɴ��豸��SCLΪ�ߵ�ƽ��ʱ������SDA��ΪӦ�� 
����ֵ��1��δӦ��
        0����Ӧ��
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


/* ����һ���ֽ����ݣ���λ���� */
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


/* ��ȡһ���ֽڣ�ack=1ʱ����ȡ��ɺ���������Ӧ�� */
uint8_t MPU_IIC_Read_Byte( uint8_t ack )
{
	uint8_t t,data=0;
	MPU_SDA_IN();
	for( t=0;t<8;t++ )
	{
		MPU_IIC_SCL_0();
		delay_us(2);//�ȴ�SDA�ı仯
		MPU_IIC_SCL_1();
		
		data<<=1;//�����ڶ�ȡǰ�棬��Ϊ֮��һλ��ȡ��Ͳ�����λ
		if( MPU_IIC_SDA_READ()==1 )
			data++;
		
		delay_us(2);//�ȴ�SDA�ı仯
		
	}

	if( !ack )
		MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK 
	return data;
}












