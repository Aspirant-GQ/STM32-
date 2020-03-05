#include "mpu6050.h"
#include "usart.h"

/**
  * @brief  ����ʼ��MPU
  * @param  ��None
  * @retval ��0����ʼ�����
*/
uint8_t MPU_Init( void )
{
	uint8_t k=1;
	
	uint8_t res;
	
	 GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//ʹ��AFIOʱ�� 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//��ʹ������IO PORTAʱ�� 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	MPU_IIC_AD0_0();
	MPU_IIC_Init();
	/* ����MPU6050��ַΪ0X68,����ʼ��IIC���� */
	
	
	/* MPU_PWR_MGMT1_REG����Դ����Ĵ��� */
	MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X80 );//��λMPU6050
//	printf("���Ĵ���ֵ��%02X\n",MPU_Read_Byte( MPU_PWR_MGMT1_REG ));
	delay_ms(100);
	MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X00 );//����MPU6050
	
	/* �������������̣���2000dps
	 ���ü��ٶȼ����̣���2g
	     ���ò���Ƶ�ʣ�50Hz����ͨ�˲���Ƶ��100Hz��   */
	MPU_Set_Gyro_Fsr( 3 );
	MPU_Set_Accel_Fsr( 0 );
	MPU_Set_Rate( 50 );
	
	MPU_Write_Byte( MPU_INT_EN_REG, 0X00 );   //�ر������ж�
	
	MPU_Write_Byte( MPU_USER_CTRL_REG, 0X00 );//�ر�IIC��ģʽ
	MPU_Write_Byte( MPU_FIFO_EN_REG, 0X00 );  //�ر�FIFO
	MPU_Write_Byte( MPU_INTBP_CFG_REG, 0X80 );//����INT���ŵ͵�ƽ��Ч
	
	
	res = MPU_Read_Byte( MPU_DEVICE_ID_REG ); //��ȡMPU6050ID
  printf("ID:%X\n",res);
	/* ȷ��ID */
	if( res==MPU_ADDR )
	{
		MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X01 );//��PLL X����Ϊʱ�Ӳο�
		MPU_Write_Byte( MPU_PWR_MGMT2_REG, 0X00 );//ʹ�������Ǻͼ��ٶȼ�
		MPU_Set_Rate( 50 );
		return 0;
	}
	else
		return 1;
	
}


////����MPU6050�����Ǵ����������̷�Χ
////fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
//{
//	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
//}
////����MPU6050���ٶȴ����������̷�Χ
////fsr:0,��2g;1,��4g;2,��8g;3,��16g
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
//{
//	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
//}
////����MPU6050�����ֵ�ͨ�˲���
////lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//uint8_t MPU_Set_LPF(uint16_t lpf)
//{
//	uint8_t data=0;
//	if(lpf>=188)data=1;
//	else if(lpf>=98)data=2;
//	else if(lpf>=42)data=3;
//	else if(lpf>=20)data=4;
//	else if(lpf>=10)data=5;
//	else data=6; 
//	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
//}
////����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
////rate:4~1000(Hz)
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//uint8_t MPU_Set_Rate(uint16_t rate)
//{
//	uint8_t data;
//	if(rate>1000)rate=1000;
//	if(rate<4)rate=4;
//	data=1000/rate-1;
//	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
// 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
//}

////�õ��¶�ֵ
////����ֵ:�¶�ֵ(������100��)
//short MPU_Get_Temperature(void)
//{
//    uint8_t buf[2]; 
//    short raw;
//	float temp;
//	MPU_Read_Continue(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
//    raw=((u16)buf[0]<<8)|buf[1];  
//    temp=36.53+((double)raw)/340;  
//    return temp*100;;
//}
////�õ�������ֵ(ԭʼֵ)
////gx,gy,gz:������x,y,z���ԭʼ����(������)
////����ֵ:0,�ɹ�
////    ����,�������
//uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
//{
//    uint8_t buf[6],res;  
//	res=MPU_Read_Continue(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
//	if(res==0)
//	{
//		*gx=((u16)buf[0]<<8)|buf[1];  
//		*gy=((u16)buf[2]<<8)|buf[3];  
//		*gz=((u16)buf[4]<<8)|buf[5];
//	} 	
//    return res;;
//}
////�õ����ٶ�ֵ(ԭʼֵ)
////gx,gy,gz:������x,y,z���ԭʼ����(������)
////����ֵ:0,�ɹ�
////    ����,�������
//uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
//{
//    uint8_t buf[6],res;  
//	res=MPU_Read_Continue(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
//	if(res==0)
//	{
//		*ax=((u16)buf[0]<<8)|buf[1];  
//		*ay=((u16)buf[2]<<8)|buf[3];  
//		*az=((u16)buf[4]<<8)|buf[5];
//	} 	
//    return res;;
//}
////IIC����д
////addr:������ַ 
////reg:�Ĵ�����ַ
////len:д�볤��
////buf:������
////����ֵ:0,����
////    ����,�������
//uint8_t MPU_Write_Continue(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{
//	uint8_t i; 
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
//	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
//	for(i=0;i<len;i++)
//	{
//		MPU_IIC_Send_Byte(buf[i]);	//��������
//		if(MPU_IIC_Wait_Ack())		//�ȴ�ACK
//		{
//			MPU_IIC_Stop();	 
//			return 1;		 
//		}		
//	}    
//    MPU_IIC_Stop();	 
//	return 0;	
//} 
////IIC������
////addr:������ַ
////reg:Ҫ��ȡ�ļĴ�����ַ
////len:Ҫ��ȡ�ĳ���
////buf:��ȡ�������ݴ洢��
////����ֵ:0,����
////    ����,�������
//uint8_t MPU_Read_Continue(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{ 
// 	MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
//	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
//    MPU_IIC_Start();
//	MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
//	while(len)
//	{
//		if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK 
//		else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK  
//		len--;
//		buf++; 
//	}    
//    MPU_IIC_Stop();	//����һ��ֹͣ���� 
//	return 0;	
//}
////IICдһ���ֽ� 
////reg:�Ĵ�����ַ
////data:����
////����ֵ:0,����
////    ����,�������
//uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
//{ 
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
//	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
//	MPU_IIC_Send_Byte(data);//��������
//	if(MPU_IIC_Wait_Ack())	//�ȴ�ACK
//	{
//		MPU_IIC_Stop();	 
//		return 1;		 
//	}		 
//    MPU_IIC_Stop();	 
//	return 0;
//}
////IIC��һ���ֽ� 
////reg:�Ĵ�����ַ 
////����ֵ:����������
//uint8_t MPU_Read_Byte(uint8_t reg)
//{
//	uint8_t res;
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
//	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
//    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
//    MPU_IIC_Start();
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
//    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
//	res=MPU_IIC_Read_Byte(0);//��ȡ����,����nACK 
//    MPU_IIC_Stop();			//����һ��ֹͣ���� 
//	return res;		
//}















/**
  * @brief  ����һ���Ĵ�����ֵ��8λ��
  * @param  ��reg���Ĵ�����ַ
  * @retval �������Ĵ���������
*/
uint8_t MPU_Read_Byte( uint8_t reg )
{
	uint8_t data;
	
	MPU_IIC_Start();
	/* ����MPU6050������ַ������д������λ��0�� */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|0 );
	MPU_IIC_Wait_Ack();
	/* д��Ĵ�����ַ */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	MPU_IIC_Start();
	/* ��MPU6050���Ͷ����� */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|1 );
	MPU_IIC_Wait_Ack();
	/* ��ȡ�Ĵ�����ֵ */
	data = MPU_IIC_Read_Byte( 0 );
	MPU_IIC_Stop();
	return data;
}


/**
  * @brief  ����ָ���Ĵ�����д�����ݣ�8λ��
  * @param  ��reg���Ĵ�����ַ
             data��Ҫд��Ĵ���������
  * @retval ����������0
*/
uint8_t MPU_Write_Byte( uint8_t reg, uint8_t data )
{
	MPU_IIC_Start();
	/* ����MPU6050������ַ������д������λ��0�� */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* д��Ĵ�����ַ */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	/* ����Ҫд������� */
	MPU_IIC_Send_Byte( data );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}



/**
  * @brief  ��������ȡ�Ĵ����е�����
  * @param  ��reg���Ĵ�����ַ
              len��Ҫ��ȡ���ݵĳ��ȣ���ByteΪ��λ��
             *buf���洢��ȡ��������
  * @retval ����������0
*/
uint8_t MPU_Read_Continue( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf )
{
	
	MPU_IIC_Start();
	/* ����MPU6050������ַ������д������λ��0�� */
	MPU_IIC_Send_Byte( (addr<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* д��Ĵ�����ַ */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	MPU_IIC_Start();
	/* ��MPU6050���Ͷ����� */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|1 );
	MPU_IIC_Wait_Ack();
	while( len )
	{
		/* ���ֻ��һλ��������Ӧ�� */
		if( len==1 )
			*buf = MPU_IIC_Read_Byte( 0 );
		else
			*buf = MPU_IIC_Read_Byte( 1 );
		len--;
		buf++;
	}
	MPU_IIC_Stop();
	return 0;
}



/**
  * @brief  �������ڼĴ�����д������
  * @param  ��reg���Ĵ�����ַ
              len��Ҫд�����ݵĳ��ȣ���ByteΪ��λ��
             *buf��Ҫд��Ĵ���������
  * @retval ����������0
*/
uint8_t MPU_Write_Continue( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf )
{
	uint8_t i;
	
	MPU_IIC_Start();
	/* ����MPU6050������ַ������д������λ��0�� */
	MPU_IIC_Send_Byte( (addr<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* д��Ĵ�����ַ */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	i=0;
	while( len )
	{
		MPU_IIC_Send_Byte( buf[i] );
		i++;
		if( MPU_IIC_Wait_Ack() )
		{
			MPU_IIC_Stop();
			return 1;
		}
		len--;
	}
	
	MPU_IIC_Stop();
		return 0;
}




/**
  * @brief  ����������������
  * @param  ��fsr:0����250dps
                  1����500dps
									2����1000dps
									3����2000dps
  * @retval ��0�����óɹ�
              1������ʧ��
*/
uint8_t MPU_Set_Gyro_Fsr( uint8_t fsr )
{
	return MPU_Write_Byte( MPU_GYRO_CFG_REG, fsr<<3 );
}


/**
  * @brief  �����ü��ٶȼ�����
  * @param  ��fsr:0����2g
                  1����4g
									2����8g
									3����16g
  * @retval ��0�����óɹ�
              1������ʧ��
*/
uint8_t MPU_Set_Accel_Fsr( uint8_t fsr )
{
	return MPU_Write_Byte( MPU_ACCEL_CFG_REG, fsr<<3 );
}


/**
  * @brief  �����õ�ͨ�˲���Ƶ��
  * @param  ��Hz��Ƶ��
  * @retval ��0�����óɹ�
              1������ʧ��
*/
uint8_t MPU_Set_LPF( uint16_t lpf )
{
	uint8_t data=0;
	
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);
}


/**
  * @brief  �����ò���Ƶ��
  * @param  ��Hz��4~1000Hz
  * @retval ��0�����óɹ�
              1������ʧ��
*/
uint8_t MPU_Set_Rate( uint16_t rate )
{
	uint8_t data;
	if(rate>1000)
		rate=1000;
	if(rate<4)
		rate=4;
	data=1000/rate-1;
	/* ����Ƶ�� */
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}


/**
  * @brief  ����ȡ�¶�ֵ
  * @param  ��None
  * @retval ��������100�����¶�ֵ(ʵ����Ϊ�˱�����λС��)
*/
short MPU_Get_Temperature( void )
{
	uint8_t buf[2];
	uint16_t raw;//�洢ԭʼ�¶�ֵ
	float temp;
	
	/* MPU_TEMP_OUTH_REG���¶�ֵ�ĸ߰�λ�Ĵ���0X41
	   MPU_TEMP_OUTL_REG���¶�ֵ�ĵͰ�λ�Ĵ���0X42
	   ��������16λ�¶�ֵ*/
	MPU_Read_Continue( MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf );
	/* ��ȡԭʼ�¶�ֵ */
	raw = ( ( uint16_t )buf[0]<<8 )|buf[1];
	/* ת���¶�ֵ */
	temp = 36.53+( (double)raw )/340;
	
	/* ������λС����С��ת��Ϊ����ʱ����ȥС������ */
	return temp*100;
}


/**
  * @brief  ����ȡ������ֵ
  * @param  ��gx��gy��gz����������x��y��z���ԭʼ������16λ��
  * @retval ��0����ȡ�ɹ�
              1����ȡʧ��
*/
uint8_t MPU_Get_Gyroscope( short *gx, short *gy, short *gz )
{
	uint8_t buf[6],res;
	
	/* MPU_GYRO_XOUTH_REG��x��߰�λ�Ĵ���
    ������ļĴ�����ַ�������ģ��ȸ�λ�����λ	*/
	if( (res=MPU_Read_Continue( MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf ))==0 )
	{
		*gx = ((uint16_t)buf[0]<<8)|buf[1];
		*gy = ((uint16_t)buf[2]<<8)|buf[3];
		*gz = ((uint16_t)buf[4]<<8)|buf[5];
	}
	
	return res;
}



/**
  * @brief  ����ȡ���ٶȼ�ֵ
  * @param  ��ax��ay��az���Ǽ��ٶȼ�x��y��z���ԭʼ������16λ��
  * @retval ��0����ȡ�ɹ�
              1����ȡʧ��
*/
uint8_t MPU_Get_Accelerometer( short *ax, short *ay, short *az )
{
	uint8_t buf[6],res;
	
	/* MPU_ACCEL_XOUTH_REG��x��߰�λ�Ĵ���
    ������ļĴ�����ַ�������ģ��ȸ�λ�����λ	*/
	if( (res=MPU_Read_Continue( MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf ))==0 )
	{
		*ax = ((uint16_t)buf[0]<<8)|buf[1];
		*ay = ((uint16_t)buf[2]<<8)|buf[3];
		*az = ((uint16_t)buf[4]<<8)|buf[5];
	}
	
	return res;
}















