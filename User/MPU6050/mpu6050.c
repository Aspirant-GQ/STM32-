#include "mpu6050.h"
#include "usart.h"

/**
  * @brief  ：初始化MPU
  * @param  ：None
  * @retval ：0：初始化完成
*/
uint8_t MPU_Init( void )
{
	uint8_t k=1;
	
	uint8_t res;
	
	 GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//先使能外设IO PORTA时钟 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	MPU_IIC_AD0_0();
	MPU_IIC_Init();
	/* 设置MPU6050地址为0X68,并初始化IIC总线 */
	
	
	/* MPU_PWR_MGMT1_REG：电源管理寄存器 */
	MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X80 );//复位MPU6050
//	printf("读寄存器值：%02X\n",MPU_Read_Byte( MPU_PWR_MGMT1_REG ));
	delay_ms(100);
	MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X00 );//唤醒MPU6050
	
	/* 设置陀螺仪量程：±2000dps
	 设置加速度计量程：±2g
	     设置采样频率：50Hz（低通滤波器频率100Hz）   */
	MPU_Set_Gyro_Fsr( 3 );
	MPU_Set_Accel_Fsr( 0 );
	MPU_Set_Rate( 50 );
	
	MPU_Write_Byte( MPU_INT_EN_REG, 0X00 );   //关闭所有中断
	
	MPU_Write_Byte( MPU_USER_CTRL_REG, 0X00 );//关闭IIC主模式
	MPU_Write_Byte( MPU_FIFO_EN_REG, 0X00 );  //关闭FIFO
	MPU_Write_Byte( MPU_INTBP_CFG_REG, 0X80 );//设置INT引脚低电平有效
	
	
	res = MPU_Read_Byte( MPU_DEVICE_ID_REG ); //读取MPU6050ID
  printf("ID:%X\n",res);
	/* 确认ID */
	if( res==MPU_ADDR )
	{
		MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0X01 );//以PLL X轴作为时钟参考
		MPU_Write_Byte( MPU_PWR_MGMT2_REG, 0X00 );//使能陀螺仪和加速度计
		MPU_Set_Rate( 50 );
		return 0;
	}
	else
		return 1;
	
}


////设置MPU6050陀螺仪传感器满量程范围
////fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
////返回值:0,设置成功
////    其他,设置失败 
//uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
//{
//	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
//}
////设置MPU6050加速度传感器满量程范围
////fsr:0,±2g;1,±4g;2,±8g;3,±16g
////返回值:0,设置成功
////    其他,设置失败 
//uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
//{
//	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
//}
////设置MPU6050的数字低通滤波器
////lpf:数字低通滤波频率(Hz)
////返回值:0,设置成功
////    其他,设置失败 
//uint8_t MPU_Set_LPF(uint16_t lpf)
//{
//	uint8_t data=0;
//	if(lpf>=188)data=1;
//	else if(lpf>=98)data=2;
//	else if(lpf>=42)data=3;
//	else if(lpf>=20)data=4;
//	else if(lpf>=10)data=5;
//	else data=6; 
//	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
//}
////设置MPU6050的采样率(假定Fs=1KHz)
////rate:4~1000(Hz)
////返回值:0,设置成功
////    其他,设置失败 
//uint8_t MPU_Set_Rate(uint16_t rate)
//{
//	uint8_t data;
//	if(rate>1000)rate=1000;
//	if(rate<4)rate=4;
//	data=1000/rate-1;
//	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
// 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
//}

////得到温度值
////返回值:温度值(扩大了100倍)
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
////得到陀螺仪值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
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
////得到加速度值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
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
////IIC连续写
////addr:器件地址 
////reg:寄存器地址
////len:写入长度
////buf:数据区
////返回值:0,正常
////    其他,错误代码
//uint8_t MPU_Write_Continue(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{
//	uint8_t i; 
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
//	if(MPU_IIC_Wait_Ack())	//等待应答
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答
//	for(i=0;i<len;i++)
//	{
//		MPU_IIC_Send_Byte(buf[i]);	//发送数据
//		if(MPU_IIC_Wait_Ack())		//等待ACK
//		{
//			MPU_IIC_Stop();	 
//			return 1;		 
//		}		
//	}    
//    MPU_IIC_Stop();	 
//	return 0;	
//} 
////IIC连续读
////addr:器件地址
////reg:要读取的寄存器地址
////len:要读取的长度
////buf:读取到的数据存储区
////返回值:0,正常
////    其他,错误代码
//uint8_t MPU_Read_Continue(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{ 
// 	MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
//	if(MPU_IIC_Wait_Ack())	//等待应答
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答
//    MPU_IIC_Start();
//	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
//    MPU_IIC_Wait_Ack();		//等待应答 
//	while(len)
//	{
//		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
//		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
//		len--;
//		buf++; 
//	}    
//    MPU_IIC_Stop();	//产生一个停止条件 
//	return 0;	
//}
////IIC写一个字节 
////reg:寄存器地址
////data:数据
////返回值:0,正常
////    其他,错误代码
//uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
//{ 
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
//	if(MPU_IIC_Wait_Ack())	//等待应答
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答 
//	MPU_IIC_Send_Byte(data);//发送数据
//	if(MPU_IIC_Wait_Ack())	//等待ACK
//	{
//		MPU_IIC_Stop();	 
//		return 1;		 
//	}		 
//    MPU_IIC_Stop();	 
//	return 0;
//}
////IIC读一个字节 
////reg:寄存器地址 
////返回值:读到的数据
//uint8_t MPU_Read_Byte(uint8_t reg)
//{
//	uint8_t res;
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
//	MPU_IIC_Wait_Ack();		//等待应答 
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答
//    MPU_IIC_Start();
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
//    MPU_IIC_Wait_Ack();		//等待应答 
//	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
//    MPU_IIC_Stop();			//产生一个停止条件 
//	return res;		
//}















/**
  * @brief  ：读一个寄存器的值（8位）
  * @param  ：reg：寄存器地址
  * @retval ：读到寄存器的数据
*/
uint8_t MPU_Read_Byte( uint8_t reg )
{
	uint8_t data;
	
	MPU_IIC_Start();
	/* 发送MPU6050器件地址，还有写命令（最低位是0） */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|0 );
	MPU_IIC_Wait_Ack();
	/* 写入寄存器地址 */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	MPU_IIC_Start();
	/* 对MPU6050发送读命令 */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|1 );
	MPU_IIC_Wait_Ack();
	/* 读取寄存器的值 */
	data = MPU_IIC_Read_Byte( 0 );
	MPU_IIC_Stop();
	return data;
}


/**
  * @brief  ：在指定寄存器中写入数据（8位）
  * @param  ：reg：寄存器地址
             data：要写入寄存器的数据
  * @retval ：正常返回0
*/
uint8_t MPU_Write_Byte( uint8_t reg, uint8_t data )
{
	MPU_IIC_Start();
	/* 发送MPU6050器件地址，还有写命令（最低位是0） */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* 写入寄存器地址 */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	/* 发送要写入的数据 */
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
  * @brief  ：连续读取寄存器中的数据
  * @param  ：reg：寄存器地址
              len：要读取数据的长度（以Byte为单位）
             *buf：存储读取到的数据
  * @retval ：正常返回0
*/
uint8_t MPU_Read_Continue( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf )
{
	
	MPU_IIC_Start();
	/* 发送MPU6050器件地址，还有写命令（最低位是0） */
	MPU_IIC_Send_Byte( (addr<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* 写入寄存器地址 */
	MPU_IIC_Send_Byte( reg );
	MPU_IIC_Wait_Ack();
	
	MPU_IIC_Start();
	/* 对MPU6050发送读命令 */
	MPU_IIC_Send_Byte( (MPU_ADDR<<1)|1 );
	MPU_IIC_Wait_Ack();
	while( len )
	{
		/* 如果只读一位，不发送应答 */
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
  * @brief  ：连续在寄存器中写入数据
  * @param  ：reg：寄存器地址
              len：要写入数据的长度（以Byte为单位）
             *buf：要写入寄存器的数据
  * @retval ：正常返回0
*/
uint8_t MPU_Write_Continue( uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf )
{
	uint8_t i;
	
	MPU_IIC_Start();
	/* 发送MPU6050器件地址，还有写命令（最低位是0） */
	MPU_IIC_Send_Byte( (addr<<1)|0 );
	if( MPU_IIC_Wait_Ack() )
	{
		MPU_IIC_Stop();
		return 1;
	}
	/* 写入寄存器地址 */
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
  * @brief  ：设置陀螺仪量程
  * @param  ：fsr:0：±250dps
                  1：±500dps
									2：±1000dps
									3：±2000dps
  * @retval ：0：设置成功
              1：设置失败
*/
uint8_t MPU_Set_Gyro_Fsr( uint8_t fsr )
{
	return MPU_Write_Byte( MPU_GYRO_CFG_REG, fsr<<3 );
}


/**
  * @brief  ：设置加速度计量程
  * @param  ：fsr:0：±2g
                  1：±4g
									2：±8g
									3：±16g
  * @retval ：0：设置成功
              1：设置失败
*/
uint8_t MPU_Set_Accel_Fsr( uint8_t fsr )
{
	return MPU_Write_Byte( MPU_ACCEL_CFG_REG, fsr<<3 );
}


/**
  * @brief  ：设置低通滤波器频率
  * @param  ：Hz：频率
  * @retval ：0：设置成功
              1：设置失败
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
  * @brief  ：设置采样频率
  * @param  ：Hz：4~1000Hz
  * @retval ：0：设置成功
              1：设置失败
*/
uint8_t MPU_Set_Rate( uint16_t rate )
{
	uint8_t data;
	if(rate>1000)
		rate=1000;
	if(rate<4)
		rate=4;
	data=1000/rate-1;
	/* 采样频率 */
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


/**
  * @brief  ：获取温度值
  * @param  ：None
  * @retval ：扩大了100倍的温度值(实际上为了保留俩位小数)
*/
short MPU_Get_Temperature( void )
{
	uint8_t buf[2];
	uint16_t raw;//存储原始温度值
	float temp;
	
	/* MPU_TEMP_OUTH_REG：温度值的高八位寄存器0X41
	   MPU_TEMP_OUTL_REG：温度值的低八位寄存器0X42
	   连续读出16位温度值*/
	MPU_Read_Continue( MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf );
	/* 读取原始温度值 */
	raw = ( ( uint16_t )buf[0]<<8 )|buf[1];
	/* 转换温度值 */
	temp = 36.53+( (double)raw )/340;
	
	/* 保留俩位小数，小数转换为整数时会舍去小数部分 */
	return temp*100;
}


/**
  * @brief  ：获取陀螺仪值
  * @param  ：gx、gy、gz：是陀螺仪x、y、z轴的原始读数（16位）
  * @retval ：0：获取成功
              1：获取失败
*/
uint8_t MPU_Get_Gyroscope( short *gx, short *gy, short *gz )
{
	uint8_t buf[6],res;
	
	/* MPU_GYRO_XOUTH_REG：x轴高八位寄存器
    三个轴的寄存器地址是连续的，先高位，后低位	*/
	if( (res=MPU_Read_Continue( MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf ))==0 )
	{
		*gx = ((uint16_t)buf[0]<<8)|buf[1];
		*gy = ((uint16_t)buf[2]<<8)|buf[3];
		*gz = ((uint16_t)buf[4]<<8)|buf[5];
	}
	
	return res;
}



/**
  * @brief  ：获取加速度计值
  * @param  ：ax、ay、az：是加速度计x、y、z轴的原始读数（16位）
  * @retval ：0：获取成功
              1：获取失败
*/
uint8_t MPU_Get_Accelerometer( short *ax, short *ay, short *az )
{
	uint8_t buf[6],res;
	
	/* MPU_ACCEL_XOUTH_REG：x轴高八位寄存器
    三个轴的寄存器地址是连续的，先高位，后低位	*/
	if( (res=MPU_Read_Continue( MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf ))==0 )
	{
		*ax = ((uint16_t)buf[0]<<8)|buf[1];
		*ay = ((uint16_t)buf[2]<<8)|buf[3];
		*az = ((uint16_t)buf[4]<<8)|buf[5];
	}
	
	return res;
}















