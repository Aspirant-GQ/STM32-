#include "stm32f10x.h"   
#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

int main(void)
{
	uint8_t x=0;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度
	
	NVIC_PriorityGroupConfig( 2 );
	delay_init();
	USART1_Init(115200);	
	printf("程序开始\n");
	
	if( MPU_Init()!=0 )
	{
		printf("MPU6050初始化错误！\n");
		return 0;
	}
		
	if( mpu_dmp_init() )
	{
		printf("DMP初始化错误！\n");
		return 0;
	}
	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		}
		delay_ms(100);
		printf("pitch:%02f  roll:%02f  yaw:%02f\n",pitch,roll,yaw);
	}
	
			
}


