#include "stm32f10x.h"   
#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

int main(void)
{
	uint8_t x=0;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�
	
	NVIC_PriorityGroupConfig( 2 );
	delay_init();
	USART1_Init(115200);	
	printf("����ʼ\n");
	
	if( MPU_Init()!=0 )
	{
		printf("MPU6050��ʼ������\n");
		return 0;
	}
		
	if( mpu_dmp_init() )
	{
		printf("DMP��ʼ������\n");
		return 0;
	}
	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		}
		delay_ms(100);
		printf("pitch:%02f  roll:%02f  yaw:%02f\n",pitch,roll,yaw);
	}
	
			
}


