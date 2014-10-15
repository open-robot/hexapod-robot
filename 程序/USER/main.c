/**********************************************************************************
本程序的功能如下：利用蓝牙模块HC05配合安卓蓝牙串口终端控制六足机器人运行；利用超声
				  波模块HC-SR04检测机器人前方障碍物距离完成自主避障功能；利用单片机
				  ADC外设实时检测系统供电电池电压；利用声音模块完成启动功能。
**********************************************************************************/ 
#include "stm32f10x.h"	    				//包含stm32库系统头文件
#include "adc.h"							//包含ADC库头文件
#include "gpio.h"							//包含GPIO库头文件
#include "usart1.h"		  					//包含串口通信设置头文件
#include "SysTick.h"						//包含系统定时器库头文件
#include "timer_count.h"					//包含定时器设置头文件
#include "ultrasonic.h"						//包含超声波设置头文件

u8 date[3];									//该变量用来存储安卓终端发来的数据
u32 value;									

extern u32 a;		  						//a用来计数，配合系统滴答定时器可检测代码运行时间
extern u8 flag;		  						//flag用作机器人超声波避障功能触发的标记
extern float distance_value;				//该变量用来存储距离值
extern uint16_t ADC_ConvertedValue; 	//该变量用来存储经过A/D转换后的数据

float sum;		 	  						//sum变量用来求和
float ADC_ConvertedValueLocal;  			//该变量用来存储经过采集到的电压数据      

/********************电池电压值采集函数********************************************/
void Power_Data(void)
{
	u8 i;

	value++;
	
	if(!(value%100))									//每隔一段时间采集数据
	{			 	
	
		sum=0;value=0;

		for(i=0;i<10;i++)
		{
			sum=sum+(float)ADC_ConvertedValue/4096*3.3; //读取转换的AD值	
		}
	
		ADC_ConvertedValueLocal=sum/10;					//得到最终电池电压值
			
	} 
	
}
/**********************************************************************************/ 
/********************获得电池电压初始值********************************************/
void Power_Value(void)
{
	u16 i;

	sum=0;

	for(i=0;i<10;i++)									//多次采集求平均值，防止误采
	{
		sum=sum+(float)ADC_ConvertedValue/4096*3.3; 	//读取转换的AD值	
	}

	ADC_ConvertedValueLocal=sum/10;	
			
}
/**********************************************************************************/ 
/********************超声波避障函数************************************************/
void Avoid_Obstacle(void)
{
	u8 i=0;
	float sum=0;	

	for(i=0;i<10;i++)									
	{
		Ultrasonic_ON();
		
		sum=sum+distance_value;
	}

	distance_value=sum/10;								//采集十次距离求平均值，防止误采

	if(distance_value<15)								//如果检测距离小于15cm，机器人后退并右转
	{
		date[2]='B';									//机器人后退
		Delay_ms(3000);									//延时3000ms
		date[2]='D';									//机器人右转
		Delay_ms(3000);									//延时3000ms
	}
	else
	{
		date[2]='A';									//否则，机器人正常前进
	}	
}
/**********************************************************************************/
/*****************************主函数***********************************************/
int main(void)
{   
	ADC1_Init();	   								   //ADC初始化
	Timer_Init();									   //定时器初始化
	SysTick_Init();									   //系统滴答定时器初始化
	GPIO_Config();									   //GPIO初始化
	USART1_Config();								   //初始化串口1
	USART3_Config(); 								   //初始化串口3
	Ultrasonic_Config();							   //初始化超声波模块

	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))	   //语音启动机器人
	{						  
		Timer_OFF();
	}
													   
	Timer_ON();										   //开启定时器

	Stand();										   //机器人站立
	Delay_ms(2000);									   //延时2000ms

	Power_Value();									   //初始采集电池电压
			 		
	while (1)
	{	
//		SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;
//		SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
//		printf("%ld \r\n",a); 

		if(ADC_ConvertedValueLocal<1.65)//经过电阻1/4分压后的电池电压小于1.75V时
		{								//机器人停止运行，并发出报警信号，此时应当进去跟换充电电池
			Timer_OFF();			
			
			GPIO_SetBits(GPIOD,GPIO_Pin_2);			 //蜂鸣器发出滴答滴答报警信号
			Delay_ms(100); 			
			GPIO_ResetBits(GPIOD,GPIO_Pin_2);
			Delay_ms(100);
		}
		else										 //否则，机器人正常运行
		{
			Timer_ON();
													 //如果超声波避障功能被触发
			if(flag)								 //机器人进入超声波避障模式
			{										
				Avoid_Obstacle();					 //超声波避障函数
			}

		}		
		
		Power_Data();								//每隔一段时间采集电池电压值
	}
} 
/**********************************************************************************/
