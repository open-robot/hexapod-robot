#include "gpio.h"
#include "usart1.h"
#include "SysTick.h"

#define MAXPWM 2500						 //舵机最大PWM控制脉宽2.5ms宏定义

#define CENTER1 1500					 //初始化舵机角度值宏定义
#define CENTER2 1640
#define CENTER3 1840

#define CENTER4 1400					 //定时器每隔1us累加一次
#define CENTER5 1680
#define CENTER6 1760

#define CENTER7 1360
#define CENTER8 1700
#define CENTER9 1900

#define CENTER10 1360
#define CENTER11 1300
#define CENTER12 1200

#define CENTER13 1560
#define CENTER14 1380
#define CENTER15 1280

#define CENTER16 1600
#define CENTER17 1340
#define CENTER18 1160

/*
#define CENTER1 1000
#define CENTER2 1140
#define CENTER3 1300

#define CENTER4 1380
#define CENTER5 1180
#define CENTER6 1240

#define CENTER7 1860
#define CENTER8 1220
#define CENTER9 1400

#define CENTER10 1860
#define CENTER11 1800
#define CENTER12 1700

#define CENTER13 1560
#define CENTER14 1880
#define CENTER15 1780

#define CENTER16 1100
#define CENTER17 1840
#define CENTER18 1660
*/

#define FHDEGREE1 160					 //机器人前进角度参数
#define FHDEGREE2 -160
#define FLDEGREE1 200
#define FLDEGREE2 -200
#define FMDEGREE1 300
#define FMDEGREE2 -300

#define DHDEGREE1 -160					 //机器人后退角度参数
#define DHDEGREE2 160
#define DLDEGREE1 200
#define DLDEGREE2 -200
#define DMDEGREE1 -300
#define DMDEGREE2 300

#define LHDEGREE1 -160					 //机器人左转角度参数
#define LHDEGREE2 -160
#define LLDEGREE1 200
#define LLDEGREE2 -200
#define LMDEGREE1 -300
#define LMDEGREE2 -300

#define RHDEGREE1 160					 //机器人右转角度参数
#define RHDEGREE2 160
#define RLDEGREE1 200
#define RLDEGREE2 -200
#define RMDEGREE1 300
#define RMDEGREE2 300

#define XHDEGREE1 500					 //机器人左走角度参数
#define XHDEGREE2 500
#define XLDEGREE1 200
#define XLDEGREE2 200
#define XMDEGREE1 0
#define XMDEGREE2 0

#define YHDEGREE1 500					 //机器人右走角度参数
#define YHDEGREE2 500
#define YLDEGREE1 200
#define YLDEGREE2 200
#define YMDEGREE1 0
#define YMDEGREE2 0

extern u8 date[3];

u8 num1,num2,num3;						 //每个变量用作延时累加
u8 count1,count2,count3;				 //每个变量用作8路舵机先后赋值控制
u8 sflag,sflag1,sflag2,sflag3;			 //每个变量用作判断8路舵机是否转动预期角度
u8 flag,fflag,dflag,lflag,rflag,xflag,yflag;//每个变量用作控制机器人动作顺序运行

int sdate[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//该数组用于保存差值

int PWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //该数组用于保存预期PWM脉冲宽度值
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};
int CPWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //该数组用于保存当前PWM脉冲宽度值
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};
int LPWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //该数组用于保存上次PWM脉冲宽度值
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};

int svalue[18]={CENTER1,CENTER2,CENTER3,CENTER4,CENTER5,CENTER6,	   //该数组用于保存机器人站立PWM脉冲宽度值
                CENTER7,CENTER8,CENTER9,CENTER10,CENTER11,CENTER12,
                CENTER13,CENTER14,CENTER15,CENTER16,CENTER17,CENTER18};

int fvalue[18]={CENTER1+FHDEGREE1,CENTER2+FLDEGREE1,CENTER3+FLDEGREE1, //该数组用于保存机器人前进PWM脉冲宽度值
                CENTER4+FMDEGREE1,CENTER5+FLDEGREE1,CENTER6+FLDEGREE1,
                CENTER7+FHDEGREE1,CENTER8+FLDEGREE1,CENTER9+FLDEGREE1,
                CENTER10+FHDEGREE2,CENTER11+FLDEGREE2,CENTER12+FLDEGREE2,
                CENTER13+FMDEGREE2,CENTER14+FLDEGREE2,CENTER15+FLDEGREE2,
                CENTER16+FHDEGREE2,CENTER17+FLDEGREE2,CENTER18+FLDEGREE2};


int dvalue[18]={CENTER1+DHDEGREE1,CENTER2+DLDEGREE1,CENTER3+DLDEGREE1, //该数组用于保存机器人后退PWM脉冲宽度值
                CENTER4+DMDEGREE1,CENTER5+DLDEGREE1,CENTER6+DLDEGREE1,
                CENTER7+DHDEGREE1,CENTER8+DLDEGREE1,CENTER9+DLDEGREE1,
                CENTER10+DHDEGREE2,CENTER11+DLDEGREE2,CENTER12+DLDEGREE2,
                CENTER13+DMDEGREE2,CENTER14+DLDEGREE2,CENTER15+DLDEGREE2,
                CENTER16+DHDEGREE2,CENTER17+DLDEGREE2,CENTER18+DLDEGREE2};

int lvalue[18]={CENTER1+LHDEGREE1,CENTER2+LLDEGREE1,CENTER3+LLDEGREE1, //该数组用于保存机器人左转PWM脉冲宽度值
                CENTER4+LMDEGREE1,CENTER5+LLDEGREE1,CENTER6+LLDEGREE1,
                CENTER7+LHDEGREE1,CENTER8+LLDEGREE1,CENTER9+LLDEGREE1,
                CENTER10+LHDEGREE2,CENTER11+LLDEGREE2,CENTER12+LLDEGREE2,
                CENTER13+LMDEGREE2,CENTER14+LLDEGREE2,CENTER15+LLDEGREE2,
                CENTER16+LHDEGREE2,CENTER17+LLDEGREE2,CENTER18+LLDEGREE2};

int rvalue[18]={CENTER1+RHDEGREE1,CENTER2+RLDEGREE1,CENTER3+RLDEGREE1, //该数组用于保存机器人右转PWM脉冲宽度值
                CENTER4+RMDEGREE1,CENTER5+RLDEGREE1,CENTER6+RLDEGREE1,
                CENTER7+RHDEGREE1,CENTER8+RLDEGREE1,CENTER9+RLDEGREE1,
                CENTER10+RHDEGREE2,CENTER11+RLDEGREE2,CENTER12+RLDEGREE2,
                CENTER13+RMDEGREE2,CENTER14+RLDEGREE2,CENTER15+RLDEGREE2,
                CENTER16+RHDEGREE2,CENTER17+RLDEGREE2,CENTER18+RLDEGREE2};

int xvalue[18]={CENTER1-XHDEGREE1,CENTER2+XLDEGREE1,CENTER3-XLDEGREE1, //该数组用于保存机器人左走PWM脉冲宽度值
                CENTER4+XMDEGREE1,CENTER5+XLDEGREE1,CENTER6-XLDEGREE1,
                CENTER7+XHDEGREE1,CENTER8+XLDEGREE1,CENTER9-XLDEGREE1,
                CENTER10+XHDEGREE2,CENTER11-XLDEGREE2,CENTER12-XLDEGREE2,
                CENTER13+XMDEGREE2,CENTER14-XLDEGREE2,CENTER15-XLDEGREE2,
                CENTER16-XHDEGREE2,CENTER17-XLDEGREE2,CENTER18-XLDEGREE2};

int yvalue[18]={CENTER1-YHDEGREE1,CENTER2+YLDEGREE1,CENTER3+YLDEGREE1, //该数组用于保存机器人右走PWM脉冲宽度值
                CENTER4+YMDEGREE1,CENTER5+YLDEGREE1,CENTER6+YLDEGREE1,
                CENTER7+YHDEGREE1,CENTER8+YLDEGREE1,CENTER9+YLDEGREE1,
                CENTER10+YHDEGREE2,CENTER11-YLDEGREE2,CENTER12+YLDEGREE2,
                CENTER13+YMDEGREE2,CENTER14-YLDEGREE2,CENTER15+YLDEGREE2,
                CENTER16-YHDEGREE2,CENTER17-YLDEGREE2,CENTER18+YLDEGREE2};
 
/************************GPIO初始化函数********************************************/
void GPIO_Config(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);//开启外设时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//选择控制引脚		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //设置引脚为推挽输出模式	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //设置引脚速率为50MHZ	
	GPIO_Init(GPIOA,&GPIO_InitStructure);			//调用库函数，初始化GPIO

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;				//选择控制引脚		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  		//设置引脚为推挽输出模式	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	//设置引脚速率为50MHZ	
	GPIO_Init(GPIOB,&GPIO_InitStructure);				//调用库函数，初始化GPIO
	
}
/**********************************************************************************/
/************************机器人第一条腿控制函数************************************/
void First_Leg(int date1,int date2,int date3)
{
	PWM[0]=date1;
	PWM[1]=date2;
	PWM[2]=date3;	
}
/**********************************************************************************/
/************************机器人第二条腿控制函数************************************/
void Second_Leg(int date1,int date2,int date3)
{
	PWM[4]=date1;
	PWM[5]=date2;
	PWM[6]=date3;	
}
/**********************************************************************************/
/************************机器人第三条腿控制函数************************************/
void Third_Leg(int date1,int date2,int date3)
{
	PWM[8]=date1;
	PWM[9]=date2;
	PWM[10]=date3;	
}
/**********************************************************************************/
/************************机器人第四条腿控制函数************************************/
void Fourth_Leg(int date1,int date2,int date3)
{
	PWM[12]=date1;
	PWM[13]=date2;
	PWM[14]=date3;	
}
/**********************************************************************************/
/************************机器人第五条腿控制函数************************************/
void Fifth_Leg(int date1,int date2,int date3)
{
	PWM[16]=date1;
	PWM[17]=date2;
	PWM[18]=date3;	
}
/**********************************************************************************/
/************************机器人第六条腿控制函数************************************/
void Sixth_Leg(int date1,int date2,int date3)
{
	PWM[20]=date1;
	PWM[21]=date2;
	PWM[22]=date3;	
}
/**********************************************************************************/
/************************机器人头部摇摆函数****************************************/
void Servo_Swing(int date1)
{
	PWM[23]=date1;	
}
/**********************************************************************************/
/************************舵机调速函数1*********************************************/
void Decode_One(void)
{
    u8 symbol=0x00;           	//symbol1为舵机转到预定位置标记，CPWM为当前PWM脉冲值   
    
    if(CPWM[0]==PWM[0])			//如果CPWM[0]=PWM[0],表明第一个舵机已转到预期角度，此时用symbol1做标记
    {
        symbol=symbol|0x01;
        LPWM[0]=CPWM[0];
    }
    else						//否则，清除标记
    {
        symbol=symbol&0x00;
    }
    if(CPWM[1]==PWM[1])			//同上
    {
        symbol=symbol|0x02;
        LPWM[1]=CPWM[1];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[2]==PWM[2])
    {
        symbol=symbol|0x04;
        LPWM[2]=CPWM[2];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[3]==PWM[3])
    {
        symbol=symbol|0x08;
        LPWM[3]=CPWM[3];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[4]==PWM[4])
    {
        symbol=symbol|0x10;
        LPWM[4]=CPWM[4];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[5]==PWM[5])
    {
        symbol=symbol|0x20;
        LPWM[5]=CPWM[5];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[6]==PWM[6])
    {
        symbol=symbol|0x40;
        LPWM[6]=CPWM[6];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[7]==PWM[7])
    {
        symbol=symbol|0x80;
        LPWM[7]=CPWM[7];
    }
    else
    {
        symbol=symbol&0x00;
    }
       
    if(symbol==0xff)			//如果symbol1=0xff，表明8路舵机已经全部转到预期位置，此时用sflag1做标记
    {
        sflag1=0x01;
    }
    else
    {
        sflag1=0x00;
    }

	sflag=sflag1&sflag2&sflag3;	

	if(sflag)					//如果sflag=1，表明24路舵机已全部转到预期位置
	{
		Update();				//此时开始更新数据

		Control_Action();	    //并根据数据控制机器人动作
	}	

}
/**********************************************************************************/
/************************舵机调速函数2*********************************************/
void Decode_Two(void)
{
    u8 symbol=0x00;            	//功能同上
    
    if(CPWM[8]==PWM[8])
    {
        symbol=symbol|0x01;
        LPWM[8]=CPWM[8];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[9]==PWM[9])
    {
        symbol=symbol|0x02;
        LPWM[9]=CPWM[9];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[10]==PWM[10])
    {
        symbol=symbol|0x04;
        LPWM[10]=CPWM[10];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[11]==PWM[11])
    {
        symbol=symbol|0x08;
        LPWM[11]=CPWM[11];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[12]==PWM[12])
    {
        symbol=symbol|0x10;
        LPWM[12]=CPWM[12];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[13]==PWM[13])
    {
        symbol=symbol|0x20;
        LPWM[13]=CPWM[13];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[14]==PWM[14])
    {
        symbol=symbol|0x40;
        LPWM[14]=CPWM[14];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[15]==PWM[15])
    {
        symbol=symbol|0x80;
        LPWM[15]=CPWM[15];
    }
    else
    {
        symbol=symbol&0x00;
    }
       
    if(symbol==0xff)
    {
        sflag2=0x01;
    }
    else
    {
        sflag2=0x00;
    }

	sflag=sflag1&sflag2&sflag3;

	if(sflag)
	{
		Update();
		
		Control_Action();	
	}	
		
}
/**********************************************************************************/
/************************舵机调速函数3*********************************************/
void Decode_Three(void)
{
    u8 symbol=0x00;           	//功能同上  
    
    if(CPWM[16]==PWM[16])
    {
        symbol=symbol|0x01;
        LPWM[16]=CPWM[16];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[17]==PWM[17])
    {
        symbol=symbol|0x02;
        LPWM[17]=CPWM[17];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[18]==PWM[18])
    {
        symbol=symbol|0x04;
        LPWM[18]=CPWM[18];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[19]==PWM[19])
    {
        symbol=symbol|0x08;
        LPWM[19]=CPWM[19];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[20]==PWM[20])
    {
        symbol=symbol|0x10;
        LPWM[20]=CPWM[20];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[21]==PWM[21])
    {
        symbol=symbol|0x20;
        LPWM[21]=CPWM[21];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[22]==PWM[22])
    {
        symbol=symbol|0x40;
        LPWM[22]=CPWM[22];
    }
    else
    {
        symbol=symbol&0x00;
    }
    if(CPWM[23]==PWM[23])
    {
        symbol=symbol|0x80;
        LPWM[23]=CPWM[23];
    }
    else
    {
        symbol=symbol&0x00;
    }		 
       
    if(symbol==0xff)
    {
        sflag3=0x01;
    }
    else
    {
        sflag3=0x00;
    }
	
	sflag=sflag1&sflag2&sflag3;
	
	if(sflag)
	{
		Update();

		Control_Action();
	}
	
}
/**********************************************************************************/
/*********************舵机角度数值计算函数1****************************************/
void Date_Calculate_One(void)
{
    sdate[0]=(PWM[0]-LPWM[0])/4;//将预期舵机PWM脉宽值分成四次执行 
    sdate[1]=(PWM[1]-LPWM[1])/4;//这样既可让舵机平滑运转，也让机器人动作流畅
    sdate[2]=(PWM[2]-LPWM[2])/4;
    sdate[3]=(PWM[3]-LPWM[3])/4;
    sdate[4]=(PWM[4]-LPWM[4])/4; 
    sdate[5]=(PWM[5]-LPWM[5])/4;
    sdate[6]=(PWM[6]-LPWM[6])/4;
    sdate[7]=(PWM[7]-LPWM[7])/4; 

    CPWM[0]=sdate[0]+CPWM[0];
    CPWM[1]=sdate[1]+CPWM[1];
    CPWM[2]=sdate[2]+CPWM[2];
    CPWM[3]=sdate[3]+CPWM[3];
    CPWM[4]=sdate[4]+CPWM[4];
    CPWM[5]=sdate[5]+CPWM[5];
    CPWM[6]=sdate[6]+CPWM[6];
    CPWM[7]=sdate[7]+CPWM[7]; 	
}
/**********************************************************************************/
/*********************舵机角度数值计算函数2****************************************/
void Date_Calculate_Two(void)
{
    sdate[8]=(PWM[8]-LPWM[8])/4;  //功能同上
    sdate[9]=(PWM[9]-LPWM[9])/4;
    sdate[10]=(PWM[10]-LPWM[10])/4;
    sdate[11]=(PWM[11]-LPWM[11])/4;
    sdate[12]=(PWM[12]-LPWM[12])/4; 
    sdate[13]=(PWM[13]-LPWM[13])/4;
    sdate[14]=(PWM[14]-LPWM[14])/4;
    sdate[15]=(PWM[15]-LPWM[15])/4; 

    CPWM[8]=sdate[8]+CPWM[8];
    CPWM[9]=sdate[9]+CPWM[9];
    CPWM[10]=sdate[10]+CPWM[10];
    CPWM[11]=sdate[11]+CPWM[11];
    CPWM[12]=sdate[12]+CPWM[12];
    CPWM[13]=sdate[13]+CPWM[13];
    CPWM[14]=sdate[14]+CPWM[14];
    CPWM[15]=sdate[15]+CPWM[15]; 	
}
/**********************************************************************************/
/*********************舵机角度数值计算函数3****************************************/
void Date_Calculate_Three(void)
{
    sdate[16]=(PWM[16]-LPWM[16])/4;//功能同上 
    sdate[17]=(PWM[17]-LPWM[17])/4;
    sdate[18]=(PWM[18]-LPWM[18])/4;
    sdate[19]=(PWM[19]-LPWM[19])/4;
    sdate[20]=(PWM[20]-LPWM[20])/4; 
    sdate[21]=(PWM[21]-LPWM[21])/4;
    sdate[22]=(PWM[22]-LPWM[22])/4;
    sdate[23]=(PWM[23]-LPWM[23])/4; 

    CPWM[16]=sdate[16]+CPWM[16];
    CPWM[17]=sdate[17]+CPWM[17];
    CPWM[18]=sdate[18]+CPWM[18];
    CPWM[19]=sdate[19]+CPWM[19];
    CPWM[20]=sdate[20]+CPWM[20];
    CPWM[21]=sdate[21]+CPWM[21];
    CPWM[22]=sdate[22]+CPWM[22];
    CPWM[23]=sdate[23]+CPWM[23]; 	
}
/**********************************************************************************/
/************************GPIO电平反转函数1*****************************************/ 
void Flip_GPIO_One(void)
{
	switch(count1)								 //将20ms的舵机控制周期分成8份，每2.5ms控制一个舵机运转
	{  											 //每个定时器控制8路舵机运转，3个定时器控制24路舵机运转
		case 1: TIM2->ARR=CPWM[0];				 //将第一个舵机脉冲宽度值赋值给定时器2
				GPIO_SetBits(GPIOA,GPIO_Pin_0);  //同时拉高控制舵机1的引脚的电平
				break;
		
		case 2:	TIM2->ARR=MAXPWM-CPWM[0]; 		 //将2.5ms减去PWM脉宽值的数据赋值定时器2
				GPIO_ResetBits(GPIOA,GPIO_Pin_0);//同时拉低控制舵机1引脚的电平 
				break;							 //控制舵机1的引脚在剩下20ms-CPM[0]时间内将一直保持低电平，舵机1按照CPWM值转动

		case 3:	TIM2->ARR=CPWM[1]; 
				GPIO_SetBits(GPIOA,GPIO_Pin_1); 
				break;
		
		case 4:	TIM2->ARR=MAXPWM-CPWM[1];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_1); 
				break;

		case 5:	TIM2->ARR=CPWM[2];  
				GPIO_SetBits(GPIOA,GPIO_Pin_2); 
				break;
		
		case 6:	TIM2->ARR=MAXPWM-CPWM[2];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_2);
				break;

		case 7:	TIM2->ARR=CPWM[3];  
				GPIO_SetBits(GPIOA,GPIO_Pin_3); 
				break;
		
		case 8:	TIM2->ARR=MAXPWM-CPWM[3];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_3);
				break;

		case 9:	TIM2->ARR=CPWM[4];  
				GPIO_SetBits(GPIOA,GPIO_Pin_4); 
				break;
		
		case 10:TIM2->ARR=MAXPWM-CPWM[4];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_4);
				break;

		case 11:TIM2->ARR=CPWM[5];  
				GPIO_SetBits(GPIOA,GPIO_Pin_5); 
				break;
		
		case 12:TIM2->ARR=MAXPWM-CPWM[5];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
				break;

		case 13:TIM2->ARR=CPWM[6];  
				GPIO_SetBits(GPIOA,GPIO_Pin_6); 
				break;
		
		case 14:TIM2->ARR=MAXPWM-CPWM[6];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
				break;

		case 15:TIM2->ARR=CPWM[7];  
				GPIO_SetBits(GPIOA,GPIO_Pin_7); 
				break;
		
		case 16:TIM2->ARR=MAXPWM-CPWM[7]; 
				GPIO_ResetBits(GPIOA,GPIO_Pin_7);
				count1=0; 
				break;
	}	
}
/**********************************************************************************/
/************************GPIO电平反转函数2*****************************************/ 
void Flip_GPIO_Two(void)
{
	switch(count2)
	{  		
		case 1: TIM3->ARR=CPWM[8];				 //功能同上
				GPIO_SetBits(GPIOA,GPIO_Pin_8);  
				break;
		
		case 2:	TIM3->ARR=MAXPWM-CPWM[8]; 
				GPIO_ResetBits(GPIOA,GPIO_Pin_8); 
				break;

		case 3:	TIM3->ARR=CPWM[9]; 
				GPIO_SetBits(GPIOA,GPIO_Pin_11); 
				break;
		
		case 4:	TIM3->ARR=MAXPWM-CPWM[9];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_11); 
				break;

		case 5:	TIM3->ARR=CPWM[10];  
				GPIO_SetBits(GPIOA,GPIO_Pin_12); 
				break;
		
		case 6:	TIM3->ARR=MAXPWM-CPWM[10];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_12);
				break;

		case 7:	TIM3->ARR=CPWM[11];  
				GPIO_SetBits(GPIOA,GPIO_Pin_15); 
				break;
		
		case 8:	TIM3->ARR=MAXPWM-CPWM[11];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_15);
				break;

		case 9:	TIM3->ARR=CPWM[12];  
				GPIO_SetBits(GPIOC,GPIO_Pin_0); 
				break;
		
		case 10:TIM3->ARR=MAXPWM-CPWM[12];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_0);
				break;

		case 11:TIM3->ARR=CPWM[13];  
				GPIO_SetBits(GPIOC,GPIO_Pin_1); 
				break;
		
		case 12:TIM3->ARR=MAXPWM-CPWM[13];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_1);
				break;

		case 13:TIM3->ARR=CPWM[14];  
				GPIO_SetBits(GPIOC,GPIO_Pin_2); 
				break;
		
		case 14:TIM3->ARR=MAXPWM-CPWM[14];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_2);
				break;

		case 15:TIM3->ARR=CPWM[15];  
				GPIO_SetBits(GPIOC,GPIO_Pin_3); 
				break;
		
		case 16:TIM3->ARR=MAXPWM-CPWM[15]; 
				GPIO_ResetBits(GPIOC,GPIO_Pin_3);
				count2=0; 
				break;
	}	
}
/**********************************************************************************/
/************************GPIO电平反转函数3*****************************************/ 
void Flip_GPIO_Three(void)
{
	switch(count3)
	{  		
		case 1: TIM4->ARR=CPWM[16];				 //功能同上
				GPIO_SetBits(GPIOC,GPIO_Pin_4);  
				break;
		
		case 2:	TIM4->ARR=MAXPWM-CPWM[16]; 
				GPIO_ResetBits(GPIOC,GPIO_Pin_4); 
				break;

		case 3:	TIM4->ARR=CPWM[17]; 
				GPIO_SetBits(GPIOC,GPIO_Pin_5); 
				break;
		
		case 4:	TIM4->ARR=MAXPWM-CPWM[17];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_5); 
				break;

		case 5:	TIM4->ARR=CPWM[18];  
				GPIO_SetBits(GPIOC,GPIO_Pin_6); 
				break;
		
		case 6:	TIM4->ARR=MAXPWM-CPWM[18];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_6);
				break;

		case 7:	TIM4->ARR=CPWM[19];  
				GPIO_SetBits(GPIOC,GPIO_Pin_7); 
				break;
		
		case 8:	TIM4->ARR=MAXPWM-CPWM[19];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_7);
				break;

		case 9:	TIM4->ARR=CPWM[20];  
				GPIO_SetBits(GPIOC,GPIO_Pin_8); 
				break;
		
		case 10:TIM4->ARR=MAXPWM-CPWM[20];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_8);
				break;

		case 11:TIM4->ARR=CPWM[21];  
				GPIO_SetBits(GPIOC,GPIO_Pin_9); 
				break;
		
		case 12:TIM4->ARR=MAXPWM-CPWM[21];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_9);
				break;

		case 13:TIM4->ARR=CPWM[22];  
				GPIO_SetBits(GPIOC,GPIO_Pin_10); 
				break;
		
		case 14:TIM4->ARR=MAXPWM-CPWM[22];  
				GPIO_ResetBits(GPIOC,GPIO_Pin_10);
				break;

		case 15:TIM4->ARR=CPWM[23];  
				GPIO_SetBits(GPIOC,GPIO_Pin_11); 
				break;
		
		case 16:TIM4->ARR=MAXPWM-CPWM[23]; 
				GPIO_ResetBits(GPIOC,GPIO_Pin_11);
				count3=0; 
				break;
	}	
}
/**********************************************************************************/   
/************************舵机控制函数1*********************************************/
void Servo1(void)
{		
	num1++;									 //num变量用来累加计数，调节舵机转速

	count1++;

	if(!(num1%50))							 //舵机脉冲宽度值每隔一段时间更新一次
	{	
		num1=0;
			
		Date_Calculate_One();				 //舵机角度计算

		Decode_One();						 //舵机调速
		
	}
	 
	Flip_GPIO_One();						 //反转IO电平

}
/**********************************************************************************/
/************************舵机控制函数2*********************************************/
void Servo2(void)
{		
	num2++;									 //功能同上

	count2++;

	if(!(num2%50))
	{	
		num2=0;
			
		Date_Calculate_Two();		

		Decode_Two();
	
	}
	
	Flip_GPIO_Two();
}
/**********************************************************************************/
/************************舵机控制函数4*********************************************/
void Servo3(void)
{		
	num3++;									 //功能同上

	count3++;

	if(!(num3%50))		 
	{	
		num3=0;
			
		Date_Calculate_Three();	

		Decode_Three();
		
	}
	
	Flip_GPIO_Three();

}
/**********************************************************************************/
void Stand(void)
{

	fflag=dflag=lflag=0;					  //清空所有动作计数
	xflag=yflag=rflag=0;

	Servo_Swing(svalue[0]);

	First_Leg(svalue[0],svalue[1],svalue[2]); //将所有舵机赋值为初始值，机器人站立待命
	Second_Leg(svalue[3],svalue[4],svalue[5]);
	Third_Leg(svalue[6],svalue[7],svalue[8]);
	
	Fourth_Leg(svalue[9],svalue[10],svalue[11]);
	Fifth_Leg(svalue[12],svalue[13],svalue[14]);
	Sixth_Leg(svalue[15],svalue[16],svalue[17]);
		
}
/**********************************************************************************/ 
/************************机器人前进函数********************************************/
void Forward(void)
{
	switch(fflag)
	{
		case 0:	First_Leg(svalue[0],fvalue[1],fvalue[2]);	//第一步：第一组腿抬脚
				Third_Leg(svalue[6],fvalue[7],fvalue[8]);
				Fifth_Leg(svalue[12],fvalue[13],fvalue[14]);//第七步：第一组腿抬脚
				break;

		case 1:	First_Leg(fvalue[0],fvalue[1],fvalue[2]);	//第二步：第一组腿前移
				Third_Leg(fvalue[6],fvalue[7],fvalue[8]);
				Fifth_Leg(fvalue[12],fvalue[13],fvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//第八步：第二组腿归位站立
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(fvalue[0],svalue[1],svalue[2]);	//第三步：第一组腿落脚
				Third_Leg(fvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(fvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],fvalue[4],fvalue[5]);	//第四步：第二组腿抬脚
				Fourth_Leg(svalue[9],fvalue[10],fvalue[11]);
				Sixth_Leg(svalue[15],fvalue[16],fvalue[17]);
				break;

		case 4:	Second_Leg(fvalue[3],fvalue[4],fvalue[5]);	//第五步：第二组腿前移
				Fourth_Leg(fvalue[9],fvalue[10],fvalue[11]);
				Sixth_Leg(fvalue[15],fvalue[16],fvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//第五步：同时第一组腿归位
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);					
				break;

		case 5: Second_Leg(fvalue[3],svalue[4],svalue[5]);	//第六步：第二组腿落脚
				Fourth_Leg(fvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(fvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}	
}
/**********************************************************************************/ 
/************************机器人后退函数********************************************/
void Draw_Back(void)
{
	switch(dflag)
	{
		case 0:	First_Leg(svalue[0],dvalue[1],dvalue[2]);	//第一步：第一组腿抬脚
				Third_Leg(svalue[6],dvalue[7],dvalue[8]);
				Fifth_Leg(svalue[12],dvalue[13],dvalue[14]);//第七步：第一组腿抬脚
				break;

		case 1:	First_Leg(dvalue[0],dvalue[1],dvalue[2]);	//第二步：第一组腿后撤
				Third_Leg(dvalue[6],dvalue[7],dvalue[8]);
				Fifth_Leg(dvalue[12],dvalue[13],dvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//第八步：第二组腿归位站立
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(dvalue[0],svalue[1],svalue[2]);	//第三步：第一组腿落脚
				Third_Leg(dvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(dvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],dvalue[4],dvalue[5]);	//第四步：第二组腿抬脚
				Fourth_Leg(svalue[9],dvalue[10],dvalue[11]);
				Sixth_Leg(svalue[15],dvalue[16],dvalue[17]);

				break;

		case 4:	Second_Leg(dvalue[3],dvalue[4],dvalue[5]);	//第五步：第二组腿后撤
				Fourth_Leg(dvalue[9],dvalue[10],dvalue[11]);
				Sixth_Leg(dvalue[15],dvalue[16],dvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//第五步：同时第一组腿归位
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(dvalue[3],svalue[4],svalue[5]);	//第六步：第二组腿落脚
				Fourth_Leg(dvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(dvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************机器人左转函数********************************************/
void Turn_Left(void)
{
	switch(lflag)
	{
		case 0:	First_Leg(svalue[0],lvalue[1],lvalue[2]);	//第一步：第一组腿抬脚
				Third_Leg(svalue[6],lvalue[7],lvalue[8]);
				Fifth_Leg(svalue[12],lvalue[13],lvalue[14]);//第七步：第一组腿抬脚
				break;

		case 1:	First_Leg(lvalue[0],lvalue[1],lvalue[2]);	//第二步：第一组腿前移
				Third_Leg(lvalue[6],lvalue[7],lvalue[8]);
				Fifth_Leg(lvalue[12],lvalue[13],lvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//第八步：第二组腿归位站立
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(lvalue[0],svalue[1],svalue[2]);	//第三步：第一组腿落脚
				Third_Leg(lvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(lvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],lvalue[4],lvalue[5]);	//第四步：第二组腿抬脚
				Fourth_Leg(svalue[9],lvalue[10],lvalue[11]);
				Sixth_Leg(svalue[15],lvalue[16],lvalue[17]);

				break;

		case 4:	Second_Leg(lvalue[3],lvalue[4],lvalue[5]);	//第五步：第二组腿后撤
				Fourth_Leg(lvalue[9],lvalue[10],lvalue[11]);
				Sixth_Leg(lvalue[15],lvalue[16],lvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//第五步：同时第一组腿归位
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(lvalue[3],svalue[4],svalue[5]);	//第六步：第二组腿落脚
				Fourth_Leg(lvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(lvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************机器人右转函数********************************************/
void Turn_Right(void)
{
	switch(rflag)
	{
		case 0:	First_Leg(svalue[0],rvalue[1],rvalue[2]);	//第一步：第一组腿抬脚
				Third_Leg(svalue[6],rvalue[7],rvalue[8]);
				Fifth_Leg(svalue[12],rvalue[13],rvalue[14]);//第七步：第一组腿抬脚
				break;

		case 1:	First_Leg(rvalue[0],rvalue[1],rvalue[2]);	//第二步：第一组腿后撤
				Third_Leg(rvalue[6],rvalue[7],rvalue[8]);
				Fifth_Leg(rvalue[12],rvalue[13],rvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//第八步：第二组腿归位站立
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(rvalue[0],svalue[1],svalue[2]);	//第三步：第一组腿落脚
				Third_Leg(rvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(rvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],rvalue[4],rvalue[5]);	//第四步：第二组腿抬脚
				Fourth_Leg(svalue[9],rvalue[10],rvalue[11]);
				Sixth_Leg(svalue[15],rvalue[16],rvalue[17]);

				break;

		case 4:	Second_Leg(rvalue[3],rvalue[4],rvalue[5]);	//第五步：第二组腿前移
				Fourth_Leg(rvalue[9],rvalue[10],rvalue[11]);
				Sixth_Leg(rvalue[15],rvalue[16],rvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//第五步：同时第一组腿归位
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(rvalue[3],svalue[4],svalue[5]);	//第六步：第二组腿落脚
				Fourth_Leg(rvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(rvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************机器人左移函数********************************************/
void Left_Tynanize(void)
{
	switch(xflag)											//重复二至五步完成行走
	{
		case 0:	First_Leg(xvalue[0],svalue[1],svalue[2]);	//第一步：保持侧走姿态
				Third_Leg(xvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],svalue[14]);
				
				Second_Leg(xvalue[3],svalue[4],svalue[5]);	
				Fourth_Leg(xvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],svalue[17]);
				
				break;

		case 1:	First_Leg(xvalue[0],xvalue[1],xvalue[2]);	//第二步：第一组腿抬腿	
				Third_Leg(xvalue[6],xvalue[7],xvalue[8]);
				Fifth_Leg(xvalue[12],xvalue[13],xvalue[14]);

				Second_Leg(xvalue[3],svalue[4],svalue[5]);	//第五步：第二组腿蹬地归位
				Fourth_Leg(xvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],svalue[17]);
				
				Servo_Swing(1000);
				break;

		case 2:	First_Leg(xvalue[0],svalue[1],xvalue[2]);	//第三步：第一组腿落地
				Third_Leg(xvalue[6],svalue[7],xvalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],xvalue[14]); 
				
				Servo_Swing(1500);
				break; 

		case 3: Second_Leg(xvalue[3],xvalue[4],xvalue[5]);	//第三步：第二组腿抬腿
				Fourth_Leg(xvalue[9],xvalue[10],xvalue[11]);
				Sixth_Leg(xvalue[15],xvalue[16],xvalue[17]);
				
				First_Leg(xvalue[0],svalue[1],svalue[2]);	//第三步：第一组腿蹬地归位
				Third_Leg(xvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],svalue[14]);

				Servo_Swing(2000);
				break;

		case 4:	Second_Leg(xvalue[3],svalue[4],xvalue[5]);	//第四步：第二组腿落地
				Fourth_Leg(xvalue[9],svalue[10],xvalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],xvalue[17]);

				Servo_Swing(1500);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************机器人右移函数********************************************/
void Right_Tynanize(void)
{
	switch(yflag)
	{
		case 0:	First_Leg(yvalue[0],svalue[1],svalue[2]);	//步态同上
				Third_Leg(yvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(yvalue[12],svalue[13],svalue[14]);
				
				Second_Leg(yvalue[3],svalue[4],svalue[5]);	
				Fourth_Leg(yvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(yvalue[15],svalue[16],svalue[17]);
				break;

		case 1:	First_Leg(yvalue[0],yvalue[1],yvalue[2]);	
				Third_Leg(yvalue[6],yvalue[7],yvalue[8]);
				Fifth_Leg(yvalue[12],yvalue[13],yvalue[14]);

				Second_Leg(yvalue[3],svalue[4],svalue[5]);	
				Fourth_Leg(yvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(yvalue[15],svalue[16],svalue[17]);

				Servo_Swing(1000);
				break;

		case 2:	First_Leg(yvalue[0],svalue[1],yvalue[2]);	
				Third_Leg(yvalue[6],svalue[7],yvalue[8]);
				Fifth_Leg(yvalue[12],svalue[13],yvalue[14]);
				
				Servo_Swing(1500);
				break; 

		case 3: Second_Leg(yvalue[3],yvalue[4],yvalue[5]);	
				Fourth_Leg(yvalue[9],yvalue[10],yvalue[11]);
				Sixth_Leg(yvalue[15],yvalue[16],yvalue[17]);
				
				First_Leg(yvalue[0],svalue[1],svalue[2]);	
				Third_Leg(yvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(yvalue[12],svalue[13],svalue[14]);

				Servo_Swing(2000);
				break;

		case 4:	Second_Leg(yvalue[3],svalue[4],yvalue[5]);	
				Fourth_Leg(yvalue[9],svalue[10],yvalue[11]);
				Sixth_Leg(yvalue[15],svalue[16],yvalue[17]);

				Servo_Swing(1500);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************蓝牙控制运动函数******************************************/
void Control_Action(void)
{
	switch(date[2])
	{
		case 'A':	Forward();								//如果蓝牙接收到数据为"ONA"
					break;									//机器人前进
		
		case 'B': 	Draw_Back();							//如果蓝牙接收到数据为"ONB"
					break;									//机器人后退
				
		case 'C': 	flag=0;									//如果蓝牙接收到数据为"ONC"
					Turn_Left();							//机器人左转
					break;
					
		case 'D': 	Turn_Right();							//如果蓝牙接收到数据为"OND"
					break;									//机器人右转

		case '1': 	flag=0;									//如果蓝牙接收到数据为"ON1"
					Left_Tynanize();						//机器人左走
					break;				

		case '2':	flag=1;									//如果蓝牙接收到数据为"ON2"
					break;									//机器人进入超声波避障模式

		case '3': 	flag=0;								   //如果蓝牙接收到数据为"ON3"
					Right_Tynanize();					   //机器人右走
					break;

		default:	Stand();							   //其它情况
					break;								   //机器人保持站立待命状态
	}
}
/**********************************************************************************/
/************************控制参数更新函数******************************************/ 
void Update(void)
{
	switch(date[2])
	{
		case 'A': 	fflag++;
						
					if(fflag>5)							   //如果步数大于5
					{
						fflag=0;						   //清零步数计数
					}
					break;
					
		case 'B': 	dflag++;							   //功能同上
						
					if(dflag>5)
					{
						dflag=0;
					}
					break;
						   
		case 'C': 	lflag++;							   //功能同上
						
					if(lflag>5)
					{
						lflag=0;
					}
					break;			

		case 'D': 	rflag++;							   //功能同上
						
					if(rflag>5)
					{
						rflag=0;
					}
					break;
		
		case '1': 	xflag++;							   //功能同上
											
					if(xflag>4)
					{
						xflag=1;
					}
					break;

		case '2':	flag=1;								   //功能同上
					break;

		case '3': 	yflag++;							   //功能同上
						
					if(yflag>4)
					{
						yflag=1;
					}
					break;
						
		default:	break;								  //其它情形退出
	}	
}
/**********************************************************************************/
