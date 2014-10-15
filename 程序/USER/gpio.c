#include "gpio.h"
#include "usart1.h"
#include "SysTick.h"

#define MAXPWM 2500						 //������PWM��������2.5ms�궨��

#define CENTER1 1500					 //��ʼ������Ƕ�ֵ�궨��
#define CENTER2 1640
#define CENTER3 1840

#define CENTER4 1400					 //��ʱ��ÿ��1us�ۼ�һ��
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

#define FHDEGREE1 160					 //������ǰ���ǶȲ���
#define FHDEGREE2 -160
#define FLDEGREE1 200
#define FLDEGREE2 -200
#define FMDEGREE1 300
#define FMDEGREE2 -300

#define DHDEGREE1 -160					 //�����˺��˽ǶȲ���
#define DHDEGREE2 160
#define DLDEGREE1 200
#define DLDEGREE2 -200
#define DMDEGREE1 -300
#define DMDEGREE2 300

#define LHDEGREE1 -160					 //��������ת�ǶȲ���
#define LHDEGREE2 -160
#define LLDEGREE1 200
#define LLDEGREE2 -200
#define LMDEGREE1 -300
#define LMDEGREE2 -300

#define RHDEGREE1 160					 //��������ת�ǶȲ���
#define RHDEGREE2 160
#define RLDEGREE1 200
#define RLDEGREE2 -200
#define RMDEGREE1 300
#define RMDEGREE2 300

#define XHDEGREE1 500					 //���������߽ǶȲ���
#define XHDEGREE2 500
#define XLDEGREE1 200
#define XLDEGREE2 200
#define XMDEGREE1 0
#define XMDEGREE2 0

#define YHDEGREE1 500					 //���������߽ǶȲ���
#define YHDEGREE2 500
#define YLDEGREE1 200
#define YLDEGREE2 200
#define YMDEGREE1 0
#define YMDEGREE2 0

extern u8 date[3];

u8 num1,num2,num3;						 //ÿ������������ʱ�ۼ�
u8 count1,count2,count3;				 //ÿ����������8·����Ⱥ�ֵ����
u8 sflag,sflag1,sflag2,sflag3;			 //ÿ�����������ж�8·����Ƿ�ת��Ԥ�ڽǶ�
u8 flag,fflag,dflag,lflag,rflag,xflag,yflag;//ÿ�������������ƻ����˶���˳������

int sdate[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//���������ڱ����ֵ

int PWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //���������ڱ���Ԥ��PWM������ֵ
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};
int CPWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //���������ڱ��浱ǰPWM������ֵ
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};
int LPWM[24]={1500,1500,1500,1500,1500,1500,1500,1500,	  //���������ڱ����ϴ�PWM������ֵ
			 1500,1500,1500,1500,1500,1500,1500,1500,
			 1500,1500,1500,1500,1500,1500,1500,1500};

int svalue[18]={CENTER1,CENTER2,CENTER3,CENTER4,CENTER5,CENTER6,	   //���������ڱ��������վ��PWM������ֵ
                CENTER7,CENTER8,CENTER9,CENTER10,CENTER11,CENTER12,
                CENTER13,CENTER14,CENTER15,CENTER16,CENTER17,CENTER18};

int fvalue[18]={CENTER1+FHDEGREE1,CENTER2+FLDEGREE1,CENTER3+FLDEGREE1, //���������ڱ��������ǰ��PWM������ֵ
                CENTER4+FMDEGREE1,CENTER5+FLDEGREE1,CENTER6+FLDEGREE1,
                CENTER7+FHDEGREE1,CENTER8+FLDEGREE1,CENTER9+FLDEGREE1,
                CENTER10+FHDEGREE2,CENTER11+FLDEGREE2,CENTER12+FLDEGREE2,
                CENTER13+FMDEGREE2,CENTER14+FLDEGREE2,CENTER15+FLDEGREE2,
                CENTER16+FHDEGREE2,CENTER17+FLDEGREE2,CENTER18+FLDEGREE2};


int dvalue[18]={CENTER1+DHDEGREE1,CENTER2+DLDEGREE1,CENTER3+DLDEGREE1, //���������ڱ�������˺���PWM������ֵ
                CENTER4+DMDEGREE1,CENTER5+DLDEGREE1,CENTER6+DLDEGREE1,
                CENTER7+DHDEGREE1,CENTER8+DLDEGREE1,CENTER9+DLDEGREE1,
                CENTER10+DHDEGREE2,CENTER11+DLDEGREE2,CENTER12+DLDEGREE2,
                CENTER13+DMDEGREE2,CENTER14+DLDEGREE2,CENTER15+DLDEGREE2,
                CENTER16+DHDEGREE2,CENTER17+DLDEGREE2,CENTER18+DLDEGREE2};

int lvalue[18]={CENTER1+LHDEGREE1,CENTER2+LLDEGREE1,CENTER3+LLDEGREE1, //���������ڱ����������תPWM������ֵ
                CENTER4+LMDEGREE1,CENTER5+LLDEGREE1,CENTER6+LLDEGREE1,
                CENTER7+LHDEGREE1,CENTER8+LLDEGREE1,CENTER9+LLDEGREE1,
                CENTER10+LHDEGREE2,CENTER11+LLDEGREE2,CENTER12+LLDEGREE2,
                CENTER13+LMDEGREE2,CENTER14+LLDEGREE2,CENTER15+LLDEGREE2,
                CENTER16+LHDEGREE2,CENTER17+LLDEGREE2,CENTER18+LLDEGREE2};

int rvalue[18]={CENTER1+RHDEGREE1,CENTER2+RLDEGREE1,CENTER3+RLDEGREE1, //���������ڱ����������תPWM������ֵ
                CENTER4+RMDEGREE1,CENTER5+RLDEGREE1,CENTER6+RLDEGREE1,
                CENTER7+RHDEGREE1,CENTER8+RLDEGREE1,CENTER9+RLDEGREE1,
                CENTER10+RHDEGREE2,CENTER11+RLDEGREE2,CENTER12+RLDEGREE2,
                CENTER13+RMDEGREE2,CENTER14+RLDEGREE2,CENTER15+RLDEGREE2,
                CENTER16+RHDEGREE2,CENTER17+RLDEGREE2,CENTER18+RLDEGREE2};

int xvalue[18]={CENTER1-XHDEGREE1,CENTER2+XLDEGREE1,CENTER3-XLDEGREE1, //���������ڱ������������PWM������ֵ
                CENTER4+XMDEGREE1,CENTER5+XLDEGREE1,CENTER6-XLDEGREE1,
                CENTER7+XHDEGREE1,CENTER8+XLDEGREE1,CENTER9-XLDEGREE1,
                CENTER10+XHDEGREE2,CENTER11-XLDEGREE2,CENTER12-XLDEGREE2,
                CENTER13+XMDEGREE2,CENTER14-XLDEGREE2,CENTER15-XLDEGREE2,
                CENTER16-XHDEGREE2,CENTER17-XLDEGREE2,CENTER18-XLDEGREE2};

int yvalue[18]={CENTER1-YHDEGREE1,CENTER2+YLDEGREE1,CENTER3+YLDEGREE1, //���������ڱ������������PWM������ֵ
                CENTER4+YMDEGREE1,CENTER5+YLDEGREE1,CENTER6+YLDEGREE1,
                CENTER7+YHDEGREE1,CENTER8+YLDEGREE1,CENTER9+YLDEGREE1,
                CENTER10+YHDEGREE2,CENTER11-YLDEGREE2,CENTER12+YLDEGREE2,
                CENTER13+YMDEGREE2,CENTER14-YLDEGREE2,CENTER15+YLDEGREE2,
                CENTER16-YHDEGREE2,CENTER17-YLDEGREE2,CENTER18+YLDEGREE2};
 
/************************GPIO��ʼ������********************************************/
void GPIO_Config(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);//��������ʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//ѡ���������		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  //��������Ϊ�������ģʽ	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //������������Ϊ50MHZ	
	GPIO_Init(GPIOA,&GPIO_InitStructure);			//���ÿ⺯������ʼ��GPIO

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

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;				//ѡ���������		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  		//��������Ϊ�������ģʽ	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	//������������Ϊ50MHZ	
	GPIO_Init(GPIOB,&GPIO_InitStructure);				//���ÿ⺯������ʼ��GPIO
	
}
/**********************************************************************************/
/************************�����˵�һ���ȿ��ƺ���************************************/
void First_Leg(int date1,int date2,int date3)
{
	PWM[0]=date1;
	PWM[1]=date2;
	PWM[2]=date3;	
}
/**********************************************************************************/
/************************�����˵ڶ����ȿ��ƺ���************************************/
void Second_Leg(int date1,int date2,int date3)
{
	PWM[4]=date1;
	PWM[5]=date2;
	PWM[6]=date3;	
}
/**********************************************************************************/
/************************�����˵������ȿ��ƺ���************************************/
void Third_Leg(int date1,int date2,int date3)
{
	PWM[8]=date1;
	PWM[9]=date2;
	PWM[10]=date3;	
}
/**********************************************************************************/
/************************�����˵������ȿ��ƺ���************************************/
void Fourth_Leg(int date1,int date2,int date3)
{
	PWM[12]=date1;
	PWM[13]=date2;
	PWM[14]=date3;	
}
/**********************************************************************************/
/************************�����˵������ȿ��ƺ���************************************/
void Fifth_Leg(int date1,int date2,int date3)
{
	PWM[16]=date1;
	PWM[17]=date2;
	PWM[18]=date3;	
}
/**********************************************************************************/
/************************�����˵������ȿ��ƺ���************************************/
void Sixth_Leg(int date1,int date2,int date3)
{
	PWM[20]=date1;
	PWM[21]=date2;
	PWM[22]=date3;	
}
/**********************************************************************************/
/************************������ͷ��ҡ�ں���****************************************/
void Servo_Swing(int date1)
{
	PWM[23]=date1;	
}
/**********************************************************************************/
/************************������ٺ���1*********************************************/
void Decode_One(void)
{
    u8 symbol=0x00;           	//symbol1Ϊ���ת��Ԥ��λ�ñ�ǣ�CPWMΪ��ǰPWM����ֵ   
    
    if(CPWM[0]==PWM[0])			//���CPWM[0]=PWM[0],������һ�������ת��Ԥ�ڽǶȣ���ʱ��symbol1�����
    {
        symbol=symbol|0x01;
        LPWM[0]=CPWM[0];
    }
    else						//����������
    {
        symbol=symbol&0x00;
    }
    if(CPWM[1]==PWM[1])			//ͬ��
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
       
    if(symbol==0xff)			//���symbol1=0xff������8·����Ѿ�ȫ��ת��Ԥ��λ�ã���ʱ��sflag1�����
    {
        sflag1=0x01;
    }
    else
    {
        sflag1=0x00;
    }

	sflag=sflag1&sflag2&sflag3;	

	if(sflag)					//���sflag=1������24·�����ȫ��ת��Ԥ��λ��
	{
		Update();				//��ʱ��ʼ��������

		Control_Action();	    //���������ݿ��ƻ����˶���
	}	

}
/**********************************************************************************/
/************************������ٺ���2*********************************************/
void Decode_Two(void)
{
    u8 symbol=0x00;            	//����ͬ��
    
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
/************************������ٺ���3*********************************************/
void Decode_Three(void)
{
    u8 symbol=0x00;           	//����ͬ��  
    
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
/*********************����Ƕ���ֵ���㺯��1****************************************/
void Date_Calculate_One(void)
{
    sdate[0]=(PWM[0]-LPWM[0])/4;//��Ԥ�ڶ��PWM����ֵ�ֳ��Ĵ�ִ�� 
    sdate[1]=(PWM[1]-LPWM[1])/4;//�����ȿ��ö��ƽ����ת��Ҳ�û����˶�������
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
/*********************����Ƕ���ֵ���㺯��2****************************************/
void Date_Calculate_Two(void)
{
    sdate[8]=(PWM[8]-LPWM[8])/4;  //����ͬ��
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
/*********************����Ƕ���ֵ���㺯��3****************************************/
void Date_Calculate_Three(void)
{
    sdate[16]=(PWM[16]-LPWM[16])/4;//����ͬ�� 
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
/************************GPIO��ƽ��ת����1*****************************************/ 
void Flip_GPIO_One(void)
{
	switch(count1)								 //��20ms�Ķ���������ڷֳ�8�ݣ�ÿ2.5ms����һ�������ת
	{  											 //ÿ����ʱ������8·�����ת��3����ʱ������24·�����ת
		case 1: TIM2->ARR=CPWM[0];				 //����һ�����������ֵ��ֵ����ʱ��2
				GPIO_SetBits(GPIOA,GPIO_Pin_0);  //ͬʱ���߿��ƶ��1�����ŵĵ�ƽ
				break;
		
		case 2:	TIM2->ARR=MAXPWM-CPWM[0]; 		 //��2.5ms��ȥPWM����ֵ�����ݸ�ֵ��ʱ��2
				GPIO_ResetBits(GPIOA,GPIO_Pin_0);//ͬʱ���Ϳ��ƶ��1���ŵĵ�ƽ 
				break;							 //���ƶ��1��������ʣ��20ms-CPM[0]ʱ���ڽ�һֱ���ֵ͵�ƽ�����1����CPWMֵת��

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
/************************GPIO��ƽ��ת����2*****************************************/ 
void Flip_GPIO_Two(void)
{
	switch(count2)
	{  		
		case 1: TIM3->ARR=CPWM[8];				 //����ͬ��
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
/************************GPIO��ƽ��ת����3*****************************************/ 
void Flip_GPIO_Three(void)
{
	switch(count3)
	{  		
		case 1: TIM4->ARR=CPWM[16];				 //����ͬ��
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
/************************������ƺ���1*********************************************/
void Servo1(void)
{		
	num1++;									 //num���������ۼӼ��������ڶ��ת��

	count1++;

	if(!(num1%50))							 //���������ֵÿ��һ��ʱ�����һ��
	{	
		num1=0;
			
		Date_Calculate_One();				 //����Ƕȼ���

		Decode_One();						 //�������
		
	}
	 
	Flip_GPIO_One();						 //��תIO��ƽ

}
/**********************************************************************************/
/************************������ƺ���2*********************************************/
void Servo2(void)
{		
	num2++;									 //����ͬ��

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
/************************������ƺ���4*********************************************/
void Servo3(void)
{		
	num3++;									 //����ͬ��

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

	fflag=dflag=lflag=0;					  //������ж�������
	xflag=yflag=rflag=0;

	Servo_Swing(svalue[0]);

	First_Leg(svalue[0],svalue[1],svalue[2]); //�����ж����ֵΪ��ʼֵ��������վ������
	Second_Leg(svalue[3],svalue[4],svalue[5]);
	Third_Leg(svalue[6],svalue[7],svalue[8]);
	
	Fourth_Leg(svalue[9],svalue[10],svalue[11]);
	Fifth_Leg(svalue[12],svalue[13],svalue[14]);
	Sixth_Leg(svalue[15],svalue[16],svalue[17]);
		
}
/**********************************************************************************/ 
/************************������ǰ������********************************************/
void Forward(void)
{
	switch(fflag)
	{
		case 0:	First_Leg(svalue[0],fvalue[1],fvalue[2]);	//��һ������һ����̧��
				Third_Leg(svalue[6],fvalue[7],fvalue[8]);
				Fifth_Leg(svalue[12],fvalue[13],fvalue[14]);//���߲�����һ����̧��
				break;

		case 1:	First_Leg(fvalue[0],fvalue[1],fvalue[2]);	//�ڶ�������һ����ǰ��
				Third_Leg(fvalue[6],fvalue[7],fvalue[8]);
				Fifth_Leg(fvalue[12],fvalue[13],fvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//�ڰ˲����ڶ����ȹ�λվ��
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(fvalue[0],svalue[1],svalue[2]);	//����������һ�������
				Third_Leg(fvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(fvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],fvalue[4],fvalue[5]);	//���Ĳ����ڶ�����̧��
				Fourth_Leg(svalue[9],fvalue[10],fvalue[11]);
				Sixth_Leg(svalue[15],fvalue[16],fvalue[17]);
				break;

		case 4:	Second_Leg(fvalue[3],fvalue[4],fvalue[5]);	//���岽���ڶ�����ǰ��
				Fourth_Leg(fvalue[9],fvalue[10],fvalue[11]);
				Sixth_Leg(fvalue[15],fvalue[16],fvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//���岽��ͬʱ��һ���ȹ�λ
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);					
				break;

		case 5: Second_Leg(fvalue[3],svalue[4],svalue[5]);	//���������ڶ��������
				Fourth_Leg(fvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(fvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}	
}
/**********************************************************************************/ 
/************************�����˺��˺���********************************************/
void Draw_Back(void)
{
	switch(dflag)
	{
		case 0:	First_Leg(svalue[0],dvalue[1],dvalue[2]);	//��һ������һ����̧��
				Third_Leg(svalue[6],dvalue[7],dvalue[8]);
				Fifth_Leg(svalue[12],dvalue[13],dvalue[14]);//���߲�����һ����̧��
				break;

		case 1:	First_Leg(dvalue[0],dvalue[1],dvalue[2]);	//�ڶ�������һ���Ⱥ�
				Third_Leg(dvalue[6],dvalue[7],dvalue[8]);
				Fifth_Leg(dvalue[12],dvalue[13],dvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//�ڰ˲����ڶ����ȹ�λվ��
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(dvalue[0],svalue[1],svalue[2]);	//����������һ�������
				Third_Leg(dvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(dvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],dvalue[4],dvalue[5]);	//���Ĳ����ڶ�����̧��
				Fourth_Leg(svalue[9],dvalue[10],dvalue[11]);
				Sixth_Leg(svalue[15],dvalue[16],dvalue[17]);

				break;

		case 4:	Second_Leg(dvalue[3],dvalue[4],dvalue[5]);	//���岽���ڶ����Ⱥ�
				Fourth_Leg(dvalue[9],dvalue[10],dvalue[11]);
				Sixth_Leg(dvalue[15],dvalue[16],dvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//���岽��ͬʱ��һ���ȹ�λ
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(dvalue[3],svalue[4],svalue[5]);	//���������ڶ��������
				Fourth_Leg(dvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(dvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************��������ת����********************************************/
void Turn_Left(void)
{
	switch(lflag)
	{
		case 0:	First_Leg(svalue[0],lvalue[1],lvalue[2]);	//��һ������һ����̧��
				Third_Leg(svalue[6],lvalue[7],lvalue[8]);
				Fifth_Leg(svalue[12],lvalue[13],lvalue[14]);//���߲�����һ����̧��
				break;

		case 1:	First_Leg(lvalue[0],lvalue[1],lvalue[2]);	//�ڶ�������һ����ǰ��
				Third_Leg(lvalue[6],lvalue[7],lvalue[8]);
				Fifth_Leg(lvalue[12],lvalue[13],lvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//�ڰ˲����ڶ����ȹ�λվ��
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(lvalue[0],svalue[1],svalue[2]);	//����������һ�������
				Third_Leg(lvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(lvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],lvalue[4],lvalue[5]);	//���Ĳ����ڶ�����̧��
				Fourth_Leg(svalue[9],lvalue[10],lvalue[11]);
				Sixth_Leg(svalue[15],lvalue[16],lvalue[17]);

				break;

		case 4:	Second_Leg(lvalue[3],lvalue[4],lvalue[5]);	//���岽���ڶ����Ⱥ�
				Fourth_Leg(lvalue[9],lvalue[10],lvalue[11]);
				Sixth_Leg(lvalue[15],lvalue[16],lvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//���岽��ͬʱ��һ���ȹ�λ
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(lvalue[3],svalue[4],svalue[5]);	//���������ڶ��������
				Fourth_Leg(lvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(lvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************��������ת����********************************************/
void Turn_Right(void)
{
	switch(rflag)
	{
		case 0:	First_Leg(svalue[0],rvalue[1],rvalue[2]);	//��һ������һ����̧��
				Third_Leg(svalue[6],rvalue[7],rvalue[8]);
				Fifth_Leg(svalue[12],rvalue[13],rvalue[14]);//���߲�����һ����̧��
				break;

		case 1:	First_Leg(rvalue[0],rvalue[1],rvalue[2]);	//�ڶ�������һ���Ⱥ�
				Third_Leg(rvalue[6],rvalue[7],rvalue[8]);
				Fifth_Leg(rvalue[12],rvalue[13],rvalue[14]);

				Second_Leg(svalue[3],svalue[4],svalue[5]);	//�ڰ˲����ڶ����ȹ�λվ��
				Fourth_Leg(svalue[9],svalue[10],svalue[11]);
				Sixth_Leg(svalue[15],svalue[16],svalue[17]);
				break;

		case 2:	First_Leg(rvalue[0],svalue[1],svalue[2]);	//����������һ�������
				Third_Leg(rvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(rvalue[12],svalue[13],svalue[14]);
				break; 

		case 3: Second_Leg(svalue[3],rvalue[4],rvalue[5]);	//���Ĳ����ڶ�����̧��
				Fourth_Leg(svalue[9],rvalue[10],rvalue[11]);
				Sixth_Leg(svalue[15],rvalue[16],rvalue[17]);

				break;

		case 4:	Second_Leg(rvalue[3],rvalue[4],rvalue[5]);	//���岽���ڶ�����ǰ��
				Fourth_Leg(rvalue[9],rvalue[10],rvalue[11]);
				Sixth_Leg(rvalue[15],rvalue[16],rvalue[17]);

				First_Leg(svalue[0],svalue[1],svalue[2]);	//���岽��ͬʱ��һ���ȹ�λ
				Third_Leg(svalue[6],svalue[7],svalue[8]);
				Fifth_Leg(svalue[12],svalue[13],svalue[14]);
				break;

		case 5: Second_Leg(rvalue[3],svalue[4],svalue[5]);	//���������ڶ��������
				Fourth_Leg(rvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(rvalue[15],svalue[16],svalue[17]);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************���������ƺ���********************************************/
void Left_Tynanize(void)
{
	switch(xflag)											//�ظ������岽�������
	{
		case 0:	First_Leg(xvalue[0],svalue[1],svalue[2]);	//��һ�������ֲ�����̬
				Third_Leg(xvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],svalue[14]);
				
				Second_Leg(xvalue[3],svalue[4],svalue[5]);	
				Fourth_Leg(xvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],svalue[17]);
				
				break;

		case 1:	First_Leg(xvalue[0],xvalue[1],xvalue[2]);	//�ڶ�������һ����̧��	
				Third_Leg(xvalue[6],xvalue[7],xvalue[8]);
				Fifth_Leg(xvalue[12],xvalue[13],xvalue[14]);

				Second_Leg(xvalue[3],svalue[4],svalue[5]);	//���岽���ڶ����ȵŵع�λ
				Fourth_Leg(xvalue[9],svalue[10],svalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],svalue[17]);
				
				Servo_Swing(1000);
				break;

		case 2:	First_Leg(xvalue[0],svalue[1],xvalue[2]);	//����������һ�������
				Third_Leg(xvalue[6],svalue[7],xvalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],xvalue[14]); 
				
				Servo_Swing(1500);
				break; 

		case 3: Second_Leg(xvalue[3],xvalue[4],xvalue[5]);	//���������ڶ�����̧��
				Fourth_Leg(xvalue[9],xvalue[10],xvalue[11]);
				Sixth_Leg(xvalue[15],xvalue[16],xvalue[17]);
				
				First_Leg(xvalue[0],svalue[1],svalue[2]);	//����������һ���ȵŵع�λ
				Third_Leg(xvalue[6],svalue[7],svalue[8]);
				Fifth_Leg(xvalue[12],svalue[13],svalue[14]);

				Servo_Swing(2000);
				break;

		case 4:	Second_Leg(xvalue[3],svalue[4],xvalue[5]);	//���Ĳ����ڶ��������
				Fourth_Leg(xvalue[9],svalue[10],xvalue[11]);
				Sixth_Leg(xvalue[15],svalue[16],xvalue[17]);

				Servo_Swing(1500);
				break;

		default: break;
	}
}
/**********************************************************************************/
/************************���������ƺ���********************************************/
void Right_Tynanize(void)
{
	switch(yflag)
	{
		case 0:	First_Leg(yvalue[0],svalue[1],svalue[2]);	//��̬ͬ��
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
/************************���������˶�����******************************************/
void Control_Action(void)
{
	switch(date[2])
	{
		case 'A':	Forward();								//����������յ�����Ϊ"ONA"
					break;									//������ǰ��
		
		case 'B': 	Draw_Back();							//����������յ�����Ϊ"ONB"
					break;									//�����˺���
				
		case 'C': 	flag=0;									//����������յ�����Ϊ"ONC"
					Turn_Left();							//��������ת
					break;
					
		case 'D': 	Turn_Right();							//����������յ�����Ϊ"OND"
					break;									//��������ת

		case '1': 	flag=0;									//����������յ�����Ϊ"ON1"
					Left_Tynanize();						//����������
					break;				

		case '2':	flag=1;									//����������յ�����Ϊ"ON2"
					break;									//�����˽��볬��������ģʽ

		case '3': 	flag=0;								   //����������յ�����Ϊ"ON3"
					Right_Tynanize();					   //����������
					break;

		default:	Stand();							   //�������
					break;								   //�����˱���վ������״̬
	}
}
/**********************************************************************************/
/************************���Ʋ������º���******************************************/ 
void Update(void)
{
	switch(date[2])
	{
		case 'A': 	fflag++;
						
					if(fflag>5)							   //�����������5
					{
						fflag=0;						   //���㲽������
					}
					break;
					
		case 'B': 	dflag++;							   //����ͬ��
						
					if(dflag>5)
					{
						dflag=0;
					}
					break;
						   
		case 'C': 	lflag++;							   //����ͬ��
						
					if(lflag>5)
					{
						lflag=0;
					}
					break;			

		case 'D': 	rflag++;							   //����ͬ��
						
					if(rflag>5)
					{
						rflag=0;
					}
					break;
		
		case '1': 	xflag++;							   //����ͬ��
											
					if(xflag>4)
					{
						xflag=1;
					}
					break;

		case '2':	flag=1;								   //����ͬ��
					break;

		case '3': 	yflag++;							   //����ͬ��
						
					if(yflag>4)
					{
						yflag=1;
					}
					break;
						
		default:	break;								  //���������˳�
	}	
}
/**********************************************************************************/
