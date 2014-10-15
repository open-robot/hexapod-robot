#ifndef __GPIO_H
#define	__GPIO_H

#include "stm32f10x.h"

void Servo1(void);
void Servo2(void);
void Servo3(void);
void Stand(void);
void Update(void);
void Forward(void);
void Draw_Back(void);
void Turn_Left(void);
void Turn_Right(void);
void Decode_One(void);
void Decode_Two(void);
void Decode_Three(void);
void GPIO_Config(void);
void Control_Action(void);
void Left_Tynanize(void);
void Right_Tynanize(void);

#endif /* __GPIO_H */
