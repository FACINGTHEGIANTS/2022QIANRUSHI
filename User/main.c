/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/04/30
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/* 头文件 */
#include "ch32v30x.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include <rtdevice.h>

#include "debug.h"

static char xj_stack[2048];                    //线程栈
static struct rt_thread xunji_thread;          //线程控制块

rt_int32_t delay1 = 100;

/* 电机驱动引脚定义 */
#ifndef Left_moto_pwm
    #define Left_moto_pwm            35  //PWM信号端  /* PB0 */  通道A
#endif                                   //或许可以用getpin直接获取引脚编号，不用关心接的是哪个引脚，直接用就行，而不用像这样查询关心接的是哪个引脚防止引脚接错。
#ifndef Right_moto_pwm
    #define Right_moto_pwm           36  //PWM信号端  /* PB1 */  通道B
#endif

#ifndef wheel_left_front
    #define wheel_left_front         55  /* PD8 */
#endif
#ifndef wheel_left_back
    #define wheel_left_back          56  /* PD9 */
#endif

#ifndef wheel_right_front
    #define wheel_right_front        57  /* PD10 */
#endif
#ifndef wheel_right_back
    #define wheel_right_back         58  /* PD11 */
#endif

/*循迹引脚定义*/

#ifndef right_xunji
    #define right_xunji              46  /* PD3 */  //右循迹
#endif

#ifndef left_xunji
    #define left_xunji               84  /* PE15 */  //左循迹
#endif

/* 小车pwm调速 */

void Right_moto_go()       //右电机向前走
{
    rt_pin_mode(wheel_right_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_front,PIN_HIGH);
    rt_pin_mode(wheel_right_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_back,PIN_LOW);
}
void Left_moto_go()        //左边电机向前走
{
    rt_pin_mode(wheel_left_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_front,PIN_LOW);
    rt_pin_mode(wheel_left_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_back,PIN_HIGH);
}
void Right_moto_back()     //右边电机向后转
{
    rt_pin_mode(wheel_right_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_front,PIN_LOW);
    rt_pin_mode(wheel_right_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_back,PIN_HIGH);
}
void Left_moto_back()      //左边电机向后走
{
    rt_pin_mode(wheel_left_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_front,PIN_HIGH);
    rt_pin_mode(wheel_left_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_back,PIN_LOW);
}
void Right_moto_Stop()     //右边电机停转
{
    rt_pin_mode(wheel_right_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_front,PIN_LOW);
    rt_pin_mode(wheel_right_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_right_back,PIN_LOW);
}
void Left_moto_Stop()      //左边电机停转
{
    rt_pin_mode(wheel_left_front,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_front,PIN_LOW);
    rt_pin_mode(wheel_left_back,PIN_MODE_OUTPUT);
    rt_pin_write(wheel_left_back,PIN_LOW);
}
/* 变量定义 */

unsigned char pwm_val_left  =0;
unsigned char push_val_left =0;

/* 初始化 */

/* 小车行驶 */

void run(void)
{
    Left_moto_go();        //左电机往前
    Right_moto_go();       //右电机往前
    rt_thread_mdelay(delay1);
}
void  backrun(void)
{
     Left_moto_back();      //左电机往后
     Right_moto_back();     //右电机往后
}

/* 小车转向 */

void  leftrun(void)
{
     Left_moto_back();   //左电机往后
     Right_moto_go();    //右电机往前
}
void  rightrun(void)
{
     Right_moto_back();   //右电机向后
     Left_moto_go();      //左电机向前
}
void stop(void)
{
    Left_moto_Stop();
    Right_moto_Stop();
}

/* 小车pwm调速 */

/* 循迹 */

static int Car_Traction(void)                    // ???`
{
    if(left_xunji==0&&right_xunji==0)           //左右均未识别
    {
        run();
    }
    else if (left_xunji==0&&right_xunji==1)     //右黑右转
    {
        rightrun();
    }
    else if (left_xunji==1&&right_xunji==0)     //左黑左转
    {
        leftrun();
    }
    else if (left_xunji==1&&right_xunji==1)     //左右都黑停车
    {
        stop();
    }
}

/* 主函数 */

/* 中断服务函数 */

/**
  ******************************************************************
  * @file    main.c
  * @author  xy,Benue
  * @version V1.0
  * @date    2022-1-13
  * @brief   使用按键 Wake_Up 控制 LED1 亮灭。
  ******************************************************************
  * @attention
  * VeriMake 用于CH32V307例程
  ******************************************************************
  */
// 包含 CH32V307 的头文件，C 标准单元库和delay()函数
/********************************************************************
* 函 数 名       : GPIO_INIT
* 函数功能       : 初始化 GPIO
* 输    入          : 无
* 输    出          : 无
********************************************************************/
void GPIO_INIT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure={0};

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/********************************************************************
* 函 数 名       : EXTI_INT_INIT
* 函数功能       : 初始化外部中断
* 输    入          : 无
* 输    出          : 无
********************************************************************/
void EXTI15_10_INT_INIT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure={0};
   EXTI_InitTypeDef EXTI_InitStructure={0};
   NVIC_InitTypeDef NVIC_InitStructure={0};

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOE,ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOE, &GPIO_InitStructure);

   GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource0);
   EXTI_InitStructure.EXTI_Line=EXTI_Line15;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    //为高电平，用上升沿
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}
void EXTI3_INT_INIT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure={0};
   EXTI_InitTypeDef EXTI_InitStructure={0};
   NVIC_InitTypeDef NVIC_InitStructure={0};

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOD,ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource0);
   EXTI_InitStructure.EXTI_Line=EXTI_Line3;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    //为高电平，用上升沿
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void EXTI_INT_INIT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure={0};
   EXTI_InitTypeDef EXTI_InitStructure={0};
   NVIC_InitTypeDef NVIC_InitStructure={0};

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* GPIOA ----> EXTI_Line0 */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
   EXTI_InitStructure.EXTI_Line=EXTI_Line0;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //按下为高电平，用上升沿
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

/********************************************************************
* 函 数 名      :  main
* 函数功能   : 主函数
* 输    入         : 无
* 输    出         : 无
*********************************************************************/
int main(void)
{
    EXTI15_10_INT_INIT(); // 初始化外部中断
    GPIO_INIT();     // 初始化 GPIO
    //while(1);        // 死循环

    rt_thread_init(&xunji_thread,
            "xunji",
            Car_Traction,
            RT_NULL,
            &xj_stack[0],
            sizeof(xj_stack),
            10,
            10);
    rt_thread_startup(&xunji_thread);
//    while(1)
//    {
//        run();
//        if(left_xunji==0&&right_xunji==0)           //左右均未识别
//            {
//                run();
//            }
//            else if (left_xunji==0&&right_xunji==1)     //右黑右转
//            {
//                rightrun();
//            }
//            else if (left_xunji==1&&right_xunji==0)     //左黑左转
//            {
//                leftrun();
//            }
//            else if (left_xunji==1&&right_xunji==1)     //左右都黑停车
//            {
//                stop();
//            }
//    }
//    rt_thread_mdelay(4000);
    run();
    rt_thread_mdelay(1000);
    backrun();
    rt_thread_mdelay(1000);
    leftrun();
    rt_thread_mdelay(1000);
    rightrun();
    rt_thread_mdelay(1000);
    stop();
    rt_thread_mdelay(1000);

}

/********************************************************************
* 函 数 名      : EXTI0_IRQHandler
* 函数功能   : 中断服务程序的函数
* 输    入         : 无
* 输    出         : 无
*********************************************************************/

//有问题
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
volatile uint16_t LED_Status = 0; // 中断里使用的变量加 volatile 可当成全局变量
void EXTI15_10_IRQHandler(void)
{
    // 置中断标志位为零
    EXTI_ClearITPendingBit(EXTI_Line0);
    if(left_xunji==0&&right_xunji==0)           //左右均未识别
        {
            run();
        }
        else if (left_xunji==0&&right_xunji==1)     //右黑右转
        {
            rightrun();
        }
        else if (left_xunji==1&&right_xunji==0)     //左黑左转
        {
            leftrun();
        }
        else if (left_xunji==1&&right_xunji==1)     //左右都黑停车
        {
            stop();
        }
//    LED_Status = !LED_Status ;  // 将 LED 状态值取反
//    GPIO_WriteBit(GPIOE, GPIO_Pin_15, LED_Status); // 配置 PE11 (即 LED1) 状态
}
