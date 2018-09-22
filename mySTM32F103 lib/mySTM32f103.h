//#include "mySTM32f103.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdio.h>

#ifndef mySTMF103_h
#define mySTMF103_h
#endif

#define LED_builtin_PORT	GPIOC
#define LED_builtin_pin		GPIO_Pin_13

#define led_builtin_togle	GPIOC->ODR ^= GPIO_Pin_13
#define led_builtin_on		GPIOC->BRR  = (1<<13)
#define led_builtin_off		GPIOC->BSRR = (1<<13)

#define button0 	GPIO_Pin_4
#define button1 	GPIO_Pin_5
#define button2 	GPIO_Pin_14
#define button3 	GPIO_Pin_15


void mySTMF103_init();

void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);
void SysTick_Handler();

void TimeTickDec();
void delay_us(u32 us);
void delay_ms(u32 ms);

void ClockSetup();
void USART_SendString(USART_TypeDef* USARTx,char *stringBuff);
void USART1_Setup();
void Timer2_Setup();

void OUT_Setup(void); 
void input_Setup(void);
