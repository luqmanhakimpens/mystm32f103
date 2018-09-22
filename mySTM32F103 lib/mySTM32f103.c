#include "mySTM32f103.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_tim.h"
//#include "misc.h"
//#include <stdio.h>

static volatile u32 delay_counter;
const  u32 Ticktime=1000000;

void USART1_IRQHandler(void) //USART1 USB_SERIAL ISR
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)//USART1 RX ISR
    {
    	led_builtin_on;
		USART_SendData(USART1,USART_ReceiveData(USART1));
    	led_builtin_off;
    }
}
void TIM2_IRQHandler(void)//Timer2 ISR
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//timer2 overflow ISR
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
void SysTick_Handler()// systick is used to generate precision system delay timing
{
  TimeTickDec();
}
void TimeTickDec()
{
  if(delay_counter!=0)
    delay_counter--;
}
void delay_us(u32 us)
{
  delay_counter=us;
  while(delay_counter);
}
void delay_ms(u32 ms)
{
  while(ms--)
    delay_us(1000);
}
void SystickInit(u32 TickTime)
{
  while(SysTick_Config(SystemCoreClock/TickTime)!=0);
}
void USART_SendString(USART_TypeDef* USARTx,char *stringBuff) //send data string via USART1
{
    while(*stringBuff)// Loop while there are more characters to send.
    {
        USART_SendData(USARTx, *stringBuff++);//
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET); //wait for next character to send
    }
}
void ClockSetup()//  system and peripheral clock setup
{
      RCC_DeInit ();                    /* RCC system reset(for debug purpose)*/
      RCC_HSEConfig (RCC_HSE_ON);       /* Enable HSE                         */

      /* Wait till HSE is ready                                               */
      while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

      RCC_HCLKConfig   (RCC_SYSCLK_Div1);   /* HCLK   = SYSCLK                */
      RCC_PCLK2Config  (RCC_HCLK_Div1);     /* PCLK2  = HCLK                  */
      RCC_PCLK1Config  (RCC_HCLK_Div2);     /* PCLK1  = HCLK/2                */
      RCC_ADCCLKConfig (RCC_PCLK2_Div6);    /* ADCCLK = PCLK2/4               */

      /* PLLCLK = 8MHz * 9 = 72 MHz                                           */
      RCC_PLLConfig (0x00010000, RCC_PLLMul_9);

      RCC_PLLCmd (ENABLE);                  /* Enable PLL                     */

      /* Wait till PLL is ready                                               */
      while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

      /* Select PLL as system clock source                                    */
      RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

      /* Wait till PLL is used as system clock source                         */
      while (RCC_GetSYSCLKSource() != 0x08);

      /* Enable USART1 and GPIOA clock                                        */
      //RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

}
void USART1_Setup()//USB SERIAL
{
	/* @brief Init USART1
	 ******************************************************************************/

      GPIO_InitTypeDef  GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;

      /* Enable clock                                                   */
      RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

      /* Configure USART1 Rx (PA10) as input floating                         */
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure USART1 Tx (PA9) as alternate function push-pull            */
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* USART1 configured as follow:
            - BaudRate = 115200 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
            - USART Clock disabled
            - USART CPOL: Clock is active low
            - USART CPHA: Data is captured on the middle
            - USART LastBit: The clock pulse of the last data bit is not output to
                             the SCLK pin
      */
      USART_InitStructure.USART_BaudRate            = 115200;
      USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits            = USART_StopBits_1;
      USART_InitStructure.USART_Parity              = USART_Parity_No ;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
      USART_Init(USART1, &USART_InitStructure);

      USART_Cmd(USART1, ENABLE);
      USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	  
			NVIC_InitTypeDef NVIC_InitStructure;

		/* Enable the USARTx Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
void Timer2_Setup()
{
	/*	clock at 24 MHz the Prescaler is computed as following:
	     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
	    and Connectivity line devices and to 24 MHz for Low-Density Value line and
	    Medium-Density Value line devices

	    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
	                                                  = 24 MHz / 666 = 36 KHz

	    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100

	    ----------------------------------------------------------------------- */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	    /* Compute the prescaler value */
	uint32_t prescaled_val=2000000; //2MHz
	uint16_t TIM3_Tick=1;		   	//20KHz
	uint16_t PrescalerVal = (uint16_t) (SystemCoreClock / prescaled_val) - 1;

	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_BaseInitStructure.TIM_Prescaler=36000;
	TIM_BaseInitStructure.TIM_Period=(2000)-1;
	TIM_BaseInitStructure.TIM_ClockDivision=0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void OUT_Setup(void) //led on GPIOB 6,7,8,9
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); //LED ON PC13

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //LED ON PC13

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|
									GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_Write(GPIOA,0xffff);//sets all pin to output high logic(3,3V)
	//GPIO_Write(GPIOB,0xffff);//sets all pin to output high logic(3,3V)
	//GPIO_Write(GPIOC,0xffff);//sets all pin to output high logic(3,3V)
}
void input_Setup(void)//push buttons on GPIOA 14,15 and GPIOB 4,5
{
      GPIO_InitTypeDef GPIO_InitStruct;

	  //Configure GPIOA
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = button3|button2;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //*Configure GPIOB
	  GPIO_InitStruct.GPIO_Pin = button0|button1;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void mySTMF103_init()
{
	ClockSetup();
	SystickInit(1000000);// systick update every 1us, 1MHz frequency
	USART1_Setup();
	//USART3_Setup();USART3_NVIC_Config();
	Timer2_Setup();TIM_Cmd(TIM2, DISABLE);
	OUT_Setup();
}	
