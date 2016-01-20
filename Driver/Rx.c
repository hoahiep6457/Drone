#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Rx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
Rx_t   Rx_Throttle;
Rx_t   Rx_Pitch;
Rx_t   Rx_Roll;
Rx_t   Rx_Yaw;
/*=====================================================================================================*/
/*=====================================================================================================*/
void Rx_Configuration(void)
{
	TIM_ICInitTypeDef				   TIM_ICInitStructure;
	GPIO_InitTypeDef           GPIO_InitStructure;
	NVIC_InitTypeDef					 NVIC_InitStructure;
  
 	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOC Configuration: TIM2 CH1 (PA0), CH2 (PA1), CH3 (PA2), CH4 (PA4)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* connect TIM2 pins to AF2*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM configuration: Input Capture mode --------------------------------------
     The external signal is connected to TIM2 CH1 pin PA0 and TIM2 CH2 PA1
                                         TIM2 CH3 pin PA2 and TIM2 CH4 PA3
     The Bothedge is used as active edge,
     The TIM CCR1, CCR2, CCR3 and CCR4 is used to compute the duty cycle and frequency value 
  ------------------------------------------------------------ */
  /* TIM-CH1 Input capture mode */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/* TIM-CH2 Input capture mode */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/* TIM-CH3 Input capture mode */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/* TIM-CH4 Input capture mode */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  /* Prepare before Calculating Interrupt */
  Rx_Throttle.Number = 0;
  Rx_Pitch.Number = 0;
  Rx_Roll.Number = 0;
  Rx_Yaw.Number = 0;
  /* TIM 2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
 
  /* Enable the CC1, CC2, CC3 and CC4 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 , ENABLE);
 }
/*=====================================================================================================*/
/*=====================================================================================================*/
void TIM2_IRQHandler(void)
{
  /* Interrupt handler for PWM Capture */
  //Calculate Throttle from Tx
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
  {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    Calculate_Rx(&Rx_Throttle, TIM2->CCR1, GPIO_Pin_0);
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) 
  {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    Calculate_Rx(&Rx_Pitch, TIM2->CCR2, GPIO_Pin_1);
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
  {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    Calculate_Rx(&Rx_Roll, TIM2->CCR3, GPIO_Pin_2);
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) 
  {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    Calculate_Rx(&Rx_Yaw, TIM2->CCR4, GPIO_Pin_3);
  }
  
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void Calculate_Rx(Rx_t *Rx, uint32_t CCRx, uint16_t GPIO_Pin_x)
{
  if ((GPIOA->IDR & GPIO_Pin_x) != 0)
    {
      //Rising
      if (Rx->Number == 0)
      {
        /* Capture Timer */
        Rx->Rising = CCRx;
        Rx->Number++;
      }
    }
    if ((GPIOA->IDR & GPIO_Pin_x) == 0)
    {
      //Falling 
      if (Rx->Number == 1)
      {
        /* Capture Timer */
        Rx->Falling = CCRx;
        //Calculate DutyCycle
        if (Rx->Falling > Rx->Rising)
        {
          Rx->Highmeasure = Rx->Falling - Rx->Rising;
        }
        else if (Rx->Falling < Rx->Rising)
        {
          Rx->Highmeasure = 0xFFFFFFFF + Rx->Falling - Rx->Rising;
        }
        else Rx->Highmeasure = 0;

        Rx->NumCCR = TIM2_TIME / (TIM2_CLOCK / Rx->Highmeasure);
        Rx->Number++;
      } 
    }
    if ((GPIOA->IDR & GPIO_Pin_x) != 0)
    {
      //Rising
      if (Rx->Number == 2)
      {
        /* Capture Timer */
        Rx->PreRising = CCRx;
        //Calculate Freqcency
        if (Rx->PreRising > Rx->Rising)
        {
          Rx->Periodmeasure = Rx->PreRising - Rx->Rising;
        }
        else if (Rx->PreRising < Rx->Rising)
        {
          Rx->Periodmeasure = 0xFFFFFFFF + Rx->PreRising - Rx->Rising;
        }
        else Rx->Periodmeasure = 0;
        Rx->Frequency = TIM2_CLOCK / Rx->Periodmeasure;
        Rx->Number = 0;
      }
    }
}
/*=====================================================================================================*/
/*=====================================================================================================*/
