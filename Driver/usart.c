#include "usart.h"
#include <stdio.h>
#include <rt_misc.h>
/*=====================================================================================================*/
/*=====================================================================================================*/
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};
 
/* Struct FILE is implemented in stdio.h */
FILE __stdout;
FILE __stdin;
/*=====================================================================================================*/
/*=====================================================================================================*/
int fputc(int ch, FILE *f) {
    /* Send your custom byte */
    /* Send byte to USART */
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
    return ch;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
int fgetc(FILE *f){
    char ch;
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    ch = USART_ReceiveData(USART1);
    return ch;
}

/*=====================================================================================================*/
/*=====================================================================================================*/
void USART_Configuration(unsigned int BaudRate)
{
	GPIO_InitTypeDef           GPIO_InitStructure;
	USART_InitTypeDef 				 USART_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART, ENABLE); 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_USART, ENABLE);
	
  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_USART_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_USART_RX;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
    
  GPIO_PinAFConfig(GPIO_USART, GPIO_Pin_USART_TX_SOURCE, GPIO_AF_USART1); 
  GPIO_PinAFConfig(GPIO_USART, GPIO_Pin_USART_RX_SOURCE, GPIO_AF_USART1); 
    
  USART_Cmd(USART1, ENABLE);  

}
/*=====================================================================================================*/
/*=====================================================================================================*/
//send to PC
void USARTx_SendChar(USART_TypeDef* USARTx, uint8_t Data)
{
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    USART_SendData(USARTx,Data);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void USARTx_SendString(USART_TypeDef* USARTx, uint8_t *Str)
{
    while(*Str)
    {
        USARTx_SendChar(USARTx,*Str);
        Str++;
    }
}
/*=====================================================================================================*/
/*=====================================================================================================*/
//receive from pc
uint8_t USARTx_GetChar(USART_TypeDef* USARTx)
{
    uint8_t Data;
    while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
    Data = (uint8_t)USART_ReceiveData(USARTx);
    return Data;
}
/*=====================================================================================================*/
/*=====================================================================================================*/