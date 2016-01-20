#include "stm32f4xx.h"
#include "quad_i2c_ctrl.h"

/*        MASTER                    SLAVE
   PB8 --- I2C1_SCL               MPU6050
   PB9 --- I2C1_SDA               HMC5883L
*/
/*=====================================================================================================*/
/*=====================================================================================================*/
void I2C_Configuration(void)
{
#ifdef FAST_I2C_MODE
#define I2C_SPEED 400000
#define I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
#define I2C_SPEED 100000
#define I2C_DUTYCYCLE I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/
  
	I2C_InitTypeDef   		     I2C_InitStructure;
	GPIO_InitTypeDef           GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1PEriph_GPIO_I2C, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_I2C, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_I2C_SCL | GPIO_Pin_I2C_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIO_I2C,GPIO_Pin_I2C_SCL_SOURCE,GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIO_I2C,GPIO_Pin_I2C_SDA_SOURCE,GPIO_AF_I2C1);	

  /* I2C De-initialize */
  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);
 /* I2C ENABLE */
  I2C_Cmd(I2C1, ENABLE); 
  /* Enable Interrupt */

}
/*=====================================================================================================*/
/*=====================================================================================================*/