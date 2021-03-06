#include "stm32f4xx.h"
#include "nRF24L01.h"
#include "delay_ctrl.h"
#include "stm32f4xx_spi.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t TxBuf[SendTimes][TxBufSize] = {0};
uint8_t RxBuf[ReadTimes][RxBufSize] = {0};

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = { 0x34,0x43,0x10,0x10,0x01 };		
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = { 0x34,0x43,0x10,0x10,0x01 };
/*=====================================================================================================*/
/*=====================================================================================================*/

void nRF24L01_HW_Init(void);
void nRF24L01_RX_Mode(void);
void nRF24L01_TX_Mode(void);

uint8_t nRF24L01_SPI_RW(uint8_t wbyte);
void nRF24L01_SPI_Write_Byte(uint8_t wbyte);
uint8_t nRF24L01_SPI_Read_Byte(void);
void nRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t nRF24L01_Read_Reg(uint8_t reg);
void nRF24L01_Write_Buf(uint8_t Addr, uint8_t *wBuf, uint8_t Bytes);
void nRF24L01_Read_Buf(uint8_t Addr, uint8_t *rBuf, uint8_t Bytes);
uint8_t nRF24L01_Check(void);
uint8_t nRF24L01_Tx_Data(uint8_t *TxBuf);
uint8_t nRF24L01_Rx_Data(uint8_t *RxBuf);

void nRF24L01_SPI_CSN_H(void);
void nRF24L01_SPI_CSN_L(void);
void nRF24L01_CE_H(void);
void nRF24L01_CE_L(void);
/*=====================================================================================================*/
/*=====================================================================================================*/

void nRF24L01_HW_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
  	GPIO_InitTypeDef GPIO_InitStructure;

  	/* Enable GPIO clocks */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_SPI, ENABLE);
	/* SPI Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_SPI, ENABLE);

	/* Configure SPI pins:  SCK ,MOSI, MISO*/
	GPIO_InitStructure.GPIO_Pin =    GPIO_Pin_CSN | GPIO_Pin_SPI_SCK | GPIO_Pin_SPI_MOSI | GPIO_Pin_SPI_MISO;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
  	GPIO_Init(GPIO_SPI, &GPIO_InitStructure);

  	/* Connect SPI pins to AF */
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_CS_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_SCK_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MOSI_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MISO_SOURCE, GPIO_AF_SPI1);
	

	/* Enable GPIO of CHIP SELECT */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_CE, ENABLE);
	/* Configure CS pin */
	SPI_SSOutputCmd(SPI, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_CE;
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;

	GPIO_Init(GPIO_CE, &GPIO_InitStructure);
	
	/* Enable GPIO of IRQ */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Configure CE pin */
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	/* SPI configuration */
  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 84000kHz/256=328kHz < 400kHz
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI, &SPI_InitStructure);

	SPI_CalculateCRC(SPI, DISABLE);

	/* SPI1 enable */
	SPI_Cmd(SPI, ENABLE);

}

/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_SPI_CSN_H(void)
{
	GPIO_SetBits(GPIO_SPI, GPIO_Pin_CSN);
}

void nRF24L01_SPI_CSN_L(void)
{
	GPIO_ResetBits(GPIO_SPI, GPIO_Pin_CSN);
}

void nRF24L01_CE_H(void)
{
	GPIO_SetBits(GPIO_CE, GPIO_Pin_CE);
}

void nRF24L01_CE_L(void)
{
	GPIO_ResetBits(GPIO_CE, GPIO_Pin_CE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	nRF24L01_CE_L();
	nRF24L01_SPI_CSN_L(); //CSN Low, start SPI transaction

	nRF24L01_SPI_Write_Byte(reg); // select register to write
	nRF24L01_SPI_Write_Byte(value); //write data to it
	nRF24L01_SPI_CSN_H();// CSN High, terminal SPI transaction
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t nRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t rbuf;
	nRF24L01_CE_L();
	nRF24L01_SPI_CSN_L(); //CSN Low, start SPI transaction
	nRF24L01_SPI_Write_Byte(reg); // select register to read
	rbuf = nRF24L01_SPI_Read_Byte(); //receive data from register
	nRF24L01_SPI_CSN_H();// CSN High, terminal SPI transaction

	return rbuf;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_Write_Buf(uint8_t Addr, uint8_t *wBuf, uint8_t Bytes)
{
	uint8_t i;

	nRF24L01_CE_L();
	nRF24L01_SPI_CSN_L(); //CSN Low, start SPI transaction

	nRF24L01_SPI_Write_Byte(Addr);

	for(i=0; i<Bytes; i++)
	{
		nRF24L01_SPI_Write_Byte(wBuf[i]);
	}
	nRF24L01_SPI_CSN_H();// CSN High, terminal SPI transaction
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_Read_Buf(uint8_t Addr, uint8_t *rBuf, uint8_t Bytes)
{
	uint8_t i;

	nRF24L01_CE_L();
	nRF24L01_SPI_CSN_L(); //CSN Low, start SPI transaction

	nRF24L01_SPI_Write_Byte(Addr);

	for(i=0; i<Bytes; i++)
	{
		rBuf[i] = nRF24L01_SPI_Read_Byte();
	}
	nRF24L01_SPI_CSN_H();// CSN High, terminal SPI transaction

}

/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t nRF24L01_SPI_RW(uint8_t wbyte)
{
	/* Loop while DR register in not emplty */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
  	SPI->DR = wbyte;

  	/* Wait to receive a byte */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

  	return SPI->DR;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_SPI_Write_Byte(uint8_t wbyte)
{
	/* Loop while DR register in not emplty */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);

  	/* Send byte through the SPIx peripheral */
  	SPI->DR = wbyte;

  	/* Wait to receive a byte */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t nRF24L01_SPI_Read_Byte(void)
{
	/* Loop while DR register in not emplty */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI->DR = 0xFF;
	/* Wait to receive a byte */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

  	return SPI->DR;
}

/*=====================================================================================================*/
/*=====================================================================================================*/

uint8_t nRF24L01_Check(void)
{
	uint8_t TestBuf[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
	uint8_t CheckBuf[5];
	uint8_t i;

	nRF24L01_Write_Buf(WRITE_nRF_REG + TX_ADDR, TestBuf, 5);
	nRF24L01_Read_Buf(TX_ADDR, CheckBuf, 5);
           
	for(i=0; i<5; i++)
		if(CheckBuf[i]!=0xC2)	break;

	if(i==5)
		return SUCCESS;		
	else
		return ERROR;			
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t nRF24L01_Tx_Data(uint8_t *TxBuf)
{
	uint8_t Sta;

	nRF24L01_CE_L();
	nRF24L01_Write_Buf(WR_TX_PLOAD, TxBuf, TX_PLOAD_WIDTH);
	nRF24L01_CE_H();


	while(nRF24l01_IRQ != 0)
	Sta = nRF24L01_Read_Reg(NRFRegSTATUS);
	nRF24L01_Write_Reg(WRITE_nRF_REG + NRFRegSTATUS, Sta);
	nRF24L01_Write_Reg(FLUSH_TX, NOP);

	if(Sta & MAX_RT)
		return MAX_RT;
	else if(Sta & TX_DS)
		return TX_DS;
	else
		return ERROR;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
uint8_t nRF24L01_Rx_Data(uint8_t *RxBuf)
{
	uint8_t Sta;

	nRF24L01_CE_H();
	while(nRF24l01_IRQ!=0)
	nRF24L01_CE_L();

	Sta = nRF24L01_Read_Reg(NRFRegSTATUS);
	nRF24L01_Write_Reg(WRITE_nRF_REG + NRFRegSTATUS, Sta);

	if(Sta&RX_DR) {
		nRF24L01_Read_Buf(RD_RX_PLOAD, RxBuf, RX_PLOAD_WIDTH);
		nRF24L01_Write_Reg(FLUSH_RX, NOP);
		return RX_DR;
	}
	else
		return ERROR;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_TX_Mode(void)
{
  	nRF24L01_CE_L();
	nRF24L01_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);		
	nRF24L01_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);	
	nRF24L01_Write_Reg(WRITE_nRF_REG + EN_AA, 0x01);		
	nRF24L01_Write_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01);	
	nRF24L01_Write_Reg(WRITE_nRF_REG + SETUP_RETR, 0x05);	
	nRF24L01_Write_Reg(WRITE_nRF_REG + RF_CH, CHANAL);		
	nRF24L01_Write_Reg(WRITE_nRF_REG + RF_SETUP, 0x0f);		
	nRF24L01_Write_Reg(WRITE_nRF_REG + CONFIG, 0x0e);			
	nRF24L01_CE_H();
	
	nRF24L01_Delay_us(200);
	
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_RX_Mode(void)
{
	nRF24L01_CE_L();

	nRF24L01_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
	nRF24L01_Write_Reg(WRITE_nRF_REG + EN_AA, 0x01);
	nRF24L01_Write_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01);
	nRF24L01_Write_Reg(WRITE_nRF_REG + RF_CH, CHANAL);
	nRF24L01_Write_Reg(WRITE_nRF_REG + RX_PW_P0, RX_PLOAD_WIDTH);
	nRF24L01_Write_Reg(WRITE_nRF_REG + RF_SETUP, 0x0f);
	nRF24L01_Write_Reg(WRITE_nRF_REG + CONFIG, 0x0f);

  	nRF24L01_CE_H();

}
/*=====================================================================================================*/
/*=====================================================================================================*/