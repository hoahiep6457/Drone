#include "stm32f4xx.h"
#include "nRF24L01.h"
#include "delay_ctrl.h"
#include "stm32f4xx_spi.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
/*=====================================================================================================*/
/*=====================================================================================================*/

void nRF24L01_HW_Init(void);
void nRF24L01_RX_Mode(void);
void nRF24L01_TX_Mode(void);
unsigned char nRF24L01_SPI_Write_Byte(unsigned char data);
unsigned char SPI_Write_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_Read_Reg(unsigned char reg);
unsigned char SPI_Write(unsigned char reg, unsigned char *wbuf, unsigned char len);
unsigned char SPI_Read(unsigned char reg, unsigned char *rbuf, unsigned char len);
unsigned char nRF24L01_Rx_Packet(unsigned char* rx_buffer);
void nRF24L01_Tx_Packet(unsigned char* tx_buffer);
void nRF24L01_SPI_NSS_H(void);
void nRF24L01_SPI_NSS_L(void);
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

	/* Connect SPI pins to AF */
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_CS_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_SCK_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MOSI_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MISO_SOURCE,GPIO_AF_SPI1);
	
	/* Configure SPI pins:  SCK ,MOSI, MISO*/
	GPIO_InitStructure.GPIO_Pin =    GPIO_Pin_CS | GPIO_Pin_SPI_SCK | GPIO_Pin_SPI_MOSI | GPIO_Pin_SPI_MISO;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
  	GPIO_Init(GPIO_SPI, &GPIO_InitStructure);

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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure CE pin */
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
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
void nRF24L01_RX_Mode(void)
{
	nRF24L01_CE_L();
	SPI_Write_Reg(WRITE_nRF_REG + CONFIG, 0x39);
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); // setup add width 5 bytes
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + RF_CH,0x02);// setup frequency
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + RF_SETUP,  0x07);// setup power and rate
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + RX_PW_P0,32); //Number of bytes in data P0
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01); //Enable data P0
	nRF24L01_Delay_us(20);
	SPI_Write(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); // write address into tx_add
	nRF24L01_Delay_us(20);
	SPI_Write(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + EN_AA, 0x00);     //disable auto-ack for all channels      
	nRF24L01_Delay_us(20);
	
	SPI_Write_Reg(WRITE_nRF_REG + CONFIG, 0x33); // enable power up and prx
	nRF24L01_Delay_us(20);
  	nRF24L01_CE_H();
	nRF24L01_Delay_us(2000);

}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_TX_Mode(void)
{
  	nRF24L01_CE_L();
	SPI_Write_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); // setup add width 5 bytes
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + CONFIG, 0x38); // enable power up and ptx
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01); //Enable data P0
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + SETUP_RETR, 0x00);//Auto Retransmit Delay: 500 us, Auto Retransmit Count: Up to 2 Re-Transmit
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + RF_CH,0x02);// setup frequency
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + RF_SETUP ,0x07); //setup power 0dbm, rate 1Mbps
	nRF24L01_Delay_us(20);
	SPI_Write(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); // write address into tx_add
	nRF24L01_Delay_us(20);
	SPI_Write(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + EN_AA, 0x00);     // Disable Auto.Ack:Pipe0
	
	nRF24L01_Delay_us(200);
	
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_SPI_NSS_H(void)
{
	GPIO_SetBits(GPIO_CE, GPIO_Pin_CS);
}

void nRF24L01_SPI_NSS_L(void)
{
	GPIO_ResetBits(GPIO_CE, GPIO_Pin_CS);
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
unsigned char nRF24L01_SPI_Write_Byte(unsigned char data)
{
	/* Loop while DR register in not emplty */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);

  	/* Send byte through the SPIx peripheral */
  	SPI_I2S_SendData(SPI, data);

  	/* Wait to receive a byte */
  	while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

  	/* Return the byte read from the SPI bus */
  	return SPI_I2S_ReceiveData(SPI);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char SPI_Write_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;

	nRF24L01_SPI_NSS_L(); //CSN Low, start SPI transaction
	nRF24L01_Delay_us(20);
	status = nRF24L01_SPI_Write_Byte(reg); // select register to write
	nRF24L01_SPI_Write_Byte(value); //write data to it
	nRF24L01_SPI_NSS_H();// CSN High, terminal SPI transaction

	return status;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char rbuf;

	nRF24L01_SPI_NSS_L(); //CSN Low, start SPI transaction
	nRF24L01_Delay_us(20);
	nRF24L01_SPI_Write_Byte(reg); // select register to read
	rbuf = nRF24L01_SPI_Write_Byte(0); //receive data from register
	nRF24L01_SPI_NSS_H();// CSN High, terminal SPI transaction

	return rbuf;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char SPI_Write(unsigned char reg, unsigned char *wbuf, unsigned char len)
{
	unsigned int status, i;

	nRF24L01_SPI_NSS_L(); //CSN Low, start SPI transaction
	nRF24L01_Delay_us(20);
	status = nRF24L01_SPI_Write_Byte(reg);
	for(i=0; i<len; i++)
	{
		nRF24L01_SPI_Write_Byte(*wbuf);
		wbuf++;
	}
	nRF24L01_SPI_NSS_H();// CSN High, terminal SPI transaction

	return status;

}
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char SPI_Read(unsigned char reg, unsigned char *rbuf, unsigned char len)
{
	unsigned int status, i;

	nRF24L01_SPI_NSS_L(); //CSN Low, start SPI transaction
	nRF24L01_Delay_us(20);
	status = nRF24L01_SPI_Write_Byte(reg);

	for(i=0; i<len; i++)
	{
		*rbuf = nRF24L01_SPI_Write_Byte(0);
		rbuf++;
	}
	nRF24L01_SPI_NSS_H();// CSN High, terminal SPI transaction

	return status;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
unsigned char nRF24L01_Rx_Packet(unsigned char* rx_buffer)
{
	unsigned char flag = 0;
	unsigned char status;

	status = SPI_Read_Reg(NRFRegSTATUS);
	nRF24L01_SPI_NSS_L(); //CSN Low, start SPI transaction
	nRF24L01_Delay_us(20);

	if(status & 0x40) //Data Ready Rx FIFO interrupt
	{
		SPI_Read(RD_RX_PLOAD, rx_buffer, RX_PLOAD_WIDTH);
		flag = 1;
	}
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + NRFRegSTATUS, 0x40); // clear bit
	nRF24L01_Delay_us(20);
	nRF24L01_SPI_NSS_L();
	nRF24L01_SPI_Write_Byte(0xE2); //Flush RX FIFO
	nRF24L01_SPI_NSS_H();
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + CONFIG, 0x33); // enable power up and prx
	nRF24L01_CE_H();
	nRF24L01_Delay_us(20);

	return flag;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void nRF24L01_Tx_Packet(unsigned char* tx_buffer)
{
	nRF24L01_CE_L();
	SPI_Write_Reg(WRITE_nRF_REG+NRFRegSTATUS, 0x7E); // Write 1 to clear bit
	nRF24L01_Delay_us(20);
	SPI_Write_Reg(WRITE_nRF_REG + CONFIG, 0x3A); // enable power up and ptx
	nRF24L01_Delay_us(20);
	nRF24L01_SPI_NSS_L();  
	nRF24L01_SPI_Write_Byte(FLUSH_TX);
	nRF24L01_SPI_Write_Byte(0x00);
	nRF24L01_SPI_NSS_H();  
	nRF24L01_Delay_us(20);
	SPI_Write(WR_TX_PLOAD, tx_buffer, TX_PLOAD_WIDTH);
	nRF24L01_CE_H();
	nRF24L01_Delay_us(300000);
	nRF24L01_CE_L();

}
/*=====================================================================================================*/
/*=====================================================================================================*/