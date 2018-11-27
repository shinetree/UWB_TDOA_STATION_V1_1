/*! ----------------------------------------------------------------------------
 * @file	dma_spi.c
 * @brief	realize fast dma spi read/write; in "safe" mode it used extra RAM buffer
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "stm32f10x_dma.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "port.h"

/***************************************************************************//**
 * Local variables, private define
**/
// Two options of DMA_SPI - with tmp_buf buffer and without
// use  SAFE_DMA_AND_BUFF if you unsure that DMA_SPI works properly
#define SAFE_DMA_AND_BUFF

#ifdef	SAFE_DMA_AND_BUFF
 #define MAX_DMABUF_SIZE	4096
static uint8 __align4 	tmp_buf[MAX_DMABUF_SIZE];
#endif

/*
 * 	do not use library function cause they are slow
 */
#define DMA_TX_CH	DMA1_Channel3
#define DMA_RX_CH	DMA1_Channel2
#define PORT_DMA_START_TX_FAST 	{DMA_TX_CH->CCR |= DMA_CCR1_EN;}
#define PORT_DMA_STOP_TX_FAST 	{DMA_TX_CH->CCR &= ~DMA_CCR1_EN;}
#define PORT_DMA_START_RX_FAST 	{DMA_RX_CH->CCR |= DMA_CCR1_EN;}
#define PORT_DMA_STOP_RX_FAST 	{DMA_RX_CH->CCR &= ~DMA_CCR1_EN;}

#define PORT_SPI_CLEAR_CS_FAST	{SPIx_CS_GPIO->BRR = SPIx_CS;}
#define PORT_SPI_SET_CS_FAST	{SPIx_CS_GPIO->BSRR = SPIx_CS;}

/***************************************************************************//**
 * Exported function prototypes
 */
void dma_init(void);

int writetospi_dma( uint16 headerLength,
			   	    const uint8 *headerBuffer,
					uint32 bodylength,
					const uint8 *bodyBuffer
				  );

int readfromspi_dma( uint16	headerLength,
			    	 const uint8 *headerBuffer,
					 uint32 readlength,
					 uint8 *readBuffer );

/***************************************************************************//**
 * @fn		dma_init()
 * @brief
 * 			init of dma module. we will not use IRQ for DMA transaction
 * 			spi_init should be executed first
 *
**/
void dma_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		/* connect DMA1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	/* connect SPI1 clock if not enable yet */

    DMA_DeInit(DMA_TX_CH);
    DMA_DeInit(DMA_RX_CH);


/* do not use library function cause it slow */
/* connect SPI to DMA in write/read functions
 * SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx , ENABLE);
 * SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx , ENABLE);
**/
    SPIx->CR2 |= SPI_I2S_DMAReq_Tx;	//connect Tx DMA_SPI
    SPIx->CR2 &= ~SPI_I2S_DMAReq_Rx;//Disconnect Rx DMA_SPI by default

/* DMA Channel SPI_TX Configuration */
    DMA_TX_CH->CCR =DMA_DIR_PeripheralDST | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |\
    				DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Mode_Normal |\
    				DMA_Priority_High | DMA_M2M_Disable ;

/* Default DMA Channel SPI_RX Configuration */
	DMA_RX_CH->CCR =DMA_DIR_PeripheralSRC | DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |\
					DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Mode_Normal |\
					DMA_Priority_High | DMA_M2M_Disable ;
}


/***************************************************************************//**
 * @fn	writetospi()
 * @brief
 * 		  Low level function to write to the SPI
 * 		  Takes two separate byte buffers for write header and write data
 * 		  Source data will be transfered from buf to spi via DMA
 *
 * @return
 * 			DWT_SUCCESS or DWT_ERROR
 *
**/
#pragma GCC optimize ("O3")
int writetospi_dma
(
	uint16 headerLength,
	const uint8 *headerBuffer,
	uint32 bodylength,
	const uint8 *bodyBuffer
)
{
    int stat ;

#ifdef SAFE_DMA_AND_BUFF
    if ((headerLength + bodylength) > MAX_DMABUF_SIZE)
    {
    	return DWT_ERROR;
    }
#endif

    stat = decamutexon() ;

/* check spi_transaction finished */
//    while((SPIx->SR & SPI_I2S_FLAG_BSY) !=0 );

    PORT_SPI_CLEAR_CS_FAST	// give extra time for SPI slave device

    DMA_TX_CH->CPAR =(uint32) &(SPIx->DR) ;

    /*header*/
    DMA_TX_CH->CMAR = (uint32)(headerBuffer) ;
    DMA_TX_CH->CNDTR = headerLength ;

    PORT_DMA_START_TX_FAST		  /* Start transfer */
    while((DMA_TX_CH->CNDTR) !=0);/* pool until last clock from SPI_TX register done */
    PORT_DMA_STOP_TX_FAST	/* do not wait SPI finished cause SPI slow. perform switching on next buffer while spi transaction in wire */

    /*data*/
    DMA_TX_CH->CMAR = (uint32)(bodyBuffer);
    DMA_TX_CH->CNDTR = bodylength ;

    PORT_DMA_START_TX_FAST		  /* Start transfer */
    while((DMA_TX_CH->CNDTR) !=0);/* pool until last clock from SPI_TX register done */
    while((SPIx->SR & SPI_I2S_FLAG_BSY) !=0 );/* Wait until last byte of transmission will leave SPI wire */
    PORT_DMA_STOP_TX_FAST

    /* finish */
    PORT_SPI_SET_CS_FAST

    SPIx->DR;	// perform a dummy read operation to clear OverRun &  RxNe flags in SPI SR register

    decamutexoff(stat);

    return DWT_SUCCESS;
}


/***************************************************************************//**
 * @fn	readfromspi()
 * @brief Source data will be transfered from buf to spi via DMA
 * 		  Low level abstract function to write to the SPI
 * 		  Takes two separate byte buffers for write header and write data
 *
 * @return
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 * 			DWT_SUCCESS or DWT_ERROR
 *
**/
#pragma GCC optimize ("O3")
int readfromspi_dma
(
	uint16       headerLength,
	const uint8 *headerBuffer,
	uint32       readlength,
	uint8       *readBuffer
)
{
    int stat ;

#ifdef	SAFE_DMA_AND_BUFF
    uint16 	count;
    count = (uint16)(headerLength + readlength);

    if (count > MAX_DMABUF_SIZE )
    {
    	return DWT_ERROR;
    }
#endif

    stat = decamutexon() ;

/* check any spi_transaction finished */
//    while((SPIx->SR & SPI_I2S_FLAG_BSY) !=0 );

// do not use library function cause it slow
    PORT_SPI_CLEAR_CS_FAST	// give extra time for SPI slave device

    DMA_TX_CH->CMAR =  (uint32)(headerBuffer); // no matter what we send to spi after header when perform a reading operation
    DMA_RX_CH->CPAR  = DMA_TX_CH->CPAR = (uint32)&(SPIx->DR);

#ifdef	SAFE_DMA_AND_BUFF

    SPIx->CR2 |= (SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx) ;	/* connect SPI_Tx & SPI_Rx to DMA */
    DMA_RX_CH->CMAR = (uint32)(&tmp_buf) ;	/* read buffer */

    DMA_RX_CH->CNDTR = DMA_TX_CH->CNDTR = count ;	//length

    /* Start write/read transfer*/
    /* read data */
    PORT_DMA_START_RX_FAST
    PORT_DMA_START_TX_FAST
    while(DMA_RX_CH->CNDTR !=0);	/* pool until last clock from SPI_RX */
    PORT_DMA_STOP_TX_FAST
    PORT_DMA_STOP_RX_FAST

    /* result of read operation*/
    for (count=headerLength; count<(headerLength+readlength); count++)
    {
    	*readBuffer++ = tmp_buf[count];
    }

#else

    SPIx->CR2 |= (SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx) ;	/* connect SPI_Tx & SPI_Rx to DMA */

    DMA_RX_CH->CMAR = (uint32) readBuffer ;	/* read buffer addr */

    DMA_TX_CH->CNDTR = headerLength+readlength;
    DMA_RX_CH->CNDTR = headerLength;

    /* Start. write command */

    /* DMA_RX Disable Memory increment and Start */
	DMA_RX_CH->CCR = DMA_CCR1_EN |	/* start Rx */
					DMA_DIR_PeripheralSRC | DMA_PeripheralInc_Disable | DMA_MemoryInc_Disable |\
					DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte | DMA_Mode_Normal |\
					DMA_Priority_High | DMA_M2M_Disable ;

    PORT_DMA_START_TX_FAST
    while(DMA_RX_CH->CNDTR !=0);	/* pool until last clock from SPI_RX */
    PORT_DMA_STOP_RX_FAST

    DMA_RX_CH->CNDTR = readlength;	/* read data length */

	DMA_RX_CH->CCR |= (DMA_MemoryInc_Enable);	 /* DMA_Rx Enable Memory increment */
/* DMA_Rx should be started after new readlength and changing of Memory increment mode */
	PORT_DMA_START_RX_FAST
	while(DMA_RX_CH->CNDTR !=0);	/* pool until last clock from SPI_RX */
	PORT_DMA_STOP_TX_FAST
	PORT_DMA_STOP_RX_FAST

#endif

    PORT_SPI_SET_CS_FAST

    SPIx->CR2 &= ~(SPI_I2S_DMAReq_Rx) ;	//Disconnect Rx from DMA_SPI !

    decamutexoff(stat);

    return DWT_SUCCESS;
}

/* eof dma_spi */


