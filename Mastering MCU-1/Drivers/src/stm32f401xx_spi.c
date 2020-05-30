#include "stm32f401xx_spi.h"
#include<stdio.h>

static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVRError_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/***********************************************************************************
 * @fn			: SPI_PeriClockControl
 *
 * @brief		: Enables or Disables Peripheral Clock for Given SPI Peripheral
 *
 * @param[in]	: Pointer to Base Address of SPI Peripheral
 * @param[in]	: ENABLE or DISABLE
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if(pSPIx == SPI4)
			SPI4_PCLK_EN();
	}
	else
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if(pSPIx == SPI4)
			SPI4_PCLK_DI();
	}
}

/***********************************************************************************
 * @fn			: SPI_Init
 *
 * @brief		: Initializes SPI Configuration
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure CR1
	uint32_t tempreg = 0;

	// 1.Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FDUPLEX)
	{
		// Disable BIDIMODE = 0
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HDUPLEX)
	{
		// Enable BIDIMODE = 1
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX)
	{
		// Disable BIDIMODE = 0,  RXONLY = 1
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure SPI Clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// 8. Configure SSI only in software ss
	tempreg |= pSPIHandle->SPIConfig.SPI_SSI << SPI_CR1_SSI;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

	tempreg = 0;
	// 9. Configure SSOE only in hardware ss
	tempreg |= pSPIHandle->SPIConfig.SPI_SS0E << SPI_CR2_SSOE;

	pSPIHandle->pSPIx->SPI_CR2 = tempreg;
}

/***********************************************************************************
 * @fn			: SPI_DeInit
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
	else if(pSPIx == SPI4)
		SPI4_REG_RESET();
}

/***********************************************************************************
 * @fn			: SPI_SendData
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until Tx buffer is empty
		while(!(pSPIx->SPI_SR & (1 << SPI_SR_TXE)));

		// 2. Check DFF bit
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )				// 16 bit Mode
		{
			// 1. Load Data to DR
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer ++;
		}
		else													// 8 bit Mode
		{
			// 1. Load Data to DR
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer ++;
		}
	}
}

/***********************************************************************************
 * @fn			: SPI_ReceiveData
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until RXNE is non empty
		while(!(pSPIx->SPI_SR & (1 << SPI_SR_RXNE)));

		// 2. Check DFF bit
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )				// 16 bit Mode
		{
			// 1. Load Data to DR
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer ++;
		}
		else													// 8 bit Mode
		{
			// 1. Load Data to DR
			*(pRxBuffer) = pSPIx->SPI_DR;
			Len--;
			pRxBuffer ++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->SPI_CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************************************
 * @fn			: SPI_IRQInterruptConfig
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)							// Program ISER0
		{
			*NVIC_ISER0  |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)		// Program ISER1
		{
			*NVIC_ISER1  |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >65 && IRQNumber <96)		// Program ISER2
		{
			*NVIC_ISER2  |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)		// Program ICER0
		{
			*NVIC_ICER0  |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)		// Program ICER1
		{
			*NVIC_ICER1  |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >65 && IRQNumber <96)		// Program ICER2
		{
			*NVIC_ICER2  |= (1 << (IRQNumber % 64));
		}
	}
}

/***********************************************************************************
 * @fn			: SPI_IRQPriorityConfig
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;
	uint8_t shift_amount = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + temp1) |= (IRQPriority << shift_amount);
}

/***********************************************************************************
 * @fn			: SPI_IRQHandling
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	// Check TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// Handle TXE
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	// Check RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);


	if(temp1 && temp2)
	{
		// Handle RXNE
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	// Check Overrun Error
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);


	if(temp1 && temp2)
	{
		// Handle RXNE
		SPI_OVRError_Interrupt_Handle(pSPIHandle);
	}
}

/***********************************************************************************
 * @fn			: SPI_IRQHandling
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark SPI state as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE falg is set
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/***********************************************************************************
 * @fn			: SPI_IRQHandling
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark SPI state as busy
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE falg is set
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF bit
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )				// 16 bit Mode
	{
		// Load Data to DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer ++;
	}
	else													// 8 bit Mode
	{
		// Load Data to DR
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer ++;
	}

	if(! pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
	}
}

void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF bit
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )				// 16 bit Mode
	{
		// Load Data to DR
		*((uint16_t*)pSPIHandle->pTxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer ++;
	}
	else													// 8 bit Mode
	{
		// Load Data to DR
		*pSPIHandle->pTxBuffer = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer ++;
	}

	if(! pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
	}
}

void SPI_OVRError_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// Clear OVR Flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// Close SPI Communication Tx Over
	// Disable TXEIE to Prevent Further Interrupts
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// Close SPI Communication Tx Over
	// Disable TXEIE to Prevent Further Interrupts
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

// Weak Implementation
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

}
