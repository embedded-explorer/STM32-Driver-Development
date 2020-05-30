#include "stm32f401xx_i2c.h"

static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

// Start Condition
static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

// Generating Stop Condition
void I2CGenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

// Send Address of Slave to write data
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr &= ~(1);				// Setting r/w bit to 0
	pI2Cx->I2C_DR = SlaveAddr;
}

// Send Address of Slave to Read data
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr |= 1;				// Setting r/w bit to 0
	pI2Cx->I2C_DR = SlaveAddr;
}

// Clearing ADDR Flag
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	// Check mode of device
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))  	// 	Master mode
	{
		// Check for application state
		if(pI2CHandle->state == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// Disable ACKing
				pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);

				// Clear ADDR Flag (read sr1, sr2)
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;
			}
		}
		else
		{
			// Clear ADDR Flag (read sr1, sr2)
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
	}
	else		// Slave Mode
	{
		// Clear ADDR Flag (read sr1, sr2)
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}
}

// Helper function to execute RXNE interrupt handler
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{

	// We have to Receive data
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Disable Acking
			pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
		}

		// Read Data
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		// Close data Reception
		// 1. Generate stop condition
		if(pI2CHandle->RepeatedStart == DISABLE)
			I2CGenerateStopCondition(pI2CHandle->pI2Cx);

		// 2. Close Reception
		I2C_CloseReception(pI2CHandle);

		// 3. Notify user application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

// Helper function to execute TXE interrupt handler
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		// 1. Load data into DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		// 2. Decrement TxLen
		pI2CHandle->TxLen--;

		// 3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

// Close data Transmission
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->state = I2C_READY;
}


// Terminate data Reception
void I2C_CloseReception(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->state = I2C_READY;

	// Enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
		pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
}

/***********************************************************************************
 * @fn			: I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}

/***********************************************************************************
 * @fn			: I2C_Init
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
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg;

	// Enable Clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// 2. Configure FREQ in CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	// 3. Store device own address in OAR1
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	// 4. Configure CCR
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSpeed_SM)
	{
		// Standard Mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// Fast Mode
		// Enable Fast Mode
		tempreg |= (1 << I2C_CCR_FS);
		// Select Duty Cycle
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		// Find CCR Value
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDUTY_2)
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		else
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempreg;

	// 5. Configure TRISE
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSpeed_SM)
	{
		// Standard Mode
		tempreg = ((RCC_GetPCLK1Value() / 1000000U) + 1);
	}
	else
	{
		// Fast Mode
		tempreg = ((RCC_GetPCLK1Value() * 300) /1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}

/***********************************************************************************
 * @fn			: I2C_DeInit
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
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		I2C1_REG_RESET();
	else if(pI2Cx == I2C2)
		I2C2_REG_RESET();
	else if(pI2Cx == I2C3)
		I2C3_REG_RESET();
}

/***********************************************************************************
 * @fn			: I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |=  (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->I2C_CR1 &=  ~(1 << I2C_CR1_PE);
	}
}

/***********************************************************************************
 * @fn			: I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart)
{
	// 1. Generate Start Condition
	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Check SB Flag to confirm completion of start generation
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB)));

	// 3. Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Check ADDR Falg to confirm completion of address phase
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR)));

	if(Len == 1)
	{
		// Disable ACKing
		pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);

		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until RXNE becomes 1
		while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE)));

		if(RepeatedStart == DISABLE)
		{
			// Generate Stop Condition
			I2CGenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	}

	if(Len > 1)
	{
		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the data until Len becomes zero
		for(uint32_t i=Len; i>0; i--)
		{
			// Wait until RXNE becomes 1
			while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE)));

			if(i == 2)
			{
				// Disable ACKing
				pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);

				if(RepeatedStart == DISABLE)
				{
					// Generate Stop Condition
					I2CGenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			// Read data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			// Increment buffer address
			pRxBuffer++;
		}
	}

	// Enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
}

/***********************************************************************************
 * @fn			: I2C_MasterSendData
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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart)
{
	// 1. Generate Start Condition
	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Check SB Flag to confirm completion of start generation
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB)));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Check ADDR Falg to confirm completion of address phase
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR)));

	// 5. Clear ADDR Flag
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send Data until Length becomes 0
	while(Len > 0)
	{
		while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE)));	// Wait Until TXE is set
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE)));
	while(!(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF)));

	if(RepeatedStart == DISABLE)
	{
		// 8. Generate STOP condition and master need not to wait for the completion of stop condition
		I2CGenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return pI2C->I2C_DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/***********************************************************************************
 * @fn			: I2C_MasterSendDataIT
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: state
 *
 * @Note		: none
 ************************************************************************************/

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart)
{
	// check state
	uint8_t busystate = pI2CHandle->state;

	// if i2c is not busy
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->state = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatedStart = RepeatedStart;

		// Generate Start Condition
		I2CGenerateStartCondition(pI2CHandle->pI2Cx);	// will trigger SB interrupt

		// Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/***********************************************************************************
 * @fn			: I2C_MasterReceiveDataIT
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
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart)
{
	// check state
	uint8_t busystate = pI2CHandle->state;

	// if i2c is not busy
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxSize = Len;
		pI2CHandle->state = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatedStart = RepeatedStart;

		// Generate Start Condition
		I2CGenerateStartCondition(pI2CHandle->pI2Cx);	// will trigger SB interrupt

		// Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/***********************************************************************************
 * @fn			: I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			: I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;
	uint8_t shift_amount = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + temp1) |= (IRQPriority << shift_amount);
}

/***********************************************************************************
 * @fn			: I2C_EV_IRQHandling
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
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling in both Master and Slave mode
	uint32_t temp1, temp2, temp3;

	// Check for ITEVTFEN
	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	// Check for ITBUFEN
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	// 1. SB Interrupt Handler (Applicable only in Master mode)
	// Check for SB Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3)
	{
		// Execute Address Phase
		if(pI2CHandle->state == I2C_BUSY_IN_TX)		// write
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->state == I2C_BUSY_IN_RX)	// read
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	// 2. ADDR Interrupt Handler
	// Master: Address is sent
	// Slave: Address is matched
	// Check ADDR Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3)
	{
		// Clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// 3. BTF(Both Transfer Finished) Interrupt Handler
	// Check BTF Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		// Check for state
		if(pI2CHandle->state == I2C_BUSY_IN_TX)		// Tx
		{
			// Make sure TxE is set
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					// Close the transmission
					if(pI2CHandle->RepeatedStart == DISABLE)
						I2CGenerateStopCondition(pI2CHandle->pI2Cx);

					// Reset all members of Handle structure
					I2C_CloseTransmission(pI2CHandle);

					// Notify application Transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
	}

	// 4. STOPF Interrupt Handler (Applicable only in slave mode)
	// Check STOPF Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);

	if(temp1 && temp3)
	{
		// Clear STOPF by reading SR1 and write to CR1
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		// Notify the application STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// 5. TXE Interrupt Handler
	// Check TXE Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE);

	if(temp1 && temp2 && temp3)
	{
		// WE need to transmit data if device is master
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// Check for state
			if(pI2CHandle->state == I2C_BUSY_IN_TX)		// Tx
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else		// Slave
		{
			// Check wether slave is transmitter mode
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// 6. RXNE Interrupt Handler
	// Check RXNE Flag
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE);

	if(temp1 && temp2 && temp3)
	{
		// Check Device Mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->state == I2C_BUSY_IN_RX)		// Rx
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/***********************************************************************************
 * @fn			: I2C_ER_IRQHandling
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
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


	// Check for Bus error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		// Clear the bus error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		// Notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	// Check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

	// Check for ACK failure  error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	// Check for Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	// Check for Time out error
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}


// Week Implementation of I2C_ApplicationCallback
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}
