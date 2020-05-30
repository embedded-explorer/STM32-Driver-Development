#ifndef INC_STM32F401XX_SPI_H_
#define INC_STM32F401XX_SPI_H_

#include "stm32f401xx.h"
// Configuration Structure for SPIx Peripherals
typedef struct
{
	uint8_t SPI_DeviceMode;			// @SPI Modes
	uint8_t SPI_BusConfig;			// @Bus Configuration
	uint8_t SPI_SclkSpeed;			// @Clock Speed
	uint8_t SPI_DFF;       			// @Data Frame Format
	uint8_t SPI_CPOL;				// @Clock Polarity
	uint8_t SPI_CPHA;				// @Clock Phase
	uint8_t SPI_SSM;				// @Slave Select Pin Management

	uint8_t SPI_SSI;   				// @Internal Slave Select
	uint8_t SPI_SS0E;   			// @External Slave Select

}SPI_Config_t;

// Handle Structure for SPIx Peripherals
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

	uint8_t		*pTxBuffer; // To Store Application Tx Buffer
	uint8_t		*pRxBuffer; // To Store Application Rx Buffer
	uint32_t	TxLen;		// To Store Tx Length
	uint32_t    RxLen;		// To Store Rx Length
	uint8_t		TxState;	// To Store Tx State
	uint8_t		RxState;	// To Store Rx State

}SPI_Handle_t;

// SPI Device Modes
#define SPI_SLAVE					0
#define SPI_MASTER					1

// Bus Configuration
#define SPI_BUS_FDUPLEX				1
#define SPI_BUS_HDUPLEX				2
#define SPI_BUS_SIMPLEX_RX			3

// Clock Speed
#define SPI_CLK_SPEED_DIV2			0
#define SPI_CLK_SPEED_DIV4			1
#define SPI_CLK_SPEED_DIV8			2
#define SPI_CLK_SPEED_DIV16			3
#define SPI_CLK_SPEED_DIV32			4
#define SPI_CLK_SPEED_DIV64			5
#define SPI_CLK_SPEED_DIV128		6
#define SPI_CLK_SPEED_DIV256		7

// Data Frame Format
#define SPI_DFF_8BIT				0
#define SPI_DFF_16BIT				1

// Clock Polarity
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

// Clock Phase
#define SPI_CPHA_SAMP1				0
#define SPI_CPHA_SAMP2				1

// Slave Select Pin Management
#define SPI_SSM_HW					0
#define SPI_SSM_SW					1

// SPI Application States
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

// SPI Application Events
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3

/**************************************** APIs Supported *******************************************/
// Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Initialization and De-Initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

// Application Callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F401XX_SPI_H_ */
