#ifndef INC_STM32F401XX_I2C_H_
#define INC_STM32F401XX_I2C_H_

#include "stm32f401xx.h"

// Configuration Structure for I2Cx Peripherals
typedef struct
{
	uint32_t I2C_SCLSpeed;			// @I2C SCLSpeed
	uint8_t	 I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;		// @ACK Control
	uint16_t I2C_FMDutyCycle;		// @I2C FM Duty Cycle
}I2C_Config_t;

// Handle Structure for I2Cx Peripherals
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;		// Store Application Tx Buffer
	uint8_t *pRxBuffer;
	uint32_t TxLen;			// Store Application Tx Length
	uint32_t RxLen;
	uint8_t state;			// @I2C State
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t RepeatedStart;
}I2C_Handle_t;

// I2C SCLSpeed
#define I2C_SCLSpeed_SM		100000
#define I2C_SCLSpeed_FM		400000

// ACK Control
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

// I2C FM Duty Cycle
#define I2C_FMDUTY_2			0
#define I2C_FMDUTY_16_9			1

// I2C State
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

// I2C Application Event Macros
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO 			4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR  			6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9


/**************************************** APIs Supported *******************************************/
// Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Initialization and De-Initialization
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t RepeatedStart);

// IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);

// Enable or Disable the Peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2CGenerateStopCondition(I2C_RegDef_t *pI2Cx);

// Application Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C, uint8_t EnorDi);


#endif /* INC_STM32F401XX_I2C_H_ */
