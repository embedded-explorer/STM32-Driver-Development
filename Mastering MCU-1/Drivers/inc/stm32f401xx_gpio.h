#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_

#include "stm32f401xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;			// Possible values @GPIO Pin Numbers
	uint8_t GPIO_PinMode;			// Possible values @GPIO Pin Modes
	uint8_t GPIO_PinSpeed;			// Possible values @GPIO Output Speeds
	uint8_t GPIO_PinPuPdControl;    // Possible values @Pull-up Pull-down Configuration
	uint8_t GPIO_PinOPType;			// Possible values @GPIO Output Types
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

// Handle Structure for GPIO pin
typedef struct{
	GPIO_RegDef_t *pGPIOx;		        // Holds Base Address of GPIO Port to Which Pin Belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// Holds GPIO Pin Configuration Settings

}GPIO_Handle_t;

// GPIO Pin Numbers
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

// GPIO Pin Modes
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_DT				6

//GPIO Output Types
#define GPIO_OP_TYPE_PP				0
#define GPIO_OP_TYPE_OD				1

//GPIO Output Speeds
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

// Pull-up Pull-down Configuration
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

/**************************************** APIs Supported *******************************************/

// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Initialization and DeInitialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


// Data Read Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);

// IRQ Configuration and Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint16_t PinNumber);


#endif /* INC_STM32F401XX_GPIO_H_ */
