#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>
#include <stddef.h>

/********************************* Processor Specific Details ***************************************/
// ARM Cortex Mx Processor NVIC ISERx Register Addresses
#define NVIC_ISER0 						(volatile uint32_t*)0xE000E100
#define NVIC_ISER1 						(volatile uint32_t*)0xE000E104
#define NVIC_ISER2 						(volatile uint32_t*)0xE000E108
#define NVIC_ISER3 						(volatile uint32_t*)0xE000E10C

// ARM Cortex Mx Processor NVIC ICERx Register Address
#define NVIC_ICER0 						(volatile uint32_t*)0XE000E180
#define NVIC_ICER1 						(volatile uint32_t*)0XE000E184
#define NVIC_ICER2 						(volatile uint32_t*)0XE000E188
#define NVIC_ICER3 						(volatile uint32_t*)0XE000E18C

// ARM Cortex Mx Processor Priority Register Addresses
#define NVIC_PR_BASEADDR				(volatile uint32_t*)0xE000E400

// ARM Coretx Mx Processor Number of Priority Bits Implemented in Priority Register
#define NO_PR_BITS_IMPLEMENTED			4

/************************************* Base Address Macros *******************************************/
// Base Addresses of Embedded Memories
#define FLASH_BASEADDR					0x08000000U			// Base address of Flash memory (512Kbytes)
#define SRAM1_BASEADDR					0x20000000U			// Base address of SRAM (96Kbytes)
#define ROM_BASEADDR					0x1FFF0000U    		// Base address of ROM (30Kbytes)
#define SRAM_BASEADDR 					SRAM1_BASEADDR

// Base Addresses of Bus Domains
#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

// Base Addresses of AHB1 Bus Peripherals
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR  					(AHB1PERIPH_BASEADDR + 0x3800)

// Base Addresses of APB1 Bus Peripherals
#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR   				(APB1PERIPH_BASEADDR + 0x4400)

// Base Addresses of APB2 Bus Peripherals
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR  				(APB2PERIPH_BASEADDR + 0x3800)

/****************************** Peripheral Register Definition Structures *******************************/
// GPIO Register Definition Structure
typedef struct{
	volatile uint32_t MODER;			// GPIO port mode register
	volatile uint32_t OTYPER;			// GPIO port output type register
	volatile uint32_t OSPEEDR;         	// GPIO port output speed register
	volatile uint32_t PUPDR;			// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;				// GPIO port input data register
	volatile uint32_t ODR;				// GPIO port output data register
	volatile uint32_t BSRR;				// GPIO port bit set/reset register
	volatile uint32_t LCKR;				// GPIO port configuration lock register
	volatile uint32_t AFR[2];			// GPIO alternate function register
}GPIO_RegDef_t;

// RCC Register Definition Structure
typedef struct{
	volatile uint32_t CR;				// RCC clock control register
	volatile uint32_t PLLCFGR;			// RCC PLL configuration register
	volatile uint32_t CFGR;				// RCC clock configuration register
	volatile uint32_t CIR;				// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset register
	volatile uint32_t Reserved0;
	volatile uint32_t Reserved1;
	volatile uint32_t APB1RSTR;			// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;			// RCC APB2 peripheral reset register
	volatile uint32_t Reserved2;
	volatile uint32_t Reserved3;
	volatile uint32_t AHB1ENR;			// RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;			// RCC AHB2 peripheral clock enable register
	volatile uint32_t Reserved4;
	volatile uint32_t Reserved5;
	volatile uint32_t APB1ENR;			// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR; 			// RCC APB2 peripheral clock enable register
	volatile uint32_t Reserved6;
	volatile uint32_t Reserved7;
	volatile uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t Reserved8;
	volatile uint32_t Reserved9;
	volatile uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;		// RCC APB2 peripheral clock enabled in low power mode register
	volatile uint32_t Reserved10;
	volatile uint32_t Reserved11;
	volatile uint32_t BDCR;				// RCC Backup domain control register
	volatile uint32_t CSR;				// RCC clock control & status register
	volatile uint32_t Reserved12;
	volatile uint32_t Reserved13;
	volatile uint32_t SSCGR;			// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR; 		// RCC PLLI2S configuration register
	volatile uint32_t Reserved14;
	volatile uint32_t DCKCFGR;			// RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;

// EXTI Register Definition Structure
typedef struct{
	volatile uint32_t IMR;				// Interrupt mask register
	volatile uint32_t EMR;				// Event mask register
	volatile uint32_t RTSR;				// Rising trigger selection register
	volatile uint32_t FTSR;				// Falling trigger selection register
	volatile uint32_t SWIER;			// Software interrupt event register
	volatile uint32_t PR;				// Pending register
}EXTI_RegDef_t;

// SYSCFG Register Definition Structure
typedef struct{
	volatile uint32_t MEMRMP;			// SYSCFG memory remap register
	volatile uint32_t PMC;				// SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];		// SYSCFG external interrupt configuration register
	volatile uint32_t Reserved[2];
	volatile uint32_t CMPCR;			// Compensation cell control register
}SYSCFG_RegDef_t;

// SPI Register Definition Structure
typedef struct{
	volatile uint32_t SPI_CR1;			// SPI control register 1
	volatile uint32_t SPI_CR2;			// SPI control register 2
	volatile uint32_t SPI_SR;			// SPI status register
	volatile uint32_t SPI_DR;			// SPI data register
	volatile uint32_t SPI_CRCPR;		// SPI CRC polynomial register
	volatile uint32_t SPI_RXCRCR;		// SPI RX CRC register
	volatile uint32_t SPI_TXCRCR;		// SPI TX CRC register
	volatile uint32_t SPI_I2SCFGR;		// SPI_I2S configuration register
	volatile uint32_t SPI_I2SPR;		// SPI_I2S prescaler register
}SPI_RegDef_t;

// I2C Register Definition Structure
typedef struct{
	volatile uint32_t I2C_CR1;			// I2C Control register 1
	volatile uint32_t I2C_CR2;			// I2C Control register 2
	volatile uint32_t I2C_OAR1;			// I2C Own address register 1
	volatile uint32_t I2C_OAR2;			// I2C Own address register 2
	volatile uint32_t I2C_DR;			// I2C Data register
	volatile uint32_t I2C_SR1;			// I2C Status register 1
	volatile uint32_t I2C_SR2;			// I2C Status register 2
	volatile uint32_t I2C_CCR;			// I2C Clock control register
	volatile uint32_t I2C_TRISE;		// I2C TRISE register
	volatile uint32_t I2C_FLTR;			// I2C FLTR register
}I2C_RegDef_t;

// USART Register Definition Structure
typedef struct{
	volatile uint32_t USART_SR;			// Status register
	volatile uint32_t USART_DR;			// Data register
	volatile uint32_t USART_BRR;		// Baud rate register
	volatile uint32_t USART_CR1;		// Control register 1
	volatile uint32_t USART_CR2;		// Control register 2
	volatile uint32_t USART_CR3;		// Control register 3
	volatile uint32_t USART_GTPR;		// Guard time and prescaler register
}USART_RegDef_t;


/****************************** Peripheral Register Definition Macros *******************************/
// Peripheral Definitions
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1  	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  	((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1  	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  	((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2	((USART_RegDef_t*)USART2_BASEADDR)
#define USART6	((USART_RegDef_t*)USART6_BASEADDR)

/****************************** GPIO Macros *******************************/
// Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

// Clock Disable Macros for GPIOx Peripherals
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

// Macros to Reset GPIO Peripherals
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 		do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 		do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 		do{ (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 		do{ (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/****************************** I2C Macros *******************************/
// Clock Enable Macros for I2Cx Peripherals
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

// Clock Disable Macros for I2Cx Peripherals
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

// Macros to Reset I2C Peripherals
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 21));	(RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 22));	(RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 23));	(RCC->APB1RSTR &= ~(1 << 23)); }while(0)

// Bit Position Macros for CR1
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

// Bit Position Macros for CR2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

// Bit Position Macros for SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

// Bit Position Macros for SR2
#define I2C_SR2_MSL  		0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

// Bit Position Macros for CCR
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS 			15

/****************************** SPI Macros *******************************/
// Clock Enable Macros for SPIx Peripherals
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

// Clock Disable Macros for SPIx Peripherals
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

// Macros to Reset SPI Peripherals
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12));	(RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14));	(RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15));	(RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 13));	(RCC->APB2RSTR &= ~(1 << 13)); }while(0)

// Bit Position Macros for CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR 				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// Bit Position Macros for CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

// Bit Position Macros for SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/****************************** USART Macros *******************************/
// Clock Enable Macros for USARTx Peripherals
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

//Clock Disable Macros for USARTx Peripherals
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

// Macros to Reset USART Peripherals
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 4));	(RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 17));	(RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 5));	(RCC->APB2RSTR &= ~(1 << 5)); }while(0)

// Bit Position Macros for SR
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

// Bit Position Macros for BRR
#define USART_BRR_DIV_FRACTION		0
#define USART_BRR_DIV_MANTISSA		4

// Bit Position Macros for CR1
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

// Bit Position Macros for CR2
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

// Bit Position Macros for CR3
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11


// Bit Position Macros for GTPR
#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8

/****************************** SYSCFG Macros *******************************/
// Clock Enable Macros for SYSCFG
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

// Clock Enable Macros for SYSCFG
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/******************************* Interrupt Macros *****************************/
// Returns port code for given gpio base address
#define GPOIO_BASEADDR_TO_CODE(x)		((x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOH)?7:0)
// IRQ Numbers
#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2						36
#define IRQ_NO_SPI3						51
#define IRQ_NO_SPI4						84

#define IRQ_NO_I2C1_EV					31
#define IRQ_NO_I2C1_ER					32
#define IRQ_NO_I2C2_EV					33
#define IRQ_NO_I2C2_ER					34
#define IRQ_NO_I2C3_EV					72
#define IRQ_NO_I2C3_ER					73

#define IRQ_NO_USART1					37
#define IRQ_NO_USART2					38
#define IRQ_NO_USART6					71


// IRQ Priority Levels
#define IRQ_PRI0						0
#define IRQ_PRI1						1
#define IRQ_PRI2						2
#define IRQ_PRI3						3
#define IRQ_PRI4						4
#define IRQ_PRI5						5
#define IRQ_PRI6						6
#define IRQ_PRI7						7
#define IRQ_PRI8						8
#define IRQ_PRI9						9
#define IRQ_PRI10						10
#define IRQ_PRI11						11
#define IRQ_PRI12						12
#define IRQ_PRI13						13
#define IRQ_PRI14						14
#define IRQ_PRI15						15

// Generic Macros
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET

#include "stm32f401xx_gpio.h"
#include "stm32f401xx_spi.h"
#include "stm32f401xx_i2c.h"
#include "stm32f401xx_usart.h"
#include "stm32f401xx_rcc.h"

#endif /* INC_STM32F401XX_H_ */
