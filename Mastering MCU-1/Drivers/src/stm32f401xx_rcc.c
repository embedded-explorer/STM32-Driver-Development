#include "stm32f401xx_rcc.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 128, 256, 512};
uint16_t APB_PreScaler[4] = {2, 4, 8, 16};

// To calculate PCLK1
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, systemclk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0)
		systemclk = 16000000;	// HSI
	else if(clksrc == 1)
		systemclk = 8000000;	// HSE
	else if(clksrc == 2)
		systemclk = RCC_GetPLLOutputClock();

	// Finding PreScaler of AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
		ahbp = 1;
	else
		ahbp = AHB_PreScaler[temp - 8];

	// Finding PreScaler of APB1
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
		apb1p = 1;
	else
		apb1p = APB_PreScaler[temp - 4];

	pclk1 = (systemclk / ahbp) / apb1p;
	return pclk1;
}

// Find PCLK2
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, systemclk;
	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0)
		systemclk = 16000000;	// HSI
	else if(clksrc == 1)
		systemclk = 8000000;	// HSE
	else if(clksrc == 2)
		systemclk = RCC_GetPLLOutputClock();

	// Finding PreScaler of AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
		ahbp = 1;
	else
		ahbp = AHB_PreScaler[temp - 8];

	// Finding PreScaler of APB2
	temp = ((RCC->CFGR >> 13) & 0x7);
	if(temp < 4)
		apb2p = 1;
	else
		apb2p = APB_PreScaler[temp - 4];

	pclk2 = (systemclk / ahbp) / apb2p;
	return pclk2;
}

// To Find PLL Clock
uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t pllclk = 0;
	return pllclk;
}
