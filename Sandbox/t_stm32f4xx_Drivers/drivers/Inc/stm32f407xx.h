/*
 * stm32f407xx.h
 *
 *  Created on: Mar 16, 2024
 *      Author: sherd
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


//Define Memory address of Flash, SRAM

#define FLAHS_BASEADDR		0x0800 0000U
#define SRAM1_BASEADDR		0x2000 0000U  //112KB (1 C000)
#define SRAM2_BASEADDR		(SRAM1_BASEADDR + 0X0001 C000)
#define ROM_BASEADDR		0x1FFF 0000  //System Memory
#define SRAM				SRAM1_BASEADDR

//AHBx and APBX Memory Addresses

#define PERIPH_BASEADDR 		0x4000 0000U
#define AHB1PERIPH_BASEADDR 	0x40020000U
#define AHB2PERIPH_BASEADDR		0x5000 0000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x4001 0000U

//Memory Addresses of all Peripherals Hanging on AHB1
#define GPIOA_BASEADDR		AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 2000)

//Memory Addresses of all Peripherals Hanging on APB1

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0X000 5000)

//Memory Addresses of all Peripherals Hanging on APB2

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0X000 3C00)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0X000 3000)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0X000 3800)

#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0X000 1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0X000 1400)

#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x000 3800)






//Peripheral Definition Structure

typedef struct {
	volatile uint32_t MODER;			//GPIO Port Mode Reg
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t GPIOA_PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];		//AFR[0] > AFRL and AFR[1] > AFRH
} GPIO_RegDef_t;

//RCC Definition Structure

typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED1;
	volatile uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED4;
	volatile uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED7;
	volatile uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED9;
	volatile uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;


#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

//Clock Enable MACRO for GPIOx
#define GPIOA_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN  		(RCC -> AHB1ENR |= (1 << 8))

//Clock Enable MACRO for I2C
#define I2C1_PCLK_EN  		(RCC -> APB2ENR |= (1 << 21))
#define I2C2_PCLK_EN  		(RCC -> APB2ENR |= (1 << 22))
#define I2C3_PCLK_EN  		(RCC -> APB2ENR |= (1 << 23))

//Clock Enable MACRO for SPI1
#define SPI1_PCLK_EN  		(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN  		(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN  		(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN  		(RCC -> APB2ENR |= (1 << 13))
//Clock Enable MACRO for USARTX
#define USART1_PCLK_EN  	(RCC -> APB2ENR |= (1 << 4))
#define USART2_PCLK_EN  	(RCC -> APB1ENR |= (1 << 17))
#define USART3_PCLK_EN  	(RCC -> APB1ENR |= (1 << 18))
#define UART4_PCLK_EN  		(RCC -> APB1ENR |= (1 << 19))
#define UART5_PCLK_EN  		(RCC -> APB1ENR |= (1 << 20))
#define USART6_PCLK_EN  	(RCC -> APB2ENR |= (1 << 5))

//GPIO_RegDef_t *pGPIOA = ((GPIO_RegDef_t*)GPIOA_BASEADDR);
GPIO_RegDef_t *pGPIOA = GPIOA;


#endif /* INC_STM32F407XX_H_ */
