/* This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/** global definitions and methods (API)
 *  @file global.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016-2017
 */
#pragma once

/** enable debugging functionalities */
#define DEBUG true

/** get the length of an array */
#define LENGTH(x) (sizeof(x) / sizeof((x)[0]))
/** concatenate 2 arguments */
#define CAT2(x,y) x##y
/** concatenate 3 arguments */
#define CAT3(x,y,z) x##y##z
/** concatenate 4 arguments */
#define CAT4(w,x,y,z) w##x##y##z

/** @defgroup reg_macro macros to  define values based on other defines values
 *  @note used when the value is calculated or isn't a value
 *  @{
 */
/** get GPIO based on GPIO identifier */
#define GPIO(x) CAT2(GPIO,x)
/** get RCC for GPIO based on GPIO identifier */
#define RCC_GPIO(x) CAT2(RCC_GPIO,x)
/** get TIM based on TIM identifier */
#define TIM(x) CAT2(TIM,x)
/** get RCC for timer based on TIM identifier */
#define RCC_TIM(x) CAT2(RCC_TIM,x)
/** get NVIC IRQ for timer base on TIM identifier */
#define NVIC_TIM_IRQ(x) CAT3(NVIC_TIM,x,_IRQ)
/** get interrupt service routine for timer base on TIM identifier */
#define TIM_ISR(x) CAT3(tim,x,_isr)
/** get port based on TIMx_CHy identifier */
#define TIM_CH_PORT(x,y) CAT4(GPIO_BANK_TIM,x,_CH,y)
/** get pin based on TIMx_CHy identifier */
#define TIM_CH_PIN(x,y) CAT4(GPIO_TIM,x,_CH,y)
/** get RCC for port based on TIMx_CHy identifier */
#define RCC_TIM_CH(x,y) CAT4(RCC_TIM,x,_CH,y)
#define RCC_TIM1_CH1 RCC_GPIOA /**< RCC for port for on TIM1_CH1 */
#define RCC_TIM1_CH2 RCC_GPIOA /**< RCC for port for on TIM1_CH2 */
#define RCC_TIM1_CH3 RCC_GPIOA /**< RCC for port for on TIM1_CH3 */
#define RCC_TIM1_CH4 RCC_GPIOA /**< RCC for port for on TIM1_CH4 */
#define RCC_TIM1_CH1N RCC_GPIOB /**< RCC for port for on TIM1_CH1N */
#define RCC_TIM1_CH2N RCC_GPIOB /**< RCC for port for on TIM1_CH2N */
#define RCC_TIM1_CH3N RCC_GPIOB /**< RCC for port for on TIM1_CH3N */
#define RCC_TIM2_CH1_ETR RCC_GPIOA /**< RCC for port for on TIM2_CH1_ETR */
#define RCC_TIM2_CH2 RCC_GPIOA /**< RCC for port for on TIM2_CH2 */
#define RCC_TIM2_CH3 RCC_GPIOA /**< RCC for port for on TIM2_CH3 */
#define RCC_TIM2_CH4 RCC_GPIOA /**< RCC for port for on TIM2_CH4 */
#define RCC_TIM3_CH1 RCC_GPIOA /**< RCC for port for on TIM3_CH1 */
#define RCC_TIM3_CH2 RCC_GPIOA /**< RCC for port for on TIM3_CH2 */
#define RCC_TIM3_CH3 RCC_GPIOB /**< RCC for port for on TIM3_CH3 */
#define RCC_TIM3_CH4 RCC_GPIOB /**< RCC for port for on TIM3_CH4 */
#define RCC_TIM4_CH1 RCC_GPIOB /**< RCC for port for on TIM4_CH1 */
#define RCC_TIM4_CH2 RCC_GPIOB /**< RCC for port for on TIM4_CH2 */
#define RCC_TIM4_CH3 RCC_GPIOB /**< RCC for port for on TIM4_CH3 */
#define RCC_TIM4_CH4 RCC_GPIOB /**< RCC for port for on TIM4_CH4 */
#define RCC_TIM5_CH1 RCC_GPIOA /**< RCC for port for on TIM5_CH1 */
#define RCC_TIM5_CH2 RCC_GPIOA /**< RCC for port for on TIM5_CH2 */
#define RCC_TIM5_CH3 RCC_GPIOA /**< RCC for port for on TIM5_CH3 */
#define RCC_TIM5_CH4 RCC_GPIOA /**< RCC for port for on TIM5_CH4 */
/** get TIM_IC based on CHx identifier */
#define TIM_IC(x) CAT2(TIM_IC,x)
/** get TIM_IC_IN_TI based on CHx identifier */
#define TIM_IC_IN_TI(x) CAT2(TIM_IC_IN_TI,x)
/** get TIM_SR_CCxIF based on CHx identifier */
#define TIM_SR_CCIF(x) CAT3(TIM_SR_CC,x,IF)
/** get TIM_DIER_CCxIE based on CHx identifier */
#define TIM_DIER_CCIE(x) CAT3(TIM_DIER_CC,x,IE)
/** get TIM_CCRy register based on TIMx_CHy identifier */
#define TIM_CCR(x,y) CAT2(TIM_CCR,y)(TIM(x))
/** get external interrupt based on pin identifier */
#define EXTI(x) CAT2(EXTI,x)
/** get NVIC IRQ for external interrupt base on external interrupt/pin */
#define NVIC_EXTI_IRQ(x) CAT3(NVIC_EXTI,x,_IRQ)
#define NVIC_EXTI5_IRQ NVIC_EXTI9_5_IRQ /**< IRQ for line 9 to 5 for pin 5 */
#define NVIC_EXTI6_IRQ NVIC_EXTI9_5_IRQ /**< IRQ for line 9 to 5 for pin 6 */
#define NVIC_EXTI7_IRQ NVIC_EXTI9_5_IRQ /**< IRQ for line 9 to 5 for pin 7 */
#define NVIC_EXTI8_IRQ NVIC_EXTI9_5_IRQ /**< IRQ for line 9 to 5 for pin 8 */
#define NVIC_EXTI9_IRQ NVIC_EXTI9_5_IRQ /**< IRQ for line 9 to 5 for pin 9 */
#define NVIC_EXTI10_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 10 */
#define NVIC_EXTI11_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 11 */
#define NVIC_EXTI12_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 12 */
#define NVIC_EXTI13_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 13 */
#define NVIC_EXTI14_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 14 */
#define NVIC_EXTI15_IRQ NVIC_EXTI15_10_IRQ /**< IRQ for line 15 to 10 for pin 15 */
/** get interrupt service routine for timer base on external interrupt/pin */
#define EXTI_ISR(x) CAT3(exti,x,_isr)
#define exti5_isr exti9_5_isr /**< isr for line 9 to 5 for pin 5 */
#define exti6_isr exti9_5_isr /**< isr for line 9 to 5 for pin 6 */
#define exti7_isr exti9_5_isr /**< isr for line 9 to 5 for pin 7 */
#define exti8_isr exti9_5_isr /**< isr for line 9 to 5 for pin 8 */
#define exti9_isr exti9_5_isr /**< isr for line 9 to 5 for pin 9 */
#define exti10_isr exti15_10_isr /**< isr for line 15 to 10 for pin 10 */
#define exti11_isr exti15_10_isr /**< isr for line 15 to 10 for pin 11 */
#define exti12_isr exti15_10_isr /**< isr for line 15 to 10 for pin 12 */
#define exti13_isr exti15_10_isr /**< isr for line 15 to 10 for pin 13 */
#define exti14_isr exti15_10_isr /**< isr for line 15 to 10 for pin 14 */
#define exti15_isr exti15_10_isr /**< isr for line 15 to 10 for pin 15 */
/** get USART based on USART identifier */
#define USART(x) CAT2(USART,x)
/** get RCC for USART based on USART identifier */
#define USART_RCC(x) CAT2(RCC_USART,x)
/** get NVIC IRQ for USART based on USART identifier */
#define USART_IRQ(x) CAT3(NVIC_USART,x,_IRQ)
/** get interrupt service routine for USART based on USART identifier */
#define USART_ISR(x) CAT3(usart,x,_isr)
/** get port for USART based on USART identifier */
#define USART_PORT(x) CAT2(USART_PORT,x)
#define USART_PORT1 GPIOA /**< USART 1 is on port A */
#define USART_PORT2 GPIOA /**< USART 2 is on port A */
#define USART_PORT3 GPIOB /**< USART 3 is on port B */
/** get RCC for USART port based on USART identifier */
#define USART_PORT_RCC(x) CAT2(RCC_USART_PORT,x)
#define RCC_USART_PORT1 RCC_GPIOA /**< USART 1 is on port A */
#define RCC_USART_PORT2 RCC_GPIOA /**< USART 2 is on port A */
#define RCC_USART_PORT3 RCC_GPIOB /**< USART 3 is on port B */
/** get transmit pin for USART based on USART identifier */
#define USART_PIN_TX(x) CAT3(GPIO_USART,x,_TX)
/** get receive pin for USART based on USART identifier */
#define USART_PIN_RX(x) CAT3(GPIO_USART,x,_RX)
/** get port based on ADC12_IN identifier */
#define ADC12_IN_PORT(x) CAT3(ADC12_IN,x,_PORT)
#define ADC12_IN0_PORT GPIOA /**< ADC12_IN0 is on PA0 */
#define ADC12_IN1_PORT GPIOA /**< ADC12_IN1 is on PA1 */
#define ADC12_IN2_PORT GPIOA /**< ADC12_IN2 is on PA2 */
#define ADC12_IN3_PORT GPIOA /**< ADC12_IN3 is on PA3 */
#define ADC12_IN4_PORT GPIOA /**< ADC12_IN4 is on PA4 */
#define ADC12_IN5_PORT GPIOA /**< ADC12_IN5 is on PA5 */
#define ADC12_IN6_PORT GPIOA /**< ADC12_IN6 is on PA6 */
#define ADC12_IN7_PORT GPIOA /**< ADC12_IN7 is on PA7 */
#define ADC12_IN8_PORT GPIOB /**< ADC12_IN8 is on PB0 */
#define ADC12_IN9_PORT GPIOB /**< ADC12_IN9 is on PB1 */
#define ADC12_IN10_PORT GPIOC /**< ADC12_IN10 is on PC0 */
#define ADC12_IN11_PORT GPIOC /**< ADC12_IN11 is on PC1 */
#define ADC12_IN12_PORT GPIOC /**< ADC12_IN12 is on PC2 */
#define ADC12_IN13_PORT GPIOC /**< ADC12_IN13 is on PC3 */
#define ADC12_IN14_PORT GPIOC /**< ADC12_IN14 is on PC4 */
#define ADC12_IN15_PORT GPIOC /**< ADC12_IN15 is on PC5 */
/** get pin based on ADC12_IN identifier */
#define ADC12_IN_PIN(x) CAT3(ADC12_IN,x,_PIN)
#define ADC12_IN0_PIN GPIO0 /**< ADC12_IN0 is on PA0 */
#define ADC12_IN1_PIN GPIO1 /**< ADC12_IN1 is on PA1 */
#define ADC12_IN2_PIN GPIO2 /**< ADC12_IN2 is on PA2 */
#define ADC12_IN3_PIN GPIO3 /**< ADC12_IN3 is on PA3 */
#define ADC12_IN4_PIN GPIO4 /**< ADC12_IN4 is on PA4 */
#define ADC12_IN5_PIN GPIO5 /**< ADC12_IN5 is on PA5 */
#define ADC12_IN6_PIN GPIO6 /**< ADC12_IN6 is on PA6 */
#define ADC12_IN7_PIN GPIO7 /**< ADC12_IN7 is on PA7 */
#define ADC12_IN8_PIN GPIO0 /**< ADC12_IN8 is on PB0 */
#define ADC12_IN9_PIN GPIO1 /**< ADC12_IN9 is on PB1 */
#define ADC12_IN10_PIN GPIO0 /**< ADC12_IN10 is on PC0 */
#define ADC12_IN11_PIN GPIO1 /**< ADC12_IN11 is on PC1 */
#define ADC12_IN12_PIN GPIO2 /**< ADC12_IN12 is on PC2 */
#define ADC12_IN13_PIN GPIO3 /**< ADC12_IN13 is on PC3 */
#define ADC12_IN14_PIN GPIO4 /**< ADC12_IN14 is on PC4 */
#define ADC12_IN15_PIN GPIO5 /**< ADC12_IN15 is on PC5 */
/** get RCC based on ADC12_IN identifier */
#define RCC_ADC12_IN(x) CAT2(RCC_ADC12_IN,x)
#define RCC_ADC12_IN0 RCC_GPIOA /**< ADC12_IN0 is on PA0 */
#define RCC_ADC12_IN1 RCC_GPIOA /**< ADC12_IN1 is on PA1 */
#define RCC_ADC12_IN2 RCC_GPIOA /**< ADC12_IN2 is on PA2 */
#define RCC_ADC12_IN3 RCC_GPIOA /**< ADC12_IN3 is on PA3 */
#define RCC_ADC12_IN4 RCC_GPIOA /**< ADC12_IN4 is on PA4 */
#define RCC_ADC12_IN5 RCC_GPIOA /**< ADC12_IN5 is on PA5 */
#define RCC_ADC12_IN6 RCC_GPIOA /**< ADC12_IN6 is on PA6 */
#define RCC_ADC12_IN7 RCC_GPIOA /**< ADC12_IN7 is on PA7 */
#define RCC_ADC12_IN8 RCC_GPIOB /**< ADC12_IN8 is on PB0 */
#define RCC_ADC12_IN9 RCC_GPIOB /**< ADC12_IN9 is on PB1 */
#define RCC_ADC12_IN10 RCC_GPIOC /**< ADC12_IN10 is on PC0 */
#define RCC_ADC12_IN11 RCC_GPIOC /**< ADC12_IN11 is on PC1 */
#define RCC_ADC12_IN12 RCC_GPIOC /**< ADC12_IN12 is on PC2 */
#define RCC_ADC12_IN13 RCC_GPIOC /**< ADC12_IN13 is on PC3 */
#define RCC_ADC12_IN14 RCC_GPIOC /**< ADC12_IN14 is on PC4 */
#define RCC_ADC12_IN15 RCC_GPIOC /**< ADC12_IN15 is on PC5 */
/** get channel based on ADC12_IN identifier */
#define ADC_CHANNEL(x) CAT2(ADC_CHANNEL,x)
/** get I2C based on I2C identifier */
#define I2C(x) CAT2(I2C,x)
/** get RCC for I2C based on I2C identifier */
#define RCC_I2C(x) CAT2(RCC_I2C,x)
/** get RCC for GPIO port for SCL based on I2C identifier */
#define RCC_I2C_SCL_PORT(x) CAT3(RCC_I2C,x,_PORT)
#define RCC_I2C1_PORT RCC_GPIOB /**< RCC for GPIO port for SCL for I2C1 */
#define RCC_I2C2_PORT RCC_GPIOB /**< RCC for GPIO port for SCL for I2C2 */
/** get RCC for GPIO port for SCL based on I2C identifier */
#define RCC_I2C_SDA_PORT(x) CAT3(RCC_I2C,x,_PORT)
#define RCC_I2C1_SDA_PORT RCC_GPIOB /**< RCC for GPIO port for SDA for I2C1 */
#define RCC_I2C2_SDA_PORT RCC_GPIOB /**< RCC for GPIO port for SDA for I2C2 */
/** get I2C port for SCL pin based on I2C identifier */
#define I2C_SCL_PORT(x) CAT3(GPIO_BANK_I2C,x,_SCL)
/** get I2C port for SDA pin based on I2C identifier */
#define I2C_SDA_PORT(x) CAT3(GPIO_BANK_I2C,x,_SDA)
/** get I2C pin for SCL pin based on I2C identifier */
#define I2C_SCL_PIN(x) CAT3(GPIO_I2C,x,_SCL)
/** get I2C port for SDA pin based on I2C identifier */
#define I2C_SDA_PIN(x) CAT3(GPIO_I2C,x,_SDA)
/** get SPI based on SPI identifier */
#define SPI(x) CAT2(SPI,x)
/** get RCC for SPI based on SPI identifier */
#define RCC_SPI(x) CAT2(RCC_SPI,x)
/** get RCC for GPIO port for SPI NSS signals */
#define RCC_SPI_NSS_PORT(x) CAT3(RCC_SPI,x,_NSS_PORT)
#define RCC_SPI1_NSS_PORT RCC_GPIOA /**< RCC for GPIO port for NSS for SPI1 */
#define RCC_SPI1_RE_NSS_PORT RCC_GPIOA /**< RCC for GPIO port for NSS for SPI1_RE */
#define RCC_SPI2_NSS_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI2 */
/** get RCC for GPIO port for SPI SCK signals */
#define RCC_SPI_SCK_PORT(x) CAT3(RCC_SPI,x,_SCK_PORT)
#define RCC_SPI1_SCK_PORT RCC_GPIOA /**< RCC for GPIO port for NSS for SPI1 */
#define RCC_SPI1_RE_SCK_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI1_RE */
#define RCC_SPI2_SCK_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI2 */
/** get RCC for GPIO port for SPI MISO signals */
#define RCC_SPI_MISO_PORT(x) CAT3(RCC_SPI,x,_MISO_PORT)
#define RCC_SPI1_MISO_PORT RCC_GPIOA /**< RCC for GPIO port for NSS for SPI1 */
#define RCC_SPI1_RE_MISO_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI1_RE */
#define RCC_SPI2_MISO_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI2 */
/** get RCC for GPIO port for SPI MOSI signals */
#define RCC_SPI_MOSI_PORT(x) CAT3(RCC_SPI,x,_MOSI_PORT)
#define RCC_SPI1_MOSI_PORT RCC_GPIOA /**< RCC for GPIO port for NSS for SPI1 */
#define RCC_SPI1_RE_MOSI_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI1_RE */
#define RCC_SPI2_MOSI_PORT RCC_GPIOB /**< RCC for GPIO port for NSS for SPI2 */
/** get SPI port for NSS signal based on SPI identifier */
#define SPI_NSS_PORT(x) CAT3(GPIO_BANK_SPI,x,_NSS)
/** get SPI port for SCK signal based on SPI identifier */
#define SPI_SCK_PORT(x) CAT3(GPIO_BANK_SPI,x,_SCK)
/** get SPI port for MISO signal based on SPI identifier */
#define SPI_MISO_PORT(x) CAT3(GPIO_BANK_SPI,x,_MISO)
/** get SPI port for MOSI signal based on SPI identifier */
#define SPI_MOSI_PORT(x) CAT3(GPIO_BANK_SPI,x,_MOSI)
/** get SPI pin for NSS signal based on SPI identifier */
#define SPI_NSS_PIN(x) CAT3(GPIO_SPI,x,_NSS)
/** get SPI pin for SCK signal based on SPI identifier */
#define SPI_SCK_PIN(x) CAT3(GPIO_SPI,x,_SCK)
/** get SPI pin for MISO signal based on SPI identifier */
#define SPI_MISO_PIN(x) CAT3(GPIO_SPI,x,_MISO)
/** get SPI pin for MOSI signal based on SPI identifier */
#define SPI_MOSI_PIN(x) CAT3(GPIO_SPI,x,_MOSI)
/** get SPI CRC polynomial register based on SPI identifier */
#define SPI_CRC_PR(x) CAT3(SPI,x,_CRCPR)
/** get SPI CRC transmit register based on SPI identifier */
#define SPI_CRC_TXR(x) CAT3(SPI,x,_TXCRCR)
/** get SPI CRC receive register based on SPI identifier */
#define SPI_CRC_RXR(x) CAT3(SPI,x,_RXCRCR)
/** get DMA based on SPI identifier */
#define DMA_SPI(x) CAT2(DMA_SPI,x)
#define DMA_SPI1 DMA1 /**< SPI1 is on DMA1 */
#define DMA_SPI2 DMA1 /**< SPI2 is on DMA1 */
#define DMA_SPI3 DMA2 /**< SPI3 is on DMA2 */
/** get RCC for DMA based on SPI identifier */
#define RCC_DMA_SPI(x) CAT2(RCC_DMA_SPI,x)
#define RCC_DMA_SPI1 RCC_DMA1 /**< SPI1 is on DMA1 */
#define RCC_DMA_SPI2 RCC_DMA1 /**< SPI2 is on DMA1 */
#define RCC_DMA_SPI3 RCC_DMA2 /**< SPI3 is on DMA2 */
/** get DMA channel for SPI TX based on SPI identifier */
#define DMA_CHANNEL_SPI_TX(x) CAT3(DMA_CHANNEL_SPI,x,_TX)
#define DMA_CHANNEL_SPI1_TX DMA_CHANNEL3 /**< SPI1 TX is on DMA channel 3 */
#define DMA_CHANNEL_SPI2_TX DMA_CHANNEL5 /**< SPI2 TX is on DMA channel 5 */
#define DMA_CHANNEL_SPI3_TX DMA_CHANNEL2 /**< SPI3 TX is on DMA channel 2 */
/** get DMA channel for SPI RX based on SPI identifier */
#define DMA_CHANNEL_SPI_RX(x) CAT3(DMA_CHANNEL_SPI,x,_RX)
#define DMA_CHANNEL_SPI1_RX DMA_CHANNEL4 /**< SPI1 RX is on DMA channel 4 */
#define DMA_CHANNEL_SPI2_RX DMA_CHANNEL2 /**< SPI2 RX is on DMA channel 2 */
#define DMA_CHANNEL_SPI3_RX DMA_CHANNEL1 /**< SPI3 RX is on DMA channel 1 */

/** get DMA channel based on SPI identifier */
/** @} */

/** @defgroup board_led board LED GPIO
 *  @{
 */
#if defined(SYSTEM_BOARD) || defined(CORE_BOARD)
	/* on system and core board LED is on pin 11/PA1 */
	#define LED_PORT A /**< GPIO port (port A) */
	#define LED_PIN	1 /**< GPIO pin (pin PA1) */
#elif defined(BLUE_PILL)
	/* on minimum system LED is on pin 2/PC13 */
	#define LED_PORT C /**< GPIO port (port C on blue pill) */
	#define LED_PIN 13 /**< GPIO pin (pin PC13 on system board) */
#elif defined (MAPLE_MINI)
	/* on maple mini LED is on pin 19/PB1 */
	#define LED_PORT B /**< GPIO port (port B on maple mini) */
	#define LED_PIN 1 /**< GPIO pin (pin PB1 on maple mini) */
#endif
/** @} */

/** @defgroup board_button board user button GPIO
 *  @{
 */
#if defined(MAPLE_MINI)
	/* on maple mini user button is on 32/PB8 */
	#define BUTTON_PORT B /**< GPIO port (port B on maple mini) */
	#define BUTTON_PIN 8 /**< GPIO pin (pin PB8 on maple mini) */
#elif defined(CORE_BOARD)
	/* on core board user button is on PA8 */
	#define BUTTON_PORT A /**< GPIO port (port A) */
	#define BUTTON_PIN 8 /**< GPIO pin (pin PA8) */
#endif
/** @} */

/** @defgroup input to force DFU mode on low, even if application is valid
 *  @{
 */
#if defined(MAPLE_MINI)
	/* use button */
	#define DFU_FORCE_PORT BUTTON_PORT /**< button port */
	#define DFU_FORCE_PIN BUTTON_PIN /**< button pin */
	#define DFU_FORCE_VALUE true /**< button is pulled low unpressed, high pressed to force DFU mode */
#elif defined(CORE_BOARD)
	/* use button */
	#define DFU_FORCE_PORT BUTTON_PORT  /**< button port */
	#define DFU_FORCE_PIN BUTTON_PIN  /**< button pin */
	#define DFU_FORCE_VALUE false /**< button floating unpressed, connected to ground pressed to force DFU mode */
#else
	/* use the JNTRST pin as JPIO (this will disable the SWJ function, but we are not using it) */
	#define DFU_FORCE_PORT B /**< JNTRST port (needs to be remapped to become PB4) */
	#define DFU_FORCE_PIN 4 /**< JNTRST pin (needs to be remapped to become PB4) */
	#define DFU_FORCE_VALUE false /**< must be high to force DFU mode, since it's low after reset */
#endif
/** @} */

/** symbol for beginning of the application
 *  @note this symbol will be provided by the linker script
 */
extern uint32_t __application_beginning;
/** symbol for end of the application
 *  @note this symbol will be provided by the linker script
 */
extern uint32_t __application_end;

extern volatile bool button_flag; /**< flag set when board user button has been pressed/released */

/** get binary representation of a number
 *  @param[in] binary number to represent in binary
 *  @param[in] rjust justify representation with leading zeros
 *  @return string with binary representation of the number
 */
char* b2s(uint64_t binary, uint8_t rjust);

/** switch on board LED */
void led_on(void);

/** switch off board LED */
void led_off(void);

/** toggle board LED */
void led_toggle(void);

/** go to sleep for some microseconds
 *  @param[in] duration sleep duration in us
 */
void sleep_us(uint32_t duration);

/** go to sleep for some milliseconds
 *  @param[in] duration sleep duration in ms
 */
void sleep_ms(uint32_t duration);

/** setup board peripherals */
void board_setup(void);
