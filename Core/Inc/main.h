/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmsis_os2.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern osMessageQueueId_t UsbRxQueueHandle;
extern osMessageQueueId_t UsbTxQueueHandle;
extern osMessageQueueId_t Can1RxQueueHandle;
extern osMessageQueueId_t Can2RxQueueHandle;
extern osMessageQueueId_t UdpTxQueueHandle;
extern osMessageQueueId_t TcpTxQueueHandle;

extern osMutexId_t tableMutexHandle;
extern osMutexId_t ajaxMutexHandle;

extern ADC_HandleTypeDef hadc1;
extern uint16_t CounterLed;
/* Anyone can request transition to the bootloader */
extern uint8_t BootloaderRequest;
extern uint8_t GlobalErrorFlags;
extern TIM_HandleTypeDef htim4;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern TIM_HandleTypeDef htim16;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern uint8_t DigitalOutput1State;
extern uint8_t DigitalOutput2State;
extern volatile unsigned long ulHighFrequencyTimerTicks;
extern osMutexId_t hethMutexHandle;
extern osMutexId_t spiSjaMutexHandle;
/*
 * Possible state of a LED.
 */
typedef enum
{
    GreenLed, /* Green on, red off */
    RedLed, /* Green off, red on */
    OrangeLed, /* Both green and red on  */
    NoLed /* Both green and red off */
} LedColor;

typedef enum
{
    LedOn = 0, /* Turn on LED */
    LedOff, /* Turn off LED */
    LedBlinkSlow, /* Blink slowly */
    LedBlinkFast, /* Blink fast */
    LedBlinkSingle /* Blink once */
} LedState;

#define QUEUE_PUT_TIMEOUT           1       /* For fast echo, long timeout not good */
#define MAX_CMD_LEN                 400U
#define USB_MAX_TRIES                   10  /* Try this much times to send response on USB */

typedef struct T_MESSAGE
{
    uint8_t Data[MAX_CMD_LEN];
    uint16_t Datalen;
} GenericMessageType;

#define LED_PERIOD      200
typedef enum
{
    SLAVE = 0, MASTER,
} masterSlave_t;

#define HAL_ETH_USE_PTP
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UsbTransmitResponse(uint8_t* pData, uint16_t length);
void UdpDataReceived(uint8_t* pData, uint16_t length);
void TcpDataReceived(uint8_t* pData, uint16_t length);
void UsbDataReceived(uint8_t* pData, uint16_t length);
/*
 * LL SPI reception callback.
 */
void SPI_Rx_Callback(SPI_TypeDef *spi);

/*
 * LL SPI error callback.
 */
void SPI_TransferError_Callback(SPI_TypeDef *spi);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DETTX2_Pin GPIO_PIN_2
#define DETTX2_GPIO_Port GPIOE
#define DETRX0_Pin GPIO_PIN_3
#define DETRX0_GPIO_Port GPIOE
#define DETRX1_Pin GPIO_PIN_4
#define DETRX1_GPIO_Port GPIOE
#define DETRX2_Pin GPIO_PIN_5
#define DETRX2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOF
#define LED5_Pin GPIO_PIN_1
#define LED5_GPIO_Port GPIOF
#define LED6_Pin GPIO_PIN_2
#define LED6_GPIO_Port GPIOF
#define LED7_Pin GPIO_PIN_3
#define LED7_GPIO_Port GPIOF
#define LED8_Pin GPIO_PIN_4
#define LED8_GPIO_Port GPIOF
#define LED9_Pin GPIO_PIN_5
#define LED9_GPIO_Port GPIOF
#define LED10_Pin GPIO_PIN_6
#define LED10_GPIO_Port GPIOF
#define LED11_Pin GPIO_PIN_7
#define LED11_GPIO_Port GPIOF
#define LED12_Pin GPIO_PIN_8
#define LED12_GPIO_Port GPIOF
#define LED13_Pin GPIO_PIN_9
#define LED13_GPIO_Port GPIOF
#define LED14_Pin GPIO_PIN_10
#define LED14_GPIO_Port GPIOF
#define LED15_Pin GPIO_PIN_0
#define LED15_GPIO_Port GPIOC
#define LED16_Pin GPIO_PIN_2
#define LED16_GPIO_Port GPIOC
#define LED17_Pin GPIO_PIN_3
#define LED17_GPIO_Port GPIOC
#define VPWR_Pin GPIO_PIN_0
#define VPWR_GPIO_Port GPIOA
#define V1V2_Pin GPIO_PIN_3
#define V1V2_GPIO_Port GPIOA
#define V3V3_Pin GPIO_PIN_4
#define V3V3_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_5
#define AI2_GPIO_Port GPIOA
#define V5V_Pin GPIO_PIN_0
#define V5V_GPIO_Port GPIOB
#define VUSB_Pin GPIO_PIN_1
#define VUSB_GPIO_Port GPIOB
#define G_INT_Pin GPIO_PIN_2
#define G_INT_GPIO_Port GPIOB
#define LED18_Pin GPIO_PIN_11
#define LED18_GPIO_Port GPIOF
#define AI1_Pin GPIO_PIN_12
#define AI1_GPIO_Port GPIOF
#define LIN_MASTER_Pin GPIO_PIN_13
#define LIN_MASTER_GPIO_Port GPIOF
#define LED19_Pin GPIO_PIN_14
#define LED19_GPIO_Port GPIOF
#define LIN_CS_Pin GPIO_PIN_15
#define LIN_CS_GPIO_Port GPIOF
#define G_RST_Pin GPIO_PIN_0
#define G_RST_GPIO_Port GPIOG
#define EEPROM_SPI_CS_Pin GPIO_PIN_11
#define EEPROM_SPI_CS_GPIO_Port GPIOE
#define CAN2STB_Pin GPIO_PIN_12
#define CAN2STB_GPIO_Port GPIOE
#define CAN1STB_Pin GPIO_PIN_13
#define CAN1STB_GPIO_Port GPIOE
#define LED21_Pin GPIO_PIN_14
#define LED21_GPIO_Port GPIOE
#define LED_RJ45_KO_Pin GPIO_PIN_15
#define LED_RJ45_KO_GPIO_Port GPIOE
#define LED_RJ45_KG_Pin GPIO_PIN_10
#define LED_RJ45_KG_GPIO_Port GPIOB
#define EEPROM_SPI_MISO_Pin GPIO_PIN_14
#define EEPROM_SPI_MISO_GPIO_Port GPIOB
#define EEPROM_SPI_MOSI_Pin GPIO_PIN_15
#define EEPROM_SPI_MOSI_GPIO_Port GPIOB
#define DBG0_Pin GPIO_PIN_8
#define DBG0_GPIO_Port GPIOD
#define DEFAULT_SET_Pin GPIO_PIN_9
#define DEFAULT_SET_GPIO_Port GPIOD
#define ETH_BOOT_Pin GPIO_PIN_10
#define ETH_BOOT_GPIO_Port GPIOD
#define DIP0_Pin GPIO_PIN_12
#define DIP0_GPIO_Port GPIOD
#define DIP1_Pin GPIO_PIN_13
#define DIP1_GPIO_Port GPIOD
#define DIP2_Pin GPIO_PIN_14
#define DIP2_GPIO_Port GPIOD
#define DIP3_Pin GPIO_PIN_15
#define DIP3_GPIO_Port GPIOD
#define LED26_Pin GPIO_PIN_2
#define LED26_GPIO_Port GPIOG
#define LED27_Pin GPIO_PIN_3
#define LED27_GPIO_Port GPIOG
#define LED23_Pin GPIO_PIN_4
#define LED23_GPIO_Port GPIOG
#define LED24_Pin GPIO_PIN_5
#define LED24_GPIO_Port GPIOG
#define LED25_Pin GPIO_PIN_6
#define LED25_GPIO_Port GPIOG
#define LED22_Pin GPIO_PIN_7
#define LED22_GPIO_Port GPIOG
#define LED20_Pin GPIO_PIN_6
#define LED20_GPIO_Port GPIOC
#define SDCD_Pin GPIO_PIN_7
#define SDCD_GPIO_Port GPIOC
#define SWITCH_RST_Pin GPIO_PIN_10
#define SWITCH_RST_GPIO_Port GPIOA
#define LIN_WAKE_Pin GPIO_PIN_15
#define LIN_WAKE_GPIO_Port GPIOA
#define SWITCH_CS_Pin GPIO_PIN_3
#define SWITCH_CS_GPIO_Port GPIOD
#define EWAKE_Pin GPIO_PIN_4
#define EWAKE_GPIO_Port GPIOD
#define E_INH_Pin GPIO_PIN_9
#define E_INH_GPIO_Port GPIOG
#define E_RST_Pin GPIO_PIN_10
#define E_RST_GPIO_Port GPIOG
#define E_INT0_Pin GPIO_PIN_14
#define E_INT0_GPIO_Port GPIOG
#define E_EN_Pin GPIO_PIN_15
#define E_EN_GPIO_Port GPIOG
#define E_INT1_Pin GPIO_PIN_4
#define E_INT1_GPIO_Port GPIOB
#define DO2_Pin GPIO_PIN_5
#define DO2_GPIO_Port GPIOB
#define DO2_FLG_Pin GPIO_PIN_8
#define DO2_FLG_GPIO_Port GPIOB
#define DO1_Pin GPIO_PIN_9
#define DO1_GPIO_Port GPIOB
#define DETTX0_Pin GPIO_PIN_0
#define DETTX0_GPIO_Port GPIOE
#define DETTX1_Pin GPIO_PIN_1
#define DETTX1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define KSZ_ADDR                    0x7
#define TJA1P0                      0x2
#define TJA1P1                      0x3
#define TJA2P0                      0x4

#define READ_DIP0()             !HAL_GPIO_ReadPin(DIP0_GPIO_Port, DIP0_Pin)
#define READ_DIP1()             !HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin)
#define READ_DIP2()             !HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin)
#define READ_DIP3()             !HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin)

#define LED0_GREEN_ON()         HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)
#define LED0_GREEN_OFF()        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)
#define LED1_RED_ON()           HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_RED_OFF()          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED2_GREEN_ON()         HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_GREEN_OFF()        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED3_RED_ON()           HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_RED_OFF()          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED4_GREEN_ON()         HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED4_GREEN_OFF()        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED5_RED_ON()           HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET)
#define LED5_RED_OFF()          HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET)
#define LED6_RED_ON()           HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET)
#define LED6_RED_OFF()          HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET)
#define LED7_GREEN_ON()         HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET)
#define LED7_GREEN_OFF()        HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET)
#define LED8_RED_ON()           HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET)
#define LED8_RED_OFF()          HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET)
#define LED9_GREEN_ON()         HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_RESET)
#define LED9_GREEN_OFF()        HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET)
#define LED10_RED_ON()          HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_RESET)
#define LED10_RED_OFF()         HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_SET)
#define LED11_GREEN_ON()        HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, GPIO_PIN_RESET)
#define LED11_GREEN_OFF()       HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, GPIO_PIN_SET)
#define LED12_RED_ON()          HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, GPIO_PIN_RESET)
#define LED12_RED_OFF()         HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, GPIO_PIN_SET)
#define LED13_GREEN_ON()        HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_RESET)
#define LED13_GREEN_OFF()       HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_SET)
#define LED14_GREEN_ON()        HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_RESET)
#define LED14_GREEN_OFF()       HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_SET)
#define LED15_RED_ON()          HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_RESET)
#define LED15_RED_OFF()         HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_SET)
#define LED16_GREEN_ON()        HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_RESET)
#define LED16_GREEN_OFF()       HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_SET)
#define LED17_RED_ON()          HAL_GPIO_WritePin(LED17_GPIO_Port, LED17_Pin, GPIO_PIN_RESET)
#define LED17_RED_OFF()         HAL_GPIO_WritePin(LED17_GPIO_Port, LED17_Pin, GPIO_PIN_SET)
#define LED18_GREEN_ON()        HAL_GPIO_WritePin(LED18_GPIO_Port, LED18_Pin, GPIO_PIN_RESET)
#define LED18_GREEN_OFF()       HAL_GPIO_WritePin(LED18_GPIO_Port, LED18_Pin, GPIO_PIN_SET)
#define LED19_RED_ON()          HAL_GPIO_WritePin(LED19_GPIO_Port, LED19_Pin, GPIO_PIN_RESET)
#define LED19_RED_OFF()         HAL_GPIO_WritePin(LED19_GPIO_Port, LED19_Pin, GPIO_PIN_SET)
#define LED20_RED_ON()          HAL_GPIO_WritePin(LED20_GPIO_Port, LED20_Pin, GPIO_PIN_RESET)
#define LED20_RED_OFF()         HAL_GPIO_WritePin(LED20_GPIO_Port, LED20_Pin, GPIO_PIN_SET)
#define LED21_GREEN_ON()        HAL_GPIO_WritePin(LED21_GPIO_Port, LED21_Pin, GPIO_PIN_RESET)
#define LED21_GREEN_OFF()       HAL_GPIO_WritePin(LED21_GPIO_Port, LED21_Pin, GPIO_PIN_SET)
#define LED22_RED_ON()          HAL_GPIO_WritePin(LED22_GPIO_Port, LED22_Pin, GPIO_PIN_RESET)
#define LED22_RED_OFF()         HAL_GPIO_WritePin(LED22_GPIO_Port, LED22_Pin, GPIO_PIN_SET)
#define LED23_GREEN_ON()        HAL_GPIO_WritePin(LED23_GPIO_Port, LED23_Pin, GPIO_PIN_RESET)
#define LED23_GREEN_OFF()       HAL_GPIO_WritePin(LED23_GPIO_Port, LED23_Pin, GPIO_PIN_SET)
#define LED24_RED_ON()          HAL_GPIO_WritePin(LED24_GPIO_Port, LED24_Pin, GPIO_PIN_RESET)
#define LED24_RED_OFF()         HAL_GPIO_WritePin(LED24_GPIO_Port, LED24_Pin, GPIO_PIN_SET)
#define LED25_GREEN_ON()        HAL_GPIO_WritePin(LED25_GPIO_Port, LED25_Pin, GPIO_PIN_RESET)
#define LED25_GREEN_OFF()       HAL_GPIO_WritePin(LED25_GPIO_Port, LED25_Pin, GPIO_PIN_SET)
#define LED26_RED_ON()          HAL_GPIO_WritePin(LED26_GPIO_Port, LED26_Pin, GPIO_PIN_RESET)
#define LED26_RED_OFF()         HAL_GPIO_WritePin(LED26_GPIO_Port, LED26_Pin, GPIO_PIN_SET)
#define LED27_GREEN_ON()        HAL_GPIO_WritePin(LED27_GPIO_Port, LED27_Pin, GPIO_PIN_RESET)
#define LED27_GREEN_OFF()       HAL_GPIO_WritePin(LED27_GPIO_Port, LED27_Pin, GPIO_PIN_SET)

#define G_PHY_ON()              HAL_GPIO_WritePin(G_RST_GPIO_Port, G_RST_Pin, GPIO_PIN_SET)
#define G_PHY_OFF()             HAL_GPIO_WritePin(G_RST_GPIO_Port, G_RST_Pin, GPIO_PIN_RESET)
#define SWITCH_ON()             HAL_GPIO_WritePin(SWITCH_RST_GPIO_Port, SWITCH_RST_Pin, GPIO_PIN_SET)
#define SWITCH_RST_OFF()        HAL_GPIO_WritePin(SWITCH_RST_GPIO_Port, SWITCH_RST_Pin, GPIO_PIN_RESET)
#define T1_PHYS_RST_OFF()       HAL_GPIO_WritePin(E_RST_GPIO_Port, E_RST_Pin, GPIO_PIN_SET)
#define T1_PHYS_RST()           HAL_GPIO_WritePin(E_RST_GPIO_Port, E_RST_Pin, GPIO_PIN_RESET)
#define T1_PHYS_ON()            HAL_GPIO_WritePin(E_EN_GPIO_Port, E_EN_Pin, GPIO_PIN_SET)
#define T1_PHYS_OFF()           HAL_GPIO_WritePin(E_EN_GPIO_Port, E_EN_Pin, GPIO_PIN_RESET)
#define T1_WAKE_UP()            HAL_GPIO_WritePin(EWAKE_GPIO_Port, EWAKE_Pin, GPIO_PIN_SET)
#define T1_SLEEP()              HAL_GPIO_WritePin(EWAKE_GPIO_Port, EWAKE_Pin, GPIO_PIN_RESET)
#define SWITCH_CS_RESET()       HAL_GPIO_WritePin(SWITCH_CS_GPIO_Port, SWITCH_CS_Pin, GPIO_PIN_RESET)
#define SWITCH_CS_SET()         HAL_GPIO_WritePin(SWITCH_CS_GPIO_Port, SWITCH_CS_Pin, GPIO_PIN_SET)

#define DO2_HIGH_STATE()        HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET)
#define DO2_Z_STATE()           HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET)
#define DO1_LOW_STATE()         HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET)
#define DO1_Z_STATE()           HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET)

#define CAN1_STBY()             HAL_GPIO_WritePin(CAN1STB_GPIO_Port, CAN1STB_Pin, GPIO_PIN_SET)
#define CAN2_STBY()             HAL_GPIO_WritePin(CAN2STB_GPIO_Port, CAN2STB_Pin, GPIO_PIN_SET)
#define CAN1_ON()               HAL_GPIO_WritePin(CAN1STB_GPIO_Port, CAN1STB_Pin, GPIO_PIN_RESET)
#define CAN2_ON()               HAL_GPIO_WritePin(CAN2STB_GPIO_Port, CAN2STB_Pin, GPIO_PIN_RESET)

#define LIN_WAKE()              HAL_GPIO_WritePin(LIN_WAKE_GPIO_Port, LIN_WAKE_Pin, GPIO_PIN_SET)
#define LIN_SLEEP()             HAL_GPIO_WritePin(LIN_WAKE_GPIO_Port, LIN_WAKE_Pin, GPIO_PIN_RESET)
#define LIN_CS_ON()             HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_SET)
#define LIN_CS_OFF()            HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_RESET)
#define LIN_MASTER()            HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_SET)
#define LIN_SLAVE()             HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_RESET)
#define LED_LIN_RED_OFF()       LED19_RED_OFF()
#define LED_LIN_RED_ON()        LED19_RED_ON()
#define LED_LIN_GREEN_OFF()     LED18_GREEN_OFF()
#define LED_LIN_GREEN_ON()      LED18_GREEN_ON()

#define LED_ETH_ORANGE_ON()     HAL_GPIO_WritePin(LED_RJ45_KO_GPIO_Port, LED_RJ45_KO_Pin, GPIO_PIN_RESET)
#define LED_ETH_ORANGE_OFF()    HAL_GPIO_WritePin(LED_RJ45_KO_GPIO_Port, LED_RJ45_KO_Pin, GPIO_PIN_SET)
#define LED_ETH_GREEN_ON()      HAL_GPIO_WritePin(LED_RJ45_KG_GPIO_Port, LED_RJ45_KG_Pin, GPIO_PIN_RESET)
#define LED_ETH_GREEN_OFF()     HAL_GPIO_WritePin(LED_RJ45_KG_GPIO_Port, LED_RJ45_KG_Pin, GPIO_PIN_SET)

#define LED_T1_1_GREEN_ON()     LED13_GREEN_ON()
#define LED_T1_1_GREEN_OFF()    LED13_GREEN_OFF()
#define LED_T1_1_RED_ON()       LED12_RED_ON()
#define LED_T1_1_RED_OFF()      LED12_RED_OFF()
#define LED_T1_2_GREEN_ON()     LED11_GREEN_ON()
#define LED_T1_2_GREEN_OFF()    LED11_GREEN_OFF()
#define LED_T1_2_RED_ON()       LED10_RED_ON()
#define LED_T1_2_RED_OFF()      LED10_RED_OFF()
#define LED_T1_3_GREEN_ON()     LED9_GREEN_ON()
#define LED_T1_3_GREEN_OFF()    LED9_GREEN_OFF()
#define LED_T1_3_RED_ON()       LED8_RED_ON()
#define LED_T1_3_RED_OFF()      LED8_RED_OFF()
#define LED_LG_4_GREEN_ON()     LED7_GREEN_ON()
#define LED_LG_4_GREEN_OFF()    LED7_GREEN_OFF()
#define LED_LG_4_RED_ON()       LED6_RED_ON()
#define LED_LG_4_RED_OFF()      LED6_RED_OFF()

#define LED_T1_1_MASTER_LED_GREEN_ON()      LED0_GREEN_ON()
#define LED_T1_1_MASTER_LED_GREEN_OFF()     LED0_GREEN_OFF()
#define LED_T1_2_MASTER_LED_GREEN_ON()      LED2_GREEN_ON()
#define LED_T1_2_MASTER_LED_GREEN_OFF()     LED2_GREEN_OFF()
#define LED_T1_3_MASTER_LED_GREEN_ON()      LED4_GREEN_ON()
#define LED_T1_3_MASTER_LED_GREEN_OFF()     LED4_GREEN_OFF()

#define LED_T1_1_MASTER_LED_RED_ON()        LED26_RED_ON()
#define LED_T1_1_MASTER_LED_RED_OFF()       LED26_RED_OFF()
#define LED_T1_2_MASTER_LED_RED_ON()        LED24_RED_ON()
#define LED_T1_2_MASTER_LED_RED_OFF()       LED24_RED_OFF()
#define LED_T1_3_MASTER_LED_RED_ON()        LED22_RED_ON()
#define LED_T1_3_MASTER_LED_RED_OFF()       LED22_RED_OFF()

#define LED_CAN1_GREEN_ON()                 LED14_GREEN_ON()
#define LED_CAN1_GREEN_OFF()                LED14_GREEN_OFF()
#define LED_CAN1_RED_ON()                   LED15_RED_ON()
#define LED_CAN1_RED_OFF()                  LED15_RED_OFF()

#define LED_CAN2_GREEN_ON()                 LED16_GREEN_ON()
#define LED_CAN2_GREEN_OFF()                LED16_GREEN_OFF()
#define LED_CAN2_RED_ON()                   LED17_RED_ON()
#define LED_CAN2_RED_OFF()                  LED17_RED_OFF()

#define LED_CAN1_GREEN_SET(x)                HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, (x))
#define LED_CAN1_RED_SET(x)                  HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, (x))
#define LED_CAN2_GREEN_SET(x)                HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, (x))
#define LED_CAN2_RED_SET(x)                  HAL_GPIO_WritePin(LED17_GPIO_Port, LED17_Pin, (x))

#define READ_ETH_BOOT()         !HAL_GPIO_ReadPin(ETH_BOOT_GPIO_Port, ETH_BOOT_Pin)

//Logical values
#define TJA1_PORT       1
#define TJA2_PORT       2
#define TJA3_PORT       3
#define KSZ_PORT        4
#define MCU_PORT        5
/* This convert the port number in binary form for VLAN port assignment*/

#define PORT_TO_BIN(PORT)    (1 << (PORT))

#define SWITCH_ID           0

#define SENSE_INPUTS        7

#define SENSE_VBAT_DIVIDER      (2.0 / 1)
#define SENSE_VBAT_MCU_DIVIDER  (3.0 / 1)
#define SENSE_VUSB_DIVIDER      (2.0 / 1)
#define ADC_CONVERSION(VDDA)    ((VDDA)/ 4095)   // 12 bit ADC with 3.29V reference

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
