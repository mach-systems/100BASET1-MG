/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/*
 *  * Product: MACH ETH
 *
 * Company: MACH SYSTEMS s.r.o.
 * Company web: www.machsystems.cz
 *
 * Product description:
 * 3x 100BASE-T1 port
 * 1x 1000BASE-T port (gigabit Ethernet)
 * All Ethernet ports are „switched“ (including the MCU)
 * 2x CAN(/FD)
 * LIN bus
 * USB 2.0
 * MicroSD card slot
 * 2x Digital output (1x high-side 5V/0.5A, 1x low-side 40V/1A)
 * 2x Analogue input (0-30 V)
 * Embedded web server for configuration and status information
 * User-programmable MCU (C language SDK available free-of-charge)
 * Use-cases:
 *  * Configurable Automotive Ethernet switch
 *  * Active TAP (frame forwarding/sniffing)
 *  * Gateway between Ethernet, CAN(/FD), LIN
 *  * User-programmable gateway / simulator
 *
 * MCU: STM32H723ZGT
 * IDE: STM32CubeIDE v1.16.1
 * HAL: STM32Cube FW_H7 V1.11.2
 *
 * Connector pinout of the device:
 * ===============================
 * Connector X1 (D-SUB Male):
 *   - 1 DO_2 HS switch (5V/1A)
 *   - 2 CAN1 L
 *   - 3 GND
 *   - 4 CAN2 L
 *   - 5 SHIELD
 *   - 6 LIN
 *   - 7 CAN1 H
 *   - 8 CAN2 H
 *   - 9 AI_2 (0-30V)
 *
 * Connector X2 (D-SUB Male):
 *   - 1 T1_1_P
 *   - 2 T1_1_N
 *   - 3 GND
 *   - 4 T1_2_P
 *   - 5 T1_2_N
 *   - 6 T1_3_P
 *   - 7 T1_3_N
 *   - 8 AI_1 (0-30V)
 *   - 9 DO_1 LS switch (35V/1A)
 *
 * SJA PORTS PHYSICAL (first is 1):
 *      MCU_PORT ...... 1
 *      TJA2_PORT ..... 2
 *      TJA3_PORT ..... 3
 *      TJA1_PORT ..... 4
 *      KSZ_PORT ...... 5
 *
 * The conversion from logical to physical ports is done in NXP_SJA1105P_config.c and affects resolution table and
 * Others SJA function works only with physical ports and the conversion must be done manually. In SwitchConfStruct the ports are stored in virtual order.
 * SJA PORTS VIRTUAL (first is 1):
 *      TJA1_PORT ..... 1
 *      TJA2_PORT ..... 2
 *      TJA3_PORT ..... 3
 *      KSZ_PORT ...... 4
 *      MCU_PORT ...... 5
 *
 * TJA to PORTS:
 *      T1_1 .......... 4
 *      T1_2 .......... 2
 *      T1_3 .......... 3
 *
 * TJA connection:
 *          1st TJA1102A P0 ..  T1_1
 *          1st TJA1102A P1 ..  T1_3
 *          2nd TJA1102A P0 ..  T1_2
 *
 * SMI devices addresses:
 *          KSZ9131 ..........  0x7
 *          1st TJA1102A P0 ..  0x2
 *          1st TJA1102A P1 ..  0x3
 *          2nd TJA1102A P0 ..  0x4
 *
 *          CAN1 -> FDCAN1
 *          CAN2 -> FDCAN2
 *
 *          LIN -> USART2
 *          LIN_TIMEOUT_TIMER -> TIM7
 *          HTTP_TIMER -> TIM1
 *          UNIVERSAL MS TIMER -> TIM6
 *          millisecond MACH protocol timeout timer -> TIM16
 *
 *
 * List of important Callbacks:
 *      LIN:
 *          void LinErrorCallback(uint8_t linId, uint8_t error);
 *          void SlaveResponseTxRxCallback(SlaveResponseDataStruct* pInputData);
 *          void LinChannelStoppedCallback();
 *          void MasterRequestTxRxCallback(LIN_Frame* pFrame);
 *          void MasterResponseTxCallback(LIN_Frame* pFrame);
 *
 *      CAN:
 *          void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);
 *          void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
 *          void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
 *
 *      TCP and UDP:
 *          void TcpDataReceived(uint8_t* pData, uint16_t length);
 *          void UdpDataReceived(uint8_t* pData, uint16_t length);
 *          void UsbDataReceived(uint8_t* pData, uint16_t length);
 *
 * Data can be send with functions:
 *      LIN: uint8_t LinWriteToTxBuffer(uint8_t linId,uint8_t msgType,uint8_t chksumType, uint8_t length, uint8_t data[]);
 *      USB: void UsbTransmitResponse(uint8_t* pData, uint16_t length);
 *      TCP: osStatus_t TcpEnqueueResponse(uint8_t* pData, uint16_t length);
 *      UDP: osStatus_t UdpEnqueueResponse(uint8_t* pData, uint16_t length);
 *      CAN: uint8_t CanSendMessage(CanMessageStruct *message);
 *
 *  * CAN-FD:
 *  Default setting for CAN is 500k, 2M, SP = 80 %.
 *  Everything received on CAN is sent to USB, UDP and to TCP connection (if there
 *  is).
 *
 *  * Bootloader:
 * CAN frame with extended frame ID 0x1fffffff and first four data bytes 0x00
 * 0x01 0x02 0x03 (on CAN1 or CAN2) will reset the device to HTTP bootloader. Also
 * shorting the ETH_B and GND pins cause entering the bootloader.
 * If you then connect to the device via Ethernet and connect from web browser
 * (Google Chrome recommended), you can upload new firmware to the device. If
 * the binary is built as without bootloader, device will reset to System
 * Bootloader. If you then connect USB cable, you can upload new firmware. See
 * below for build possibilities. Furthermore, see manual for more info.
 *
 *  * Virtual COM Port:
 * If there is anything received, there is a frame transmitted on CAN1.
 *
 *  * EEPROM:
 * There is running an EEPROM emulation in flash. Program tries to load conf-
 * iguration from there.
 *
 *  * Analog Input:
 * There is one analog input that can measure 0 to 5 V. You can get its value
 * using GetAdcMeasurement().
 *
 *  * Output:
 *  There is one 5 V HS switch (DO_2) and one LS switch (DO_1). There are macros defined in main.h
 *  DO2_HIGH_STATE()  DO2_Z_STATE()
 *  DO1_LOW_STATE() DO1_Z_STATE()
 *
 * NOTE: When cache is enabled, in Middlewares/Third_Party/FatFs/src/ff.h,
 * there must be set changed lines 125 and 170:
 * 125: BYTE    win[_MAX_SS] __attribute__((aligned(32)));
 * 170: BYTE   buf[_MAX_SS] __attribute__((aligned(32)));
 * (else there is FR_INT_ERR)
 * Credit
 *   community.st.com/s/question/0D50X00009sTr0fSAC/stm32f7xx-cubemx-sd-disk-io-dma-with-rtos-driver-sdread-function-invalid-address-access
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NXP_SJA1105P_spi.h"
#include "NXP_SJA1105P_config.h"
#include "NXP_SJA1105P_switchCore.h"
#include "NXP_SJA1105P_diagnostics.h"
#include "NXP_SJA1105P_ethIf.h"
#include "NXP_SJA1105P_addressResolutionTable.h"
#include "string.h"
#include "tools.h"
#include "kszSetting.h"
#include "sjaSetting.h"
#include "lwip/udp.h"
#include <string.h>
#include "tjaSetting.h"
#include "adc.h"
#include "lin.h"
#include "configuration.h"
#include "tcpServer.h"
#include "can.h"
#include "switch.h"
#include "config.h"
#include "sharedParams.h"
#include "udpServer.h"
#include "eeprom.h"
#include "eepromConfig.h"
#include <usbd_cdc_if.h>    /* CDC_Transmit_HS() */
#include "timestamp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// For LATENCY 2 WS, WRHIGHFREQ should be set to 2 (page 161 in RM0468)
#define FLASH_PROGRAM_DELAY     FLASH_PROGRAMMING_DELAY_2

#define SYSTEM_BOOT_ADDR        0x1FF09800
#define HTTP_BOOT_ADDR          0x08000000

#define BOOT_DELAY              100       /* 100 millisecond delay before device is
                                           initialized (just to be sure on devi-
                                           ces where the reset circuit was re-
                                           moved) */

#define SHARED_RAM_USB_JUMP     11  /* Address in the shared memory section that is used when
                                       jumping to System Bootloader */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ITCM_HEAP     65520
#define DTCM_HEAP     127936
// RAM_D2 and RAM_D3 seemed not to work

static uint8_t __attribute__((section(".itcm"))) ucHeapItcm[ITCM_HEAP];
static uint8_t __attribute__((section(".dtcm"))) ucHeapDtcm[DTCM_HEAP];
static uint8_t ucHeap[configTOTAL_HEAP_SIZE];   // 32768

// Must be sorted by memory start, from lowest to highest
HeapRegion_t usrHeapRegions[] = {
  { ucHeapItcm, ITCM_HEAP         },
  { ucHeapDtcm, DTCM_HEAP         },
  { ucHeap, configTOTAL_HEAP_SIZE },
  { NULL,   0                     }
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DTS_HandleTypeDef hdts;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim24;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fatFsTask */
osThreadId_t fatFsTaskHandle;
const osThreadAttr_t fatFsTask_attributes = {
  .name = "fatFsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for checkDeviceStat */
osThreadId_t checkDeviceStatHandle;
const osThreadAttr_t checkDeviceStat_attributes = {
  .name = "checkDeviceStat",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usbProtoTaskRx */
osThreadId_t usbProtoTaskRxHandle;
const osThreadAttr_t usbProtoTaskRx_attributes = {
  .name = "usbProtoTaskRx",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for usbProtoTaskTx */
osThreadId_t usbProtoTaskTxHandle;
const osThreadAttr_t usbProtoTaskTx_attributes = {
  .name = "usbProtoTaskTx",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UsbTxQueue */
osMessageQueueId_t UsbTxQueueHandle;
const osMessageQueueAttr_t UsbTxQueue_attributes = {
  .name = "UsbTxQueue"
};
/* Definitions for UsbRxQueue */
osMessageQueueId_t UsbRxQueueHandle;
const osMessageQueueAttr_t UsbRxQueue_attributes = {
  .name = "UsbRxQueue"
};
/* Definitions for TcpTxQueue */
osMessageQueueId_t TcpTxQueueHandle;
const osMessageQueueAttr_t TcpTxQueue_attributes = {
  .name = "TcpTxQueue"
};
/* Definitions for UdpTxQueue */
osMessageQueueId_t UdpTxQueueHandle;
const osMessageQueueAttr_t UdpTxQueue_attributes = {
  .name = "UdpTxQueue"
};
/* Definitions for tableMutex */
osMutexId_t tableMutexHandle;
const osMutexAttr_t tableMutex_attributes = {
  .name = "tableMutex"
};
/* Definitions for ajaxMutex */
osMutexId_t ajaxMutexHandle;
const osMutexAttr_t ajaxMutex_attributes = {
  .name = "ajaxMutex"
};
/* Definitions for hethMutex */
osMutexId_t hethMutexHandle;
const osMutexAttr_t hethMutex_attributes = {
  .name = "hethMutex"
};
/* Definitions for spiSjaMutex */
osMutexId_t spiSjaMutexHandle;
const osMutexAttr_t spiSjaMutex_attributes = {
  .name = "spiSjaMutex"
};
/* USER CODE BEGIN PV */
uint8_t LinGlobalData[LIN_DATA_LENGTH];
uint16_t CounterLed;

uint8_t DigitalOutput1State;
uint8_t DigitalOutput2State;
/* Request to go to bootloader */
uint8_t BootloaderRequest = 0;
uint8_t GlobalErrorFlags = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM24_Init(void);
static void MX_DTS_Init(void);
static void MX_IWDG1_Init(void);
void StartDefaultTask(void *argument);
void FatFsTask(void *argument);
void CheckDeviceStatus(void *argument);
extern void UsbProtocolTaskRx(void *argument);
extern void UsbProtocolTaskTx(void *argument);

/* USER CODE BEGIN PFP */
static void getDipStatus(dip_stat * dipswitchStatus);
volatile unsigned long ulHighFrequencyTimerTicks;

/*
 * Jump to the system bootloader.
 * Almost entirely from:
 * https://community.st.com/s/article/STM32H7-bootloader-jump-from-application
 * Other source:
 * https://www.programmersought.com/article/56553023460/
 * https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
 */
void jumpToBootloader(uint32_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    if (GetSharedData(SHARED_RAM_USB_JUMP) == 1)
    {
      // Should not go here - jump is to be done by the bootloader (except for the NoBootloader config)
      SetSharedData(SHARED_RAM_USB_JUMP, 0);
      jumpToBootloader(SYSTEM_BOOT_ADDR);
    }
    hiwdg1.Instance = IWDG1;
    WDG_REFRESH();    // Watchdog might have been enabled by the bootloader
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN2_Init();
  MX_SDMMC1_SD_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM24_Init();
  MX_DTS_Init();
  MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(BOOT_DELAY);
  HAL_DTS_Start(&hdts);
  EepromInit();
  InitNonVolatileData();
  TimestampTimerInit();
  /* Power on peripherals */
  G_PHY_ON();
  SWITCH_ON();
  T1_PHYS_ON();
  T1_PHYS_RST_OFF();
  T1_WAKE_UP();
  CAN1_ON();
  CAN2_ON();
  LIN_WAKE();
  AdcInit();
  CanInit();

  /* Wait until all devices are ready*/
  HAL_Delay(300);
  WDG_REFRESH();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of tableMutex */
  tableMutexHandle = osMutexNew(&tableMutex_attributes);

  /* creation of ajaxMutex */
  ajaxMutexHandle = osMutexNew(&ajaxMutex_attributes);

  /* creation of hethMutex */
  hethMutexHandle = osMutexNew(&hethMutex_attributes);

  /* creation of spiSjaMutex */
  spiSjaMutexHandle = osMutexNew(&spiSjaMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UsbTxQueue */
  UsbTxQueueHandle = osMessageQueueNew (16, sizeof(GenericMessageType), &UsbTxQueue_attributes);

  /* creation of UsbRxQueue */
  UsbRxQueueHandle = osMessageQueueNew (16, sizeof(GenericMessageType), &UsbRxQueue_attributes);

  /* creation of TcpTxQueue */
  TcpTxQueueHandle = osMessageQueueNew (64, sizeof(GenericMessageType), &TcpTxQueue_attributes);

  /* creation of UdpTxQueue */
  UdpTxQueueHandle = osMessageQueueNew (64, sizeof(GenericMessageType), &UdpTxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fatFsTask */
  fatFsTaskHandle = osThreadNew(FatFsTask, NULL, &fatFsTask_attributes);

  /* creation of checkDeviceStat */
  checkDeviceStatHandle = osThreadNew(CheckDeviceStatus, NULL, &checkDeviceStat_attributes);

  /* creation of usbProtoTaskRx */
  usbProtoTaskRxHandle = osThreadNew(UsbProtocolTaskRx, NULL, &usbProtoTaskRx_attributes);

  /* creation of usbProtoTaskTx */
  usbProtoTaskTxHandle = osThreadNew(UsbProtocolTaskTx, NULL, &usbProtoTaskTx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 3;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DTS Initialization Function
  * @param None
  * @retval None
  */
static void MX_DTS_Init(void)
{

  /* USER CODE BEGIN DTS_Init 0 */

  /* USER CODE END DTS_Init 0 */

  /* USER CODE BEGIN DTS_Init 1 */

  /* USER CODE END DTS_Init 1 */
  hdts.Instance = DTS;
  hdts.Init.QuickMeasure = DTS_QUICKMEAS_DISABLE;
  hdts.Init.RefClock = DTS_REFCLKSEL_PCLK;
  hdts.Init.TriggerInput = DTS_TRIGGER_HW_NONE;
  hdts.Init.SamplingTime = DTS_SMP_TIME_1_CYCLE;
  hdts.Init.Divider = 0;
  hdts.Init.HighThreshold = 0x0;
  hdts.Init.LowThreshold = 0x0;
  if (HAL_DTS_Init(&hdts) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DTS_Init 2 */

  /* USER CODE END DTS_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 8;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 32;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 2;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 8;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxFifo1ElmtsNbr = 64;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.TxEventsNbr = 32;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */
#if 0
  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */
#endif
  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 10;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_32BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**SPI2 GPIO Configuration
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  PA9   ------> SPI2_SCK
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI2 interrupt Init */
  NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR4;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 39999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 49999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM24 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM24_Init(void)
{

  /* USER CODE BEGIN TIM24_Init 0 */

  /* USER CODE END TIM24_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM24_Init 1 */

  /* USER CODE END TIM24_Init 1 */
  htim24.Instance = TIM24;
  htim24.Init.Prescaler = 199;
  htim24.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim24.Init.Period = 4294967295;
  htim24.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim24.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim24) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim24, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim24, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM24_Init 2 */

  /* USER CODE END TIM24_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**USART2 GPIO Configuration
  PD5   ------> USART2_TX
  PD6   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_SetLINBrkDetectionLen(USART2, LL_USART_LINBREAK_DETECT_10B);
  LL_USART_DisableOverrunDetect(USART2);
  LL_USART_DisableDMADeactOnRxErr(USART2);
  LL_USART_ConfigLINMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED0_Pin|EEPROM_SPI_CS_Pin|CAN2STB_Pin|CAN1STB_Pin
                          |LED21_Pin|LED_RJ45_KO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|LED15_Pin
                          |LED16_Pin|LED17_Pin|LED20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin
                          |LED8_Pin|LED9_Pin|LED10_Pin|LED11_Pin
                          |LED12_Pin|LED13_Pin|LED14_Pin|LED18_Pin
                          |LED19_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LIN_MASTER_Pin|LIN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, G_RST_Pin|E_RST_Pin|E_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RJ45_KG_GPIO_Port, LED_RJ45_KG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DBG0_Pin|SWITCH_CS_Pin|EWAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED26_Pin|LED27_Pin|LED23_Pin|LED24_Pin
                          |LED25_Pin|LED22_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SWITCH_RST_Pin|LIN_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO2_Pin|DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DETTX2_Pin DETRX0_Pin DETRX1_Pin DETRX2_Pin
                           DETTX0_Pin DETTX1_Pin */
  GPIO_InitStruct.Pin = DETTX2_Pin|DETRX0_Pin|DETRX1_Pin|DETRX2_Pin
                          |DETTX0_Pin|DETTX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin CAN2STB_Pin CAN1STB_Pin LED21_Pin
                           LED_RJ45_KO_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|CAN2STB_Pin|CAN1STB_Pin|LED21_Pin
                          |LED_RJ45_KO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED15_Pin
                           LED16_Pin LED17_Pin LED20_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED15_Pin
                          |LED16_Pin|LED17_Pin|LED20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED5_Pin LED6_Pin LED7_Pin
                           LED8_Pin LED9_Pin LED10_Pin LED11_Pin
                           LED12_Pin LED13_Pin LED14_Pin LED18_Pin
                           LIN_MASTER_Pin LED19_Pin LIN_CS_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin
                          |LED8_Pin|LED9_Pin|LED10_Pin|LED11_Pin
                          |LED12_Pin|LED13_Pin|LED14_Pin|LED18_Pin
                          |LIN_MASTER_Pin|LED19_Pin|LIN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : G_INT_Pin E_INT1_Pin */
  GPIO_InitStruct.Pin = G_INT_Pin|E_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : G_RST_Pin LED26_Pin LED27_Pin LED23_Pin
                           LED24_Pin LED25_Pin LED22_Pin E_RST_Pin
                           E_EN_Pin */
  GPIO_InitStruct.Pin = G_RST_Pin|LED26_Pin|LED27_Pin|LED23_Pin
                          |LED24_Pin|LED25_Pin|LED22_Pin|E_RST_Pin
                          |E_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_SPI_CS_Pin */
  GPIO_InitStruct.Pin = EEPROM_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(EEPROM_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RJ45_KG_Pin DO2_Pin DO1_Pin */
  GPIO_InitStruct.Pin = LED_RJ45_KG_Pin|DO2_Pin|DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DBG0_Pin SWITCH_CS_Pin EWAKE_Pin */
  GPIO_InitStruct.Pin = DBG0_Pin|SWITCH_CS_Pin|EWAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DEFAULT_SET_Pin ETH_BOOT_Pin DIP0_Pin DIP1_Pin
                           DIP2_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = DEFAULT_SET_Pin|ETH_BOOT_Pin|DIP0_Pin|DIP1_Pin
                          |DIP2_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDCD_Pin */
  GPIO_InitStruct.Pin = SDCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDCD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_RST_Pin LIN_WAKE_Pin */
  GPIO_InitStruct.Pin = SWITCH_RST_Pin|LIN_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E_INH_Pin E_INT0_Pin */
  GPIO_InitStruct.Pin = E_INH_Pin|E_INT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : DO2_FLG_Pin */
  GPIO_InitStruct.Pin = DO2_FLG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DO2_FLG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void getDipStatus(dip_stat * dipswitchStatus)
{
    dipswitchStatus->dip1 = READ_DIP0();
    dipswitchStatus->dip2 = READ_DIP1();
    dipswitchStatus->dip3 = READ_DIP2();
    dipswitchStatus->dip4 = READ_DIP3();
}

void linTest(void)
{
    LinInitStruct linInit =
    {
            .AMLR = AUTOLEN,
            .Baudrate = BAUDRATE_19200,
            .ChecksumType = CHECKSUM_ENHANCED,
            .DeviceMode = LIN_MODE_MASTER,
            .autoStart = true,
            .payload = 0,
    };
    LIN_Frame frame = {
    .ID = 0x10,
    .ChecksumType = CHECKSUM_ENHANCED,
    .Data = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8},
    .Length = 0x8,
    .Type = MASTER_RESPONSE};

    LinWriteToTxBuffer(frame.ID,frame.Type ,frame.ChecksumType, frame.Length, frame.Data);
    InitLin(&linInit);
}

void TcpDataReceived(uint8_t* pData, uint16_t length)
{
  length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  SendToCan(pData, length);
}

void UdpDataReceived(uint8_t* pData, uint16_t length)
{
  length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  SendToCan(pData, length);
}

void UsbProtocolTaskRx(void* arg)
{
  while (((USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData)->TxState != 0)
    osDelay(1000);
  while (1)
  {
    GenericMessageType usbRxBuffer;     /* Reception buffer */
    osStatus_t queueState;

    /* Get received data from queue */
    while ((queueState = osMessageQueueGet(UsbRxQueueHandle, &usbRxBuffer, NULL, 0xffff)) != osOK);
    uint16_t length = usbRxBuffer.Datalen > MAX_CMD_LEN ? MAX_CMD_LEN : usbRxBuffer.Datalen;
    SendToCan(usbRxBuffer.Data, length);
  }
}

void UsbDataReceived(uint8_t* pData, uint16_t length)
{
  GenericMessageType usbRxBuffer;
  length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  memcpy(usbRxBuffer.Data, pData, length);
  usbRxBuffer.Datalen = length;
  uint32_t timeout = (__get_IPSR() != 0U) ? 0 : QUEUE_PUT_TIMEOUT;
  /* Just put the data to queue */
  /* Should return osStatus_t osOK. If not, Flash write is probably ongoing - user's problem */
  osMessageQueuePut(UsbRxQueueHandle, &usbRxBuffer, 0, timeout);
}

void UsbProtocolTaskTx(void* arg)
{
  while (1)
  {
    uint8_t timeout = 0;
    GenericMessageType usbTxBuffer;     /* Transmission buffer */
    osStatus_t queueState;

    /* Get received data from queue */
    while ((queueState = osMessageQueueGet(UsbTxQueueHandle, &usbTxBuffer, NULL, 0xffff)) != osOK);
    uint16_t length = usbTxBuffer.Datalen > MAX_CMD_LEN ? MAX_CMD_LEN : usbTxBuffer.Datalen;
    // If fail - USB probably not connected yet
    while (CDC_Transmit_HS(usbTxBuffer.Data, length) != 0 && timeout++ < USB_MAX_TRIES);
  }
}

void UsbTransmitResponse(uint8_t* pData, uint16_t length)
{
    GenericMessageType usbTxBuffer;
    length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
    memcpy(usbTxBuffer.Data, pData, length);
    usbTxBuffer.Datalen = length;
    uint32_t timeout = (__get_IPSR() != 0U) ? 0 : QUEUE_PUT_TIMEOUT;
    osMessageQueuePut(UsbTxQueueHandle, &usbTxBuffer, 0, timeout);
}

void jumpToBootloader(uint32_t address)
{


  uint32_t i=0;
  void (*SysMemBootJump)(void);

  /* Set the address of the entry point to bootloader */
  volatile uint32_t BootAddr = address;

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Deinitialize all the peripherals */
  HAL_DeInit();

  /* Must disable the caches! */
  SCB_DisableDCache();
  SCB_DisableICache();

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  uint8_t cnt = (sizeof(NVIC->ICER) / sizeof(*NVIC->ICER));
  for (i = 0; i < cnt; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* Set vector table offset to 0 */
  SCB->VTOR = 0;

  /* Re-enable all interrupts */
  __enable_irq();

  /* Set up the jump to booloader address + 4 */
  SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((BootAddr + 4))));

  /* Set the main stack pointer to the bootloader stack */
  __set_MSP(*(uint32_t *)BootAddr);

  /* RTOS in engineering, this statement is very important to privileged mode, use the MSP pointer */
  __set_CONTROL(0);

  /* Call the function to jump to bootloader location */
  SysMemBootJump();

  /* Jump is done successfully */
  while (1)
  {
    /* Code should never reach this loop */
  }
}
void SPI_Rx_Callback(SPI_TypeDef *spi)
{
    // For some reason, both EOT and TXTF flags must be cleared for SPI to work correctly
    if (LL_SPI_IsActiveFlag_EOT(spi) != 0U)
    {
        LL_SPI_ClearFlag_EOT(spi);
    }
    if (LL_SPI_IsActiveFlag_TXTF(spi) != 0U)
    {
        LL_SPI_ClearFlag_TXTF(spi);
    }

    if (EE_SPI == spi)
    {
        (void) EepromSpiCallback();
    }
}

void SPI_TransferError_Callback(SPI_TypeDef *spi)
{
    LL_SPI_ClearFlag_OVR(spi);
    LL_SPI_ClearFlag_UDR(spi);
    LL_SPI_ClearFlag_CRCERR(spi);
}

void SpiInterruptCallback()
{
    EepromSpiCallback();
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  dip_stat dipswitchStatus;
  KszInit(&heth);
  InitDip(0);
  InitDip(1);
  InitDip(2);
  InitDip(3);
  /* Init T1 PHYs */
  TjaInit(&heth,TJA1P0);
  TjaInit(&heth,TJA1P1);
  TjaInit(&heth,TJA2P0);

  TcpServerInit(TcpTxQueueHandle);
  UdpServerInit();

  WDG_REFRESH();
  //Apply values loaded from EEPROM
  ApplySwitchConfiguration(GetSwitchConfigurationAddr());

  /* Infinite loop */
  for (;;) {
      getDipStatus(&dipswitchStatus);
      SetConfigurationByDipswitch(&heth, dipswitchStatus, GetSwitchConfigurationAddr());
      if (READ_ETH_BOOT())
        BootloaderRequest = 2;
      /* Check jump to bootloader conditions */
      if (1U == BootloaderRequest)
      {
          (void) USBD_DeInit(&hUsbDeviceHS);
          (void) SetSharedData(SHARED_RAM_USB_JUMP, 1);
          HAL_NVIC_SystemReset();
        jumpToBootloader(SYSTEM_BOOT_ADDR);
      }
      else if (2U == BootloaderRequest)
      {
        InitSharedParams();
        SetSharedData(0, 1);
        jumpToBootloader(HTTP_BOOT_ADDR);
      }
      WDG_REFRESH();
      osDelay(500);
  }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_FatFsTask */
/**
* @brief Function implementing the fatFsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FatFsTask */
void FatFsTask(void *argument)
{
  /* USER CODE BEGIN FatFsTask */
  /* Infinite loop */
  for(;;)
  {
   /* FatFs middleware main background task */
    MX_FATFS_Process();
    osDelay(100);
  }
  /* USER CODE END FatFsTask */
}

/* USER CODE BEGIN Header_CheckDeviceStatus */
/**
* @brief Function implementing the checkDeviceStat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CheckDeviceStatus */
void CheckDeviceStatus(void *argument)
{
  /* USER CODE BEGIN CheckDeviceStatus */
  /* Infinite loop */
  for(;;)
  {
    if (TakeHethAccess()){
        TjaIsLinkUp(&heth,TJA1P0);
        TjaIsLinkUp(&heth,TJA1P1);
        TjaIsLinkUp(&heth,TJA2P0);
        TjaReadError(&heth,TJA1P0);
        TjaReadError(&heth,TJA1P1);
        TjaReadError(&heth,TJA2P0);
        TjaIsPolarityInvert(&heth,TJA1P0);
        TjaIsPolarityInvert(&heth,TJA1P1);
        TjaIsPolarityInvert(&heth,TJA2P0);
        TjaGetSqi(&heth,TJA1P0);
        TjaGetSqi(&heth,TJA1P1);
        TjaGetSqi(&heth,TJA2P0);
        IsKszMaster(&heth);
        GiveAccessToHeth();
    }
    osDelay(1000);
  }
  /* USER CODE END CheckDeviceStatus */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM6) {
        CounterLed++;
        if (CounterLed > LED_PERIOD)
            CounterLed = 0;
        TjaSetLedActivity(TJA1P0);
        TjaSetLedActivity(TJA1P1);
        TjaSetLedActivity(TJA2P0);

        /* LED CAN blink control timer tick */
        CanLedTimerCallback();
    }
    if (TIM1 == htim->Instance)
    {

    }
    if (TIM4 == htim->Instance)
    {
        ulHighFrequencyTimerTicks++;
    }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
