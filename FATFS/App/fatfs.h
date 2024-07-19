/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
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
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */

/* USER CODE BEGIN Includes */
#define FATFS_DEBUG_TEST
/* USER CODE END Includes */

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */

void MX_FATFS_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum {
APPLICATION_IDLE = 0, /* Idle state - SD card initialized and mounted/mounting */
APPLICATION_INIT,     /* Init state - before SD card initialization*/
APPLICATION_DEINIT,   /* SD card disconnected - request deinit */
APPLICATION_REINIT,   /* Reinitialization after disconnect/connect */
APPLICATION_CREATE_FILE,  /* Log file creation requested */
APPLICATION_RUNNING,  /* Device is logging */
APPLICATION_CLOSE_FILE,   /* File closing was requested */
APPLICATION_SD_UNPLUGGED, /* SD card is unplugged */
APPLICATION_ERROR,        /* Recoverable error */
APPLICATION_ERROR_PERSISTENT  /* Irrecoverable error - card was removed when device was logging */
} FS_FileOperationsTypeDef;
/*
* Format the drive. It is not needed to call this if there is a valid filesystem
* on the card.
*/
FRESULT FormatDrive(void);
void MX_FATFS_Process(void);
/*
* Initialize listing. MUST be called before file listing.
*/
FRESULT InitListing(DIR* pDir);
/*
* Read next filename (when device is already mounted). Return filename length:
* >0 if success
* == 0 if error or all filenames were already read.
*/
uint8_t GetNextFileName(char* pStr, FSIZE_t* pSize, DIR* pDir);
/*
* Read how many log files are on the SD card. Init and deinit do not have to be
* called (function itself does it).
*/
uint16_t GetLogFileCount(void);
/*
* Convert log file index to filename.
*/
uint8_t LogFileIndexToFilename(uint16_t index, TCHAR* pName);
/*
* Deinitialize file listing. Should be called after listing is done.
*/
FRESULT DeinitListing(DIR* pDir);
/**
* @brief  BSP SD Callback.
* @param  Instance SD Instance
* @param  Status   Pin status
* @retval None.
*/
void BSP_SD_DetectCallback(uint32_t Instance, uint32_t Status);
/*
* @brief  Get current state of FatFs application.
* @param  none
* @retval Current state
*/
FS_FileOperationsTypeDef SD_GetState(void);
/*
* Status defines
*/
#define APP_OK                      0
#define APP_ERROR                   -1
#define APP_SD_UNPLUGGED            -2
#define APP_INIT                    1
#define NO_DATA_TO_WRITE            -3
#define MAX_FILENAME                13
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */
