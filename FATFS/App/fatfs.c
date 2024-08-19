/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
  * Mount fail -> red on
  * Start log without SD inserted -> red on
  * Irrecoverable error -> red blink
  *
  * Logging running -> blink green
  * Playback running -> blink green
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

#include <string.h>
#include <stdlib.h> // itoa
#include "main.h"
// Private typedef
// empty
//#define FATFS_DEBUG_TEST
// Private defines
#define FATFS_MKFS_ALLOWED 0    /* Set to 1 to always start by formatting the drive */
// Private function prototypes
static int32_t FS_FileOperations(void);
#ifdef FATFS_DEBUG_TEST
static int32_t FS_WriteData(void);
#endif // FATFS_DEBUG_TEST
static int32_t FS_CreateFile(uint16_t cnt);
static int32_t FS_CloseFile(void);
static FRESULT FS_Mount(void);
static void SD_Init(void);

/*
* Check that first three characters (if filename is longer than 3) are "log"
*/
static uint8_t checkFilename(TCHAR* pName);
FATFS SDFatFs;    /* File system object for SD logical drive */
FIL SDFile;       /* File  object for SD */
char SDPath[4];   /* SD logical drive path */
FS_FileOperationsTypeDef Appli_state = APPLICATION_INIT;
uint8_t workBuffer[_MAX_SS];
uint8_t mounted = 0; /* Set to 1 if SD Card is mounted */
uint32_t is_initialized = 0;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
int FatFsIterations;
FRESULT FormatDrive(void)
{
return f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
}
#ifndef FATFS_DEBUG_TEST
/**
* @brief  FatFs application main process
* @param  None
* @retval Process result
*/
void MX_FATFS_Process(void)
{
int32_t process_res = APP_OK;
/* Mass Storage Application State Machine */
switch(Appli_state)
{
case APPLICATION_INIT:
  BSP_SD_ITConfig();
  if(BSP_SD_IsDetected() == SD_PRESENT)
  {
      SD_Init();    /* Initialize the SD card */
      //Appli_state = APPLICATION_CREATE_FILE;
      Appli_state = APPLICATION_IDLE;
#if FATFS_MKFS_ALLOWED
    FRESULT res;
    // Disable millisecond timer for f_mkfs (for concurrent SPI transactions
    //  the duration is too long).
    taskENTER_CRITICAL();
    res = f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
    taskEXIT_CRITICAL();
    if (res != FR_OK)
    {
      process_res = APP_ERROR;
    }
    else
    {
      process_res = APP_INIT;
      Appli_state = APPLICATION_RUNNING;
    }
//#else
//        process_res = APP_INIT;
//        Appli_state = APPLICATION_CREATE_FILE;
#endif
  }
  else
  {
    Appli_state = APPLICATION_SD_UNPLUGGED;
  }
  break;
case APPLICATION_DEINIT:
    //f_mount(NULL, (TCHAR const*)"", 0); // Unmount the drive
    //f_mount(NULL, SDPath, 1); // Unmount the drive
    Appli_state = APPLICATION_SD_UNPLUGGED;
    break;
case APPLICATION_REINIT:
    if ((process_res = SD_Reinit()) == 1)
    {
        Appli_state = APPLICATION_IDLE;
        // When error, try it again
    }
    else if (2 == process_res)
        Appli_state = APPLICATION_ERROR;
    break;
case APPLICATION_CREATE_FILE:
  LogSetCloseFileFlag(0);
  uint16_t count = GetLogFileCount();
  while (-2 == (process_res = FS_CreateFile(count)))
  {
      // If by any chance the file exists, skip its number
      count++;
  }
  if (0 != process_res)
  {
      Appli_state = APPLICATION_ERROR;
  }
  else
      Appli_state = APPLICATION_RUNNING;
break;
case APPLICATION_RUNNING:
  if (BtnClick || AdcSupplyVoltageLow())
      Appli_state = APPLICATION_CLOSE_FILE;
  else if (LogGetCloseFileFlag() || /*!SentLogEnabled() || */!SentLoggingOn())
  {
    while (!FS_FileOperations()); // Read until there are not any data
    Appli_state = APPLICATION_CLOSE_FILE;
  }
  else
  {
    process_res = FS_FileOperations();
    if (FR_OK != process_res && NO_DATA_TO_WRITE != process_res)
        Appli_state = APPLICATION_ERROR;
  }
break;
case APPLICATION_CLOSE_FILE:
  process_res = FS_CloseFile();
  if (FR_OK != process_res)
      Appli_state = APPLICATION_ERROR;
  else if (!BtnClick && !AdcSupplyVoltageLow())
      Appli_state = APPLICATION_IDLE;
  else
  {
      Appli_state = APPLICATION_CREATE_FILE;
      BtnClick = 0;
  }
  break;
case APPLICATION_SD_UNPLUGGED:
  process_res = APP_SD_UNPLUGGED;
  break;
case APPLICATION_IDLE:
  if (!mounted)
  {
     // SdLedBlink(LedOff, RedLed);
      if (FS_Mount() == FR_OK)
       //   SdLedBlink(LedBlinkSingle, GreenLed);
      else
       //   SdLedBlink(LedOn, RedLed);
  }
  if (SentLoggingOn() && /*SentLogEnabled() && */!AdcSupplyVoltageLow())
  {
    /*  SdLedBlink(LedOff, RedLed);
      SdLedBlink(LedOff, GreenLed);
      SdLedBlink(LedBlinkSlow, GreenLed);*/
      Appli_state = APPLICATION_CREATE_FILE;
  }
  break;
case APPLICATION_ERROR:
    SdLedBlink(LedOn, RedLed);
    break;
case APPLICATION_ERROR_PERSISTENT:
default:
  break;
}
}
#else // FATFS_DEBUG_TEST defined
/* Debug test application */
int FatFsIterations;

void MX_FATFS_Process(void)
{
  int32_t process_res = APP_OK;
  /* Mass Storage Application State Machine */
  switch(Appli_state)
  {
    case APPLICATION_INIT:
      if(BSP_SD_IsDetected() == SD_PRESENT)
      {
          SD_Init();   /* Initialize the SD card */
          process_res = APP_INIT;
          if (!mounted)
              FS_Mount();
          if (mounted)
              Appli_state = APPLICATION_CREATE_FILE;

#if FATFS_MKFS_ALLOWED
        FRESULT res;

        // Disable millisecond timer for f_mkfs (for concurrent SPI transactions
        //  the duration is too long).
        taskENTER_CRITICAL();
        res = f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
        taskEXIT_CRITICAL();

        if (res != FR_OK)
        {
          process_res = APP_ERROR;
        }
        else
        {
          process_res = APP_INIT;
          Appli_state = APPLICATION_RUNNING;
        }
#endif
      }
      else
      {
        Appli_state = APPLICATION_SD_UNPLUGGED;
      }
      break;

    case APPLICATION_CREATE_FILE:;
      //LogSetCloseFileFlag(0);
      uint16_t count = GetLogFileCount();
      process_res = FS_CreateFile(count);
      if (FR_OK != process_res)
      {
          Appli_state = APPLICATION_ERROR;
      }
      else
          Appli_state = APPLICATION_RUNNING;
    break;

    case APPLICATION_RUNNING:
      if (FatFsIterations < 100)
      {
          FatFsIterations++;
          if (FS_WriteData() != FR_OK)
              Appli_state = APPLICATION_ERROR;
      }
      else
      {
          Appli_state = APPLICATION_CLOSE_FILE;
      }
    break;

    case APPLICATION_CLOSE_FILE:
      process_res = FS_CloseFile();
      if (FR_OK != process_res)
          Appli_state = APPLICATION_ERROR;
      else
          Appli_state = APPLICATION_IDLE;
      break;

    case APPLICATION_SD_UNPLUGGED:
      process_res = APP_SD_UNPLUGGED;
      break;

    case APPLICATION_IDLE:
    case APPLICATION_ERROR:
    default:
      break;
  }
  return ;
}
#endif // FATFS_DEBUG_TEST

/**
 * @retval Not compatible with FRESULT.
 */
static  int32_t __attribute__((unused)) FS_FileOperations(void)
{
  FRESULT res; /* FatFs function common result code */
  uint32_t byteswritten; /* File write/read counts */
  uint16_t dataToWriteSize = 0;
  uint8_t* dataToWrite = 0;
  int8_t returnVal = 0;

 // LogGetData(&dataToWrite, &dataToWriteSize);
  if (dataToWriteSize > 0)
  {
    /* Write data to the text file */
    res = f_write(&SDFile, dataToWrite, dataToWriteSize, (void*) &byteswritten);
    if ((byteswritten > 0) && (res == FR_OK))
      returnVal = 0;
    else
      returnVal = -1;
    res = f_sync(&SDFile); // Sync file (flush cache without closing)
  }
  else
      returnVal = NO_DATA_TO_WRITE;
  return returnVal;
}

#ifdef FATFS_DEBUG_TEST
static int32_t FS_WriteData(void)
{
  FRESULT res; /* FatFs function common result code */
  uint32_t byteswritten; /* File write/read counts */
  uint8_t dataToWrite[] = "Abcdefghijklmnopqrstuvwxyz0123456789\r\n";
  int8_t returnVal = 0;

  /* Write data to the text file */
  res = f_write(&SDFile, dataToWrite, sizeof(dataToWrite) - 1, (void*) &byteswritten);
  if ((byteswritten > 0) && (res == FR_OK))
  {
    returnVal = 0;
  }
  return returnVal;
}
#endif // FATFS_DEBUG_TEST

uint8_t usrItoa(uint32_t value, char* buffer, size_t len)
{
    itoa(value, buffer, 10);
    uint8_t digits = 5;
    if (value < 10)         // <0; 9>
        digits = 1;
    else if (value < 100)   // <9; 99>
        digits = 2;
    else if (value < 1000)  // <100; 999>
        digits = 3;
    else if (value < 10000) // <1000; 9999>
        digits = 4;
    return digits;

    /*uint16_t div = 1;
    uint8_t i;
    for (i = 0; i < digits && i < len - 1; i++)
    {
        buffer[i] = (digits / div) + '0';
        div *= 10;
    }
    buffer[i] = '\0';
    return i;*/
}

static int32_t FS_CreateFile(uint16_t cnt)
{
  int8_t returnVal = -1;
  FRESULT res;

  char num[6];    // 16bit max + '\0'
  uint8_t len = usrItoa(cnt, num, 5);
  TCHAR path[MAX_FILENAME];
  path[0] = 'l';
  path[1] = 'o';
  path[2] = 'g';
  uint8_t i;
  // i < 5
  for (i = 0; i < len && i < MAX_FILENAME - 3 - 4 - 1; i++)
  {
    path[i + 3] = num[i];
  }
  path[i + 3] = '.';
  path[i + 4] = 'b';
  path[i + 5] = 'i';
  path[i + 6] = 'n';
  path[i + 7] = '\0';
  /* Create and Open a new file object with write access */
  if ((res = f_open(&SDFile, path, FA_CREATE_NEW | FA_WRITE)) == FR_OK)
  {
    /* Success*/
    returnVal = 0;
  }
  if (FR_EXIST == res)
    returnVal = -2;
  /* Error */
  return returnVal;
}

static int32_t FS_CloseFile(void)
{
  return f_close(&SDFile);
}

static FRESULT FS_Mount(void)
{
    FRESULT res = FR_OK;
    if (!mounted)
    {
        res = f_mount(&SDFatFs, SDPath, 1);
    }
    if (res == FR_OK)
    {
        mounted = 1;
    }
    return res;
}

/**
  * @brief  BSP SD Callback.
  * @param  Instance SD Instance
  * @param  Status   Pin status
  * @retval None.
  */
void BSP_SD_DetectCallback(uint32_t Instance, uint32_t Status)
{
  if (Status == SD_PRESENT /*&& APPLICATION_ERROR_PERSISTENT != Appli_state*/)
  {
//    BSP_SD_Init(0);
//    BSP_SD_DetectITConfig(0);
    Appli_state = APPLICATION_REINIT;
  }
  else
  {
   /* if (SentLoggingOn()
        && (APPLICATION_RUNNING == Appli_state || APPLICATION_ERROR == Appli_state))
    {
        Appli_state = APPLICATION_ERROR_PERSISTENT;
     //   SdLedBlink(LedBlinkSlow, RedLed);
    }
    else
    {
        //f_mount(NULL, SDPath, 0); // Unmount the drive
        f_mount(NULL, (TCHAR const*)"", 0); // Unmount the drive*/
        Appli_state = APPLICATION_DEINIT;
   // }

    mounted = 0;
  }
}

FRESULT InitListing(DIR* pDir)
{
    return f_opendir(pDir, (TCHAR const*)SDPath);
}

FRESULT DeinitListing(DIR* pDir)
{
    return f_closedir(pDir);
}

uint8_t GetNextFileName(char* pStr, FSIZE_t* pSize, DIR* pDir)
{
    FILINFO fno;
    FRESULT res = f_readdir(pDir, &fno);
    uint8_t ret = 0;

    if (FR_OK == res && fno.fname[0] != '\0')
    {
        strncpy(pStr, fno.fname, MAX_FILENAME);
        ret = strlen(pStr);
        *pSize = fno.fsize;
    }
    return ret;
}

static uint8_t checkFilename(TCHAR* pName)
{
    uint8_t i = 0, good = 1;
    char c;
    while ((c = pName[i]) != '\0' && good)
    {
        good = 0;
        if (0 == i)
        {
            if ('L' == c)
                good = 1;
            else
                good = 0;
        }
        else if (1 == i)
        {
            if ('O' == c)
                good = 1;
            else
                good = 0;
        }
        else if (2 == i)
        {
            if ('G' == c)
                good = 1;
            else
                good = 0;
        }
        else
        {
            good = 1;
        }
        i++;
    }
    if (!good || i < 5 || pName[i - 1] != 'N' || pName[i - 2] != 'I'
        || pName[i - 3] != 'B' || pName[i - 4] != '.')
        good = 0;
    return good;
}

uint16_t GetLogFileCount(void)
{
    DIR dir;
    InitListing(&dir);
    FILINFO fno;
    FRESULT res = FR_OK;
    uint16_t logCnt = 0;

    while ((res = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != '\0')
    {
        if (checkFilename(fno.fname))
            logCnt++;
    }
    DeinitListing(&dir);
    return logCnt;
}

uint8_t LogFileIndexToFilename(uint16_t index, TCHAR* pName)
{
    uint8_t ret = 0;
    if (pName)
    {
        DIR dir;
        FILINFO fno;
        uint16_t curIndex = 0;
        f_opendir(&dir, (TCHAR const*)SDPath);

        while (curIndex <= index
               && f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0')
        {
            if (checkFilename(fno.fname))
            {
                if (curIndex == index)
                {
                    // Found the name
                    strncpy(pName, fno.fname, MAX_FILENAME);  // Buffer length is hard-coded!
                    break;
                }
                else
                    curIndex++;
            }
        }

        f_closedir(&dir);
        if (curIndex > index)
        {
            pName[0] = '\0';
            ret = 0;
        }
        else
            ret = 1;
    }
    return ret;
}

/**
  * @brief  BSP SD Initialization
  * @param  None
  * @retval None.
  */
static void SD_Init(void)
{
  if (!is_initialized)
  {
     BSP_SD_Init();
     //BSP_SD_ITConfig();
     is_initialized = 1;
  }
}


FS_FileOperationsTypeDef SD_GetState(void)
{
    return Appli_state;
}
/* USER CODE END Application */
