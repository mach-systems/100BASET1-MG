/*
 * eeprom.c
 *
 *  Created on: Jan 11, 2023
 *      Author: Karel Hevessy
 *
 */

#include "eeprom.h"
#include "main.h"
#include "defs.h"   /* EEPROM_ERROR */
#include "FreeRTOS.h"
#include "task.h"   /* Get scheduler state */

/* Buffer size for EEPROM testing; same as size of the whole EEPROM */
#define EEPROM_SIZE         2048U

#define EEPROM_WRITE_BLOCK  16U /* Device 25LC160C has 16B pages */

/*
 * Command set
 */
#define EEPROM_CMD_READ     0x3U    /* 0b00000011 */
#define EEPROM_CMD_WRITE    0x2U    /* 0b00000010 */
#define EEPROM_CMD_WRDI     0x4U    /* 0b00000100 */
#define EEPROM_CMD_WREN     0x6U    /* 0b00000110 */
#define EEPROM_CMD_RDSR     0x5U    /* 0b00000101 */
#define EEPROM_CMD_WRSR     0x1U    /* 0b00000001 */

#define EE_TIMEOUT          (3000000 / 166) /* Timeout 6 ms when reading status register (Twc) */
#define EEPROM_TIMEOUT_CNT  500U            /* Wait this many timer ticks when waiting for EEPROM idle */
/*#define EEPROM_DELAY_CNT    5*/               /* Minimum delay after CS = 1 is 50 ns */

/* Minimal CS pin timings - values for Vdd 3.3 V */
#define EE_CS_SETUP           0 /* CS low before the transaction - 150 ns*/
#define EE_CS_HOLD            1 /* CS low after the transaction - 250 ns*/
#define EE_CS_DISABLE         0 /* CS must stay high for at least 50 ns */

/*
 * Currently ongoing operation
 */
static EeOperation EepromMode;

/*
 * Hardware macros
 */
#define EE_CS_ON()          HAL_GPIO_WritePin(EEPROM_SPI_CS_GPIO_Port, EEPROM_SPI_CS_Pin, GPIO_PIN_RESET)   // Minimum 150 ns setup (could add eeDelay(EE_CS_SETUP) after)
#define EE_CS_OFF()         HAL_GPIO_WritePin(EEPROM_SPI_CS_GPIO_Port, EEPROM_SPI_CS_Pin, GPIO_PIN_SET)     // Minimum 250 ns hold (could add eeDelay(EE_CS_HOLD) before)

/*
 * Transmit a byte using LL SPI API.
 */
static void eepromSpiTransmitByte(uint8_t byte);

/*
 * End SPI transaction (disable the peripheral).
 */
static void eepromSpiTransactionEnd(void);


static void (*EepromCallback)(int16_t);
static uint8_t* EepromBuffer;
static uint16_t EepromBufferIndex;
static uint16_t EepromAddress;
static int16_t EepromCnt;
static int16_t EepromLen;       /* Amount of data yet to read/write */

static uint8_t txByte;
static uint8_t rxByte;


/*
 * 0 ... 180 ns, 1 ... 300 ns, 10 ... 800 ns
 */
static void eeDelay(uint16_t time)
{
    volatile uint16_t max = time;
    volatile uint16_t i = 0;
    while (i < max)
    {
        i++;
    }
}

int8_t EEPROMReadStatus(uint8_t* st, void (*callback)(int16_t))
{
    int8_t ret = -1;
    if ((EeIdle == EepromMode) || (EeTimeout == EepromMode))
    {
        ret = 0;
        EepromCallback = callback;
        EepromBuffer = st;
        EepromBufferIndex = 0;
        EepromCnt = -1;
        EE_CS_ON();
        txByte = EEPROM_CMD_RDSR;
        EepromMode = EeReadSt;
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        // Invalid state
    }
    return ret;
}

int8_t EEPROMWriteStatus(uint8_t* st, void (*callback)(int16_t))
{
    int8_t ret = -1;
    if ((EeIdle == EepromMode) || (EeTimeout == EepromMode))
    {
        ret = 0;
        EepromCallback = callback;
        EepromBuffer = st;
        EepromBufferIndex = 0;
        EepromAddress = 0;
        EepromCnt = -2;
        EepromLen = 0;
        EE_CS_ON();
        txByte = EEPROM_CMD_WREN;
        EepromMode = EeWriteSt;
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        // Invalid state
    }
    return ret;
}

int8_t EEPROMWait(void)
{
    uint32_t timeoutCnt = 0;
    int8_t ret = 0;
    while ((EeIdle != EepromMode) && (EeTimeout != EepromMode)
           && (timeoutCnt < EEPROM_TIMEOUT_CNT))
    {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            osDelay(1);
        }
        else
        {
            HAL_Delay(1);
        }
        ++timeoutCnt;
    }
    if (timeoutCnt >= EEPROM_TIMEOUT_CNT)
    {
        ret = -1;
    }
    return ret;
}

EeOperation EEPROMStat(void)
{
    return EepromMode;
}

int8_t EEPROMReadData(uint16_t addr, uint8_t* pBuf, uint16_t len, void (*callback)(int16_t))
{
    int8_t ret = -1;
    if ((EeIdle == EepromMode) || (EeTimeout == EepromMode))
    {
        ret = 0;
        EepromCallback = callback;
        EepromBuffer = pBuf;
        EepromBufferIndex = 0;
        EepromAddress = addr;
        EepromCnt = -3;
        EepromLen = len;
        EE_CS_ON();
        txByte = EEPROM_CMD_READ;
        EepromMode = EeRead;
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        // Invalid state
    }
    return ret;
}

int8_t EEPROMWriteData(uint16_t addr, uint8_t* pBuf, uint16_t len, void (*callback)(int16_t))
{
    int8_t ret = -1;
    if (((EeIdle == EepromMode) || (EeTimeout == EepromMode)) && (0U != len) && (NULL != pBuf))
    {
        ret = 0;
        EepromCallback = callback;
        EepromBuffer = pBuf;
        EepromBufferIndex = 0;
        EepromAddress = addr;
        EepromCnt = -3;
        EepromLen = len;
        EE_CS_ON();
        txByte = EEPROM_CMD_WREN;
        EepromMode = EeWrite;
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        // Invalid state or bad parameters
    }
    return ret;
}

int8_t EEPROMErase(uint16_t addr, uint16_t len, void (*callback)(int16_t))
{
    int8_t ret = -1;
    if ((EeIdle == EepromMode) || (EeTimeout == EepromMode))
    {
        ret = 0;
        EepromCallback = callback;
        EepromAddress = addr;
        EepromCnt = -3;
        EepromLen = len;
        EE_CS_ON();
        txByte = EEPROM_CMD_WREN;
        EepromMode = EeErase;
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        // Invalid state
    }
    return ret;
}

uint8_t EepromSpiCallback(void)
{
    rxByte = LL_SPI_ReceiveData8(EE_SPI);

    static int16_t EepromWCnt = 0;
    uint8_t ret = 0xffU;
    int16_t callbackArg = 0;

    if (EeRead == EepromMode)
    {
        if (-3 == EepromCnt)  /* Begin by transmitting the address */
        {
            txByte = (uint8_t) ((EepromAddress >> 8) & 0xFFU);
        }
        else if (-2 == EepromCnt)
        {
            txByte = (uint8_t) (EepromAddress & 0xFFU);
        }
        else if (-1 == EepromCnt)   /* Address transfered, we will receive the data */
        {
            txByte = 0;
        }
        else
        {
            txByte = 0;
            EepromBuffer[EepromBufferIndex + (uint16_t) EepromCnt] = rxByte;

            if (EepromCnt == (EepromLen - 1))   /* Read complete */
            {
                EepromMode = EeIdle;
                EE_CS_OFF();
                callbackArg = EepromLen;
                ret = 0;
            }
            else
            {
                txByte = 0;
            }
        }
        EepromCnt++;
    }
    else if ((EeWrite == EepromMode) || (EeErase == EepromMode))  /* Writing to EEPROM */
    {
        if (EepromCnt == -3)
        {
            /* After write enable command, we must cycle the CS pin */
            EE_CS_OFF();
            eeDelay(EE_CS_HOLD);
            EE_CS_ON();
            txByte = EEPROM_CMD_WRITE;
        }
        else if (EepromCnt == -2) /* Transfer the address first */
        {
            txByte = (uint8_t) ((EepromAddress >> 8) & 0xFFU);
        }
        else if (EepromCnt == -1)
        {
            txByte = (uint8_t) (EepromAddress & 0xFFU);
        }
        else    /* We can send the data */
        {
            /* Write page ended - new write to the write enable latch needed */
            if (((((uint16_t) EepromCnt + EepromAddress) & (EEPROM_WRITE_BLOCK - 1U)) == 0U)
                && (EepromCnt > 0))
            {
                // Page boundary
                EE_CS_OFF();
                EepromLen -= EepromCnt;
                EepromAddress += (uint16_t) EepromCnt;
                EepromBufferIndex += (uint16_t) EepromCnt;
                eeDelay(EE_CS_HOLD);
                EE_CS_ON();
                txByte = EEPROM_CMD_RDSR;
                if (EeWrite == EepromMode)
                {
                    EepromMode = EeWriteWait;
                }
                else
                {
                    EepromMode = EeEraseWait;
                }
                EepromWCnt = 0;
          }
          else if (EepromCnt < EepromLen) // We can transfer more data in this cycle
          {
              if (EepromMode == EeWrite)
              {
                  txByte = EepromBuffer[EepromBufferIndex + (uint16_t) EepromCnt];
              }
              else
              {
                  txByte = 0xffU;
              }
          }
          else
          {
              // Transaction done
              EE_CS_OFF();
              eeDelay(EE_CS_HOLD);
              EepromLen = 0;
              EE_CS_ON();
              // Data transfer complete, we can wait for the write to finish and exit
              txByte = EEPROM_CMD_RDSR;
              if (EeWrite == EepromMode)
              {
                  EepromMode = EeWriteWait;
              }
              else
              {
                  EepromMode = EeEraseWait;
              }
              EepromWCnt = 0;
        }
      }
      EepromCnt++;
    }
    else if ((EeWriteWait == EepromMode) || (EeEraseWait == EepromMode))    // Wait until the write is complete
    {
        if (0 != (EepromWCnt & 1))
        {
            EE_CS_OFF();
            // Waiting until page write / erase is really done
            if (EepromWCnt > EE_TIMEOUT)
            {
                EepromMode = EeTimeout;
                // Timeout of the communication - EEPROM not responding, error
                GlobalErrorFlags |= EEPROM_ERROR;
                callbackArg = -1;
                ret = 0;
            }
            else if (0U != (rxByte & 1U))   // Write was not done yet
            {
                EE_CS_ON();
                txByte = EEPROM_CMD_RDSR;
            }
            else    // Write was done successfully
            {
                if (0 != EepromLen)     // There are some more data
                {
                    EE_CS_ON();
                    EepromCnt = -3;
                    txByte = EEPROM_CMD_WREN;
                    if (EeWriteWait == EepromMode)
                    {
                        EepromMode = EeWrite;
                    }
                    else
                    {
                        EepromMode = EeErase;
                    }
                }
                else
                {
                    EepromMode = EeIdle;
                    callbackArg = EepromWCnt;
                    ret = 1;
                }
            }
        }
        else
        {
            txByte = 0;
        }
        EepromWCnt++;
    }
    else if (EeReadSt == EepromMode)
    {
        if (EepromCnt == -1)
        {
            txByte = 0;
        }
        else
        {
            EepromMode = EeIdle;
            EE_CS_OFF();
            callbackArg = rxByte;
            ret = 0;
        }
        EepromCnt++;
    }
    else if (EeWriteSt == EepromMode)
    {
        if (EepromCnt == -2)
        {
            EE_CS_OFF();
            eeDelay(EE_CS_HOLD);
            EE_CS_ON();
            txByte = EEPROM_CMD_WRSR;
        }
        else if (EepromCnt == -1)
        {
            txByte = EepromBuffer[EepromBufferIndex];
        }
        else    /* Write of the status register complete */
        {
            EE_CS_OFF();
            EepromLen = 0;
            eeDelay(EE_CS_HOLD);
            EE_CS_ON();
            txByte = EEPROM_CMD_RDSR;
            EepromMode = EeWriteWait;
            EepromWCnt = 0;
        }
        EepromCnt++;
    }
    else
    {
        // Invalid state
    }

    if (0xffU == ret)
    {
        eepromSpiTransmitByte(txByte);
    }
    else
    {
        eepromSpiTransactionEnd();
        if (NULL != EepromCallback)
        {
            EepromCallback(callbackArg);
        }
    }

    return ret;
}

void TestEepromAccess(void)
{
  /* Buffers for eeprom testing */
  static uint8_t eeTestRxData[EEPROM_SIZE];
  static uint8_t eeTestTxData[EEPROM_SIZE];

  /* Init the test tx array */
  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
  {
    eeTestTxData[i] = i + 1U;
  }

  (void) EEPROMWriteData(0, eeTestTxData, EEPROM_SIZE, NULL);
  (void) EEPROMWait();

  (void) EEPROMReadData(0, eeTestRxData, EEPROM_SIZE, NULL);
  (void) EEPROMWait();

  volatile uint8_t good = 1;

  /* Test the read data */
  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
  {
    if (eeTestRxData[i] != eeTestTxData[i])
    {
      good = 0;
      break;
    }
  }
  /* Try erasing the eeprom */
  (void) EEPROMErase(0, EEPROM_SIZE, NULL);
  (void) EEPROMWait();

  /* Test that all the data is 0xff */
  (void) EEPROMReadData(0, eeTestRxData, EEPROM_SIZE, NULL);
  (void) EEPROMWait();
  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
  {
    if (eeTestRxData[i] != 0xffU)
    {
      good = 0;
      break;
    }
  }

  (void) EEPROMWriteData(0, eeTestTxData, 1, NULL);
  (void) EEPROMWait();
  (void) EEPROMReadData(0, eeTestRxData, 1, NULL);
  (void) EEPROMWait();
  if (eeTestRxData[0] != eeTestTxData[0])
  {
      good = 0;
  }

  (void) good;
//  if (!good)
//    LED1_LG_RED_ON();
//  else
//    LED1_LG_GREEN_ON();
  while (1) { }
}

void eepromSpiTransmitByte(uint8_t byte)
{
    eepromSpiTransactionEnd();
    LL_SPI_SetTransferSize(EE_SPI, 1);
    LL_SPI_Enable(EE_SPI);
    LL_SPI_TransmitData8(EE_SPI, byte);
    LL_SPI_EnableIT_EOT(EE_SPI);
    LL_SPI_EnableIT_CRCERR(EE_SPI);
    LL_SPI_EnableIT_UDR(EE_SPI);
    LL_SPI_EnableIT_OVR(EE_SPI);
    LL_SPI_StartMasterTransfer(EE_SPI);
}

void eepromSpiTransactionEnd(void)
{
    LL_SPI_Disable(EE_SPI);
    LL_SPI_DisableIT_EOT(EE_SPI);
}
