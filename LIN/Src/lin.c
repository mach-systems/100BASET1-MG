/*
 * lin.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#include <string.h>   /* memset(), memcpy() */
#include "lin.h"
#include "lin_timer.h"
#include "lin_uart.h"
#include "main.h"     /* Pin name definitions */
#include "config.h"

/* Current scheduler delay */
uint16_t LinDelay;

uint8_t linIsTransmitting;  /* 1 when the transmission is in progress */

uint8_t schedulerRunning;
uint8_t schedulerIndex;

LinScheduler scheduler;

/* Frame to send */
LIN_Frame txFrame;
uint8_t txFrameDataIndex;
/* Data to receive (raw data) */
uint8_t rxData[RX_BUFFER_LENGTH];
uint8_t rxDataIndex;

/* What part of the LIN message we will be transmitting */
LinState linTransmitState;
LinState linSlaveReceptionState;

uint8_t linStopChannelRequestFlag;

/* Determines if we will transmit anything or will be only receiving */
uint8_t linStopTx;

/* One if we are receiving the slave response */
uint8_t linReceiving;

/* Response timeout counter variables */
uint16_t responseTimeoutValue;
uint16_t responseTimeoutMax;

/* Timeout of receiving break (in case of short of LIN to Vbat) */
uint8_t breakTmrRunning;
uint16_t breakTmrVal;

/* LIN channel settings - mode and so on */
LinInitStruct LinSettings;

/* TxFrameBuffer write index*/
uint8_t LinWriteIndex;

/*TxFrameBuffer read index*/
uint8_t LinReadIndex;

uint8_t linTransmittionPending;

uint8_t channelToResponse;

uint16_t slaveResponseMaxIndex; /* Number of used slave responses */
uint16_t slaveResponseIndex;    /* Index to the buffer we are currently working with */
uint8_t slaveResponseDataIndex; /* Index to currently received slave data (index to the data in the buffer) */

/* Buffers for slave responses */
SlaveResponseDataStruct SlaveResponse[LIN_ID_COUNT + 1];  /* One free space for other id's data */

/* Index to loopback control data */
uint8_t linControlDataIndex;

/* Frame for loopback checking of sent data and loopback control */
LinControlStruct linLoopbackControl;
LinState LinControlState;

/*  */
uint8_t restartLinUart;

/* Time counters for Led blink*/
uint16_t counterLinBlinkRed;
uint16_t counterLinBlinkGreen;

/* Led Flags for blinking */
bool ledLinRedBlink;
bool ledLinGreenBlink;

/* If true send frame asynchronous else send frame from TX buffer*/
uint8_t transmitAsyncSet;

/* Timeout of receiving brake (in case of short of LIN to Vbat) */
uint8_t breakTmrRunning;
uint16_t breakTmrVal;

uint8_t loopbackTmrRunning;
uint16_t loopbackTmrVal;

uint8_t slaveTxBuffer[LIN_DATA_LENGTH + 1];         /* Slave data, that we are currently sending */
uint8_t slaveTxBufferLength;      /* Number of bytes to be sent in the slaveTxBuffer */
uint8_t slaveTxBufferReadIndex;   /* Index to currently sent byte in the slaveTxBuffer*/

LIN_Frame TxFrameBuffer[LIN_FRAME_TX_BUFFER_LENGTH];    //buffer of LIN Tx frames

bool linEchoTx = true;
bool linEchoRx = true;


bool linRun;


/* This is called when LIN error occurred*/
void LinErrorCallback(uint8_t linId, uint8_t error)
{
    blinkLedLin(RED_LIN_LED);
}

void SchedulerTimerCallback(void)
{
  /* LinMaintenance */
  if(linStopChannelRequestFlag && !linIsTransmitting  && !linTransmittionPending )
  {
      LinStop();
      LinChannelStoppedCallback();
  }
  if(restartLinUart)
  {
      restartLinUart = 0;
      LinUartReset();
  }


  if (schedulerRunning)
  {
    LinDelay--;
    if (LinDelay == 0)
    {
        uint8_t txOk = 0;
        LIN_Frame *pFrame = &scheduler.Row[schedulerIndex].Frame;
        txOk = LinWriteToTxBuffer(pFrame->ID, pFrame->Type, pFrame->ChecksumType,
                pFrame->Length, pFrame->Data);
      if (txOk)
      {
        schedulerIndex = (schedulerIndex + 1) % scheduler.NumberOfFrames;
      }
      LinDelay = scheduler.Row[schedulerIndex].Delay;
    }
  }
}

void LinSchedulerStart(void)
{
  if (scheduler.NumberOfFrames)
  {
    linIsTransmitting = 0;
    schedulerIndex = 0;
    schedulerRunning = 1;
    LinDelay = scheduler.Row[schedulerIndex].Delay;
  }
}

void LinSchedulerStop(void)
{
  schedulerRunning = 0;
}


uint8_t InitLin(LinInitStruct * ConfLinSettings)
{

  if(linRun == true){
      return 1;
  }

  //Set led
  LED_LIN_RED_OFF();
  LED_LIN_GREEN_ON();

  LinSettings.AMLR = ConfLinSettings->AMLR;
  LinSettings.Baudrate = ConfLinSettings->Baudrate;
  LinSettings.ChecksumType = ConfLinSettings->ChecksumType;
  LinSettings.DeviceMode = ConfLinSettings->DeviceMode;

  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_SET);
    /* Initialize frames in the scheduler */
    initLinScheduler();
  }
  else
    HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_RESET);

  /* Configure the uart and timer peripherals */
  InitLinUartBaudrate(LinSettings.Baudrate);
  LinTimeoutTimerInit(LinSettings.Baudrate);

  /* Configure the UART interrupts */
  LinUartEnableBreakInterrupt();
  LinUartEnableRxInterrupt();
  LinUartDisableTxInterrupt();
  if (LinSettings.DeviceMode == LIN_MODE_SLAVE)
    LinUartDisableRxInterrupt();
  linStopChannelRequestFlag = 0;
  resetStateMachine();
  linRun = true;
  LinWriteIndex = 0;
  LinReadIndex = 0;
  //LinSchedulerStart();
  return 0;
}

void initLinScheduler(void)
{
  scheduler.Row[0].Frame.ID = 0x1;
  scheduler.Row[0].Frame.Length = 3;
  memset((void*) scheduler.Row[0].Frame.Data, 0xab, scheduler.Row[0].Frame.Length);
  scheduler.Row[0].Frame.ChecksumType = ENHANCED_CHECKSUM;
  scheduler.Row[0].Frame.Type = MASTER_RESPONSE;
  scheduler.Row[0].Delay = 25;

  scheduler.Row[1].Frame.ID = 0x2;
  scheduler.Row[1].Frame.Length = 2;
  scheduler.Row[1].Frame.ChecksumType = ENHANCED_CHECKSUM;
  scheduler.Row[1].Frame.Type = MASTER_REQUEST;
  scheduler.Row[1].Delay = 50;

  scheduler.NumberOfFrames = 2;
}


void LinTimeoutTimerCallback(void)
{
  if (breakTmrRunning)
  {
      /* Waiting for break - in case when LIN is shorted to Vbat */
      breakTmrVal++;
      if (breakTmrVal >= BREAK_TIMEOUT_MAX)
      {
          LinTimeoutTimerStop();
          breakTmrVal = breakTmrRunning = 0;
          LinReadIndex = LinWriteIndex = 0;
          linResetAll();
      }
  }
  else if (loopbackTmrRunning)
  {
    /* Waiting for loopback */
    loopbackTmrVal++;
    if (loopbackTmrVal >= BREAK_TIMEOUT_MAX)
    {
      LinTimeoutTimerStop();
      loopbackTmrVal = loopbackTmrRunning = 0;
      LinReadIndex = LinWriteIndex = 0;
      linResetAll();
    }
  }
  else if (linReceiving) /* Timeout for response running */
  {
    if (responseTimeoutValue < responseTimeoutMax)
      responseTimeoutValue++;
    else
    {
      LinTimeoutTimerStop();
      linReceiving = 0;

      if (LinSettings.DeviceMode == LIN_MODE_MASTER)
      {
        linIsTransmitting = 0;
        linTransmittionPending = 0;

        /* Hard-coded length of messages - timeout occurred */
        if (LinSettings.AMLR == HARDCODED)
        {
          LinErrorCallback(TxFrameBuffer[LinReadIndex].ID, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
        }
        else  /* Variable message length - timeout did not have to occur */
        {
          /* Response length is with the sync + id bytes and checksum ... some data received*/
          if (TxFrameBuffer[LinReadIndex].ResponseLength > 3)
          {
            if (processMasterReceivedFrame(&TxFrameBuffer[LinReadIndex], rxData))
              MasterRequestTxRxCallback(&TxFrameBuffer[LinReadIndex]);
            else
              LinErrorCallback(TxFrameBuffer[LinReadIndex].ID & 0x3f, PROTOCOL_ERR_LIN_CHECKSUM);
          }
          /* No data received */
          else
          {
            LinErrorCallback(TxFrameBuffer[LinReadIndex].ID & 0x3f, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
          }
        }

        transmitAsyncSet=0;
        if (LinReadIndex != LinWriteIndex)
            LinReadIndex++;

        if(LinReadIndex>=LIN_FRAME_TX_BUFFER_LENGTH)
        {
            LinReadIndex = 0;
        }

        if((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0) )
        {
          setLinIsTransmitting(1);
        }
        else
        {
           if(linStopChannelRequestFlag == 0)
           {
               setLinIsTransmitting(0);
           }
           else
           {
               LinStop();
               LinChannelStoppedCallback();
           }
        }

      }
      else  /* Device is in slave mode */
      {
        /* Waiting for sync field after sync break */
        if (linSlaveReceptionState == STATE_SYNC_FIELD)
        {
          LinErrorCallback(0x00, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
          resetSlaveStateMachine();
          restartLinUart = 1;
        }
        else
        {
          if (LinSettings.AMLR == HARDCODED)
          {
            LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
            restartLinUart = 1;
          }
          else /* Variable message length */
          {
            /* RxDataLength including checksum - some data received */
            if (SlaveResponse[slaveResponseIndex].RxDataLength > 0)
            {
              uint8_t index = slaveResponseIndex;
              /* Buffer overflow prevention */
              index %= LIN_ID_COUNT + 1;
              uint8_t rxdl = SlaveResponse[index].RxDataLength;
              uint8_t linIdProtected = getParityIdentifier(SlaveResponse[index].LinID);
              uint8_t temp;

              if (SlaveResponse[index].Data[rxdl - 1] == (temp = calcChecksum(SlaveResponse[index].Data, SlaveResponse[index].RxDataLength - 1,
                                                                      SlaveResponse[index].ChecksumType,
                                                                      linIdProtected)))
              {
                /* Data length without checksum */
                SlaveResponse[index].RxDataLength -= 1;
                SlaveResponseTxRxCallback(&SlaveResponse[index]);
              }
              else
              {
                LinErrorCallback(SlaveResponse[index].LinID, PROTOCOL_ERR_LIN_CHECKSUM);
              }
            }
            else  /* No data received */
            {
              LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
              restartLinUart = 1;
            }
          }
        }
        resetSlaveStateMachine();
        LinUartDisableRxInterrupt();  /* So that RX interrupt is not before LBD */
      } /* Slave */
    } /* Timeout has elapsed */
  } /* linReceiving */
}


void LinRxCallback(uint8_t byteReceived)
{
  rxData[rxDataIndex] = byteReceived;

  if (linLoopbackControl.Enabled)
  {
    loopbackStateMachine(byteReceived);
  }
  else /* loopback disabled */
  {
    if (LinSettings.DeviceMode == LIN_MODE_MASTER)
    {
      if (TxFrameBuffer[LinReadIndex].Type == MASTER_REQUEST && linReceiving)
      {
        /* Note: ResponseLength is without the sync break */
        TxFrameBuffer[LinReadIndex].ResponseLength++;

        if (LinSettings.AMLR == HARDCODED)
        {
          /* Last byte received */
          if (rxDataIndex >= TxFrameBuffer[LinReadIndex].Length + 4 - 1)
          {
            /* length = data bytes + break 1 B + sync 1 B + id 1 B + checksum 1 B - 1 (it is an index) */
            linTransmittionPending = 0;
            linReceiving=0;
            if (processMasterReceivedFrame(&TxFrameBuffer[LinReadIndex], rxData))
              MasterRequestTxRxCallback(&TxFrameBuffer[LinReadIndex]);
            else
              LinErrorCallback(TxFrameBuffer[LinReadIndex].ID, PROTOCOL_ERR_LIN_CHECKSUM);

            if (LinReadIndex != LinWriteIndex)
                LinReadIndex=(LinReadIndex+1)%LIN_FRAME_TX_BUFFER_LENGTH;

            if((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0))
            {
                setLinIsTransmitting(1);
            }
            else
            {
                if(linStopChannelRequestFlag == 0)
                {
                    linIsTransmitting = 0;
                }
                else
                {
                    LinStop();
                    LinChannelStoppedCallback();
                }
            }


          }
        }
        else  /* AMLR == AUTOLEN */
        {
          responseTimeoutValue = 0; /* Auto message length detection - just collect data */
        }
      }
    }
    else /* mode == LIN_MODE_SLAVE */
    {
      uint8_t linIdFound;
      uint8_t frameId;
      uint8_t protectedId;
      uint8_t localChecksum;

      switch (linSlaveReceptionState)
      {
        case STATE_SYNC_BREAK:
          if (byteReceived == 0x00)
          {
            linSlaveReceptionState = STATE_SYNC_FIELD;
            responseTimeoutMax = 20;  /* Timeout for 1 byte */
            timeoutTimerStart();
          }
          else
          {
            restartLinUart = 1;
          }
          break;

        case STATE_SYNC_FIELD:
          LinTimeoutTimerStop();
          linSlaveReceptionState = STATE_ID;
          break;

        case STATE_ID:
          linIdFound = 0;
          frameId = (rxData[rxDataIndex] & 0x3f);
          protectedId = getParityIdentifier(frameId);
          if (protectedId == rxData[rxDataIndex])
          {
            /* Id is OK, check if it is in SlaveResponse buffer */
            slaveResponseIndex = 0;
            for (uint8_t i = 0; i < slaveResponseMaxIndex; i++)
            {
              if (SlaveResponse[i].LinID == frameId)
              {
                slaveResponseIndex = i;
                linIdFound = 1;
                break;
              }
            }

            /* We want all the data on the bus (even belonging to other IDs) in sniffer mode */
            if (LinSettings.DeviceMode == LIN_SNIFFER_MODE)
            {
              slaveResponseIndex = LIN_ID_COUNT;   /* Let's use index 64 to receive data (will always be free) */
              SlaveResponse[slaveResponseIndex].LinID = frameId;

              if (LinSettings.ChecksumType == CHECKSUM_CLASSICAL || frameId == 0x3c || frameId == 0x3d
                                                               || frameId == 0x3e || frameId == 0x3f)
              {
                SlaveResponse[slaveResponseIndex].ChecksumType = CLASSICAL_CHECKSUM;
              }
              else
              {
                SlaveResponse[slaveResponseIndex].ChecksumType = ENHANCED_CHECKSUM;
              }

              /* Pretend that buffer with this frame ID is configured */
              SlaveResponse[slaveResponseIndex].BufferDir = BUFFER_DIR_RX;
              linIdFound = 1;
            }

            /* This if does not have to be in this implementation, as lin id is
             * always found (even when the id does not exist, see above). */
            if (linIdFound)
            {
              if (SlaveResponse[slaveResponseIndex].BufferDir == BUFFER_DIR_TX)
              {
                // Device will send data from SlaveResponseBuffer
                LinControlState = STATE_DATA;
                linControlDataIndex = 0;
                linReceiving = 0;
                linLoopbackControl.Enabled = 1;
                SendSlaveResponse(slaveResponseIndex);
                resetSlaveStateMachine();
                LinUartEnableTxInterrupt();
              }
              else
              {
                // Device will receive data to SlaveBuffer
                if (LinSettings.AMLR == AUTOLEN)
                {
                  SlaveResponse[slaveResponseIndex].RxDataLength = 0;
                  responseTimeoutMax = 14;   /* Timeout for 1 byte */
                }
                else
                {
                  SlaveResponse[slaveResponseIndex].RxDataLength=frameId & 0x20 ? (frameId & 0x10 ? 8:4) : 2;
                  calcAndUpdateTimeoutValue(SlaveResponse[slaveResponseIndex].RxDataLength);
                }
                timeoutTimerStart();
                linReceiving = 1;
                linSlaveReceptionState = STATE_DATA;
              }
            }
            else
            {
              timeoutTimerStop(); /* Stop delay timer to prevent Delay error for other IDs */
              resetSlaveStateMachine();
            }
          }
          else
          {
            /* Id is not ok */
            timeoutTimerStop();   /* Stop delay timer to prevent Delay error for other IDs */
            resetSlaveStateMachine();
          }

          slaveResponseDataIndex = 0;
          break;

        case STATE_DATA:
          SlaveResponse[slaveResponseIndex].Data[slaveResponseDataIndex] = rxData[rxDataIndex];
          slaveResponseDataIndex++;

          if (LinSettings.AMLR == HARDCODED)
          {
            /* Reception of data done */
            if (slaveResponseDataIndex >= SlaveResponse[slaveResponseIndex].RxDataLength)
              linSlaveReceptionState = STATE_CHECKSUM;
          }
          else /* Autolen */
          {
            responseTimeoutValue = 0;   /* Reset timeout for the next byte */
            SlaveResponse[slaveResponseIndex].RxDataLength++;
          }
          break;

        case STATE_CHECKSUM:
          LinTimeoutTimerStop();
          linReceiving = 0;
          localChecksum = calcChecksum(&SlaveResponse[slaveResponseIndex].Data[0],
                                        SlaveResponse[slaveResponseIndex].RxDataLength,
                                        SlaveResponse[slaveResponseIndex].ChecksumType,
                                        SlaveResponse[slaveResponseIndex].LinID);

          if (rxData[rxDataIndex] == localChecksum)
          {
              linTransmittionPending = 0;     //Transmittion ended, silent on bus
              /* Checksum OK, slave reception done */
              SlaveResponseTxRxCallback(&SlaveResponse[slaveResponseIndex]);
          }
          else
          {
              /* Checksum not ok */
              LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_CHECKSUM);
          }

          resetSlaveStateMachine();
          LinUartDisableRxInterrupt();
          break;

        default:
          break;
      }
    }
  }

  rxDataIndex++;
  rxDataIndex %= RX_BUFFER_LENGTH;
}

uint8_t loopbackStateMachine(uint8_t byteReceived)
{
  uint8_t LinControlError = 0;

  switch (LinControlState)
  {
    case STATE_SYNC_BREAK:
      LinControlState = STATE_SYNC_FIELD;
      if (byteReceived != 0x00)
      {
        LinControlError = 1;
      }
      break;

    case STATE_SYNC_FIELD:
      LinControlState = STATE_ID;
      if (byteReceived != 0x55)
      {
        LinControlError = 1;
      }
      break;

    case STATE_ID:
      LinControlState = STATE_DATA;
      linControlDataIndex = 0;
      if (byteReceived != getParityIdentifier(linLoopbackControl.LinControlFrame.ID))
      //if (byteReceived != linLoopbackControl.LinControlFrame.ID)
      {
        LinControlError = 1;
      }
      break;

    case STATE_DATA:
      if (byteReceived != linLoopbackControl.LinControlFrame.Data[linControlDataIndex])
      {
        LinControlError = 1;
      }
      linControlDataIndex++;
      if (linControlDataIndex >= linLoopbackControl.LinControlFrame.Length)
      {
        LinControlState = STATE_CHECKSUM;
      }
      break;

    case STATE_CHECKSUM:
      LinControlState = STATE_SYNC_BREAK;
      linLoopbackControl.Enabled = 0; /* After checksum, disable loopback control */
      if (loopbackTmrRunning)
      {
        loopbackTmrRunning = 0;
        LinTimeoutTimerStop();
      }

      if (byteReceived != linLoopbackControl.LinControlFrame.Checksum)
      {
        LinControlError = 1;
      }

      if (!LinControlError)
      {

        if (LinSettings.DeviceMode == LIN_MODE_SLAVE) /* All ok, slave transmission done */
        {
          SlaveResponseTxRxCallback(&SlaveResponse[slaveResponseIndex]);
          resetSlaveLoopbackControl();
        }
        else /* Master mode */
        {
          linTransmittionPending = 0;     //Transmittion ended, silent on bus
          MasterResponseTxCallback(&linLoopbackControl.LinControlFrame);
          if (LinReadIndex != LinWriteIndex)
          LinReadIndex++;

          if(LinReadIndex>=LIN_FRAME_TX_BUFFER_LENGTH)
          {
              LinReadIndex = 0;
          }
          if((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0) )
          {
              setLinIsTransmitting(1);
          }
          else
          {
            if(linStopChannelRequestFlag == 0)
            {
                setLinIsTransmitting(0);
            }
            else
            {
                LinStop();
                LinChannelStoppedCallback();
            }
          }

        }
      }
      break;

    /* Invalid state*/
    default:
      LinControlError = 1;
      break;
  }

  if (LinControlError) /* Some error happened */
  {
    LinControlError = 0;
    linTransmittionPending = 0;
    if (LinSettings.DeviceMode == LIN_MODE_SLAVE) /* Slave error - bus collision */
    {
      LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_BUS_ERROR);
      resetSlaveLoopbackControl();
      restartLinUart = 1;
    }
    else /* Master error */
    {
      /* We haven't received what we sent, skip the packet */
      resetStateMachine();
      LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_BUS_ERROR); /* Report bus error */
    }

    if (LinReadIndex != LinWriteIndex)
    {
        LinReadIndex++;                         // skip the packet
        if (LinReadIndex >= LIN_FRAME_TX_BUFFER_LENGTH)
            LinReadIndex = 0;
    }
    if((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0))
    {
        setLinIsTransmitting(1);
    }
    else
    {
        if(linStopChannelRequestFlag == 0)
        {
            setLinIsTransmitting(0);
        }
        else
        {
            LinStop();
            LinChannelStoppedCallback();
        }
    }
  }
  return LinControlError;
}



uint8_t GetConfigurationLin(void){
    uint8_t linConfigurationByte = 0;
    linConfigurationByte |= (GetConfigurationLINAddr()->Baudrate+1);
    if(GetConfigurationLINAddr()->DeviceMode==LIN_MODE_MASTER){
        linConfigurationByte |= 4;
    }
    linConfigurationByte |= (GetConfigurationLINAddr()->autoStart<<4);
    linConfigurationByte |= (GetConfigurationLINAddr()->AMLR << 5);
    linConfigurationByte |= (GetConfigurationLINAddr()->ChecksumType << 6);
    linConfigurationByte |= (GetConfigurationLINAddr()->txEcho << 7);
    return linConfigurationByte;
}

void LinTxCallback(void)
{
  LIN_Frame * txFrameToSend;
  if (transmitAsyncSet){
    txFrameToSend = &txFrame;
  }else{
    txFrameToSend = &TxFrameBuffer[LinReadIndex];
  }
  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    if (linStopTx) /* Stop transmitting (done or waiting for the response) */
    {

      if (txFrameToSend->Type == MASTER_REQUEST)
      {
        linReceiving = 1;
        setLinIsTransmitting(0);
        if (LinSettings.AMLR == HARDCODED){
          txFrameToSend->Length = txFrameToSend->ID & 0x20 ? (txFrameToSend->ID & 0x10 ? 8:4) : 2;
        }
        calcAndUpdateTimeoutValue(txFrameToSend->Length);
        timeoutTimerStart();
        /* Disable the tx interrupt */
        LinUartDisableTxInterrupt();
      }
      else
      {
        linIsTransmitting = linReceiving = 0;
        /* Disable the tx interrupt */
        LinUartDisableTxInterrupt();
      }
    }
    else /* Continue with the transmission */
    {
      linTransmittionPending = 1;
      switch (linTransmitState)
      {
        case STATE_SYNC_BREAK:
          LinUartSendBreak();
          linTransmitState = STATE_SYNC_FIELD;
          /* Trigger timer waiting for break (for the situation when LIN is shorted to Vcc) */
          breakTmrRunning = 1;
          breakTmrVal = 0;
          timeoutTimerStart();
          LinControlState = STATE_SYNC_BREAK;
          linLoopbackControl.Enabled = 1;
          /* Disable the tx interrupt until the break appears */
          LinUartDisableTxInterrupt();
          break;

        case STATE_ID:
          if (txFrameToSend->Type == MASTER_RESPONSE)
          {
            if (txFrameToSend->Length != 0)
            {
              linTransmitState = STATE_DATA;
              txFrameDataIndex = 0;
            }
            else
              linTransmitState = STATE_CHECKSUM;
          }
          else if (txFrameToSend->Type == MASTER_REQUEST)  /* Now is LIN request sent on bus send echo */
          {
            linTransmitState = STATE_SYNC_BREAK;
            /* Do not transmit anything else, wait for response */
            linStopTx = 1;
            setLinIsTransmitting(0);                             // header sent, wait for SLAVE response
          }
          else
            linResetAll();

          /* Continue only if reset did not happen */
          if (txFrameToSend->Type == MASTER_REQUEST || txFrameToSend->Type == MASTER_RESPONSE)
          {
            LinUartPutChar(txFrameToSend->ID);
            /* For master request, we do not need loopback */
            linLoopbackControl.Enabled = !linStopTx;
            if (!linStopTx)
            {
              /* Set up the parameters needed by the loopback */
              linLoopbackControl.LinControlFrame.ID = txFrameToSend->ID;
              linLoopbackControl.LinControlFrame.Length = txFrameToSend->Length;
            }
          }
          break;

        case STATE_DATA:
          linLoopbackControl.LinControlFrame.Data[txFrameDataIndex] = txFrameToSend->Data[txFrameDataIndex];
          if (txFrameDataIndex + 1 >= txFrameToSend->Length)
          {
            /* Last data byte will be transmitted, then the checksum */
            linTransmitState = STATE_CHECKSUM;
          }
          LinUartPutChar(txFrameToSend->Data[txFrameDataIndex]);
          txFrameDataIndex++;
          break;

        case STATE_CHECKSUM:
          linLoopbackControl.LinControlFrame.Checksum = txFrameToSend->Checksum;
          linTransmitState = STATE_SYNC_BREAK;
          linStopTx = 1;  /* Last byte will be sent */
          timeoutTimerStart();
          loopbackTmrRunning = 1;
          LinUartPutChar(txFrameToSend->Checksum);
          setLinIsTransmitting(0);                               //stop
          break;

        default: /* Bad state */
          linResetAll();
          break;
      }
    }
  }
  else /* module is in slave mode */
  {
    //LinUartDisableRxInterrupt();
    LinUartEnableBreakInterrupt();
    if (slaveTxBufferReadIndex >= slaveTxBufferLength)
    {
      /* Transmission was done - disable the interrupt */
      LinUartDisableTxInterrupt();
      //LinUartDisableRxInterrupt();    /* RX interrupt might arrive before break */
      LinUartEnableBreakInterrupt();
      resetSlaveStateMachine();
    }
    else
    {
      LinUartPutChar(slaveTxBuffer[slaveTxBufferReadIndex]);
      slaveTxBufferReadIndex++;
    }
  }
}

void LinLineBreakCallback(void)
{
  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    if (linTransmitState == STATE_SYNC_FIELD)
    {
      /* Stop the timer waiting for line break */
      LinTimeoutTimerStop();
      breakTmrRunning = 0;
      /* Enable the tx interrupt again */
      LinUartEnableTxInterrupt();
      linTransmitState = STATE_ID;
      LinUartPutChar(0x55);
    }
    else
    {
      linResetAll();
    }
  }
  else // mode == LIN_SLAVE_MODE
  {
    if (linSlaveReceptionState != STATE_SYNC_BREAK)
    {
      /* Caution: when both RX and LBD interrupts are enabled, RX arrives first! */
      resetSlaveStateMachine();
      LinUartDisableRxInterrupt();
    }
    else
    {
      linSlaveReceptionState = STATE_SYNC_BREAK;  /* Wait for the complete syncbreak */
      /* Enable the rx interrupt */
      LinUartEnableRxInterrupt();
    }
  }
}

void linResetAll(void)
{
  LinTimeoutTimerStop();
  LinUartReset();
  resetStateMachine();
  linTransmittionPending = 0;
  if(LinReadIndex != LinWriteIndex)
      setLinIsTransmitting(1);
  else
      setLinIsTransmitting(0);
}

void setLinIsTransmitting(uint8_t state)
{
    if(state)
    {
        linIsTransmitting = 1;
        LinUartEnableTxInterrupt();
    }
    else
    {
        linIsTransmitting = 0;
    }
}


void resetStateMachine(void)
{
  linStopTx = linIsTransmitting = linReceiving = linTransmittionPending = 0;
  rxDataIndex = 0;
  linTransmitState = STATE_SYNC_BREAK;
}

void LinFramingErrorCallback(void)
{
  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    if (linTransmitState == STATE_SYNC_FIELD)
    {
      /* Enable the tx interrupt again */
      LinUartEnableTxInterrupt();
      linTransmitState = STATE_ID;
      LinUartPutChar(0x55);
    }
    else
    {
      /* Line break detected when not wanted - error */
      linResetAll();
    }
  }
  else /* module is in slave mode */
  {
    linSlaveReceptionState = STATE_SYNC_BREAK;
  }
}

uint8_t getParityIdentifier(uint8_t id)
{
   uint8_t newId = id & 0x3Fu;
   newId |= (  (id >> 0u & 0x1u) ^ (id >> 1u & 0x1u)
             ^ (id >> 2u & 0x1u) ^ (id >> 4u & 0x1u)) << 6u;
   newId |= (  (id >> 1u & 0x1u) ^ (id >> 3u & 0x1u)
             ^ (id >> 4u & 0x1u) ^ (id >> 5u & 0x1u) ^ 0x1u) << 7u;
   return newId;
}

uint8_t calcChecksum(uint8_t* pAddress, uint8_t size, uint8_t checksumType,
                     uint8_t linId)
{
  uint16_t sum = 0x0000;
  uint8_t i = 0;
  if (ENHANCED_CHECKSUM == checksumType)
  {
    sum += getParityIdentifier(linId);
  }

  for (i = 0; i < size; i++)
    sum += pAddress[i];

  sum = (sum & 0x00FFu) + ((sum & 0xFF00u) >> 8u);
  if (sum & 0xFF00u) /* Did adding the carry bits result in a carry? */
  {
    sum += 1;  /* Add the last carry */
  }

   sum &= 0x00FFu;
   return (uint8_t)(~sum);
}

uint8_t UpdateTxFrameData(uint8_t ID, uint8_t* pData, uint8_t size)
{
  uint8_t ret = 0;
  /* Prevent buffer overflow */
  if (size > LIN_DATA_LENGTH)
    size = LIN_DATA_LENGTH;

  /* Try to find the frame and update it */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_RESPONSE)
    {
      memcpy(scheduler.Row[i].Frame.Data, pData, size);
      scheduler.Row[i].Frame.Length = size;
      ret = 1;
      break;
    }
  }
  return ret;
}

LIN_Frame* FindRxFrame(uint8_t ID)
{
  LIN_Frame* ret = NULL;
  /* Try to find the frame */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_REQUEST)
    {
      ret = &scheduler.Row[i].Frame;
      break;
    }
  }
  return ret;
}

void timeoutTimerStart(void)
{
  responseTimeoutValue = 0;

  /* Reset the counter value and enable the timer and its interrupt */
  LinTimeoutTimerStart();
}

void timeoutTimerStop(void)
{
  /* Stop the timer and its interrupt */
  LinTimeoutTimerStop();
}

int16_t LinGetFreeTx(void)
{
    int16_t r;
    r = (int16_t) LinReadIndex - (int16_t) LinWriteIndex;
    if (r <= 0)
        r += LIN_FRAME_TX_BUFFER_LENGTH;
    return r-1;
}

uint8_t LinWriteToTxBuffer(uint8_t linId,uint8_t msgType,uint8_t chksumType, uint8_t length, uint8_t data[])
{
      uint8_t i;
    linLoopbackControl.ExpectedDataLength = 0;
    linLoopbackControl.ReceivedDataLength = 0;

    if (LinGetFreeTx() >= 1)
    {
        TxFrameBuffer[LinWriteIndex].Type = msgType;
        TxFrameBuffer[LinWriteIndex].ID = getParityIdentifier(linId);
        TxFrameBuffer[LinWriteIndex].Length = length;
        TxFrameBuffer[LinWriteIndex].ChecksumType = chksumType;

        if(msgType == MASTER_RESPONSE)
        {
            for(i=0;i<length;i++)
            {
              TxFrameBuffer[LinWriteIndex].Data[i]=data[i];
            }
            if ( (linId == 0x3c) || (linId == 0x3d) || linId == 0x3e || linId == 0x3f){
                TxFrameBuffer[LinWriteIndex].Checksum = calcChecksum(&data[0],length,CLASSICAL_CHECKSUM,linId);
            }else{
                TxFrameBuffer[LinWriteIndex].Checksum = calcChecksum(&data[0],length,chksumType,linId);
            }

        }

        if (msgType == MASTER_REQUEST)
          TxFrameBuffer[LinWriteIndex].ResponseLength = 0; /* No response yet */

        LinWriteIndex++;

        if(linTransmittionPending == 0)
        {
            rxDataIndex=0;
            linTransmitState=STATE_SYNC_BREAK;
            linIsTransmitting = 1;
            linStopTx = 0;
            LinUartEnableTxInterrupt();

        }

        if(LinWriteIndex >= LIN_FRAME_TX_BUFFER_LENGTH)
        {
            LinWriteIndex = 0;
        }
        return 1;
    }
    else
    {
        return 0;
    }
}


uint8_t WriteToLinSlaveResponseBuffer(uint8_t linId, uint8_t bufferDir, uint8_t length,
                                      uint8_t chksumType, uint8_t* pData)
{
  uint8_t i;
  uint8_t index = slaveResponseMaxIndex;

  if (slaveResponseMaxIndex < LIN_ID_COUNT)
  {
    for (i = 0; i < slaveResponseMaxIndex; i++)
    {
      if (SlaveResponse[i].LinID == (linId & 0x3F))    //check if Lin ID is exists, if yes -> rewrite the data
      {
        index = i;
        break;
      }
    }

    SlaveResponse[index].LinID = linId & 0x3F;
    SlaveResponse[index].BufferDir = bufferDir;
    SlaveResponse[index].ChecksumType = chksumType;

    if (bufferDir == BUFFER_DIR_TX)
    {
      SlaveResponse[index].TxDataLength = length;
      SlaveResponse[index].RxDataLength = 0;
      SlaveResponse[index].Checksum = calcChecksum(&pData[0],length,chksumType,linId);

      for (i = 0; i < length; i++)
      {
        SlaveResponse[index].Data[i] = pData[i];
      }
    }
    else
    {
      SlaveResponse[index].TxDataLength = 0;
      SlaveResponse[index].RxDataLength = length;
    }

    // new response added only when corresponding index not found
    if (index == slaveResponseMaxIndex )
      slaveResponseMaxIndex++;
    return 1;
  }
  else
  {
    return 0;
  }
}

void resetSlaveStateMachine(void)
{
  linSlaveReceptionState = STATE_SYNC_BREAK;
}

void SendSlaveResponse(uint8_t index)
{
  blinkLedLin(GREEN_LIN_LED);
  uint8_t i;
  slaveTxBufferReadIndex = 0;
  slaveTxBufferLength = 0;
  linLoopbackControl.LinControlFrame.Length = 0;
  /* Fill buffer */
  for (i = 0; i < SlaveResponse[index].TxDataLength && i < LIN_DATA_LENGTH; i++)
  {
    linLoopbackControl.LinControlFrame.Length++;
    linLoopbackControl.LinControlFrame.Data[i] = SlaveResponse[index].Data[i];
    slaveTxBuffer[i] = SlaveResponse[index].Data[i];
  }
  slaveTxBuffer[i] = SlaveResponse[index].Checksum;
  linLoopbackControl.LinControlFrame.Checksum = SlaveResponse[index].Checksum;
  i++;
  slaveTxBufferLength = i;  /* Number of bytes */
  /* Enable interrupt */
  LinUartEnableTxInterrupt();
}

void calcAndUpdateTimeoutValue(uint8_t numberOfBytes)
{
  responseTimeoutMax = (numberOfBytes + 1) * 14 + 1;
}

void SlaveResponseTxRxCallback(SlaveResponseDataStruct* pInputData)
{
  blinkLedLin(GREEN_LIN_LED);
  if (pInputData->BufferDir == BUFFER_DIR_RX)
  {
    /* Slave received some data */
    if (pInputData->RxDataLength >= 2)
    {

    }
  }
  else if (pInputData->BufferDir == BUFFER_DIR_TX)
  {

  }
}


void resetSlaveLoopbackControl(void)
{
  linLoopbackControl.Enabled = 0;
  /* We do not need to receive anything anymore */
  LinUartDisableRxInterrupt();
}


void MasterRequestTxRxCallback(LIN_Frame* pFrame)
{
  blinkLedLin(GREEN_LIN_LED);
  /* Try to find correct buffer in the scheduler for the response */
  LIN_Frame* pFr = FindRxFrame(pFrame->ID & 0x3f);
  /* Copy the data to the scheduler frame buffer */
  if (pFr != NULL)
    memcpy(pFr->Data, pFrame->Data, pFrame->Length);

  if (pFrame->ResponseLength > 8)
  {
      LinErrorCallback(pFrame->ID & 0x3f,PROTOCOL_ERR_LIN_DATA_TOO_LONG);
  }
  else
  {
      if (pFrame->ResponseLength > 0)
      {
          /* Receive data */
      }
  }
}

void MasterResponseTxCallback(LIN_Frame* pFrame)
{
  blinkLedLin(GREEN_LIN_LED);
}

uint8_t processMasterReceivedFrame(LIN_Frame* pTxFrame, uint8_t* pRxData)
{
  blinkLedLin(GREEN_LIN_LED);
  uint8_t ret = 0;
  /* Correct checksum - data successfully received */
  /* Prevent buffer overflow */
  pTxFrame->ResponseLength %= RX_BUFFER_LENGTH;
  uint8_t index = 3;
  index %= RX_BUFFER_LENGTH;
  LinChecksum checksumType=pTxFrame->ChecksumType;
  uint8_t unprotectID=pTxFrame->ID & 0x3f;
  if (unprotectID == 0x3c ||unprotectID == 0x3d || unprotectID == 0x3e ||unprotectID == 0x3f ){
    checksumType=CHECKSUM_CLASSICAL;
  }
  if (pRxData[pTxFrame->ResponseLength] == calcChecksum(pRxData + index, pTxFrame->ResponseLength - 3,
      checksumType,
      pTxFrame->ID))
  {
    pTxFrame->ResponseLength -= 3;
    /* We know that LIN_DATA_LENGTH < RX_BUFFER_LENGTH - 3*/
    for (uint8_t i = 0; i < pTxFrame->ResponseLength && i < LIN_DATA_LENGTH; i++)
      pTxFrame->Data[i] = rxData[i + 3];
    ret = 1;
  }
  return ret;
}




uint8_t LinStop(){
    linRun = false;
    LinUartDisableRxInterrupt();
    LinUartDisableTxInterrupt();
    LinUartDisableBreakInterrupt();
    LinTimeoutTimerStop();
    linReceiving = 0;
    linTransmittionPending = 0;
    linStopChannelRequestFlag = 0;
    ledLinRedBlink = 0;
    ledLinGreenBlink = 0;
    return 0;
}
void LinAutoStart(){
    linEchoTx = (GetConfigurationLINAddr()->txEcho);
    if (GetConfigurationLINAddr()->autoStart){
        InitLin(GetConfigurationLINAddr());
    }
}

uint8_t LinStopChannelRequest(uint8_t linChannel, uint8_t channelToTXResponse){
  channelToResponse=channelToTXResponse;
  switch(linChannel)
  {
      case 0:
          linStopChannelRequestFlag = 1;
          return 0;
          break;
      default:
        return 1;
        break;

  }
}

void LinChannelStoppedCallback(){
  //Set led
  LED_LIN_RED_OFF();
  LED_LIN_GREEN_OFF();
}

void LinLedTimerCallback(){
  if (ledLinGreenBlink){
    counterLinBlinkGreen++;
    if ( counterLinBlinkGreen<GREEN_BLINK_DURATION )
      LED_LIN_GREEN_OFF();
    else
      LED_LIN_GREEN_ON();
    if ( counterLinBlinkGreen>=GREEN_BLINK_DURATION*2 )
      counterLinBlinkGreen=ledLinGreenBlink=0;
  }else if (ledLinRedBlink){
    counterLinBlinkRed++;
    if ( counterLinBlinkRed<RED_BLINK_DURATION )
      LED_LIN_RED_ON();
    else
      LED_LIN_RED_OFF();
    if ( counterLinBlinkRed>=RED_BLINK_DURATION*2 )
      counterLinBlinkRed=ledLinRedBlink=0;
  }
}
uint8_t blinkLedLin(uint8_t Led){
  switch(Led){

    case RED_LIN_LED:
      if (ledLinRedBlink){
        return 1;
      }else{
        ledLinRedBlink=1;
        return 0;
      }
      break;

    case GREEN_LIN_LED:
      if (ledLinGreenBlink){
        return 1;
      }else{
        ledLinGreenBlink=1;
        return 0;
      }
    default:
      break;
  }
  return 1;
}

uint8_t SetConfigurationLin(uint8_t Register){


    uint8_t ack;
    switch(Register & 0x03)
    {
        case 0x01:
            ack = 1;
            GetConfigurationLINAddr()->Baudrate = BAUDRATE_9600;
            break;
        case 0x02:
            ack = 1;
            GetConfigurationLINAddr()->Baudrate = BAUDRATE_19200;
            break;
        default:
            ack = 0;
    }

    GetConfigurationLINAddr()->autoStart=(Register>>4)&0x01;

    switch((Register>>2)&0x03)
    {
        case 1:
            GetConfigurationLINAddr()->DeviceMode = LIN_MODE_MASTER;
            break;
        case 0:
            GetConfigurationLINAddr()->DeviceMode = LIN_MODE_SLAVE;
            break;
        case LIN_SNIFFER_MODE:
            GetConfigurationLINAddr()->DeviceMode = LIN_SNIFFER_MODE;
            break;
        default:
            ack = 0;
    }

    if(ack)
    {
        if((Register >> 5)&0x01)
        {
            GetConfigurationLINAddr()->AMLR = AUTOLEN;
        }
        else
        {
            GetConfigurationLINAddr()->AMLR = HARDCODED;
        }
        if((Register >> 6)&0x01)
        {
            GetConfigurationLINAddr()->ChecksumType = CHECKSUM_ENHANCED;
            if (GetConfigurationLINAddr()->AMLR == HARDCODED)
                ack = 0;
        }
        else
        {
            GetConfigurationLINAddr()->ChecksumType = CHECKSUM_CLASSICAL;
        }
        linEchoTx = (GetConfigurationLINAddr()->txEcho = ((Register & 0x80) != 0));
    }
  return !ack;
}
