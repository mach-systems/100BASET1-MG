/*
 * can.c
 *
 *  Created on: 9. 8. 2021
 *      Author: petr_kolar
 */

#include "string.h"
#include "config.h"
#include "stdbool.h"
#include "can.h"
#include "timestamp.h"
#include "tools.h"
#include "tcpServer.h"
#include "udpServer.h"
#include <stdio.h>
#define TDC_FILTER  0x0
#define FDCAN_COMPENSATION_THRESHOLD    2000000 /* Delay compensation shall be used for Data baud rate above 2 Mbit */

#define MAX_DATA_SEG1_LEN               32      /* Maximum value of time quanta in Data Bit Segment 1 (HW constraint) */
#define MAX_DATA_SEG2_LEN               16      /* Maximum value of time quanta in Data Bit Segment 2 (HW constraint) */



/* Inspired by the similar variable in stm32h7xx_hal_fdcan.c */
static const uint8_t DLCtoBytes[] = {0, 1,  2,  3,  4,  5,  6,  7,
                                     8, 12, 16, 20, 24, 32, 48, 64};

CanChannelStatusTypedef Can1ChannelState = {0, true, false, UINT8_MAX};
CanChannelStatusTypedef Can2ChannelState = {0, true, false, UINT8_MAX};

CanChannnelProperties Can1Property = {CAN1};
CanChannnelProperties Can2Property = {CAN2};

uint8_t canConfigUnlock;



/*
 * Convert SamplePoint enum to its numerical meaning. Does not check for value
 * sanity, this should be enforced by the caller.
 */
static double samplePointToCoef(SamplePoint sp);

/*
 * Convert sample point numerical value (in tenths of percent) to SamplePoint
 * enum value. Does not check for value sanity, this should be enforced by
 * caller.
 */
static SamplePoint coefToSamplePoint(uint16_t c);

/*
 * Convert integer value to HAL FDCAN_DLC value.
 */
static uint8_t IntToDLC(uint8_t dataLength, uint32_t *DLC);

/*
 *  Get can configuration values for specified baudrate. Computation works for
 *  80 MHz clock; sample point is accurate up to baud rate 2 MHz, then it is
 *  rounded.
 *  For data part, actual sample point may change - values above 82.5 % are
 *  rounded to 5 % due to hardware constraints (=> 87.5 % is not possible).
 */
static uint8_t BaudToParametrs(uint8_t baudRate, uint8_t* pSPoint, uint8_t *prescaler,
                               uint8_t *tseg1, uint8_t *tseg2, uint8_t dataBr);


/*
 * Convert HAL message to standard message.
 */
static void TxMessageToFormat(FDCAN_TxEventFifoTypeDef *pTxHeader, uint8_t* pTxData,
        CanMessageStruct *pMessage, uint8_t channel);

/*
 * Convert HAL message to standard message.
 */
static void RxMessageToFormat(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t pRxData[],
        CanMessageStruct *pMessage, uint8_t channel);

/*
 * Configure only basic CAN settings.
 */
static CAN_RET ConfigureCanSimple(uint8_t channel);

/*
 * Calculate CAN baud rate from supplied configuration.
 */
static uint32_t paramsToBaud(CanInitStruct* pConf);

/*
 * Count the difference between supplied data length and possible
 * CAN FD data length.
 */
uint8_t getPaddingLength(uint8_t datalen);

/* Enable/Disable the CAN LED, Color Red/Green, status On/Off */
static void enableLedCan(uint8_t chanChannel, LedColor color, LedState status);

void CanInit(void)
{
    Can1ChannelState.TimestampIndex = TimestampInit();
    Can2ChannelState.TimestampIndex = TimestampInit();
    CanStartChannel(CAN1_NUM, 1);
    CanStartChannel(CAN2_NUM, 1);
}





uint8_t SetConfigurationCan(uint8_t *pConfRegisters)
{
    uint8_t ret;
    uint8_t channelNum = pConfRegisters[0] & 0x03;
    if ((channelNum) != CAN1_NUM || (channelNum) != CAN2_NUM)
    {
        ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    }
    else if (( (channelNum == CAN1_NUM) && Can1ChannelState.CanChannelRunning) ||
            ((channelNum == CAN2_NUM) && Can2ChannelState.CanChannelRunning))
    {
        ret = CAN_IS_ALREADY_RUNNING;
    }
    else
    {
        ret = CAN_OK;
        CanInitStruct *pSetCanConf;
        if(channelNum == CAN1_NUM)
            pSetCanConf = &GetCanConfigurationAddr(CAN1_NUM)->CanConfiguration;
        else
            pSetCanConf = &GetCanConfigurationAddr(CAN2_NUM)->CanConfiguration;
        pSetCanConf->PreciseTimingSet = false;

        switch (pConfRegisters[1] & 0x0F)
        {
            case 0:
                pSetCanConf->ArbitrationSPoint = SP_60;
                break;
            case 1:
                pSetCanConf->ArbitrationSPoint = SP_62_5;
                break;
            case 2:
                pSetCanConf->ArbitrationSPoint = SP_65;
                break;
            case 3:
                pSetCanConf->ArbitrationSPoint = SP_67_5;
                break;
            case 4:
                pSetCanConf->ArbitrationSPoint = SP_70;
                break;
            case 5:
                pSetCanConf->ArbitrationSPoint = SP_72_5;
                break;
            case 6:
                pSetCanConf->ArbitrationSPoint = SP_75;
                break;
            case 7:
                pSetCanConf->ArbitrationSPoint = SP_77_5;
                break;
            case 8:
                pSetCanConf->ArbitrationSPoint = SP_80;
                break;
            case 9:
                pSetCanConf->ArbitrationSPoint = SP_82_5;
                break;
            case 10:
                pSetCanConf->ArbitrationSPoint = SP_85;
                break;
            case 11:
                pSetCanConf->ArbitrationSPoint = SP_87_5;
                break;
            case 12:
                pSetCanConf->ArbitrationSPoint = SP_90;
                break;
            default:    // Reserved
                ret = CAN_ERROR;
                break;
        }

        if (pConfRegisters[1] & 0x10)
        {
            pSetCanConf->Mode = SilentMode;
        }
        else
        {
            pSetCanConf->Mode = NormalMode;
        }

        pSetCanConf->AutoStart = pConfRegisters[1] & 0x20;
        if ((pConfRegisters[1] & 0xc0) == 0)
        {
            pSetCanConf->Protocol = CAN;
        }
        else if ((pConfRegisters[1] & 0xc0) == 0x40)
        {
            pSetCanConf->Protocol = ISO_CAN_FD;
        }
        else
        {
            // Reserved
            ret = CAN_ERROR;
        }

        switch ((pConfRegisters[2] & 0x07))
        {
            case 0:
                pSetCanConf->ArbitrationBaud = BAUDRATE_125k;
                break;
            case 1:
                pSetCanConf->ArbitrationBaud = BAUDRATE_250k;
                break;
            case 2:
                pSetCanConf->ArbitrationBaud = BAUDRATE_500k;
                break;
            case 3:
                pSetCanConf->ArbitrationBaud = BAUDRATE_1M;
                break;
            default:
                ret = CAN_ERROR;
                break;
        }

        pSetCanConf->ArbitrationSJW = (pConfRegisters[3] & 0x7F) + 1;
        pSetCanConf->DataSJW = (pConfRegisters[4] & 0x0F) + 1;

        switch ((pConfRegisters[4] & 0x70) >> 4)
        {
            case 0:
                pSetCanConf->DataBaud = BAUDRATE_1M;
                break;
            case 1:
                pSetCanConf->DataBaud = BAUDRATE_2M;
                break;
            case 2:
                pSetCanConf->DataBaud = BAUDRATE_4M;
                break;
            case 3:
                pSetCanConf->DataBaud = BAUDRATE_8M;
                break;
            default:
                ret = CAN_ERROR;
                break;
        }

        switch ((pConfRegisters[5] & 0x0F))
        {
            case 0:
                pSetCanConf->DataSPoint = SP_60;
                break;
            case 1:
                pSetCanConf->DataSPoint = SP_62_5;
                break;
            case 2:
                pSetCanConf->DataSPoint = SP_65;
                break;
            case 3:
                pSetCanConf->DataSPoint = SP_67_5;
                break;
            case 4:
                pSetCanConf->DataSPoint = SP_70;
                break;
            case 5:
                pSetCanConf->DataSPoint = SP_72_5;
                break;
            case 6:
                pSetCanConf->DataSPoint = SP_75;
                break;
            case 7:
                pSetCanConf->DataSPoint = SP_77_5;
                break;
            case 8:
                pSetCanConf->DataSPoint = SP_80;
                break;
            case 9:
                pSetCanConf->DataSPoint = SP_82_5;
                break;
            case 10:
                pSetCanConf->DataSPoint = SP_85;
                break;
            case 11:
                pSetCanConf->DataSPoint = SP_87_5;
                break;
            case 12:
                pSetCanConf->DataSPoint = SP_90;
                break;
            default:
                ret = CAN_ERROR;
                break;
        }

        // In this implementation, NominalTimeSeg1 won't be > 255
        uint8_t tmp = (uint8_t) pSetCanConf->ArbitrationTimeSegment1;
        if (BaudToParametrs(pSetCanConf->ArbitrationBaud, &pSetCanConf->ArbitrationSPoint,
                        &pSetCanConf->ArbitrationPrescaler, &tmp,
                        &pSetCanConf->ArbitrationTimeSegment2, 0) == CAN_INCORRECT_PARAMETER)
        {
            ret = CAN_ERROR;
        }
        else
            pSetCanConf->ArbitrationTimeSegment1 = tmp;

        if (BaudToParametrs(pSetCanConf->DataBaud, &pSetCanConf->DataSPoint,
                        &pSetCanConf->DataPrescaler, &pSetCanConf->DataTimeSegment1,
                        &pSetCanConf->DataTimeSegment2, 1) == CAN_INCORRECT_PARAMETER)
        {
            ret = CAN_ERROR;
        }
    }
    return ret;
}

uint8_t SetConfigurationCanTim(uint8_t *pConfRegisters)
{
    uint8_t ret = CAN_OK;
    CanInitStruct *pSetCanConf;
    uint8_t channelNum = pConfRegisters[0] & 0x03;
    if ((channelNum) != CAN1_NUM || (channelNum) != CAN2_NUM)
    {
        ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    }
    else if (( (channelNum == CAN1_NUM) && Can1ChannelState.CanChannelRunning) ||
            ((channelNum == CAN2_NUM) && Can2ChannelState.CanChannelRunning))
    {
        ret = CAN_IS_ALREADY_RUNNING;
    }
    else
    {
        pSetCanConf = &GetCanConfigurationAddr(channelNum)->CanConfiguration;

        pSetCanConf->PreciseTimingSet = true;

        if (pConfRegisters[1] & 0x10)
            pSetCanConf->Mode = SilentMode;
        else
            pSetCanConf->Mode = NormalMode;

        pSetCanConf->AutoStart = pConfRegisters[1] & 0x20;
        if ((pConfRegisters[1] & 0xc0) == 0)
        {
            pSetCanConf->Protocol = CAN;
        }
        else if ((pConfRegisters[1] & 0xc0) == 0x40)
        {
            pSetCanConf->Protocol = ISO_CAN_FD;
        }
        else
        {
            // Reserved
        }

        pSetCanConf->ArbitrationTimeSegment1 = pConfRegisters[2] + 1;
        pSetCanConf->ArbitrationTimeSegment2 = (pConfRegisters[3] & 0x7F) + 1;
        pSetCanConf->ArbitrationPrescaler = pConfRegisters[4] + 1;
        pSetCanConf->ArbitrationSJW = (pConfRegisters[5] & 0x7F) + 1;
        pSetCanConf->DataTimeSegment1 = (pConfRegisters[6] & 0x1F) + 1;
        pSetCanConf->DataTimeSegment2 = (pConfRegisters[7] & 0x0F) + 1;
        pSetCanConf->DataSJW = ((pConfRegisters[7] & 0xF0) >> 4) + 1;
        pSetCanConf->DataPrescaler = (pConfRegisters[8] & 0x1F) + 1;
    }
    return ret;
}

CAN_RET ConfigureCAN(uint8_t channel)
{
    CAN_RET ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        FDCAN_HandleTypeDef *pFdcan = (channel==CAN1_NUM?CAN1:CAN2);
        CONFIGURATION_CAN *canConf = GetCanConfigurationAddr(channel);

        HAL_FDCAN_DeInit(pFdcan);

        // Set parameters
        if (canConf->CanConfiguration.Mode)
            pFdcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
        else
            pFdcan->Init.Mode = FDCAN_MODE_NORMAL;

        if (canConf->CanConfiguration.Protocol == CAN)
            pFdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        else if (canConf->CanConfiguration.Protocol == ISO_CAN_FD)
            pFdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;

        pFdcan->Init.NominalSyncJumpWidth = canConf->CanConfiguration.ArbitrationSJW;
        pFdcan->Init.DataSyncJumpWidth = canConf->CanConfiguration.DataSJW;

        pFdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_64;
        pFdcan->Init.TxBuffersNbr = 0;
        pFdcan->Init.RxBuffersNbr = 0;
        pFdcan->Init.RxFifo0ElmtsNbr = RX_FIFO_SIZE;
        pFdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
        pFdcan->Init.RxFifo1ElmtsNbr = 0;
        pFdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
        pFdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        pFdcan->Init.StdFiltersNbr = MAX_FILTERED_CAN_IDS;
        pFdcan->Init.ExtFiltersNbr = MAX_FILTERED_CAN_IDS;
        if (CAN1_NUM == channel)
        {
            pFdcan->Init.MessageRAMOffset = 0;
        }
        else    // CAN2_NUM == channel
        {
            pFdcan->Init.MessageRAMOffset = 1280;
        }
        pFdcan->Init.AutoRetransmission = ENABLE;
        pFdcan->Init.TxFifoQueueElmtsNbr = TX_FIFO_SIZE;
        pFdcan->Init.TxEventsNbr = TX_FIFO_SIZE;
        pFdcan->Init.NominalPrescaler = canConf->CanConfiguration.ArbitrationPrescaler;
        pFdcan->Init.NominalTimeSeg1 = canConf->CanConfiguration.ArbitrationTimeSegment1;
        pFdcan->Init.NominalTimeSeg2 = canConf->CanConfiguration.ArbitrationTimeSegment2;
        pFdcan->Init.DataPrescaler = canConf->CanConfiguration.DataPrescaler;
        pFdcan->Init.DataTimeSeg1 = canConf->CanConfiguration.DataTimeSegment1;
        pFdcan->Init.DataTimeSeg2 = canConf->CanConfiguration.DataTimeSegment2;

        if (HAL_FDCAN_Init(pFdcan) != HAL_OK)
          ret = CAN_ERROR;
        // Throw away frames that do not match any filter
        HAL_FDCAN_ConfigGlobalFilter(pFdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

        if (paramsToBaud(&canConf->CanConfiguration) > FDCAN_COMPENSATION_THRESHOLD)
        {
            uint8_t tdo = (canConf->CanConfiguration.DataTimeSegment1 + 1) & 0x7f;
            HAL_FDCAN_ConfigTxDelayCompensation(pFdcan, tdo, TDC_FILTER);
            HAL_FDCAN_EnableTxDelayCompensation(pFdcan);
        }
    }
    return ret;
}

CAN_RET ConfigureCanSimple(uint8_t channel)
{
    CAN_RET ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        FDCAN_HandleTypeDef *pFdcan = (channel==CAN1_NUM?CAN1:CAN2);
        CONFIGURATION_CAN *canConf = GetCanConfigurationAddr(channel);

        HAL_FDCAN_DeInit(pFdcan);

        // Set parameters
        pFdcan->Init.Mode = FDCAN_MODE_NORMAL;

        if (canConf->CanConfiguration.Protocol == CAN)
            pFdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        else if (canConf->CanConfiguration.Protocol == ISO_CAN_FD)
            pFdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;

        pFdcan->Init.NominalSyncJumpWidth = canConf->CanConfiguration.ArbitrationSJW;
        pFdcan->Init.DataSyncJumpWidth = canConf->CanConfiguration.DataSJW;
        pFdcan->Init.NominalPrescaler = canConf->CanConfiguration.ArbitrationPrescaler;
        pFdcan->Init.NominalTimeSeg1 = canConf->CanConfiguration.ArbitrationTimeSegment1;
        pFdcan->Init.NominalTimeSeg2 = canConf->CanConfiguration.ArbitrationTimeSegment2;
        pFdcan->Init.DataPrescaler = canConf->CanConfiguration.DataPrescaler;
        pFdcan->Init.DataTimeSeg1 = canConf->CanConfiguration.DataTimeSegment1;
        pFdcan->Init.DataTimeSeg2 = canConf->CanConfiguration.DataTimeSegment2;

        if (HAL_FDCAN_Init(pFdcan) != HAL_OK)
          ret = CAN_ERROR;

        if (paramsToBaud(&canConf->CanConfiguration) > FDCAN_COMPENSATION_THRESHOLD)
        {
            uint8_t tdo = (canConf->CanConfiguration.DataTimeSegment1 + 1) & 0x7f;
            HAL_FDCAN_ConfigTxDelayCompensation(pFdcan, tdo, TDC_FILTER);
            HAL_FDCAN_EnableTxDelayCompensation(pFdcan);
        }
    }
    return ret;
}

uint8_t CanStartChannel(uint8_t channel, uint8_t runningAsInterface)
{
    uint8_t ret;
    FDCAN_HandleTypeDef *phfdcan;
    HAL_StatusTypeDef status;
    CanChannelStatusTypedef * stateCanChannel;
    if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        stateCanChannel = (channel==CAN1_NUM?&Can1ChannelState:&Can2ChannelState);
        if (stateCanChannel->CanChannelRunning)
            ret = CAN_IS_ALREADY_RUNNING;
        else
        {
            phfdcan = (channel==CAN1_NUM?CAN1:CAN2);
            TimestampStart(stateCanChannel->TimestampIndex);

            ConfigureCAN(channel);
            status = HAL_FDCAN_ConfigInterruptLines(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                                             | FDCAN_TT_FLAG_ERROR_LEVEL_CHANGE
                                                             | FDCAN_IT_ARB_PROTOCOL_ERROR
                                                             | FDCAN_IT_TX_EVT_FIFO_NEW_DATA,
                                                             FDCAN_INTERRUPT_LINE0);
            status |= HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                                             | FDCAN_TT_FLAG_ERROR_LEVEL_CHANGE
                                                             | FDCAN_IT_ARB_PROTOCOL_ERROR
                                                             | FDCAN_IT_TX_EVT_FIFO_NEW_DATA,
                                                             0);

            status |= CanReconfigureFilter(!runningAsInterface, channel);
            status |= HAL_FDCAN_Start(phfdcan);

            if (status != HAL_OK)
                ret = CAN_ERROR;
            else
            {
                // Setup Echo configuration
                stateCanChannel->CanEchoRxChannel = GetCanConfigurationAddr(channel)->CanConfiguration.EchoConfig & 1;
                stateCanChannel->CanEchoTxChannel = GetCanConfigurationAddr(channel)->CanConfiguration.EchoConfig & 2;
                stateCanChannel->CanChannelRunning = runningAsInterface;
                if (stateCanChannel->CanChannelRunning)
                {
                    if (channel == CAN1_NUM)
                    {
                        LED_CAN1_GREEN_ON();
                        LED_CAN1_RED_OFF();
                    }
                    else
                    {
                        LED_CAN2_GREEN_ON();
                        LED_CAN2_RED_OFF();
                    }
                }

                ret = CAN_OK;
            }
        }
    }
    else
    {
        ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    }

    return ret;
}

uint8_t CanStopChannel(uint8_t channel)
{
    uint8_t ret;
    if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        ret = CAN_OK;
        //HAL_FDCAN_Stop(&CAN1);
        CanReconfigureFilter(1,channel);
        Can1ChannelState.CanChannelRunning = false;
        LED_CAN1_GREEN_OFF();
        LED_CAN1_RED_OFF();
        TimestampDeinit(CanGetChannelState(channel)->TimestampIndex);
    }
    else
        ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    return ret;
}

uint8_t CanGetChannelTimestamp(uint8_t channel, uint8_t *pMessageLength, uint8_t* pDataToSend, uint8_t errorFrameTimestamp)
{
    uint8_t ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    if (channel == CAN1_NUM || channel == CAN2_NUM || channel == CAN_ALL_CHANNELS)
    {
        uint8_t startOfTimestamp = errorFrameTimestamp ? 2 : 1;
        ret = CAN_OK;
        pDataToSend[0] = channel;
        uint64_t ts = (CanGetChannelState(channel)->CanChannelRunning) ? CanGetTimestamp(channel)
                                                                                       : 0;
        FillTimestamp(&pDataToSend[startOfTimestamp], NULL, &ts);
        *pMessageLength = MESSAGE_TIMESTAMP_SIZE + startOfTimestamp;
    }
    return ret;
}

uint8_t CanSendMessage(CanMessageStruct *message)
{
    bool messageProtocol = false; // Check if message must be send by FDCAN
    FDCAN_TxHeaderTypeDef txHeader;
    HAL_StatusTypeDef halRet;
    uint8_t ret = CAN_OK;
    uint8_t canChannel;
    CanChannnelProperties * canProp;

    txHeader.Identifier = message->Id;


    // Note that when requirement for maximum value for ID is not met, weird things
    // can happen (e.g. inadvertently transmitted Remote or Error frames!)
    if (!message->EXTId)
    {
        txHeader.Identifier &= CAN_ID_MASK;
        txHeader.IdType = FDCAN_STANDARD_ID;
    }
    else
    {
        txHeader.Identifier &= CAN_EXTID_MASK;
        txHeader.IdType = FDCAN_EXTENDED_ID;
    }

    if (!message->RTR)
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
    else
        txHeader.TxFrameType = FDCAN_REMOTE_FRAME;

    if (!message->BRS)
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    else
    {
        txHeader.BitRateSwitch = FDCAN_BRS_ON;
        messageProtocol = true;
    }

    if (!message->ESI)
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    else
    {
        txHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
        messageProtocol = true;
    }

    if (!message->FDF)
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    else
    {
        txHeader.FDFormat = FDCAN_FD_CAN;
        messageProtocol = true;
    }

    if (IntToDLC(message->DLC, &txHeader.DataLength))
    {
        ret = CAN_WRONG_DLC;
    }
    if (message->CANChannel == CAN1_NUM || message->CANChannel == CAN2_NUM)
    {
        canChannel = message->CANChannel;
    }
    else
    {
        ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    }
    if (CAN_OK == ret)
    {
        canProp  =  CanGetChannelProperty(canChannel); /* Get property of CAN channel*/
        txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
        if (ret == CAN_OK)
        {
            txHeader.MessageMarker = canProp->canTxDataIndexChannel;
            memcpy(canProp->canTxDataChannel[canProp->canTxDataIndexChannel], message->Data, message->DLC);
            canProp->canTxDataIndexChannel = (canProp->canTxDataIndexChannel + 1) % TX_FIFO_SIZE;
            if (GetCanConfigurationAddr(canChannel)->CanConfiguration.Protocol == CAN && messageProtocol)
                ret = CAN_WRONG_PROTOCOL;
            else
            {
                halRet = HAL_FDCAN_AddMessageToTxFifoQ(canProp->CanHandler, &txHeader, message->Data);
                if (halRet != HAL_OK)
                {
                    if (HAL_FDCAN_ERROR_FIFO_FULL == (canProp->CanHandler)->ErrorCode)
                    {
                        /* Message for CAN gateway send the buffer full here with this format*/
                        ret = CAN_ERROR_FIFO_FULL;

                    }
                    canProp->counterCanBlinkRed = 1;
                    ret = (canProp->CanHandler)->ErrorCode;
                }
                else
                {
                    canProp->counterCanBlinkGreen = 1;
                    ret = CAN_OK;
                }
            }
        }
    }
    return ret;
}
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    FDCAN_TxEventFifoTypeDef txEvent;
    uint8_t* TxData;
    CanMessageStruct message;
    uint8_t channel = 0;
    CanChannnelProperties * canProp;
    uint64_t ts;

    if (CAN1 == hfdcan)
        channel = CAN1_NUM;
    else if (CAN2 == hfdcan)
        channel = CAN2_NUM;

    canProp  =  CanGetChannelProperty(channel); /* Get property of CAN channel*/
    canProp->errorFrameBurstChannel = 0;
    ts = CanGetTimestamp(channel);

    if ((TxEventFifoITs & FDCAN_FLAG_TX_EVT_FIFO_NEW_DATA ) != 0)
    {
        uint8_t ret = HAL_FDCAN_GetTxEvent(hfdcan, &txEvent);
        if (ret == HAL_OK)
        {

            enableLedCan(channel, RedLed, LedOff);
            TxData = canProp->canTxDataChannel[txEvent.MessageMarker];
            TxMessageToFormat(&txEvent, TxData, &message, channel);
            message.TimeStamp = ts; // Take the earliest time stamp we have
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    CanMessageStruct message;
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[CAN_MAX_DATALEN];
    uint8_t channel = CAN1_NUM;
    CanChannnelProperties * canProp;
    uint64_t ts;

    if (CAN1 == hfdcan)
        channel = CAN1_NUM;
    else if (CAN2 == hfdcan)
        channel = CAN2_NUM;

    canProp  =  CanGetChannelProperty(channel); /* Get property of CAN channel*/
    ts = CanGetTimestamp(channel);

    canProp->errorFrameBurstChannel = 0;

    /* Retrieve Rx messages from RX FIFO0 if possible */
    while ((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        && HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
            enableLedCan(channel, RedLed, LedOff);
            canProp->LedCanGreenBlink = 1;
            message.CANChannel = channel;
            RxMessageToFormat(&RxHeader, RxData, &message, channel);
            message.TimeStamp = ts; // Take the earliest time stamp we have

            uint8_t datalen = DLCtoBytes[RxHeader.DataLength];

            /* It is strongly suggested to keep the possibility to jump to System Booloader from application */
            if (RxHeader.Identifier == 0x1fffffff && datalen == 4 && RxData[0] == 0
                    && RxData[1] == 1 && RxData[2] == 2 && RxData[3] == 3)
            {
        #ifdef Bootloader   /* HTTP Bootloader */
                /* Cannot go to bootloader directly from ISR */
                BootloaderRequest = 2;
        #else
        #ifdef NoBootloader /* STM System Bootloader */
                BootloaderRequest = 1;
        #endif
        #endif

                return;
            }

            /* Send the information about received CAN frame to the virtual COM port */
            char sendBuffer[MAX_CMD_LEN];
            uint16_t dataWritten = 0;
            dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                    "ID: %lx, ", RxHeader.Identifier);
            char *chan = (hfdcan == CAN1) ? "CAN1" : "CAN2";
            dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                    "channel: %s, ", chan);
            dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                    "datalen: %d, ", datalen);
            dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                    "data: ");
            for (uint8_t i = 0; i < datalen; i++)
            {
                dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                        "%x", RxData[i]);
                if (i != datalen - 1)
                    dataWritten += snprintf(sendBuffer + dataWritten,
                            MAX_CMD_LEN, ", ");
            }
            dataWritten += snprintf(sendBuffer + dataWritten, MAX_CMD_LEN,
                    "\r\n");

            TcpEnqueueResponse((uint8_t*) sendBuffer, dataWritten);
            UdpEnqueueResponse((uint8_t*) sendBuffer, dataWritten);
            UsbTransmitResponse((uint8_t*) sendBuffer, dataWritten);
      }
}

HAL_StatusTypeDef CanReconfigureFilter(uint8_t activateProtocol, uint8_t canChannel)
{
    HAL_StatusTypeDef ret = HAL_OK;
    // Reconfigure the filter to receive only protocol messages / all messages
    FDCAN_FilterTypeDef filter;
    FDCAN_HandleTypeDef * canInterface;
    canInterface = (canChannel==CAN1_NUM?CAN1:CAN2);
    if (activateProtocol)
    {
        uint32_t id = GetCanConfigurationAddr(canChannel)->CanConfiguration.CanRxId.Id;

        filter.IdType = (id >> 31) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        if (filter.IdType == FDCAN_EXTENDED_ID)
        {
            filter.IdType = FDCAN_STANDARD_ID;
            filter.FilterIndex = 0;
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterConfig = FDCAN_FILTER_DISABLE;
            filter.FilterID1 = 0x00000000;
            filter.FilterID2 = 0x00000000;
            filter.IsCalibrationMsg = 0;
            ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Disable standard
            filter.IdType = FDCAN_EXTENDED_ID;
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            filter.FilterID1 = id & CAN_EXTID_MASK;
            filter.FilterID2 = CAN_EXTID_MASK;
            ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Enable extended
        }
        else
        {
            filter.IdType = FDCAN_EXTENDED_ID;
            filter.FilterIndex = 0;
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterConfig = FDCAN_FILTER_DISABLE;
            filter.FilterID1 = 0;
            filter.FilterID2 = 0;
            filter.IsCalibrationMsg = 0;
            ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Disable extended
            filter.IdType = FDCAN_STANDARD_ID;
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            filter.FilterID1 = id & CAN_ID_MASK;
            filter.FilterID2 = CAN_ID_MASK;
            ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Enable standard
        }
    }
    else
    {
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0x00000000;
        filter.FilterID2 = 0x00000000;
        ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Enable all standard
        filter.IdType = FDCAN_EXTENDED_ID;
        ret |= HAL_FDCAN_ConfigFilter(canInterface, &filter);   // Enable all extended
    }

    return ret;
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef ProtocolStatus;
    HAL_FDCAN_GetProtocolStatus(hfdcan, &ProtocolStatus);
    uint8_t channel;
    uint8_t good = 1;
    CanChannnelProperties * canProp;

    if (CAN1 == hfdcan)
        channel = CAN1_NUM;
    else if (CAN2 == hfdcan)
        channel = CAN2_NUM;
    else
    {
        good = 0;
    }
    if (good)
    {
        canProp  =  CanGetChannelProperty(channel); /* Get property of CAN channel i*/
        if (canProp->errorFrameBurstChannel < CAN_ERROR_FRAME_BURST_SIZE)
        {
            canProp->errorFrameBurstChannel++;
        }
        if (canProp->errorFrameFilterSendChannel)
        {
            canProp->errorFrameFilterSendChannel = 0;
        }

        if (ProtocolStatus.RxBRSflag
            && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
            && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE )
        {
            canProp->LedCanRedBlink = 1;
        }
        if (ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
            && ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
            canProp->LedCanRedBlink = 1;
        if (ProtocolStatus.ErrorPassive == 1)
            enableLedCan(channel, RedLed, LedOn);

        if ((ProtocolStatus.RxBRSflag
             && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
             && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
            || (ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
                && ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE))
        {
            // Manually clear the error flag in the peripheral
            if (hfdcan->ErrorCode & FDCAN_IR_PED)
                hfdcan->ErrorCode &= ~FDCAN_IR_PED;
            if (hfdcan->ErrorCode & FDCAN_IR_PEA)
                hfdcan->ErrorCode &= ~FDCAN_IR_PEA;
        }

        if (1U == ProtocolStatus.BusOff)
        {
            CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        }

        if (!CanIsRunning(channel))
        {
            HAL_FDCAN_Stop(hfdcan);
            HAL_FDCAN_Start(hfdcan);    // Restart the peripheral to abort the request
        }
    }
}

void RxMessageToFormat(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t pRxData[],
        CanMessageStruct *pMessage, uint8_t channel) {
    /*
     * The time counter is driven by CAN time and the smallest unit is bit time -> we must correct this
     * value to get uS.
     */

    pMessage->Id = pRxHeader->Identifier;
    pMessage->EXTId = pRxHeader->IdType != FDCAN_STANDARD_ID;
    pMessage->RTR = pRxHeader->RxFrameType != FDCAN_DATA_FRAME;
    pMessage->BRS = pRxHeader->BitRateSwitch != FDCAN_BRS_OFF;
    pMessage->ESI = pRxHeader->ErrorStateIndicator != FDCAN_ESI_ACTIVE;
    pMessage->FDF = pRxHeader->FDFormat != FDCAN_CLASSIC_CAN;
    pMessage->DLC = DLCtoBytes[pRxHeader->DataLength];
    pMessage->Data = pRxData;
    pMessage->TimeStamp = CanGetTimestamp(channel);
}

uint64_t CanGetTimestamp(uint8_t channel)
{
    uint64_t timestamp = 0;
    if (CAN1_NUM == channel || CAN2_NUM == channel)
    {
        timestamp = GetTimestamp(CanGetChannelState(channel)->TimestampIndex);
    }
    return timestamp;
}


uint8_t IntToDLC(uint8_t dataLength, uint32_t *pDLC)
{
    switch (dataLength)
    {
        case 0:
            *pDLC = FDCAN_DLC_BYTES_0;
            break;
        case 1:
            *pDLC = FDCAN_DLC_BYTES_1;
            break;
        case 2:
            *pDLC = FDCAN_DLC_BYTES_2;
            break;
        case 3:
            *pDLC = FDCAN_DLC_BYTES_3;
            break;
        case 4:
            *pDLC = FDCAN_DLC_BYTES_4;
            break;
        case 5:
            *pDLC = FDCAN_DLC_BYTES_5;
            break;
        case 6:
            *pDLC = FDCAN_DLC_BYTES_6;
            break;
        case 7:
            *pDLC = FDCAN_DLC_BYTES_7;
            break;
        case 8:
            *pDLC = FDCAN_DLC_BYTES_8;
            break;
        case 12:
            *pDLC = FDCAN_DLC_BYTES_12;
            break;
        case 16:
            *pDLC = FDCAN_DLC_BYTES_16;
            break;
        case 20:
            *pDLC = FDCAN_DLC_BYTES_20;
            break;
        case 24:
            *pDLC = FDCAN_DLC_BYTES_24;
            break;
        case 32:
            *pDLC = FDCAN_DLC_BYTES_32;
            break;
        case 48:
            *pDLC = FDCAN_DLC_BYTES_48;
            break;
        case 64:
            *pDLC = FDCAN_DLC_BYTES_64;
            break;
        default:
            return CAN_WRONG_DLC;
            break;
    }
    return CAN_OK;
}

uint8_t BaudToParametrs(uint8_t baudRate, uint8_t *pSPoint, uint8_t *prescaler,
                        uint8_t *pTseg1, uint8_t *pTseg2, uint8_t dataBr)
{
    uint8_t ret;
    if (NULL == pSPoint || NULL == prescaler || NULL == pTseg1 || NULL == pTseg2)
        ret = CAN_INCORRECT_PARAMETER;
    else
    {
        ret = CAN_OK;
        uint32_t tq;
        switch (baudRate)
        {
            case BAUDRATE_125k:
                *prescaler = 16;
                tq = (FDCAN_CLOCK / *prescaler / 125000);   // 40
                break;

            case BAUDRATE_250k:
                *prescaler = 8;
                tq = (FDCAN_CLOCK / *prescaler / 250000);   // 40
                break;

            case BAUDRATE_500k:
                *prescaler = 4;
                tq = (FDCAN_CLOCK / *prescaler / 500000);   // 40
                break;

            case BAUDRATE_1M:
                *prescaler = 2;
                tq = (FDCAN_CLOCK / *prescaler / 1000000);  // 40
                break;

            case BAUDRATE_2M:
                *prescaler = 1;
                tq = (FDCAN_CLOCK / *prescaler / 2000000);  // 40
                break;

            case BAUDRATE_4M:
                *prescaler = 1;
                // Sample point rounded to next lowest multiple of 5%
                tq = (FDCAN_CLOCK / *prescaler / 4000000);  // 20
                break;

            case BAUDRATE_8M:
                *prescaler = 1;
                // Sample point rounded to next lowest multiple of 10%
                tq = (FDCAN_CLOCK / *prescaler / 8000000);  // 10
                break;

            default:
                ret = CAN_INCORRECT_PARAMETER;
                break;
        }

        if (CAN_OK == ret)
        {
            *pTseg1 = tq * samplePointToCoef(*pSPoint);
            *pTseg2 = tq - *pTseg1;

            // Recalculate the values when out of range - SP can be changed
            if (dataBr)
            {
                if ((*pTseg1 - 1) > MAX_DATA_SEG1_LEN || *pTseg2 > MAX_DATA_SEG2_LEN)
                {
                    *prescaler *= 2;
                    tq /= 2;

                    *pTseg1 = tq * samplePointToCoef(*pSPoint);
                    *pTseg2 = tq - *pTseg1;
                }
                uint16_t sp = (uint16_t) 1000 * (*pTseg1) / (*pTseg1 + *pTseg2);
                // Adjust Sample Point to its real value - it might have changed slightly
                *pSPoint = coefToSamplePoint(sp);
            }
            (*pTseg1)--;
        }
    }

    return ret;
}



void CanLedTimerCallback(void)
{
    CanChannnelProperties * canProp;
    for (uint8_t i = 0; i < CAN_NUMBER_OF_CHANNELS; i++)
    {
        canProp  =  CanGetChannelProperty(i); /* Get property of CAN channel i*/
        if (canProp->LedCanGreenBlink)
        {
            canProp->counterCanBlinkGreen++;
            if (canProp->counterCanBlinkGreen < GREEN_BLINK_DURATION)
                enableLedCan(i, GreenLed, LedOff);
            else
                enableLedCan(i, GreenLed, LedOn);
            if (canProp->counterCanBlinkGreen >= GREEN_BLINK_DURATION*2)
            {
                canProp->counterCanBlinkGreen = canProp->LedCanGreenBlink = 0;
                if (!CanGetChannelState(i)->CanChannelRunning)
                    enableLedCan(i, GreenLed, LedOff); // End blink by OFF
            }
        }
        if (canProp->LedCanRedBlink)
        {
            canProp->counterCanBlinkRed++;
            if (canProp->counterCanBlinkRed<RED_BLINK_DURATION)
                enableLedCan(i, RedLed, LedOn);
            else
                enableLedCan(i, RedLed, LedOff);
            if (canProp->counterCanBlinkRed >= RED_BLINK_DURATION * 2)
                canProp->counterCanBlinkRed = canProp->LedCanRedBlink = 0;
        }
        /* This counter control sending of the error frames */
        if (canProp->errorFrameBurstChannel == CAN_ERROR_FRAME_BURST_SIZE)
        {
           canProp->errorFramePauseCounterChannel++;
          if (CAN_DELAY_BETWEEN_ERROR_FRAMES == canProp->errorFramePauseCounterChannel)
          {
              canProp->errorFramePauseCounterChannel = 0;
              canProp->errorFrameFilterSendChannel = 1;
          }
        }
        else
        {
            canProp->errorFramePauseCounterChannel = 0;
        }
    }
}

double samplePointToCoef(SamplePoint sp)
{
    return 0.6 + sp * 0.025;
}

SamplePoint coefToSamplePoint(uint16_t c)
{
    return ((c - 600) / 25);
}

CAN_RET CanGetSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen)
{
    CAN_RET ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    if (datalen < 2)
        ret = PROTOCOL_ERR_DATALEN;
    else if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        ret = CAN_OK;
        CanInitStruct* pStruct = &GetCanConfigurationAddr(channel)->CanConfiguration;

        pData[0] = (pStruct->ArbitrationSPoint & 0xf) | (pStruct->Protocol << 6);
        pData[1] = (pStruct->ArbitrationBaud & 0x7);
        if (datalen > 3)
        {
            if (pStruct->Protocol == ISO_CAN_FD)
            {
                pData[2] = (pStruct->DataBaud - 3) << 4;
                pData[3] = pStruct->DataSPoint & 0xf;
            }
            else
                pData[2] = pData[3] = 0xff;
        }
    }
    return ret;
}

CAN_RET CanCheckSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen)
{
    CAN_RET ret = CAN_CHANNEL_NOT_IMPLEMENTED;
    if (datalen < 2)
        ret = PROTOCOL_ERR_DATALEN;
    else if (channel == CAN1_NUM || channel == CAN2_NUM)
    {
        ret = CAN_OK;

        if ((pData[1] & 0x7) > BAUDRATE_1M || (pData[0] & 0xf) > SP_90
            || (pData[0] >> 6) > ISO_CAN_FD)
        {
            ret = CAN_ERROR;
        }

        if ((pData[0] >> 6) == ISO_CAN_FD && datalen >= 4)
        {
            // DBAUD starts at 0 == 1 M
            if (((pData[2] >> 4) + 3) < BAUDRATE_1M || ((pData[2] >> 4) + 3) > BAUDRATE_8M
                || (pData[3] & 0xf) > SP_90)
            {
                ret = CAN_ERROR;
            }
        }
    }
    return ret;
}

CAN_RET CanSetSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen)
{
   CAN_RET ret = CAN_CHANNEL_NOT_IMPLEMENTED;
   if (datalen < 2)
       ret = PROTOCOL_ERR_DATALEN;
   else if (channel == CAN1_NUM || channel == CAN2_NUM)
   {
       // CanGetLockState() ensured by caller
       ret = CAN_OK;

       CanInitStruct* pStruct = &GetCanConfigurationAddr(channel)->CanConfiguration;
       pStruct->PreciseTimingSet = 0;
       pStruct->ArbitrationSPoint = pData[0] & 0xf;
       pStruct->Protocol = pData[0] >> 6;
       pStruct->ArbitrationBaud = pData[1] & 0x7;

       // In this implementation, NominalTimeSeg1 won't be > 255
       uint8_t tmp = (uint8_t) pStruct->ArbitrationTimeSegment1;
       if (BaudToParametrs(pStruct->ArbitrationBaud, &pStruct->ArbitrationSPoint,
                       &pStruct->ArbitrationPrescaler, &tmp,
                       &pStruct->ArbitrationTimeSegment2, 0) == CAN_INCORRECT_PARAMETER)
       {
           ret = CAN_ERROR;
       }
       else
           pStruct->ArbitrationTimeSegment1 = tmp;
       if (pStruct->ArbitrationBaud > BAUDRATE_1M)
           ret = CAN_ERROR;

       if (pStruct->Protocol == ISO_CAN_FD && datalen >= 4)
       {
           pStruct->DataBaud = (pData[2] >> 4) + 3; // DBAUD starts at 0 == 1 M
           pStruct->DataSPoint = pData[3] & 0xf;
           if (BaudToParametrs(pStruct->DataBaud, &pStruct->DataSPoint,
                           &pStruct->DataPrescaler, &pStruct->DataTimeSegment1,
                           &pStruct->DataTimeSegment2, 1) == CAN_INCORRECT_PARAMETER)
           {
               ret = CAN_ERROR;
           }
           if (pStruct->DataBaud < BAUDRATE_1M)
               ret = CAN_ERROR;
       }
       if (CAN_OK == ret)
           ConfigureCanSimple(channel);
   }
   return ret;
}






uint32_t CanWaitTrEnd(uint8_t channel)
{
    uint32_t ret = 0;
    FDCAN_HandleTypeDef * pCanHandle;

    if (CAN1_NUM == channel || CAN2_NUM == channel )
    {
        pCanHandle =  CAN1_NUM == channel ? CAN1 : CAN2;
        uint8_t attempts = 0;
        while (HAL_FDCAN_GetTxFifoFreeLevel(pCanHandle) != TX_FIFO_SIZE)
        {
            osDelay(10);
            if (++attempts >= CAN_WAIT_THRESHOLD)
                break;
        }
        ret = (attempts < CAN_WAIT_THRESHOLD);
    }
    return ret;
}

void CanToggleLock(uint8_t change)
{

    if (change == 1)
        canConfigUnlock = 1;
    else
        canConfigUnlock = 0;

}

uint8_t CanGetLockState(void)
{
    uint8_t state;
    state = canConfigUnlock == 1;
    return (state);
}

uint32_t paramsToBaud(CanInitStruct* pConf)
{
    uint32_t ret = 0;
    if (NULL != pConf && pConf->DataPrescaler != 0 && pConf->DataTimeSegment1 + pConf->DataTimeSegment2 + 1 != 0)
        ret = (FDCAN_CLOCK / (pConf->DataPrescaler) / (pConf->DataTimeSegment1 + pConf->DataTimeSegment2 + 1));
    return ret;
}

uint8_t getPaddingLength(uint8_t datalen)
{
    // Valid CAN FD length: 0..8, 12, 16, 20, 24, 32, 48, 64
    uint8_t ret = 0;
    if (datalen > 8)
    {
        if (datalen <= 12)
            ret = 12 - datalen;
        else if (datalen <= 16)
            ret = 16 - datalen;
        else if (datalen <= 20)
            ret = 20 - datalen;
        else if (datalen <= 24)
            ret = 24 - datalen;
        else if (datalen <= 32)
            ret = 32 - datalen;
        else if (datalen <= 48)
            ret = 48 - datalen;
        else if (datalen <= 64)
            ret = 64 - datalen;
    }
    return ret;
}

void TxMessageToFormat(FDCAN_TxEventFifoTypeDef *pTxHeader, uint8_t* pTxData,
        CanMessageStruct *pMessage, uint8_t channel)
{
    /* The time counter is driven by CAN time and the smallest unit is bit time -> we must correct this
     * value to get uS. */

    pMessage->CANChannel = channel;
    pMessage->Id = pTxHeader->Identifier;
    pMessage->EXTId = pTxHeader->IdType != FDCAN_STANDARD_ID;
    pMessage->RTR = pTxHeader->TxFrameType != FDCAN_DATA_FRAME;
    pMessage->BRS = pTxHeader->BitRateSwitch != FDCAN_BRS_OFF;
    pMessage->ESI = pTxHeader->ErrorStateIndicator != FDCAN_ESI_ACTIVE;
    pMessage->FDF = pTxHeader->FDFormat != FDCAN_CLASSIC_CAN;
    pMessage->DLC = DLCtoBytes[pTxHeader->DataLength];
    pMessage->Data = pTxData;
    pMessage->TimeStamp = CanGetTimestamp(channel);
}


inline CanChannelStatusTypedef * CanGetChannelState(uint8_t channel)
{
    CanChannelStatusTypedef * retType;
    if (channel==CAN1_NUM)
        retType = &Can1ChannelState;
    else
        retType = &Can2ChannelState;
    return retType;
}


inline CanChannnelProperties * CanGetChannelProperty(uint8_t channel)
{
    CanChannnelProperties * retType;
    if (channel==CAN1_NUM)
        retType = &Can1Property;
    else
        retType = &Can2Property;
    return retType;
}

inline uint8_t CanIsRunning(uint8_t channel)
{
    return (CAN1_NUM == channel || CAN2_NUM == channel) ? CanGetChannelState(channel)->CanChannelRunning : 0;
}

void enableLedCan(uint8_t chanChannel, LedColor color, LedState status)
{
    if (chanChannel == CAN1_NUM)
    {
        if (color == GreenLed)
        {
            LED_CAN1_GREEN_SET(status);
        }
        else if (color == RedLed)
        {
            LED_CAN1_RED_SET(status);
        }
    }
    else if (chanChannel == CAN2_NUM)
    {
        if (color == GreenLed)
        {
            LED_CAN2_GREEN_SET(status);
        }
        else if (color == RedLed)
        {
            LED_CAN2_RED_SET(status);
        }
    }
}

void SendToCan(uint8_t* pData, uint16_t length)
{
  uint8_t retCode=0;
  for (int i = 0; i < length; i+=8)
  {
    CanMessageStruct message;
    message.BRS = false;
    message.CANChannel  = CAN1_NUM;
    message.DLC = (length-i) > 8 ? 8 : (length-i);
    message.Data  = &pData[i];
    message.ESI = false;
    message.EXTId = false;
    message.FDF = false;
    message.Id  = 0x11;
    message.RTR = false;
    retCode = CanSendMessage(&message);
    while (retCode == CAN_ERROR_FIFO_FULL)
    {
        osDelay(10);
        retCode = CanSendMessage(&message);
    }
  }
  return;
}
