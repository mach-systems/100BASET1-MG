/*
 * can.h
 *
 *  Created on: 9. 8. 2021
 *      Author: petr_kolar
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stdbool.h"
#include "main.h"



#define CAN_MAX_DATALEN                 64

#define CAN1                            &hfdcan1
#define CAN2                            &hfdcan2
#define FDCAN_CLOCK                     80000000    /* Peripheral has 80 MHz clock */

#define CAN1_NUM                        0
#define CAN2_NUM                        1
#define CAN_ALL_CHANNELS                0xFF
#define CAN_NUMBER_OF_CHANNELS          2
#define CAN_CHANNEL_SELECTION_MASK      0x03        /* Two most LSb */

#define RX_FIFO_SIZE                    32
#define TX_FIFO_SIZE                    32
#define CAN_WAIT_THRESHOLD              50          /* This many iterations use
                                                       when waiting for end */


#define CAN_RET                         uint8_t     /* Type of CAN driver return value */
#define CAN_OK                          0
#define CAN_ERROR                       1
#define CAN_CHANNEL_NOT_IMPLEMENTED     2
#define CAN_WRONG_DLC                   3
#define CAN_WRONG_PROTOCOL              4
#define CAN_IS_NOT_RUNNING              5
#define CAN_IS_ALREADY_RUNNING          7
#define CAN_INCORRECT_PARAMETER         6
#define CAN_START_ERROR                 8
#define PROTOCOL_ERR_DATALEN            9

#define CORRECTION_CONST                80          /* FDCAN CLK in MHz */

#define GREEN_BLINK_DURATION            100
#define RED_BLINK_DURATION              300

#define GREEN_CAN1_LED                  0
#define RED_CAN1_LED                    1
#define GREEN_CAN2_LED                  2
#define RED_CAN2_LED                    3

#define CAN_ID_LENGTH                   5
#define CAN_PROTOCOL_MAX_RESPONSE_LEN   64

#define IS_CAN_ID_EXT(ID)   ((ID >> 31) & 0x1)
#define IS_CAN_ID_FD(ID)    ((ID >> 30) & 0x1)
#define IS_CAN_ID_BRS(ID)   ((ID >> 29) & 0x1)

#define CAN_ID_MASK                     0x7ff       /* Standard ID: 11 bits */
#define CAN_EXTID_MASK                  0x1fffffff  /* Extended ID: 29 bits */

#define CAN_ERROR_STUFF                 0   /* CAN Bit Stuff error */
#define CAN_ERROR_FORM                  1   /* CAN Form error */
#define CAN_ERROR_ACK                   2   /* CAN Acknowledge error */
#define CAN_ERROR_BIT                   3   /* CAN Bit error */
#define CAN_ERROR_CRC                   4   /* CAN CRC error */
#define CAN_ERROR_FIFO_FULL             5   /* CAN FIFO full error use only for gateway*/

#define CAN_ERROR_FRAME_BURST_SIZE      10
#define CAN_DELAY_BETWEEN_ERROR_FRAMES  100

#define MAX_FILTERED_CAN_IDS       15 /* Number of protocols that can be used with CAN Gateway */

#define PROTOCOL_COUNT             3  /* Number of protocols that can be used with CAN Gateway */
#define MAX_FILTERED_CAN_IDS       15 /* Number of protocols that can be used with CAN Gateway */


/*
 * Baud rate - 125k to 8M
 */
typedef enum
{
    BAUDRATE_125k = 0,
    BAUDRATE_250k = 1,
    BAUDRATE_500k = 2,
    BAUDRATE_1M = 3,
    BAUDRATE_2M = 4,
    BAUDRATE_4M = 5,
    BAUDRATE_8M = 6
} CanBaud;

typedef enum
{
    SP_60 = 0,
    SP_62_5 = 1,
    SP_65 = 2,
    SP_67_5 = 3,
    SP_70 = 4,
    SP_72_5 = 5,
    SP_75 = 6,
    SP_77_5 = 7,
    SP_80 = 8,
    SP_82_5 = 9,
    SP_85 = 10,
    SP_87_5 = 11,
    SP_90 = 12
} SamplePoint;

typedef enum
{
    NormalMode = 0,
    SilentMode = 1
} AcknowledgeMode;

typedef enum
{
    CAN = 0,
    ISO_CAN_FD = 1
} CanProtocol;

typedef struct S_CAN_ID_CONFIG
{
    uint32_t Id;     /* CAN Frame Id  */
                     /* Value is 0 if not used */

} CanIdConfigTypedef;

typedef struct
{
    CanIdConfigTypedef FastDataFrame;   /* Alternative frame id to use for fast data frame echo */
                                        /* Value is 0 if not used */

} CanIdConfiguration;

typedef struct
{
    uint32_t Id         :29;
    uint8_t  Extended   :1;
} CanId_t;

/*
 * Settings of the CAN Gateway
 */
typedef struct
{
    uint8_t Channels[PROTOCOL_COUNT];
    CanId_t FilteredCanIds[MAX_FILTERED_CAN_IDS];
    uint8_t FilterEnabled       :1;
    uint8_t FilterIdsCount      :4;
    uint16_t Port;
    uint8_t IpAddress[4];
} CanGateway_t;

/*
 * Settings of the CAN channel.
 */
typedef struct
{
    CanProtocol Protocol;
    CanBaud DataBaud;
    CanBaud ArbitrationBaud;
    SamplePoint DataSPoint;
    SamplePoint ArbitrationSPoint;
    AcknowledgeMode Mode;
    uint8_t DataSJW;
    uint8_t ArbitrationSJW;
    bool AutoStart;
    bool PreciseTimingSet;
    uint16_t ArbitrationTimeSegment1;
    uint8_t ArbitrationTimeSegment2;
    uint8_t ArbitrationPrescaler;
    uint8_t DataTimeSegment1;
    uint8_t DataTimeSegment2;
    uint8_t DataPrescaler;
    uint8_t EchoConfig;
    CanIdConfigTypedef CanRxId; /* ID for CAN reception */
    CanIdConfigTypedef CanTxId; /* ID for CAN transmission */
    CanGateway_t CanGateway;
} CanInitStruct;

typedef struct
{
    bool FDF;
    bool ESI;
    bool BRS;
    bool RTR;
    bool EXTId;
    uint8_t CANChannel;
    uint8_t DLC;
    uint32_t Id;            /* CAN ID */
    uint8_t* Data;
    uint64_t TimeStamp;

} CanMessageStruct;


typedef struct
{
    FDCAN_HandleTypeDef * CanHandler;
    bool LedCanGreenBlink;
    bool LedCanRedBlink;
    uint8_t canTxDataIndexChannel;
    uint8_t canTxDataChannel[TX_FIFO_SIZE][CAN_MAX_DATALEN];
    uint16_t counterCanBlinkRed;
    uint16_t counterCanBlinkGreen;
    uint8_t errorFrameBurstChannel; /* Count the error messages if the value is grater than CAN_ERROR_FRAME_BURST_SIZE it indicate that the device is in error state and the error frames should be send every 100ms */
    uint8_t errorFrameFilterSendChannel; /* Indicate that the error frame should be send now */
    uint8_t errorFramePauseCounterChannel;
} CanChannnelProperties;

typedef struct
{
    uint8_t CanChannelRunning;         /* Flag to tell if CAN channel is running. Three different status running for Protocol CAN, running as interface, CAN running as Gateway */
    bool CanEchoTxChannel;
    bool CanEchoRxChannel;
    uint8_t TimestampIndex;
} CanChannelStatusTypedef;


void CanInit(void);

/*
 * Start CAN channel.
 * Parameter runningAsInterface determines if the channel is started for MACH
 * protocol usage or to be used as CAN interface.
 */
uint8_t CanStartChannel(uint8_t channel, uint8_t runningAsInterface);

/*
 * Stop CAN channel - meaning that gateway functionality is turned off.
 */
uint8_t CanStopChannel(uint8_t channel);

/*
 * Start CAN channel if its autoStart bit is set.
 */
void CanAutoStart(void);


/*
 *  Configure CAN. For configuration is use CONFIGURATION_CAN structure.
 */
CAN_RET ConfigureCAN(uint8_t channel);

/*
 * Send CAN message.
 */
uint8_t CanSendMessage(CanMessageStruct *message);

////Auxiliary functions
//
/*
 * Get message data from input data.
 */
uint8_t CanFillMessageFormat(uint8_t *data, uint8_t dataLen, CanMessageStruct *message);

/*
 * Fill output message with can configuration. NOTE that pData must have allocated
 * at least twelve bytes.
 */
uint8_t GetConfigurationCan(uint8_t channel, uint8_t *pData);
//
/*
 * Process the configuration data to actual configuration of CAN
 */
uint8_t SetConfigurationCan(uint8_t *ConfRegisters);

/*
 * Process the configuration data with explicit timing setting to actual configuration of CAN
 */
uint8_t SetConfigurationCanTim(uint8_t *ConfRegisters);

/*
 * Get time from startup of CAN channel in microseconds.
 * Send 9 byte message: first byte represents the channel number, next eight
 * bytes represent timestamp (LSB order).
 */
uint64_t CanGetTimestamp(uint8_t channel);

/*
 * Get actual timestamp of channel.
 */
uint8_t CanGetChannelTimestamp(uint8_t channel, uint8_t * pMessageLength, uint8_t DataToSend[], uint8_t errorFrameTimestamp);

/*
 * Millisecond timer callback for CAN LEDs.
 */
void CanLedTimerCallback(void);


/*
 * Get CAN simple configuration. Simple means that only channel mode, baudrate
 * (arbitration/data) and sample point (arbitration/data) is touched.
 * Data format:
 *
 *  Data 0
 *  +---------------------------------------------+
 *  | Bit 7 | Bits 6..4    | Bits 3..1 | Bit 0    |
 *  |=============================================|
 *  | x     | Sample point | Baudrate  | Protocol |
 *  +---------------------------------------------+
 *
 *  Data 1 (valid for CAN FD, otherwise 0)
 *  +--------------------------------------+
 *  | Bits 7..6 | Bits 5..3    | Bits 0..2 |
 *  |======================================|
 *  | x         | Sample point | Baud rate |
 *  +--------------------------------------+
 *
 */
CAN_RET CanGetSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen);

/*
 * Check CAN simple configuration for correctness.
 */
CAN_RET CanCheckSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen);

/*
 * Set CAN simple configuration. Simple means that only channel mode, baudrate
 * (arbitration/data) and sample point (arbitration/data) is touched.
 * Format same as in CanGetSimpleConfig().
 */
CAN_RET CanSetSimpleConfig(uint8_t channel, uint8_t* pData, uint8_t datalen);



/*
 * Send bootup packet with information about protocol RX ID.
 */
void CanSendBootupPkt(void);

/*
 * Try to wait for transmission end (transmit FIFO is empty). Only wait for
 * CAN_WAIT_THRESHOLD iterations, then fail.
 */
uint32_t CanWaitTrEnd(uint8_t channel);

/*
 * Enable / disable receiving only protocol messages.
 */
HAL_StatusTypeDef CanReconfigureFilter(uint8_t activateProtocol, uint8_t canInterface);

/*
 * Unlock / lock CAN configuration changes (from CAN).
 * When change == 1, unlock the lock. Lock it otherwise.
 */
void CanToggleLock(uint8_t change);

/*
 * Get state of the CAN configuration lock.
 * Return value: 0 - configuration is locked
 *               1 - configuration is unlocked
 */
uint8_t CanGetLockState(void);

/*
 * Turn on / off logging on all the channels.
 * Start stop CAN channel for logging when requested (or after power up).
 */
CAN_RET CanToggleLogging(uint8_t data);

/*
 * CAN channel status flags getters.
 */
uint8_t CanIsRunning(uint8_t channel);
uint8_t CanIsLogging(uint8_t channel);
CanChannelStatusTypedef * CanGetChannelState(uint8_t channel);

CanChannnelProperties * CanGetChannelProperty(uint8_t channel);


/*
 * Exported variables
 */
extern CanChannelStatusTypedef Can1ChannelState;
extern CanChannelStatusTypedef Can2ChannelState;

/* Simple send to CAN version for SDK*/
void SendToCan(uint8_t* pData, uint16_t length);
#endif /* INC_CAN_H_ */
