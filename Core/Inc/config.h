/*
 * config.h
 *
 *  Created on: Mar 22, 2023
 *      Author: Karel Hevessy
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdint.h>
#include <stddef.h> /* size_t */
#include "lin.h"
#include "can.h"
#include "switch.h"
#include "sjaSetting.h"
#include "kszSetting.h"
/* Default IP address and IPv4 length */
#define IP_ADDRESS_LENGTH           4
#define DEFAULT_IP0                 192
#define DEFAULT_IP1                 168
#define DEFAULT_IP2                 1
#define DEFAULT_IP3                 100
#define IP_MASK_LENGTH              4
#define DEFAULT_MASK0               255
#define DEFAULT_MASK1               255
#define DEFAULT_MASK2               255
#define DEFAULT_MASK3               0
#define DEFAULT_GW0                 0
#define DEFAULT_GW1                 0
#define DEFAULT_GW2                 0
#define DEFAULT_GW3                 0

/* Default CAN1 interface setting */
#define DEFAULT_DBAUD_CAN_CH1                   BAUDRATE_2M
#define DEFAULT_NBAUD_CAN_CH1                   BAUDRATE_500k
#define DEFAULT_MODE_CAN_CH1                    NormalMode
#define DEFAULT_DSJW_CAN_CH1                    4
#define DEFAULT_NSJW_CAN_CH1                    8
#define DEFAULT_DSP_CAN_CH1                     SP_80
#define DEFAULT_NSP_CAN_CH1                     SP_80
#define DEFAULT_PROTOCOL_CAN_CH1                ISO_CAN_FD
#define DEFAULT_AUTOSTART_CAN_CH1               false
#define DEFAULT_PRECISE_TIM_CAN_CH1             false
#define DEFAULT_DATA_PRESCALER_CAN_CH1          1
#define DEFAULT_DATA_TIME_SEG1_CAN_CH1          31
#define DEFAULT_DATA_TIME_SEG2_CAN_CH1          8
#define DEFAULT_ARBITRATION_PRESCALER_CAN_CH1   4
#define DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH1   31
#define DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH1   8
#define DEFAULT_ECHO_CONF_CH1                   3
#define DEFAULT_CAN_RXID                        0x123
#define DEFAULT_CAN_TXID                        0x321



#define DEFAULT_DBAUD_CAN_CH2                   BAUDRATE_2M
#define DEFAULT_NBAUD_CAN_CH2                   BAUDRATE_500k
#define DEFAULT_MODE_CAN_CH2                    NormalMode
#define DEFAULT_DSJW_CAN_CH2                    4
#define DEFAULT_NSJW_CAN_CH2                    8
#define DEFAULT_DSP_CAN_CH2                     SP_80
#define DEFAULT_NSP_CAN_CH2                     SP_80
#define DEFAULT_PROTOCOL_CAN_CH2                ISO_CAN_FD
#define DEFAULT_AUTOSTART_CAN_CH2               false
#define DEFAULT_PRECISE_TIM_CAN_CH2             false
#define DEFAULT_DATA_PRESCALER_CAN_CH2          1
#define DEFAULT_DATA_TIME_SEG1_CAN_CH2          31
#define DEFAULT_DATA_TIME_SEG2_CAN_CH2          8
#define DEFAULT_ARBITRATION_PRESCALER_CAN_CH2   4
#define DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH2   31
#define DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH2   8
#define DEFAULT_ECHO_CONF_CH2                   3

/* Default switch setting */
#define DEFAULT_ADDRESS_LEARNING                1
#define DEFAULT_EGRESS_ENABLED                  1
#define DEFAULT_MIRRORING_INGRESS_ENABLED       0
#define DEFAULT_MIRRORING_EGRESS_ENABLED        0
#define DEFAULT_INGRESS_ENABLED                 1
#define DEFAULT_MASTER_FORCED                   0
#define DEFAULT_MASTER_SELECTED                 1
#define DEFAULT_MIRROR_PORT                     5  /* Non valid port means disable mirroring */
#define DEFAULT_PORT_SPEED                      SPEED_SJA1105_100M
#define DEFAULT_PORT_SPEED_KSZ                  SPEED_KSZ_AUTO
#define DEFAULT_VLAN_ID                         0x22b
#define DEFAULT_VLAN_TAG_MIRRORED_INGRESS       0x0
#define DEFAULT_VLAN_TAG_MIRRORED_EGRESS        0x0

/* Default LIN interface setting */
#define DEFAULT_MODE_LIN            LIN_MODE_MASTER
#define DEFAULT_CHECKSUM_LIN        CHECKSUM_ENHANCED
#define DEFAULT_BAUDRATE_LIN        BAUDRATE_19200
#define DEFAULT_AMLR_LIN            AUTOLEN
#define DEFAULT_AUTOSTART_LIN       false
#define DEFAULT_TX_ECHO_LIN         1  /* Echo enabled */

/* Default TCP port */
#define DEFAULT_PORT                8000

/* MAC address length */
#define MAC_ADDRESS_LENGTH          6

#define CONFIG_BASE_ADDR            1                                       /* Address for saving configuration - first address is 1 */
#define INFO_BASE_ADDR              CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION) + 1             /* Save the information right after the configuration (and checksums) */
#define EXTRA_CONFIG_BASE_ADDR      INFO_BASE_ADDR \
                                    + sizeof(PRODUCT_INFORMATION) + 1
#define CAN1_CONFIG_BASE_ADDR       EXTRA_CONFIG_BASE_ADDR \
                                    + sizeof(EXTRA_CONFIGURATION) + 1      /* CAN1 configuration */
#define CAN2_CONFIG_BASE_ADDR       CAN1_CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION_CAN) + 1      /* CAN2 configuration */
#define SWITCH_CONFIG_BASE_ADDR     CAN2_CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION_CAN) + 1      /* SWITCH configuration */
#define LIN_BASE_ADDR               SWITCH_CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION_SWITCH) + 1      /* LIN configuration */

/* Lengths of the PRODUCT_INFORMATION arrays */
#define SN_LENGTH                   4
#define HW_INFO_LENGTH              6

#define EXTRA_CONFIG_RESERVED_SIZE  252     /* Total size of extra configuration: 256 */
#define CAN_CONFIG_RESERVED_SIZE    80      /* 128 - 48 = 80 */
#define LIN_CONFIG_RESERVED_SIZE    108     /* 128 - 20 = 108 */
#define SWITCH_CONFIG_RESERVED_SIZE 121     /* 128 - 27 = 121 */

#define VERSION_MINOR               0x09
#define VERSION_MAJOR               0x00

#define SIGNATURE_VALUE             0x55aa55aa55aa55aa
#define PRODUCT_ID                  5
#define VARIANT_ID                  1

/* Discovery protocol defines */
#define ID_DISCOVERY                0
#define ID_CHANGE_IP                1
#define ID_CHANGE_PORT              2
#define ID_CHANGE_ALL               3
#define ID_DEFAULT_CONFIG           4
#define ID_REBOOT                   5
#define ID_CHANGE_GW                6

#define RESPONSE_SUCCESS            0
#define RESPONSE_CHKSM_ERR          1
#define RESPONSE_ID_ERR             2

#define CRC_DATATYPE                uint8_t


#define T1_NUMBER_OF_PORTS         3
/*
 * Other modules may want to read the firmware version constant variable.
 */
extern const volatile uint16_t FirmwareVersion;

/*
 * Sum bytes in the supplied array.
 */
uint8_t ArraySum(const uint8_t* pArr, uint8_t size);

/*
 * Count one-byte sum of the supplied array.
 */
CRC_DATATYPE ArrayChecksum(const uint8_t* pArr, uint16_t size);

/*
 * Add bytes to checksum crc8. Function taken from project CAN-LIN-ECU.
 * Credit for algorithm: rcgldr from https://stackoverflow.com/
 * Polynomial: 0x07
 */
void AddToCrc(CRC_DATATYPE* crc, const uint8_t* pData, size_t length);



/*
 * Structure for configuration of the device.
 * Size of this struct must be always even! Or the writing to the EEPROM won't work
 */
typedef struct SConfig
{
    uint8_t IpAddress[IP_ADDRESS_LENGTH];
    uint8_t IpMask[IP_MASK_LENGTH];
    uint16_t Port;
    uint8_t MacAddress[MAC_ADDRESS_LENGTH];

} CONFIGURATION;

/*
 * Should be called at the beginning of the program.
 */
void InitNonVolatileData(void);

/*
 * Additional configuration. Total size is 1024 bytes.
 */
typedef struct SMoreConfig
{
    uint8_t DefaultGateway[IP_ADDRESS_LENGTH];
    uint8_t Reserved[EXTRA_CONFIG_RESERVED_SIZE];

} EXTRA_CONFIGURATION;

/*
 * CAN configuration plus reserved space.
 */
typedef struct CANConfig
{
    CanInitStruct CanConfiguration;
    uint8_t Reserved[CAN_CONFIG_RESERVED_SIZE];

} CONFIGURATION_CAN;


/*
 * LIN configuration plus reserved space.
 */
typedef struct LinConfig
{
    LinInitStruct LinConfiguration;
    uint8_t Reserved[LIN_CONFIG_RESERVED_SIZE];
} CONFIGURATION_LIN;


/*
 * SWITCH configuration plus reserved space.
 */
typedef struct SwitchConfig
{
    SwitchConfStruct SwitchConfiguration;
    uint8_t Reserved[SWITCH_CONFIG_RESERVED_SIZE];
} CONFIGURATION_SWITCH;

/*
 * Structure for product information (serial number and HW info number).
 */
typedef struct SProductInfo
{
    uint8_t SerialNumber[SN_LENGTH];
    uint8_t HardwareInfo[HW_INFO_LENGTH];
} PRODUCT_INFORMATION;


/*
 * Set hardware info.
 */
void SetHwInfo(uint8_t HwInfo[HW_INFO_LENGTH]);
/*
 * Set the device serial number.
 */
void SetSerialNumber(uint8_t SN[SN_LENGTH]);

/*
 * Getter for product information.
 */
PRODUCT_INFORMATION GetInformation(void);

/*
 * Save product information to FLASH.
 */
uint8_t WriteInformation(void);
/*
 * Change device MAC address (only in configuration array).
 */
uint8_t ConfigChangeMac(uint8_t* pData);

/*
 * Change IPv4 address (only in configuration array).
 */
uint8_t ConfigChangeIp(const uint8_t* pData);

/*
 * Change default gateway (only in the configuration array).
 */
uint8_t ConfigChangeGw(const uint8_t* pData);

/*
 * Convert mask from one-byte encoded value (e.g. 24) to the array value
 * (e.g. 255.255.255.0).
 */
void MaskToArray(uint8_t encoded, uint8_t* pArr);

/*
 * Convert one-byte encoded value to mask (see previous function).
 */
void MaskFromArray(uint8_t* pEncoded, uint8_t* pArr);

/*
 * Change subnet mask (only in configuration array). Supplied mask is in one-byte
 * encoded format.
 */
uint8_t ConfigChangeMask(uint8_t encoded);

/*
 * Change used TCP port (only in configuration array.
 */
uint8_t ConfigChangePort(const uint16_t num);

/*
 * Check that supplied array has data bytes same as our configured MAC address.
 * Caller is responsible that the array is at least 6 bytes.
 */
uint8_t CheckMacMatch(const uint8_t* pArr);
/*
 * Configuration changes via Multicast (or normal UDP communication). Also, build
 * a response to the second supplied buffer.
 * Return value: return 1 if response is to be sent.
 */
uint8_t CorrectMulticastReceived(const uint8_t* pRxBuf, const uint16_t* rxSize,
                       uint8_t* pTxBuf, uint16_t* pTxSize);

/*
 * Called from the UDP server after response to the configuration request was
 * sent.
 */
void MulticastResponseSent(const uint8_t* pRxBuf, const uint16_t* pRxSize,
                           const uint8_t* pTxBuf, const uint16_t* pTxSize);

/*
 * Try to load the configuration from flash. !!! Changes the global configuration
 * value even if the checksum is incorrect !!!
 */
uint8_t ReadConfiguration(void);

/*
 * Read extra configuration from eeprom
 */
uint8_t ReadExtraConfiguration(void);
/*
 * Read LIN configuration from eeprom
 */
uint8_t ReadConfigurationLin(void);
/*
 * Apply default configuration by the defines in this file (config.h).
 */
void DefaultConfiguration(void);

/*
 * Get LIN configuration from EEPROM
 */
uint8_t ReadConfigurationLIN(void);

/*
 * Read CAN channel configuration. Only supported channel is CAN1.
 */
uint8_t ReadConfigurationCan(uint8_t channel);

/*
 * Read Switch configuration from EEPROM
 */
uint8_t ReadConfigurationSwitch(void);
/*
 * Save configuration to the FLASH. Return 0 on success, 1 on failure.
 */
uint8_t WriteConfiguration(void);
/*
 * Apply default extra configuration by the defines in this file (config.h).
 */
void DefaultExtraConfiguration(void);
/*
 * Save extra configuration to the FLASH.
 */
uint8_t WriteExtraConfiguration(void);
/*
 * Save LIN configuration to the FLASH.
 */
uint8_t WriteConfigurationLin(void);

/*
 * Save CAN configuration to the FLASH.
 */
uint8_t WriteConfigurationCan(uint8_t channel);

/*
 * Save Switch configuration to the FLASH.
 */
uint8_t WriteSwitchConfiguration(void);

/*
 * Apply default configuration by the defines in this file (config.h) to CAN1 only.
 */
void DefaultConfigurationCanCh1(void);

/*
 * Apply default configuration by the defines in this file (config.h) to CAN2 only.
 */
void DefaultConfigurationCanCh2(void);
/*
 * Set Echo configuration CAN1
 */
void SetEchoConfigurationCanCh1(uint8_t echo );

/*
 * Apply default configuration by the defines in this file (config.h) to LIN
 */
void DefaultConfigurationLin(void);
/*
 * Load default switch configuration
 */
uint8_t DefaultConfigurationSwitch(void);

/*
 * Getter for the configuration.
 */
CONFIGURATION GetConfiguration(void);

/*
 * Get pointer to extra configuration
 */
const EXTRA_CONFIGURATION* GetExtraConfigurationAddr(void);

/*
 * Get pointer to LIN configuration structure
 */
LinInitStruct * GetConfigurationLINAddr(void);

/*
 *  Get CAN1 configuration
 */
CONFIGURATION_CAN GetConfigurationCAN1(void);

/*
 *  Get CAN2 configuration
 */
CONFIGURATION_CAN GetConfigurationCAN2(void);

/*
 * Get CAN configuration
 */
CONFIGURATION_CAN* GetCanConfigurationAddr(uint8_t channel);

extern CONFIGURATION Config;               // IP settings and MAC address
#endif /* INC_CONFIG_H_ */
