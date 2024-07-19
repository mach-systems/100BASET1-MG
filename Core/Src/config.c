/*
 * config.c
 *
 *  Created on: Mar 22, 2023
 *      Author: Karel Hevessy
 */

#include "config.h"
#include "eepromConfig.h"
#include "can.h"
#include "string.h"



// Load address is 0x80fffe0
const volatile uint16_t __attribute__((section(".version"))) FirmwareVersion  = VERSION_MINOR + (256 * VERSION_MAJOR);
#ifdef Bootloader
    const volatile uint64_t __attribute__((section(".signature"))) Signature = SIGNATURE_VALUE;
    // Use two separate sections to ensure correct order
    const uint8_t __attribute__((section(".product"))) ProductId = PRODUCT_ID;
    const uint8_t __attribute__((section(".variant"))) VariantId = VARIANT_ID;
#endif


    CONFIGURATION Config;               // IP settings and MAC address
    EXTRA_CONFIGURATION ExtraConfig;    // Default gateway
    PRODUCT_INFORMATION ProductInfo;    // SN and HW info
    CONFIGURATION_CAN configCan1;        // CAN configuration
    CONFIGURATION_CAN configCan2;        // CAN configuration
    CONFIGURATION_LIN configLin;        // LIN configuration
    CONFIGURATION_SWITCH configSwitch;  // SWITCH configuration

uint8_t ArraySum(const uint8_t* pArr, uint8_t size)
{
  uint8_t sum = 0;
  uint16_t i;
  for (i = 0; i < size; i++)
    sum += pArr[i];
  return sum;
}

CRC_DATATYPE ArrayChecksum(const uint8_t* pArr, uint16_t size)
{
  CRC_DATATYPE crc = 0;
  AddToCrc(&crc, pArr, size);
  return crc;
}

void AddToCrc(CRC_DATATYPE* crc, const uint8_t* pData, size_t length)
{
    size_t i, j;
    for (i = 0; i < length; i++)
    {
        *crc ^= pData[i];
        for (j = 0; j < 8; j++)
        {
            if ((*crc & 0x80) != 0)
                *crc = (uint8_t)((*crc << 1) ^ 0x07);
            else
                *crc <<= 1;
        }
    }
}

void InitNonVolatileData(void)
{
  //TestEepromAccess();

  // Prevent the hard-coded version to be optimized out (if the section is set
  // as KEEP() in the linker, it will be preserved either way)
  (void) FirmwareVersion;

  uint8_t pinState = HAL_GPIO_ReadPin(DEFAULT_SET_GPIO_Port, DEFAULT_SET_Pin) == GPIO_PIN_RESET;
  if (!ReadConfiguration() || pinState)
    DefaultConfiguration();
  if (!ReadExtraConfiguration() || pinState)
    DefaultExtraConfiguration();
  if (!ReadConfigurationSwitch() || pinState)
    DefaultConfigurationSwitch();
  if (!ReadConfigurationCan(CAN1_NUM) || pinState)
    DefaultConfigurationCanCh1();
  if (!ReadConfigurationCan(CAN2_NUM) || pinState)
    DefaultConfigurationCanCh2();
  if (!ReadConfigurationLin() || pinState)
      DefaultConfigurationLin();

  EECfgReadItem(INFO_BASE_ADDR, (uint8_t*) &ProductInfo, sizeof(PRODUCT_INFORMATION));

}

uint8_t WriteConfiguration(void)
{

    int8_t status = 0;
    size_t len = sizeof(CONFIGURATION);
    status = EECfgWriteItem(CONFIG_BASE_ADDR, (uint8_t*) &Config, len);

    if (status == 0)
      status |= EECfgWriteByte(CONFIG_BASE_ADDR + len, ArrayChecksum((uint8_t*) &Config, len));


  return (status == 0);
}

uint8_t WriteConfigurationLin(void)
{

    int8_t status = 0;
    size_t len = sizeof(CONFIGURATION_LIN);
    status = EECfgWriteItem(LIN_BASE_ADDR, (uint8_t*) &configLin, len);

    if (status == 0)
      status |= EECfgWriteByte(LIN_BASE_ADDR + len, ArrayChecksum((uint8_t*) &configLin, len));


  return (status == 0);
}

uint8_t WriteExtraConfiguration(void)
{

    uint8_t status = 0;
    size_t len = sizeof(EXTRA_CONFIGURATION);
    EECfgWriteItem(EXTRA_CONFIG_BASE_ADDR, (uint8_t*) &ExtraConfig, len);

    if (status == 0)
        status |= EECfgWriteByte(EXTRA_CONFIG_BASE_ADDR + len,
                                 ArrayChecksum((uint8_t*) &ExtraConfig, len));


  return (status == 0);
}

uint8_t WriteConfigurationCan(uint8_t channel)
{
    uint8_t ret = 0;
    size_t len = sizeof(CONFIGURATION_CAN);
    uint8_t status = 0;
    if (CAN1_NUM == channel)
    {
        EECfgWriteItem(CAN1_CONFIG_BASE_ADDR, (uint8_t*) &configCan1, len);

        if (status == 0)
            status |= EECfgWriteByte(CAN1_CONFIG_BASE_ADDR + len,
                                     ArrayChecksum((uint8_t*) &configCan1, len));


        ret = (status == 0);
    }
    else if (CAN2_NUM == channel)
    {

        EECfgWriteItem(CAN2_CONFIG_BASE_ADDR, (uint8_t*) &configCan2, len);

        if (status == 0)
            status |= EECfgWriteByte(CAN2_CONFIG_BASE_ADDR + len,
                                     ArrayChecksum((uint8_t*) &configCan2, len));


        ret = (status == 0);
    }
    return ret;
}

uint8_t CreateConfiguration(uint8_t IpAddress[IP_ADDRESS_LENGTH], uint16_t Port, uint8_t MacAddress[MAC_ADDRESS_LENGTH])
{
  Config.IpAddress[0] = IpAddress[0];
  Config.IpAddress[1] = IpAddress[1];
  Config.IpAddress[2] = IpAddress[2];
  Config.IpAddress[3] = IpAddress[3];

  Config.Port = Port;

  Config.MacAddress[0] = MacAddress[0];
  Config.MacAddress[1] = MacAddress[1];
  Config.MacAddress[2] = MacAddress[2];
  Config.MacAddress[3] = MacAddress[3];
  Config.MacAddress[4] = MacAddress[4];
  Config.MacAddress[5] = MacAddress[5];
  return 1;
}

void SetHwInfo(uint8_t HwInfo[HW_INFO_LENGTH])
{
  ProductInfo.HardwareInfo[0] = HwInfo[0];
  ProductInfo.HardwareInfo[1] = HwInfo[1];
  ProductInfo.HardwareInfo[2] = HwInfo[2];
  ProductInfo.HardwareInfo[3] = HwInfo[3];
  ProductInfo.HardwareInfo[4] = HwInfo[4];
  ProductInfo.HardwareInfo[5] = HwInfo[5];
}

PRODUCT_INFORMATION GetInformation(void)
{
  return ProductInfo;
}

uint8_t ReadConfiguration(void)
{
// Real EEPROM
  uint8_t ret = 0;
  uint8_t readData, status = 0;
  CONFIGURATION tmp;
  size_t len = sizeof(CONFIGURATION);

  status = EECfgReadItem(CONFIG_BASE_ADDR, (uint8_t*) &tmp, len);
  if (0 == status)
      status |= EECfgReadItem(CONFIG_BASE_ADDR + len, &readData, 1);

  Config = tmp;
  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION)))
    ret = 1;

  return ret;
}

uint8_t ReadConfigurationLin(void)
{
// Real EEPROM
  uint8_t ret = 0;
  uint8_t readData, status = 0;
  CONFIGURATION_LIN tmp;
  size_t len = sizeof(CONFIGURATION_LIN);

  status = EECfgReadItem(LIN_BASE_ADDR, (uint8_t*) &tmp, len);
  if (0 == status)
      status |= EECfgReadItem(LIN_BASE_ADDR + len, &readData, 1);

  configLin = tmp;
  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_LIN)))
    ret = 1;

  return ret;
}

uint8_t ReadExtraConfiguration(void)
{

  uint8_t ret = 0;
  uint8_t readData, status = 0;
  EXTRA_CONFIGURATION tmp;
  size_t len = sizeof(EXTRA_CONFIGURATION);
  status |= EECfgReadItem(EXTRA_CONFIG_BASE_ADDR, (uint8_t*) &tmp, len);

  if (status == 0)
    status |= EECfgReadItem(EXTRA_CONFIG_BASE_ADDR + len, &readData, 1);

  ExtraConfig = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(EXTRA_CONFIGURATION)))
    ret = 1;

  return ret;
}

uint8_t CheckMacMatch(const uint8_t* pArr)
{
  return (Config.MacAddress[0] == pArr[0] && Config.MacAddress[1] == pArr[1]
          && Config.MacAddress[2] == pArr[2] && Config.MacAddress[3] == pArr[3]
          && Config.MacAddress[4] == pArr[4] && Config.MacAddress[5] == pArr[5]);
}

void DefaultConfiguration(void)
{
  Config.IpAddress[0] = DEFAULT_IP0;
  Config.IpAddress[1] = DEFAULT_IP1;
  Config.IpAddress[2] = DEFAULT_IP2;
  Config.IpAddress[3] = DEFAULT_IP3;
  Config.IpMask[0] = DEFAULT_MASK0;
  Config.IpMask[1] = DEFAULT_MASK1;
  Config.IpMask[2] = DEFAULT_MASK2;
  Config.IpMask[3] = DEFAULT_MASK3;

  Config.Port = DEFAULT_PORT;

  // MAC address does not have any default value
}

uint8_t CorrectMulticastReceived(const uint8_t* pRxBuf, const uint16_t* pRxSize,
                       uint8_t* pTxBuf, uint16_t* pTxSize)
{
  uint8_t ret = 1;
  if (*pRxSize == 1 && pRxBuf[0] == ID_DISCOVERY)
  {
    CONFIGURATION Config = GetConfiguration();

    uint8_t mask;
    MaskFromArray(&mask, Config.IpMask);

    pTxBuf[0] = ID_DISCOVERY;
    pTxBuf[1] = Config.IpAddress[0];
    pTxBuf[2] = Config.IpAddress[1];
    pTxBuf[3] = Config.IpAddress[2];
    pTxBuf[4] = Config.IpAddress[3];
    MaskFromArray(pTxBuf + 5, Config.IpMask);
    pTxBuf[6] = Config.Port & 0xff;
    pTxBuf[7] = (Config.Port >> 8) & 0xff;
    pTxBuf[8] = Config.MacAddress[0];
    pTxBuf[9] = Config.MacAddress[1];
    pTxBuf[10] = Config.MacAddress[2];
    pTxBuf[11] = Config.MacAddress[3];
    pTxBuf[12] = Config.MacAddress[4];
    pTxBuf[13] = Config.MacAddress[5];
    pTxBuf[14] = ExtraConfig.DefaultGateway[0];
    pTxBuf[15] = ExtraConfig.DefaultGateway[1];
    pTxBuf[16] = ExtraConfig.DefaultGateway[2];
    pTxBuf[17] = ExtraConfig.DefaultGateway[3];
    pTxBuf[18] = ArraySum(pTxBuf + 1, 17);
    *pTxSize = 19;
  }
  else if (*pRxSize >= 7 && (pRxBuf[0] == ID_CHANGE_IP
                             || pRxBuf[0] == ID_CHANGE_PORT
                             || pRxBuf[0] == ID_CHANGE_ALL
                             || pRxBuf[0] == ID_DEFAULT_CONFIG
                             || pRxBuf[0] == ID_REBOOT
                             || pRxBuf[0] == ID_CHANGE_GW))
  {
    if (CheckMacMatch(pRxBuf + 1))
    {
      uint8_t checksum;

      if (pRxBuf[0] == ID_CHANGE_IP && *pRxSize >= 13)       // Change IP and mask
      {
        // Datalen = 1 (cmd) + 6 (MAC) + 5 + 1 = 13
        checksum = ArraySum(pRxBuf + 7, 5);
        if (pRxBuf[12] == checksum)
        {
          ConfigChangeIp(pRxBuf + 7);
          ConfigChangeMask(pRxBuf[11]);
          WriteConfiguration();
          *pTxSize = 1;
          pTxBuf[0] = RESPONSE_SUCCESS;
        }
        else
        {
          *pTxSize = 3;
          pTxBuf[0] = RESPONSE_CHKSM_ERR;
          pTxBuf[1] = pRxBuf[12];
          pTxBuf[2] = checksum;
        }
      }
      else if (pRxBuf[0] == ID_CHANGE_PORT && *pRxSize >= 10)  // Change port
      {
        // Datalen = 1 (cmd) + 6 (MAC) + 2 + 1 = 10
        checksum = ArraySum(pRxBuf + 7, 2);
        if (pRxBuf[9] == checksum)
        {
            // Port is not checked
          ConfigChangePort(pRxBuf[7] + (((uint16_t) pRxBuf[8]) << 8));
          WriteConfiguration();
          *pTxSize = 1;
          pTxBuf[0] = RESPONSE_SUCCESS;
        }
        else
        {
          *pTxSize = 3;
          pTxBuf[0] = RESPONSE_CHKSM_ERR;
          pTxBuf[1] = pRxBuf[9];
          pTxBuf[2] = checksum;
        }
      }
      else if (pRxBuf[0] == ID_CHANGE_ALL && *pRxSize >= 15)  // Change IP, mask and port
      {
        // Datalen = 1 (cmd) + 6 (MAC) + 7 + 1 = 15
        checksum = ArraySum(pRxBuf + 7, 7);
        if (pRxBuf[14] == checksum)
        {
          ConfigChangeIp(pRxBuf + 7);
          ConfigChangeMask(pRxBuf[11]);
          // Port is not checked
          ConfigChangePort(pRxBuf[12] + (((uint16_t) pRxBuf[13]) << 8));
          WriteConfiguration();
          *pTxSize = 1;
          pTxBuf[0] = RESPONSE_SUCCESS;
        }
        else
        {
          *pTxSize = 3;
          pTxBuf[0] = RESPONSE_CHKSM_ERR;
          pTxBuf[1] = pRxBuf[14];
          pTxBuf[2] = checksum;
        }
      }
      else if (pRxBuf[0] == ID_DEFAULT_CONFIG)
      {
        DefaultConfiguration();
        WriteConfiguration();
        *pTxSize = 1;
        pTxBuf[0] = RESPONSE_SUCCESS;
      }
      else if (pRxBuf[0] == ID_REBOOT)
      {
        *pTxSize = 1;
        pTxBuf[0] = RESPONSE_SUCCESS;
      }
      else if (pRxBuf[0] == ID_CHANGE_GW && *pRxSize >= 12)  // Change default gw
      {
        // Datalen = 1 (cmd) + 6 (MAC) + 5 = 12
        checksum = ArraySum(pRxBuf + 7, 4);
        if (pRxBuf[11] == checksum)
        {
          ConfigChangeGw(pRxBuf + 7);
          WriteExtraConfiguration();
          *pTxSize = 1;
          pTxBuf[0] = RESPONSE_SUCCESS;
        }
        else
        {
          *pTxSize = 3;
          pTxBuf[0] = RESPONSE_CHKSM_ERR;
          pTxBuf[1] = pRxBuf[11];
          pTxBuf[2] = checksum;
        }
      }
    }
    else
      ret = 0;  // MAC address did not match - do not send anything
  }
  else
    ret = 0;    /* Do not respond on unknown ID */
  return ret;
}

uint8_t ConfigChangeMac(uint8_t* pData)
{
  Config.MacAddress[0] = pData[0];
  Config.MacAddress[1] = pData[1];
  Config.MacAddress[2] = pData[2];
  Config.MacAddress[3] = pData[3];
  Config.MacAddress[4] = pData[4];
  Config.MacAddress[5] = pData[5];
  return 1;
}

uint8_t ConfigChangeIp(const uint8_t* pData)
{
  Config.IpAddress[0] = pData[0];
  Config.IpAddress[1] = pData[1];
  Config.IpAddress[2] = pData[2];
  Config.IpAddress[3] = pData[3];
  return 1;
}

uint8_t ConfigChangeGw(const uint8_t* pData)
{
    ExtraConfig.DefaultGateway[0] = pData[0];
    ExtraConfig.DefaultGateway[1] = pData[1];
    ExtraConfig.DefaultGateway[2] = pData[2];
    ExtraConfig.DefaultGateway[3] = pData[3];
    return 1;
}

void MaskToArray(uint8_t encoded, uint8_t* pArr)
{
  encoded = encoded > 32 ? 32 : encoded;

  uint32_t ones = 1 << (32 - encoded);
  ones--;
  ones = ~ones;
  for (uint8_t i = 0; i < 4; i++)
    pArr[i] = (ones >> (24 - 8 * i) & 0xff);
}

void MaskFromArray(uint8_t* pEncoded, uint8_t* pArr)
{
  uint32_t raw = 0;
  uint8_t i;
  for (i = 0; i < 4; i++)
    raw += ((uint32_t) pArr[i] << (24 - 8 * i));
  for (i = 0; i < 32; i++)
    if (((raw >> (31 - i)) & 0x1) == 0)
      break;
  *pEncoded = i;
}

uint8_t ConfigChangeMask(uint8_t encoded)
{
  uint8_t ipMask[IP_MASK_LENGTH];
  MaskToArray(encoded, ipMask);

  Config.IpMask[0] = ipMask[0];
  Config.IpMask[1] = ipMask[1];
  Config.IpMask[2] = ipMask[2];
  Config.IpMask[3] = ipMask[3];
  return 1;
}

uint8_t ConfigChangePort(const uint16_t num)
{
    uint8_t ret = 0;
    Config.Port = num;
    ret = 1;
    return ret;
}

const EXTRA_CONFIGURATION* GetExtraConfigurationAddr(void)
{
  return &ExtraConfig;
}

void MulticastResponseSent(const uint8_t* pRxBuf, const uint16_t* pRxSize,
                           const uint8_t* pTxBuf, const uint16_t* pTxSize)
{
  if (*pRxSize >= 7 && pRxBuf[0] == ID_REBOOT)
  {
    if (CheckMacMatch(pRxBuf + 1) && pRxBuf[0] == ID_REBOOT)
    {
      // Reset the device so as to apply the new settings
      HAL_NVIC_SystemReset();
    }
  }
}
CONFIGURATION GetConfiguration(void)
{
  return Config;
}

inline LinInitStruct * GetConfigurationLINAddr(void){
    return &(configLin.LinConfiguration);
}
inline CONFIGURATION_CAN* GetCanConfigurationAddr(uint8_t channel)
{
  CONFIGURATION_CAN * retConf;
  if (CAN1_NUM == channel)
      retConf =  &configCan1;
  else
      retConf =  &configCan2;
  return retConf;
}

void SetSerialNumber(uint8_t SN[SN_LENGTH])
{
  ProductInfo.SerialNumber[0] = SN[0];
  ProductInfo.SerialNumber[1] = SN[1];
  ProductInfo.SerialNumber[2] = SN[2];
  ProductInfo.SerialNumber[3] = SN[3];
}

uint8_t ReadConfigurationCan(uint8_t channel)
{
  uint8_t ret = 0;
  if (CAN1_NUM == channel)
  {

      uint8_t readData, status = 0;
      CONFIGURATION_CAN tmp;
      status = EECfgReadItem(CAN1_CONFIG_BASE_ADDR, (uint8_t*) &tmp, sizeof(CONFIGURATION_CAN));

      status |= EECfgReadItem(CAN1_CONFIG_BASE_ADDR + sizeof(CONFIGURATION_CAN), &readData, 1);
      configCan1 = tmp;


      if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_CAN)))
        ret = 1;
  }
  else if (CAN2_NUM == channel)
  {

      uint8_t readData, status = 0;
      CONFIGURATION_CAN tmp;
      status = EECfgReadItem(CAN2_CONFIG_BASE_ADDR, (uint8_t*) &tmp, sizeof(CONFIGURATION_CAN));

      status |= EECfgReadItem(CAN2_CONFIG_BASE_ADDR + sizeof(CONFIGURATION_CAN), &readData, 1);
      configCan2 = tmp;


      if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_CAN)))
        ret = 1;
  }
  return ret;
}

void DefaultExtraConfiguration(void)
{
  ExtraConfig.DefaultGateway[0] = DEFAULT_GW0;
  ExtraConfig.DefaultGateway[1] = DEFAULT_GW1;
  ExtraConfig.DefaultGateway[2] = DEFAULT_GW2;
  ExtraConfig.DefaultGateway[3] = DEFAULT_GW3;

  memset((void*) ExtraConfig.Reserved, 0xff, EXTRA_CONFIG_RESERVED_SIZE);
}

void DefaultConfigurationCanCh1(void)
{
    configCan1.CanConfiguration.DataBaud = DEFAULT_DBAUD_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationBaud = DEFAULT_NBAUD_CAN_CH1;
    configCan1.CanConfiguration.Mode = DEFAULT_MODE_CAN_CH1;
    configCan1.CanConfiguration.DataSJW = DEFAULT_DSJW_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationSJW = DEFAULT_NSJW_CAN_CH1;
    configCan1.CanConfiguration.DataSPoint = DEFAULT_DSP_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationSPoint = DEFAULT_NSP_CAN_CH1;
    configCan1.CanConfiguration.AutoStart = DEFAULT_AUTOSTART_CAN_CH1;
    configCan1.CanConfiguration.Protocol = DEFAULT_PROTOCOL_CAN_CH1;
    configCan1.CanConfiguration.PreciseTimingSet = DEFAULT_PRECISE_TIM_CAN_CH1;
    configCan1.CanConfiguration.DataPrescaler = DEFAULT_DATA_PRESCALER_CAN_CH1;
    configCan1.CanConfiguration.DataTimeSegment1 = DEFAULT_DATA_TIME_SEG1_CAN_CH1;
    configCan1.CanConfiguration.DataTimeSegment2 = DEFAULT_DATA_TIME_SEG2_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationPrescaler = DEFAULT_ARBITRATION_PRESCALER_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationTimeSegment1 = DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH1;
    configCan1.CanConfiguration.ArbitrationTimeSegment2 = DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH1;
    configCan1.CanConfiguration.EchoConfig = DEFAULT_ECHO_CONF_CH1;
    configCan1.CanConfiguration.CanRxId.Id = DEFAULT_CAN_RXID;
    configCan1.CanConfiguration.CanTxId.Id = DEFAULT_CAN_TXID;

}

void DefaultConfigurationCanCh2(void)
{
    configCan2.CanConfiguration.DataBaud = DEFAULT_DBAUD_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationBaud = DEFAULT_NBAUD_CAN_CH2;
    configCan2.CanConfiguration.Mode = DEFAULT_MODE_CAN_CH2;
    configCan2.CanConfiguration.DataSJW = DEFAULT_DSJW_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationSJW = DEFAULT_NSJW_CAN_CH2;
    configCan2.CanConfiguration.DataSPoint = DEFAULT_DSP_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationSPoint = DEFAULT_NSP_CAN_CH2;
    configCan2.CanConfiguration.AutoStart = DEFAULT_AUTOSTART_CAN_CH2;
    configCan2.CanConfiguration.Protocol = DEFAULT_PROTOCOL_CAN_CH2;
    configCan2.CanConfiguration.PreciseTimingSet = DEFAULT_PRECISE_TIM_CAN_CH2;
    configCan2.CanConfiguration.DataPrescaler = DEFAULT_DATA_PRESCALER_CAN_CH2;
    configCan2.CanConfiguration.DataTimeSegment1 = DEFAULT_DATA_TIME_SEG1_CAN_CH2;
    configCan2.CanConfiguration.DataTimeSegment2 = DEFAULT_DATA_TIME_SEG2_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationPrescaler = DEFAULT_ARBITRATION_PRESCALER_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationTimeSegment1 = DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH2;
    configCan2.CanConfiguration.ArbitrationTimeSegment2 = DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH2;
    configCan2.CanConfiguration.EchoConfig = DEFAULT_ECHO_CONF_CH2;
    configCan2.CanConfiguration.CanRxId.Id = DEFAULT_CAN_RXID;
    configCan2.CanConfiguration.CanTxId.Id = DEFAULT_CAN_TXID;

}

void DefaultConfigurationLin(void)
{
  configLin.LinConfiguration.DeviceMode = DEFAULT_MODE_LIN;
  configLin.LinConfiguration.ChecksumType = DEFAULT_CHECKSUM_LIN;
  configLin.LinConfiguration.Baudrate = DEFAULT_BAUDRATE_LIN;
  configLin.LinConfiguration.AMLR = DEFAULT_AMLR_LIN;
  configLin.LinConfiguration.autoStart = DEFAULT_AUTOSTART_LIN;
  configLin.LinConfiguration.txEcho = DEFAULT_TX_ECHO_LIN;
}

void SetEchoConfigurationCanCh1(uint8_t echo)
{
    configCan1.CanConfiguration.EchoConfig =  echo;
}

void SetEchoConfigurationCanCh2(uint8_t echo)
{
    configCan2.CanConfiguration.EchoConfig =  echo;
}
uint8_t ReadConfigurationSwitch(void)
{
    uint8_t ret = 0;
    uint8_t readData, status = 0;
    CONFIGURATION_SWITCH tmp;
     status = EECfgReadItem(SWITCH_CONFIG_BASE_ADDR, (uint8_t*) &tmp, sizeof(CONFIGURATION_SWITCH));

    status |= EECfgReadItem(SWITCH_CONFIG_BASE_ADDR + sizeof(CONFIGURATION_SWITCH), &readData, 1);
    configSwitch = tmp;


    if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_SWITCH)))
      ret = 1;

return ret;
}

uint8_t WriteSwitchConfiguration(void)
{


    uint8_t status = 0;
    size_t len = sizeof(CONFIGURATION_SWITCH);
    EECfgWriteItem(SWITCH_CONFIG_BASE_ADDR, (uint8_t*) &configSwitch, len);

    if (status == 0)
        status |= EECfgWriteByte(SWITCH_CONFIG_BASE_ADDR + len,
                                 ArrayChecksum((uint8_t*) &configSwitch, len));


  return (status == 0);
    return 1;
}


uint8_t DefaultConfigurationSwitch(void)
{
    SwitchConfStruct * p_SwitchConfig = GetSwitchConfigurationAddr();
    for (uint8_t i = 0; i < SWITCH_PORTS_COUNT;i++)
    {
        p_SwitchConfig->ports[i].addressLearning = DEFAULT_ADDRESS_LEARNING;
        p_SwitchConfig->ports[i].egressEnabled = DEFAULT_EGRESS_ENABLED;
        p_SwitchConfig->ports[i].egressMirroringEnabled = DEFAULT_MIRRORING_EGRESS_ENABLED;
        p_SwitchConfig->ports[i].ingressEnabled = DEFAULT_INGRESS_ENABLED;
        p_SwitchConfig->ports[i].ingressMirroringEnabled = DEFAULT_MIRRORING_INGRESS_ENABLED;
        p_SwitchConfig->ports[i].masterForced = DEFAULT_MASTER_FORCED;
        p_SwitchConfig->ports[i].masterSelected = DEFAULT_MASTER_SELECTED;

        if(i==KSZ_PORT-1)
        {
            p_SwitchConfig->ports[i].portSpeed = DEFAULT_PORT_SPEED_KSZ;
        }
        else
        {
            p_SwitchConfig->ports[i].portSpeed = DEFAULT_PORT_SPEED;
        }
        p_SwitchConfig->ports[i].vlanId = DEFAULT_VLAN_ID;
        p_SwitchConfig->ports[i].vlanTagMirroredIngress = DEFAULT_VLAN_TAG_MIRRORED_INGRESS;
    }
    p_SwitchConfig->mirrorPort = DEFAULT_MIRROR_PORT;
    p_SwitchConfig->vlanTagMirroredEgress = DEFAULT_VLAN_TAG_MIRRORED_EGRESS;

    return 0;
}

SwitchConfStruct* GetSwitchConfigurationAddr(void)
{
  return &(configSwitch.SwitchConfiguration);
}
uint8_t WriteInformation(void)
{

    if (EECfgWriteItem(INFO_BASE_ADDR, (uint8_t*) &ProductInfo, sizeof(PRODUCT_INFORMATION)) != 0)
        return 0;

  return 1;
}


