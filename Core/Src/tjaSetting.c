/*
 * tjaSetting.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Petr Kolář
 */


#include "tjaSetting.h"
#include "tools.h"
#include "main.h"
#include "lwip.h"

/* Status of T1 devices */
t1Status_t StatusT1[T1_CHANNELS];

/* First half of array is normal activity second is error activity*/
static uint8_t activityOnChannel[T1_CHANNELS * 2];

/*
 * @brief Enable LED that indicates master. LEDs are located under the left DSUB.
 * @param color color of LED
 * @param tjaAddr address of tja
 * @retval none
 */
static void tjaEnableLedLink(LedColor color,uint32_t tjaAddr);

/*
 * @brief Enable LED that indicates master. LEDs are located under the Dipswitch.
 * @param color color of LED
 * @param tjaAddr address of tja
 * @retval none
 */
static void tjaEnableLedMaster(LedColor color, uint32_t tjaAddr);

/*
 * @brief check if thete is activity on sense TX and RX channels
 * @param color color of LED
 * @param tjaAddr address of tja
 * @retval none
 */
static void checkActivityOnChannel(uint32_t tjaAddr);


void TjaInit(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    if (TjaAddrToChannel[tjaAddr])
        StatusT1[TjaAddrToChannel[tjaAddr]-1].port = TjaAddrToPort[tjaAddr];
    TjaUnlock(heth, tjaAddr);
    TjaLinkUpAndNormalMode(heth, tjaAddr);
    TjaLock(heth, tjaAddr);
}
void TjaSetMaster(ETH_HandleTypeDef * heth, uint32_t tjaAddr){
    uint32_t regVal;

    TjaUnlock(heth, tjaAddr);
    /*
     * Configuration register 1  (18 dec)
     *  MASTER_SLAVE [15] = 1 PHY configured as Master
     */
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x12, &regVal);
    regVal = regVal | BIT(15);
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x12, regVal);
    TjaLinkUpAndNormalMode(heth, tjaAddr);
    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
        StatusT1[TjaAddrToChannel[tjaAddr]-1].masterSlave = MASTER;
    tjaEnableLedMaster(GreenLed, tjaAddr);
    TjaLock(heth, tjaAddr);
}

void TjaSetSlave(ETH_HandleTypeDef * heth, uint32_t tjaAddr){
    uint32_t regVal;

    TjaUnlock(heth, tjaAddr);
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x12, &regVal);
    /*
     * Configuration register 1  (18 dec)
     *  MASTER_SLAVE [15] = 0 PHY configured as Slave
     */
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x12, &regVal);
    regVal = regVal & (~BIT(15));
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x12, regVal);
    TjaLinkUpAndNormalMode(heth, tjaAddr);
    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
        StatusT1[TjaAddrToChannel[tjaAddr]-1].masterSlave = SLAVE;
    tjaEnableLedMaster(NoLed, tjaAddr);
    TjaLock(heth, tjaAddr);
}

void TjaLock(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint32_t regVal;

    /*
     * Extended control register (17 dec)
     * CONFIG_EN [2] = 0 disable    configuration register access
     */
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x11, &regVal);
    regVal = regVal & (~BIT(2));
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
}


void TjaUnlock(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{

    uint32_t regVal;
    /*
     * Extended control register (17 dec)
     * LINK_CONTROL [15]    = 0 link control disabled
     * POWER_MODE [14:11]   = 1100 Standby mode (command)
     * CONFIG_EN [2]        = 1 enable    configuration register access
     */

    //Read if conf. is locked
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x11, &regVal);


    if (!(regVal & 0x0004) || (regVal & 0xE000))
    {
        //Unlock device
        regVal = (regVal | BIT(2) | BIT(14) | BIT(13)) & ~(BIT(15) | BIT(12) | BIT(11));
        HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
    }

}

void TjaLinkUpAndNormalMode(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint32_t regVal;
    /*
    * Extended control register (17 dec)
    * LINK_CONTROL [15]    = 1 link control enabled
    * POWER_MODE [14:11]   = 0011 Normal mode (command)
    *
    */
    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
        StatusT1[TjaAddrToChannel[tjaAddr]-1].testMode = 0;
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x11, &regVal);
    regVal = ((regVal & 0x07FF) | 0x9800);
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
}


void TjaDiagnose(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint8_t regAddr = 0;
    uint32_t regVal = 0;
    for (regAddr = 0; regAddr < 29; regAddr++)
    {
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, regAddr, &regVal);
    }

}

bool TjaIsMaster(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint32_t regVal = 0;
    bool retVal = 0;
    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
    {
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x12, &regVal);
        retVal = (BIT(15)&regVal) != 0;
        StatusT1[TjaAddrToChannel[tjaAddr]-1].masterSlave = retVal;
        tjaEnableLedMaster(retVal == MASTER ? GreenLed : NoLed, tjaAddr);
    }
    return retVal;
}

bool TjaIsLinkUp(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    //LINK_STATUS
    uint32_t regVal = 0;
    bool retVal = 0;
    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
    {
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x1, &regVal);
        retVal = (BIT(2)&regVal) != 0;
        StatusT1[TjaAddrToChannel[tjaAddr]-1].linkUp = retVal;
    }
    return retVal ;
}

bool TjaIsPolarityInvert(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    //LINK_STATUS
    uint32_t regVal = 0;
    bool retVal = 0;
    if (TjaAddrToChannel[tjaAddr])
    {
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x19, &regVal);
        retVal = (BIT(6)&regVal) != 0;
        StatusT1[TjaAddrToChannel[tjaAddr]-1].invertPolarity = retVal;
    }
    return (BIT(6)&regVal) != 0;
}


void TjaSetTestMode(ETH_HandleTypeDef * heth, uint32_t tjaAddr, t1Testmode_t testMode)
{
    uint32_t regVal = 0;
    TjaUnlock(heth, tjaAddr);
    testMode = (testMode & 0x7);

    /*
     * Extended control register (17 dec)
     * LINK_CONTROL [15]    = 0 link control disabled (enabled for test mode = 0)
     * POWER_MODE [14:11]   = 0011 Normal mode (command)
     * CONFIG_EN [2]        = 1 enabled    configuration register access
     */
    regVal = (regVal | BIT(2) | BIT(12) | BIT(11));
    regVal = (regVal | BIT(2) | testMode << 6 | BIT(12) | BIT(11));
    if (testMode == NORMAL_MODE)
        regVal|= BIT(15);
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
    StatusT1[TjaAddrToChannel[tjaAddr]-1].testMode = testMode;
    TjaLock(heth, tjaAddr);

}

void TjaSetCableTest(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint32_t regVal = 0;
    /*
     * Extended control register (17 dec)
     * LINK_CONTROL [15]    = 0 link control disabled
     * POWER_MODE [14:11]   = 0011 Normal mode (command)
     * CABLE_TEST [5]       = 1 enabled
     * CONFIG_EN [2]        = 1 enabled    configuration register access
     */
    regVal = (regVal | BIT(2) | BIT(12) | BIT(11));
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
    regVal = (regVal | BIT(2) | BIT(12) | BIT(11)| BIT(5));
    HAL_ETH_WritePHYRegister(heth, tjaAddr, 0x11, regVal);
}

uint8_t TjaGetCableTestResults(ETH_HandleTypeDef * heth, uint32_t tjaAddr, uint8_t * polarityAndShortDetect)
{
    uint8_t retVal = 0;
    uint32_t regVal = 0;
    *polarityAndShortDetect = 0;
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x11, &regVal);
    if ((regVal &  BIT(5)) == 0)
    {
        /*
        External status register (register 25)
        SHORT_DETECT [8]   =
        OPEN_DETECT  [7]   =
        */
        retVal = 0;
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x19, &regVal);
        TjaLinkUpAndNormalMode(heth,tjaAddr);
        if (regVal & (BIT(7))){
            *polarityAndShortDetect = 1;
        }
        if (regVal & (BIT(8))){
            if (*polarityAndShortDetect == 1)
                *polarityAndShortDetect = 0;  //connected to active link partner both short and open detected
            else
                *polarityAndShortDetect = 2;
        }
    }
    else
        retVal = 1;

    return retVal;
}
uint16_t TjaReadError(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    uint32_t regVal = 0;
    /*
     * Symbol error counter register (20 dec)
     * SYM_ERR_CNT [15:0]
     */
    if (TjaAddrToChannel[tjaAddr])
    {
        /*When the device is disconnected the value is always 0xFFFF so don't increment that*/
        HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x14, &regVal);
        if (regVal == 0xFFFF)
            regVal = 1;
        StatusT1[TjaAddrToChannel[tjaAddr]-1].linkErr = add_o(regVal,StatusT1[TjaAddrToChannel[tjaAddr]-1].linkErr);
        if (regVal != 0)
            // Set error activity
            activityOnChannel[TjaAddrToChannel[tjaAddr]-1 + T1_CHANNELS] = 1;
    }
    return regVal;
}


void TjaGetSqi(ETH_HandleTypeDef * heth, uint32_t tjaAddr)
{
    /*
     * Communication status register  (23 dec)
     * SQI  [7:5]   = 001  class A SQI (unstable link)
     *                     ....
     */
    uint32_t regVal = 0;
    uint8_t SQI;
    HAL_ETH_ReadPHYRegister(heth, tjaAddr, 0x17, &regVal);
    SQI = (regVal>>5) & 0x7;
    if (StatusT1[TjaAddrToChannel[tjaAddr]-1].linkUp)
        StatusT1[TjaAddrToChannel[tjaAddr]-1].SQ=SQI+1;
    else
        StatusT1[TjaAddrToChannel[tjaAddr]-1].SQ=0;
}


static void tjaEnableLedMaster(LedColor color,uint32_t tjaAddr)
{
    switch (TjaAddrToChannel[tjaAddr])
    {
        case 1:
            switch (color)
            {
                case GreenLed:
                    LED_T1_1_MASTER_LED_GREEN_ON();
                    LED_T1_1_MASTER_LED_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_1_MASTER_LED_GREEN_OFF();
                    LED_T1_1_MASTER_LED_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_1_MASTER_LED_GREEN_ON();
                    LED_T1_1_MASTER_LED_RED_ON();
                    break;
                case NoLed:
                    LED_T1_1_MASTER_LED_GREEN_OFF();
                    LED_T1_1_MASTER_LED_RED_OFF();
                    break;
            }
            break;
        case 2:
            switch (color)
            {
                case GreenLed:
                    LED_T1_2_MASTER_LED_GREEN_ON();
                    LED_T1_2_MASTER_LED_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_2_MASTER_LED_GREEN_OFF();
                    LED_T1_2_MASTER_LED_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_2_MASTER_LED_GREEN_ON();
                    LED_T1_2_MASTER_LED_RED_ON();
                    break;
                case NoLed:
                    LED_T1_2_MASTER_LED_GREEN_OFF();
                    LED_T1_2_MASTER_LED_RED_OFF();
                    break;
            }
            break;
        case 3:
            switch (color)
            {
                case GreenLed:
                    LED_T1_3_MASTER_LED_GREEN_ON();
                    LED_T1_3_MASTER_LED_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_3_MASTER_LED_GREEN_OFF();
                    LED_T1_3_MASTER_LED_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_3_MASTER_LED_GREEN_ON();
                    LED_T1_3_MASTER_LED_RED_ON();
                    break;
                case NoLed:
                    LED_T1_3_MASTER_LED_GREEN_OFF();
                    LED_T1_3_MASTER_LED_RED_OFF();
                    break;
            }
            break;
        default:
            break;
    }
}

static void tjaEnableLedLink(LedColor color,uint32_t tjaAddr)
{
    switch (TjaAddrToPort[tjaAddr])
    {
        case 1:
            switch (color)
            {
                case GreenLed:
                    LED_T1_1_GREEN_ON();
                    LED_T1_1_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_1_GREEN_OFF();
                    LED_T1_1_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_1_GREEN_ON();
                    LED_T1_1_RED_ON();
                    break;
                case NoLed:
                    LED_T1_1_GREEN_OFF();
                    LED_T1_1_RED_OFF();
                    break;
            }
            break;
        case 2:
            switch (color)
            {
                case GreenLed:
                    LED_T1_2_GREEN_ON();
                    LED_T1_2_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_2_GREEN_OFF();
                    LED_T1_2_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_2_GREEN_ON();
                    LED_T1_2_RED_ON();
                    break;
                case NoLed:
                    LED_T1_2_GREEN_OFF();
                    LED_T1_2_RED_OFF();
                    break;
            }
            break;
        case 3:
            switch (color)
            {
                case GreenLed:
                    LED_T1_3_GREEN_ON();
                    LED_T1_3_RED_OFF();
                    break;
                case RedLed:
                    LED_T1_3_GREEN_OFF();
                    LED_T1_3_RED_ON();
                    break;
                case OrangeLed:
                    LED_T1_3_GREEN_ON();
                    LED_T1_3_RED_ON();
                    break;
                case NoLed:
                    LED_T1_3_GREEN_OFF();
                    LED_T1_3_RED_OFF();
                    break;
            }
            break;
        default:
            break;
    }
}

static void checkActivityOnChannel(uint32_t tjaAddr){

    uint16_t pinRx, pinTx;

    //Check if address exist
    if (TjaAddrToChannel[tjaAddr])
    {
        switch (TjaAddrToChannel[tjaAddr])
        {
            case 1:
                pinRx = DETRX0_Pin;
                pinTx = DETTX0_Pin;
                break;
            case 2:
                pinRx = DETRX2_Pin;
                pinTx = DETTX2_Pin;
                break;
            case 3:
                pinRx = DETRX1_Pin;
                pinTx = DETTX1_Pin;
                break;
            default:
                pinRx = DETRX0_Pin;
                pinTx = DETTX0_Pin;
                break;
        }

        /* Check if there is activity */
        if (__HAL_GPIO_EXTI_GET_IT(pinRx) != 0x00U)
        {
            activityOnChannel[TjaAddrToChannel[tjaAddr]-1] = 1;
          __HAL_GPIO_EXTI_CLEAR_IT(pinRx);
        }

        if (__HAL_GPIO_EXTI_GET_IT(pinTx) != 0x00U)
        {
            activityOnChannel[TjaAddrToChannel[tjaAddr]-1] = 1;
          __HAL_GPIO_EXTI_CLEAR_IT(pinTx);
        }
    }
}


void TjaSetLedActivity(uint32_t tjaAddr)
{

    uint8_t *errorActivity, *activity;
     //Check if address exist
     if (TjaAddrToChannel[tjaAddr])
     {
         errorActivity = &activityOnChannel[TjaAddrToChannel[tjaAddr]-1 + T1_CHANNELS];
         activity = &activityOnChannel[TjaAddrToChannel[tjaAddr]-1];

         if (StatusT1[TjaAddrToChannel[tjaAddr]-1].linkUp || *errorActivity )
         {
            checkActivityOnChannel(tjaAddr);
            if (*errorActivity)
            {
                if (CounterLed == 0)
                    tjaEnableLedLink(RedLed,tjaAddr);
                else if (CounterLed == LED_PERIOD / 2)
                {
                    *errorActivity = 0;
                    tjaEnableLedLink(GreenLed,tjaAddr);
                }
            }
            else if(*activity)
            {
              if (CounterLed == 0)
                  tjaEnableLedLink(NoLed,tjaAddr);
              else if (CounterLed == LED_PERIOD / 2)
              {
                  *activity = 0;
                  tjaEnableLedLink(GreenLed,tjaAddr);
              }
            }
            else
                tjaEnableLedLink(GreenLed,tjaAddr);
        }
        else
        {
            tjaEnableLedLink(NoLed,tjaAddr);
        }
     }
}

 t1Status_t * GetStatusT1(uint32_t tjaAddr)
 {
     t1Status_t * status = 0;
     //Check if address exist
     if (TjaAddrToChannel[tjaAddr])
     {
         status = &StatusT1[TjaAddrToChannel[tjaAddr]-1];
     }
     return status;
 }


 uint8_t CableTestRun(ETH_HandleTypeDef * heth, uint8_t port, uint8_t * pTestResult){
     uint8_t retVal=0;
     if (TakeHethAccess()){
         TjaSetCableTest(heth,PortToTjaAddr[port]);
         uint8_t i = 0;
         while (i<10){
             if(!TjaGetCableTestResults(heth,PortToTjaAddr[port],pTestResult)){
                 retVal = 1;
                 break; //Cable results ready
             }
             osDelay(30);
             i++;
         }
         GiveAccessToHeth();
     }
     return retVal;
 }



