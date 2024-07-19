/*
 * kszSetting.c
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */
#include "kszSetting.h"
#include "main.h"
#include "tools.h"

#define KSZ9131_SKEW_5BIT_MAX   2400
#define KSZ9131_SKEW_4BIT_MAX   800
#define KSZ9131_OFFSET      700
#define KSZ9131_STEP        100

#define MII_KSZ9031RN_TX_DATA_PAD_SKEW  6
#define MII_KSZ9031RN_CLK_PAD_SKEW      8
#define MII_KSZ9031RN_RX_DATA_PAD_SKEW  5
#define MII_KSZ9031RN_CONTROL_PAD_SKEW  4

#define KSZ9131RN_MMD_COMMON_CTRL_REG   2
#define KSZ9131RN_RXC_DLL_CTRL      76
#define KSZ9131RN_TXC_DLL_CTRL      77
#define KSZ9131RN_DLL_CTRL_BYPASS   BIT_MASK(12)
#define KSZ9131RN_DLL_ENABLE_DELAY  0
#define KSZ9131RN_DLL_DISABLE_DELAY BIT(12)
struct phy_device PhyDev;


static int kszConfigRgmiizDelay(struct phy_device *phydev, ETH_HandleTypeDef * heth);
static uint8_t kszLedErrata(ETH_HandleTypeDef * heth);

ksz_err KszInit(ETH_HandleTypeDef * heth)
{
    uint8_t ret = KSZ_OK;

    PhyDev.autonegComplete = 0;
    PhyDev.link = 0;
    PhyDev.masterSlave = SLAVE;
    //RGMII delays TX and RX are set on KSZ for better performance
    PhyDev.interface = PHY_INTERFACE_MODE_RGMII_ID;//PHY_INTERFACE_MODE_RGMII_RXID; //PHY_INTERFACE_MODE_RGMII_TXID; ;
    PhyDev.port = KSZ_PORT;
    PhyDev.speed = 0;
    PhyDev.speedLimit = 0;
    PhyDev.state = PHY_DOWN;

    //Read device id
    if(KszReadReg(heth, 0x2) != 0x0022)
        ret |= KSZ_erWRONG_ID;

    KszMmdWrite(heth, 0x7, 0x3c, 0); //Disable EEE advertisement it caused the idle ERROR




    ret |= kszConfigRgmiizDelay(&PhyDev, heth);

    /*
     * Load skew values default values only RXx are changed
     */
    KszMmdWrite(heth, 0x2, MII_KSZ9031RN_CLK_PAD_SKEW, 0xE7);

    KszMmdWrite(heth, 0x2, MII_KSZ9031RN_CONTROL_PAD_SKEW, 0x077);

    KszMmdWrite(heth, 0x2, MII_KSZ9031RN_RX_DATA_PAD_SKEW, 0x7777); //0x8888

    KszMmdWrite(heth, 0x2, MII_KSZ9031RN_TX_DATA_PAD_SKEW, 0x7777);


    //Set correct autonegotiation advertisement
    /*
     * AUTO-NEGOTIATION ADVERTISEMENT REGISTER 0x4
     * Asymmetric Pause [11] 1 = Asymmetric PAUSE toward link partner advertised
     *
     * Symmetric Pause [10] 1 = Symmetric PAUSE toward link partner advertised
     *
     * 100BASE-X Full Duplex [8] 1 = 100BASE-X full duplex ability  advertised
     *
     * 10BASE-T Full Duplex[6]  1 = 10BASE-T full duplex ability advertised
     *
     * Selector Field[4:0] 00001 This field identifies the type of message being sent by Auto-Negotiation.
     */

    ret |= kszLedErrata(heth);


    KszWriteReg(heth, 0x4, 0xD41);

    return ret;

}





/*
 * Set RGMII delay correct one depends on second PHY but there must be one
 * delay on both PHY if is not done by lenght of trace on PCB.
 */
static int kszConfigRgmiizDelay(struct phy_device *phydev, ETH_HandleTypeDef * heth)
{
    uint16_t rxcdll_val, txcdll_val;


    switch (phydev->interface) {
    case PHY_INTERFACE_MODE_RGMII:
        rxcdll_val = KSZ9131RN_DLL_DISABLE_DELAY;
        txcdll_val = KSZ9131RN_DLL_DISABLE_DELAY;
        break;
    case PHY_INTERFACE_MODE_RGMII_ID:
        rxcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
        txcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
        break;
    case PHY_INTERFACE_MODE_RGMII_RXID:
        rxcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
        txcdll_val = KSZ9131RN_DLL_DISABLE_DELAY;
        break;
    case PHY_INTERFACE_MODE_RGMII_TXID:
        rxcdll_val = KSZ9131RN_DLL_DISABLE_DELAY;
        txcdll_val = KSZ9131RN_DLL_ENABLE_DELAY;
        break;
    default:
        return 0;
    }
    /* rxdll_tap_sel = 0x1B */
    /* rxdll_tap_adj = 0x11 */
    rxcdll_val |=  0x06d1;

    /* txdll_tap_sel = 0x1B */
    /* txdll_tap_adj = 0x11 */
    txcdll_val |=  0x06d1;

    KszMmdWrite(heth, KSZ9131RN_MMD_COMMON_CTRL_REG,KSZ9131RN_RXC_DLL_CTRL, rxcdll_val);

    KszMmdWrite(heth, KSZ9131RN_MMD_COMMON_CTRL_REG, KSZ9131RN_TXC_DLL_CTRL, txcdll_val);

    return 0;
}

uint16_t KszMmdRead(ETH_HandleTypeDef * heth, uint8_t device, uint16_t registr)
{
    return MmdRead(heth, KSZ_ADDR, device, registr);
}

void KszMmdWrite(ETH_HandleTypeDef * heth, uint8_t device, uint16_t registr,  uint16_t data)
{
    MmdWrite(heth, KSZ_ADDR, device, registr, data);
}

uint32_t KszReadReg(ETH_HandleTypeDef *heth, uint16_t registr)
{
    uint32_t val;
    HAL_ETH_ReadPHYRegister(heth, KSZ_ADDR, registr, &val);
    return val;
}

void KszWriteReg(ETH_HandleTypeDef * heth, uint16_t registr, uint16_t data)
{
    HAL_ETH_WritePHYRegister(heth, KSZ_ADDR, registr, data);
}


/* Silicon Errata DS80000693B
 *
 * When LEDs are configured in Individual Mode, LED1 is ON in a no-link
 * condition. Workaround is to set register 0x1e, bit 9, this way LED1 behaves
 * according to the datasheet (off if there is no link).
 */
static uint8_t kszLedErrata(ETH_HandleTypeDef * heth)
{
    uint32_t val;
    uint16_t reg;
    uint8_t retVal = 0;
    reg = KszMmdRead(heth, 0x2, 0x0);


    if (!(reg & BIT(4)))
        retVal = 0;
    else
    {
        val = KszReadReg(heth,30);
        KszWriteReg(heth,0x1e, (uint16_t) val | BIT(9));
        reg = KszReadReg(heth,30);
    }
    return retVal;
}


bool IsKsz1000BtLinkUp(ETH_HandleTypeDef * heth){
    uint16_t regVal;
    regVal = KszReadReg(heth, 0x13);
    return regVal & 2;
}

bool IsKsz100BtLinkUp(ETH_HandleTypeDef * heth){
    uint16_t regVal;
    regVal = KszReadReg(heth, 0x13);
    return regVal & 1;
}

bool IsKszMaster(ETH_HandleTypeDef * heth){
    bool retVal;
    retVal = (KszReadReg(heth, 0xA) & BIT(14)) != 0;
    PhyDev.masterSlave = retVal;
    return retVal;
}

uint16_t GetErrorCounter(ETH_HandleTypeDef * heth)
{
    uint16_t retVal = KszReadReg(heth,0xA) & 0xFF;
    PhyDev.linkErr = retVal != 0;
    return retVal;
}
bool IsKsz10BtLinkUp(ETH_HandleTypeDef * heth){

    return (KszReadReg(heth, 0x1) & BIT(2)
            && KszReadReg(heth, 0x1f) & BIT(4));
}

void DisableKszAutoMDI(ETH_HandleTypeDef * heth)
{
    KszWriteReg(heth, 0x1c, 0x0040);
}

void EnableKszAutoMDI(ETH_HandleTypeDef * heth)
{
    KszWriteReg(heth, 0x1c, 0x0000);
}


void SetSpeedKsz(ETH_HandleTypeDef * heth, ksz_speed speed){
    switch (speed){
        case SPEED_KSZ_AUTO:
            //Advertise Auto-negotiation
            KszWriteReg(heth, 0x4, 0xD41);
            KszWriteReg(heth, 0x9, 0x0200);
            KszWriteReg(heth, 0x0, 0x1300);  //Autonegotiation enabled
        break;

        case  SPEED_KSZ_10M:
            //Advertise only 10Mbit/s
            KszWriteReg(heth, 0x9, 0x0);
            KszWriteReg(heth, 0x4, 0x0041);
            KszWriteReg(heth, 0x0, 0x1300);  //Autonegotiation enabled
        break;

        case  SPEED_KSZ_100M:
            //Advertise only 100Mbit/s
            KszWriteReg(heth, 0x9, 0x0);
            KszWriteReg(heth, 0x4, 0x0d01);
            KszWriteReg(heth, 0x0, 0x1300); //Autonegotiation enabled
        break;

        case  SPEED_KSZ_1G:
            //Advertise 1000Mbit/s
            KszWriteReg(heth, 0x9, 0x0200);
            KszWriteReg(heth, 0x4, 0x0d01);
            KszWriteReg(heth, 0x0, 0x1300); //Autonegotiation enabled
        break;
    }
}


