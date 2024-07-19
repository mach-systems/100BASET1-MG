/*
 * kszSetting.h
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */

#ifndef INC_KSZSETTING_H_
#define INC_KSZSETTING_H_
#include "stdbool.h"
#include "main.h"



typedef enum  {
    PHY_DOWN = 0,
    PHY_READY,
    PHY_HALTED,
    PHY_UP,
    PHY_RUNNING,
    PHY_NOLINK,
    PHY_CABLETEST,
}phyState_t;

typedef enum errStateKsz {
    KSZ_OK = 0,
    KSZ_erWRONG_ID,
} ksz_err;

typedef enum
{
    SPEED_KSZ_1G = 1,
    SPEED_KSZ_100M,
    SPEED_KSZ_10M,
    SPEED_KSZ_AUTO,
} ksz_speed;


/* type of rgmii delays*/
typedef enum {
    PHY_INTERFACE_MODE_RGMII,
    PHY_INTERFACE_MODE_RGMII_ID,
    PHY_INTERFACE_MODE_RGMII_RXID,
    PHY_INTERFACE_MODE_RGMII_TXID,
} phy_interface_t;

extern struct phy_device PhyDev;

struct phy_device {


    /* The most recently read link state */
    uint8_t link                :1;
    uint8_t linkErr             :1;
    uint8_t autonegComplete     :1;
    /* Interrupts are enabled */
    uint8_t interrupts          :1; //For future use

    phyState_t state;  //For future use

    phy_interface_t interface;
    /*
     * forced speed & duplex (no autoneg)
     * partner speed & duplex & pause (autoneg)
     */
    ksz_speed speed; //This is actual speed
    ksz_speed speedLimit; //This is selected speed for autonegotiation the autonegotiation can be limited to 10, 100 or 1000.
                          //Other possible value is SPEED_KSZ_AUTO and the is no limitation for autonegotiation
    uint8_t port;
    masterSlave_t masterSlave;
};

/*
 * @brief This function init the Ksz9131. The function setup the RGMII delay, disable EEE, set correct auto negotiation advertisement and apply the errata
 * @param heth ethernet handler
 * @retval {0: successful, else: failed}
 */
uint8_t KszInit(ETH_HandleTypeDef * heth);



/*
 * @brief Write function for the MMD registers
 * @param heth ethernet handler
 * @retval none
 */
void KszMmdWrite(ETH_HandleTypeDef * heth, uint8_t device, uint16_t registr, uint16_t data);

/*
 * @brief Read function for the MMD registers
 * @param heth ethernet handler
 * @retval register val
 */
uint16_t KszMmdRead(ETH_HandleTypeDef * heth, uint8_t device, uint16_t registr);




/*
 * @brief Write standard KSZ registers through mdio
 * @param heth ethernet handler
 * @retval none
 */
void KszWriteReg(ETH_HandleTypeDef * heth, uint16_t registr, uint16_t data);

/*
 * @brief Read standard KSZ registers through mdio
 * @param heth ethernet handler
 * @retval register value
 */
uint32_t KszReadReg(ETH_HandleTypeDef *heth, uint16_t registr);



/*
 * @brief Check is the 1000B-T Link is Up
 * @param heth ethernet handler
 * @retval true if LinkUp
 */
bool IsKsz1000BtLinkUp(ETH_HandleTypeDef * heth);

/*
 * @brief Check is the 100B-T Link is Up
 * @param heth ethernet handler
 * @retval true if LinkUp
 */
bool IsKsz100BtLinkUp(ETH_HandleTypeDef * heth);

/*
 * @brief Check is the 10B-T Link is Up
 * @param heth ethernet handler
 * @retval true if LinkUp
 */
bool IsKsz10BtLinkUp(ETH_HandleTypeDef * heth);
/*
 *
 * @brief Disable automatic cable crossover detection and correction
 * @param heth ethernet handler
 * @retval none
 */
void DisableKszAutoMDI(ETH_HandleTypeDef * heth);

/*
 * @brief Enable automatic cable crossover detection and correction
 * @param heth ethernet handler
 * @retval none
 */
void EnableKszAutoMDI(ETH_HandleTypeDef * heth);

/*
 * @brief AUTO-NEGOTIATION MASTER SLAVE STATUS valid for 1Gb/s connection
 * @param heth ethernet handler
 * @retval true if master
 */
bool IsKszMaster(ETH_HandleTypeDef * heth);

/*
 * @brief Idle counter value others counters are not valid (errata)
 * @param heth ethernet handler
 * @retval Counter valuer
 */
uint16_t GetErrorCounter(ETH_HandleTypeDef * heth);

/*
 * @brief the speed is always autonegotiation
 * @param heth ethernet handler
 * @param speed on which the autoneotiation will be restricted
 * @retval none
 */
void SetSpeedKsz(ETH_HandleTypeDef * heth, ksz_speed speed);

#endif /* INC_KSZSETTING_H_ */
