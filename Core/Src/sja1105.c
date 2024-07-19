/*
 * sja1105nl.c
 *
 * SJA1105NL ethernet phy driver. Inspired by lan8742.c and .h from STM.
 *
 *  Created on: May 14, 2021
 *      Author: Karel Hevessy
 */

/**
  ******************************************************************************
  * @file    lan8742.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the LAN742
  *          PHY devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <sja1105.h>
#include "main.h"   /* Reset pin */


/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LAN8742 LAN8742
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup LAN8742_Private_Defines LAN8742 Private Defines
  * @{
  */
#define SJA1105_SW_RESET_TO    ((uint32_t)500U)
#define SJA1105_INIT_TO        ((uint32_t)2000U)
#define SJA1105_PHADDR         ((uint32_t)7U)   /* Porting: edit this address
                                                   according to strap-ins. Not
                                                   configurable in software   */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup LAN8742_Private_Functions LAN8742 Private Functions
  * @{
  */

/**
  * @brief  Register IO functions to component object
  * @param  pObj: device object  of LAN8742_Object_t.
  * @param  ioctx: holds device IO functions.
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_ERROR if missing mandatory function
  */
int32_t  SJA1105_RegisterBusIO(sja1105_Object_t *pObj, sja1105_IOCtx_t *ioctx)
{
  if(!pObj || !ioctx->ReadReg || !ioctx->WriteReg || !ioctx->GetTick)
  {
    return SJA1105_STATUS_ERROR;
  }

  pObj->IO.Init = ioctx->Init;
  pObj->IO.DeInit = ioctx->DeInit;
  pObj->IO.ReadReg = ioctx->ReadReg;
  pObj->IO.WriteReg = ioctx->WriteReg;
  pObj->IO.GetTick = ioctx->GetTick;

  return SJA1105_STATUS_OK;
}

/**
  * @brief  Initialize the lan8742 and configure the needed hardware resources
  * @param  pObj: device object LAN8742_Object_t.
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_ADDRESS_ERROR if cannot find device address
  *         SJA1105_STATUS_READ_ERROR if cannot read register
  *         SJA1105_STATUS_WRITE_ERROR if cannot write to register
  *         SJA1105_STATUS_RESET_TIMEOUT if cannot perform a software reset
  */
 int32_t SJA1105_Init(sja1105_Object_t *pObj)
 {
   uint32_t tickstart = 0, regvalue = 0, addr = 0;
   int32_t status = SJA1105_STATUS_OK;
   return SJA1105_STATUS_OK;
   if(pObj->Is_Initialized == 0)
   {
//     if(pObj->IO.Init != 0)
//     {
//       /* GPIO and Clocks initialization */
//       pObj->IO.Init();
//     }
//
//     /* for later check */
//     pObj->DevAddr = LAN8742_MAX_DEV_ADDR + 1;
//
//     /* Get the device address from special mode register */
//     for(addr = 0; addr <= LAN8742_MAX_DEV_ADDR; addr ++)
//     {
//       if(pObj->IO.ReadReg(addr, LAN8742_SMR, &regvalue) < 0)
//       {
//         status = LAN8742_STATUS_READ_ERROR;
//         /* Can't read from this device address
//            continue with next address */
//         continue;
//       }
//
//       if((regvalue & LAN8742_SMR_PHY_ADDR) == addr)
//       {
//         pObj->DevAddr = addr;
//         status = LAN8742_STATUS_OK;
//         break;
//       }
//     }
//
//     if(pObj->DevAddr > LAN8742_MAX_DEV_ADDR)
//     {
//       status = LAN8742_STATUS_ADDRESS_ERROR;
//     }
//
//     /* if device address is matched */
//     if(status == LAN8742_STATUS_OK)
//     {
     pObj->DevAddr = addr = SJA1105_PHADDR;

#ifdef USE_HARDWARE_RESET
     HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
     HAL_Delay(SJA1105_INIT_TO);
     HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
#endif /* USE_HARDWARE_RESET */

       /* set a software reset  */
       if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_BCR, SJA1105_BCR_SOFT_RESET) >= 0)
       {
         /* get software reset status */
         if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &regvalue) >= 0)
         {
           tickstart = pObj->IO.GetTick();

           /* wait until software reset is done or timeout occured  */
           while(regvalue & SJA1105_BCR_SOFT_RESET)
           {
             if((pObj->IO.GetTick() - tickstart) <= SJA1105_SW_RESET_TO)
             {
               if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &regvalue) < 0)
               {
                 status = SJA1105_STATUS_READ_ERROR;
                 break;
               }
             }
             else
             {
               status = SJA1105_STATUS_RESET_TIMEOUT;
               break;
             }
           }
         }
         else
         {
           status = SJA1105_STATUS_READ_ERROR;
         }
       }
       else
       {
         status = SJA1105_STATUS_WRITE_ERROR;
       }
//     }
   }

   if(status == SJA1105_STATUS_OK)
   {
     tickstart =  pObj->IO.GetTick();

     /* Wait for 2s to perform initialization */
     while((pObj->IO.GetTick() - tickstart) <= SJA1105_INIT_TO)
     {
     }
     pObj->Is_Initialized = 1;
   }

   return status;
 }

/**
  * @brief  De-Initialize the sja1105 and it's hardware resources
  * @param  pObj: device object SJA1105_Object_t.
  * @retval None
  */
int32_t SJA1105_DeInit(sja1105_Object_t *pObj)
{
  if(pObj->Is_Initialized)
  {
    if(pObj->IO.DeInit != 0)
    {
      if(pObj->IO.DeInit() < 0)
      {
        return SJA1105_STATUS_ERROR;
      }
    }

    pObj->Is_Initialized = 0;
  }

  return SJA1105_STATUS_OK;
}

/**
  * @brief  Disable the LAN8742 power down mode.
  * @param  pObj: device object LAN8742_Object_t.
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_READ_ERROR if connot read register
  *         SJA1105_STATUS_WRITE_ERROR if connot write to register
  */
int32_t SJA1105_DisablePowerDownMode(sja1105_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &readval) >= 0)
  {
    readval &= ~SJA1105_BCR_POWER_DOWN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_BCR, readval) < 0)
    {
      status =  SJA1105_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Enable the LAN8742 power down mode.
  * @param  pObj: device object LAN8742_Object_t.
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_READ_ERROR if connot read register
  *         SJA1105_STATUS_WRITE_ERROR if connot write to register
  */
int32_t SJA1105_EnablePowerDownMode(sja1105_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &readval) >= 0)
  {
    readval |= SJA1105_BCR_POWER_DOWN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_BCR, readval) < 0)
    {
      status =  SJA1105_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Start the auto negotiation process.
  * @param  pObj: device object LAN8742_Object_t.
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_READ_ERROR if connot read register
  *         SJA1105_STATUS_WRITE_ERROR if connot write to register
  */
int32_t SJA1105_StartAutoNego(sja1105_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &readval) >= 0)
  {
    readval |= SJA1105_BCR_AUTONEGO_EN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_BCR, readval) < 0)
    {
      status =  SJA1105_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Get the link state of LAN8742 device.
  * @param  pObj: Pointer to device object.
  * @param  pLinkState: Pointer to link state
  * @retval SJA1105_STATUS_LINK_DOWN  if link is down
  *         SJA1105_STATUS_AUTONEGO_NOTDONE if Auto nego not completed
  *         SJA1105_STATUS_100MBITS_FULLDUPLEX if 100Mb/s FD
  *         SJA1105_STATUS_100MBITS_HALFDUPLEX if 100Mb/s HD
  *         SJA1105_STATUS_10MBITS_FULLDUPLEX  if 10Mb/s FD
  *         SJA1105_STATUS_10MBITS_HALFDUPLEX  if 10Mb/s HD
  *         SJA1105_STATUS_READ_ERROR if connot read register
  *         SJA1105_STATUS_WRITE_ERROR if connot write to register
  */
int32_t SJA1105_GetLinkState(sja1105_Object_t *pObj)
{
  uint32_t readval = 0;
  return SJA1105_STATUS_100MBITS_FULLDUPLEX;
  /* Read Status register  */
  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BSR, &readval) < 0)
  {
    return SJA1105_STATUS_READ_ERROR;
  }

  /* Read Status register again */
  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BSR, &readval) < 0)
  {
    return SJA1105_STATUS_READ_ERROR;
  }

  if((readval & SJA1105_BSR_LINK_STATUS) == 0)
  {
    /* Return Link Down status */
    return SJA1105_STATUS_LINK_DOWN;
  }

  /* Check Auto negotiaition */
  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &readval) < 0)
  {
    return SJA1105_STATUS_READ_ERROR;
  }

  if((readval & SJA1105_BCR_AUTONEGO_EN) != SJA1105_BCR_AUTONEGO_EN)
  {
    if(((readval & SJA1105_BCR_SPEED_SELECT) == SJA1105_BCR_SPEED_SELECT) && ((readval & SJA1105_BCR_DUPLEX_MODE) == SJA1105_BCR_DUPLEX_MODE))
    {
      return SJA1105_STATUS_100MBITS_FULLDUPLEX;
    }
    else if ((readval & SJA1105_BCR_SPEED_SELECT) == SJA1105_BCR_SPEED_SELECT)
    {
      return SJA1105_STATUS_100MBITS_HALFDUPLEX;
    }
    else if ((readval & SJA1105_BCR_DUPLEX_MODE) == SJA1105_BCR_DUPLEX_MODE)
    {
      return SJA1105_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
      return SJA1105_STATUS_10MBITS_HALFDUPLEX;
    }
  }
  else /* Auto Nego enabled */
  {
    if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_CTRL2, &readval) < 0)
    {
      return SJA1105_STATUS_READ_ERROR;
    }

    /* Check if auto nego not done */
    if((readval & SJA1105_CTRL2_AUTONEGO_DONE) == 0)
    {
      return SJA1105_STATUS_AUTONEGO_NOTDONE;
    }

    if((readval & SJA1105_CTRL2_HCDSPEEDMASK) == SJA1105_CTRL2_100BTX_FD)
    {
      return SJA1105_STATUS_100MBITS_FULLDUPLEX;
    }
    else if ((readval & SJA1105_CTRL2_HCDSPEEDMASK) == SJA1105_CTRL2_100BTX_HD)
    {
      return SJA1105_STATUS_100MBITS_HALFDUPLEX;
    }
    else if ((readval & SJA1105_CTRL2_HCDSPEEDMASK) == SJA1105_CTRL2_10BT_FD)
    {
      return SJA1105_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
      return SJA1105_STATUS_10MBITS_HALFDUPLEX;
    }
  }
}

/**
  * @brief  Set the link state of LAN8742 device.
  * @param  pObj: Pointer to device object.
  * @param  pLinkState: link state can be one of the following
  *         LAN8742_STATUS_100MBITS_FULLDUPLEX if 100Mb/s FD
  *         LAN8742_STATUS_100MBITS_HALFDUPLEX if 100Mb/s HD
  *         LAN8742_STATUS_10MBITS_FULLDUPLEX  if 10Mb/s FD
  *         LAN8742_STATUS_10MBITS_HALFDUPLEX  if 10Mb/s HD
  * @retval LAN8742_STATUS_OK  if OK
  *         LAN8742_STATUS_ERROR  if parameter error
  *         LAN8742_STATUS_READ_ERROR if connot read register
  *         LAN8742_STATUS_WRITE_ERROR if connot write to register
  */
int32_t SJA1105_SetLinkState(sja1105_Object_t *pObj, uint32_t LinkState)
{
  uint32_t bcrvalue = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_BCR, &bcrvalue) >= 0)
  {
    /* Disable link config (Auto nego, speed and duplex) */
    bcrvalue &= ~(SJA1105_BCR_AUTONEGO_EN | SJA1105_BCR_SPEED_SELECT | SJA1105_BCR_DUPLEX_MODE);

    if(LinkState == SJA1105_STATUS_100MBITS_FULLDUPLEX)
    {
      bcrvalue |= (SJA1105_BCR_SPEED_SELECT | SJA1105_BCR_DUPLEX_MODE);
    }
    else if (LinkState == SJA1105_STATUS_100MBITS_HALFDUPLEX)
    {
      bcrvalue |= SJA1105_BCR_SPEED_SELECT;
    }
    else if (LinkState == SJA1105_STATUS_10MBITS_FULLDUPLEX)
    {
      bcrvalue |= SJA1105_BCR_DUPLEX_MODE;
    }
    else
    {
      /* Wrong link status parameter */
      status = SJA1105_STATUS_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  if(status == SJA1105_STATUS_OK)
  {
    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_BCR, bcrvalue) < 0)
    {
      status = SJA1105_STATUS_WRITE_ERROR;
    }
  }

  return status;
}

///**
//  * @brief  Enable loopback mode.
//  * @param  pObj: Pointer to device object.
//  * @retval LAN8742_STATUS_OK  if OK
//  *         LAN8742_STATUS_READ_ERROR if connot read register
//  *         LAN8742_STATUS_WRITE_ERROR if connot write to register
//  */
//int32_t LAN8742_EnableLoopbackMode(lan8742_Object_t *pObj)
//{
//  uint32_t readval = 0;
//  int32_t status = LAN8742_STATUS_OK;
//
//  if(pObj->IO.ReadReg(pObj->DevAddr, LAN8742_BCR, &readval) >= 0)
//  {
//    readval |= LAN8742_BCR_LOOPBACK;
//
//    /* Apply configuration */
//    if(pObj->IO.WriteReg(pObj->DevAddr, LAN8742_BCR, readval) < 0)
//    {
//      status = LAN8742_STATUS_WRITE_ERROR;
//    }
//  }
//  else
//  {
//    status = LAN8742_STATUS_READ_ERROR;
//  }
//
//  return status;
//}
//
///**
//  * @brief  Disable loopback mode.
//  * @param  pObj: Pointer to device object.
//  * @retval LAN8742_STATUS_OK  if OK
//  *         LAN8742_STATUS_READ_ERROR if connot read register
//  *         LAN8742_STATUS_WRITE_ERROR if connot write to register
//  */
//int32_t LAN8742_DisableLoopbackMode(lan8742_Object_t *pObj)
//{
//  uint32_t readval = 0;
//  int32_t status = LAN8742_STATUS_OK;
//
//  if(pObj->IO.ReadReg(pObj->DevAddr, LAN8742_BCR, &readval) >= 0)
//  {
//    readval &= ~LAN8742_BCR_LOOPBACK;
//
//    /* Apply configuration */
//    if(pObj->IO.WriteReg(pObj->DevAddr, LAN8742_BCR, readval) < 0)
//    {
//      status =  LAN8742_STATUS_WRITE_ERROR;
//    }
//  }
//  else
//  {
//    status = LAN8742_STATUS_READ_ERROR;
//  }
//
//  return status;
//}

/**
  * @brief  Enable IT source.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT source to be enabled
  *         should be a value or a combination of the following:
  *         SJA1105_JABBER_IE
  *         SJA1105_RECEIVE_ERROR_IE
  *         SJA1105_PAGE_RECEIVED_IE
  *         SJA1105_PARALLEL_DETECTION_FAULT_IE
  *         SJA1105_LINK_PARTNER_ACK_IE
  *         SJA1105_LINK_DOWN_IE
  *         SJA1105_REMOTE_FAULT_IE
  *         SJA1105_LINK_UP_IE
  * @retval SJA1105_STATUS_OK  if OK
  *         SJA1105_STATUS_READ_ERROR if cannot read register
  *         SJA1105_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t SJA1105_EnableIT(sja1105_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_ICSR, &readval) >= 0)
  {
    readval |= Interrupt;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_ICSR, readval) < 0)
    {
      status =  SJA1105_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Disable IT source.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT source to be disabled
  *         should be a value or a combination of the following:
  *         SJA1105_JABBER_IF
  *         SJA1105_RECEIVE_ERROR_IF
  *         SJA1105_PAGE_RECEIVED_IF
  *         SJA1105_PARALLEL_DETECTION_FAULT_IF
  *         SJA1105_LINK_PARTNER_ACK_IF
  *         SJA1105_LINK_DOWN_IF
  *         SJA1105_REMOTE_FAULT_IF
  *         SJA1105_LINK_UP_IF
  * @retval LAN8742_STATUS_OK  if OK
  *         LAN8742_STATUS_READ_ERROR if cannot read register
  *         LAN8742_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t SJA1105_DisableIT(sja1105_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = SJA1105_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_ICSR, &readval) >= 0)
  {
    readval &= ~Interrupt;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, SJA1105_ICSR, readval) < 0)
    {
      status = SJA1105_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

///**
//  * @brief  Clear IT flag.
//  * @param  pObj: Pointer to device object.
//  * @param  Interrupt: IT flag to be cleared
//  *         should be a value or a combination of the following:
//  *         LAN8742_WOL_IT
//  *         LAN8742_ENERGYON_IT
//  *         LAN8742_AUTONEGO_COMPLETE_IT
//  *         LAN8742_REMOTE_FAULT_IT
//  *         LAN8742_LINK_DOWN_IT
//  *         LAN8742_AUTONEGO_LP_ACK_IT
//  *         LAN8742_PARALLEL_DETECTION_FAULT_IT
//  *         LAN8742_AUTONEGO_PAGE_RECEIVED_IT
//  * @retval LAN8742_STATUS_OK  if OK
//  *         LAN8742_STATUS_READ_ERROR if connot read register
//  */
//int32_t  LAN8742_ClearIT(lan8742_Object_t *pObj, uint32_t Interrupt)
//{
//  uint32_t readval = 0;
//  int32_t status = LAN8742_STATUS_OK;
//
//  if(pObj->IO.ReadReg(pObj->DevAddr, LAN8742_ISFR, &readval) < 0)
//  {
//    status =  LAN8742_STATUS_READ_ERROR;
//  }
//
//  return status;
//}
//
///**
//  * @brief  Get IT Flag status.
//  * @param  pObj: Pointer to device object.
//  * @param  Interrupt: IT Flag to be checked,
//  *         should be a value or a combination of the following:
//  *         LAN8742_WOL_IT
//  *         LAN8742_ENERGYON_IT
//  *         LAN8742_AUTONEGO_COMPLETE_IT
//  *         LAN8742_REMOTE_FAULT_IT
//  *         LAN8742_LINK_DOWN_IT
//  *         LAN8742_AUTONEGO_LP_ACK_IT
//  *         LAN8742_PARALLEL_DETECTION_FAULT_IT
//  *         LAN8742_AUTONEGO_PAGE_RECEIVED_IT
//  * @retval 1 IT flag is SET
//  *         0 IT flag is RESET
//  *         LAN8742_STATUS_READ_ERROR if connot read register
//  */
//int32_t LAN8742_GetITStatus(lan8742_Object_t *pObj, uint32_t Interrupt)
//{
//  uint32_t readval = 0;
//  int32_t status = 0;
//
//  if(pObj->IO.ReadReg(pObj->DevAddr, LAN8742_ISFR, &readval) >= 0)
//  {
//    status = ((readval & Interrupt) == Interrupt);
//  }
//  else
//  {
//    status = LAN8742_STATUS_READ_ERROR;
//  }
//
//  return status;
//}

/**
  * @brief  Get all IT Flag status.
  *         !!! Reading the register 0Bh also clears all the interrupt flags !!!
  * @param  pObj: Pointer to device object.
  * @retval Bit vector of IT flags that are active. Interrupt: IT Flag to be checked,
  *         should be a value or a combination of the following:
  *         LAN8742_WOL_IT
  *         LAN8742_ENERGYON_IT
  *         LAN8742_AUTONEGO_COMPLETE_IT
  *         LAN8742_REMOTE_FAULT_IT
  *         LAN8742_LINK_DOWN_IT
  *         LAN8742_AUTONEGO_LP_ACK_IT
  *         LAN8742_PARALLEL_DETECTION_FAULT_IT
  *         LAN8742_AUTONEGO_PAGE_RECEIVED_IT
  * @retval 1 IT flag is SET
  *         0 IT flag is RESET
  *         LAN8742_STATUS_READ_ERROR if connot read register
  */
int32_t SJA1105_GetITStatusClear(sja1105_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = 0;

  if(pObj->IO.ReadReg(pObj->DevAddr, SJA1105_ICSR, &readval) >= 0)
  {
    status = readval & 0xff;
  }
  else
  {
    status = SJA1105_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#define A

#ifdef B


#ifdef A

#endif

#endif



