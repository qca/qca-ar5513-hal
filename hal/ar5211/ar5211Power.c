/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific power management routines.
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Power.c#1 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "vport.h"
#include "wlanPhy.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Power.h"
#include "ar5211.h"

/**************************************************************
 * ar5211SetPowerMode
 *
 * Set power mgt to the requested mode, and conditionally set
 * the chip as well
 */
A_STATUS
ar5211SetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip)
{
    A_STATUS status = A_HARDWARE;

    switch (powerRequest) {

    case NETWORK_SLEEP:
        /*
         * Notify Power Mgt is enabled in self-generated frames.
         * If requested, set Power Mode of chip to auto/normal.
         * Duration in units of 128us (1/8 TU)
         */
        A_REG_SET_BIT(pDev, MAC_STA_ID1, PWR_SAV);

#if defined(PCI_INTERFACE)
        if (setChip) {
            A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_NORMAL);
        }
        status = A_OK;
#endif
        break;

    case AWAKE:
        /*
         * Notify Power Mgt is disabled in self-generated frames.
         * If requested, force chip awake.
         *
         * Returns A_OK if chip is awake or successfully forced awake.
         *
         * WARNING WARNING WARNING
         * There is a problem with the chip where sometimes it will not wake up.
         */
#if defined(PCI_INTERFACE)
        if (setChip) {
            int i;

            A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_FWAKE);
            udelay(10);  // Give chip the chance to awake

            for (i = 0; i < POWER_UP_TIME / 200; i++) {
                if ((A_REG_RD(pDev, MAC_PCICFG) & MAC_PCICFG_SPWR_DN) == 0) {
                    status = A_OK;
                    break;
                }
                udelay(200);
                A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_FWAKE);
            }
            ASSERT(status == A_OK);
            if (status != A_OK) {
                uiPrintf("ar5211SetPowerModeAwake: Failed to leave sleep\n");
            }
        } else {
            status = A_OK;
        }

        if (status == A_OK) {
            A_REG_CLR_BIT(pDev, MAC_STA_ID1, PWR_SAV);
        }
#else
        status = A_OK;
#endif
        break;

    case FULL_SLEEP:
        /*
         * Notify Power Mgt is enabled in self-generated frames.
         * If requested, force chip to sleep.
         */
        A_REG_SET_BIT(pDev, MAC_STA_ID1, PWR_SAV);

#if defined(PCI_INTERFACE)
        if (setChip) {
            A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_FSLEEP);
        }
#endif
        status = A_OK;
        break;

    default:
        status = A_ENOTSUP;
        break;
    }

    return status;
}

/**************************************************************
 * ar5211GetPowerMode
 *
 * Return the current sleep mode of the chip
 */
A_UINT32
ar5211GetPowerMode(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
    // Just so happens the h/w maps directly to the abstracted value
    return A_REG_RD_FIELD(pDev, MAC_SCR, SLMODE);
#else
    return AWAKE;
#endif
}

/**************************************************************
 * ar5211GetPowerStatus
 *
 * Return the current sleep state of the chip
 * TRUE = sleeping
 */
A_BOOL
ar5211GetPowerStatus(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
    return A_REG_IS_BIT_SET(pDev, MAC_PCICFG, SPWR_DN);
#else
    return FALSE;
#endif
}

/**************************************************************
 * ar5211SetupPSPollDesc
 *
 * Initialize for PS-Polls
 */
void
ar5211SetupPsPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc)
{
    VPORT_BSS *pVportBaseBss = GET_BASE_BSS(pDev);
    AR5211_TX_CONTROL *pTxControl = TX_CONTROL(pDesc);

    // Send PS-Polls at 6mbps.
    pTxControl->transmitRate =
        pVportBaseBss->bss.pRateTable->info[PSPOLL_RATE_INDEX].rateCode;

    pTxControl->antModeXmit = 0;

    // PS-Poll descriptor points to itself with the VEOL bit set.
    pTxControl->noAck         = 0;
    pTxControl->clearDestMask = 1;
    pTxControl->PktType       = HAL_DESC_PKT_TYPE_PSPOLL;
    pTxControl->VEOL          = 1;
    pDesc->nextPhysPtr        = pDesc->thisPhysPtr;
}

#endif // #ifdef BUILD_AR5211
