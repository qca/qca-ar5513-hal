/*
 *  Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific power management routines.
 */

#ifdef BUILD_AR5513

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Power.c#4 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanPhy.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"
#include "vport.h"

/* Headers for HW private items */
#include "ar5513MacReg.h"
#include "ar5513Power.h"
#include "ar5513Mac.h"

/**************************************************************
 * ar5513SetPowerMode
 *
 * Set power mgt to the requested mode, and conditionally set
 * the chip as well
 */

#if defined(PCI_INTERFACE)

#define FALCON_PCI_DEADLOCK_BUG 1
BOOLEAN 
ar5513SetPowerAwake(PVOID pContext)
{
    WLAN_DEV_INFO *pDev    = (WLAN_DEV_INFO *) pContext;
    A_REG_WR_FIELD(pDev, MAC_SCR_ALIAS, SLMODE, MAC_SLMODE_FWAKE);
    return TRUE;
}

BOOLEAN 
ar5513SetPowerSleep(PVOID pContext)
{
    WLAN_DEV_INFO *pDev    = (WLAN_DEV_INFO *) pContext;
    A_REG_WR_FIELD(pDev, MAC_SCR_ALIAS, SLMODE, MAC_SLMODE_FSLEEP);
    return TRUE;
}


BOOLEAN 
ar5513SetPowerPS(PVOID pContext)
{
    WLAN_DEV_INFO *pDev    = (WLAN_DEV_INFO *) pContext;
    A_REG_WR_FIELD(pDev, MAC_SCR_ALIAS, SLMODE, MAC_SLMODE_NORMAL);
    return TRUE;
}

#else
#undef  FALCON_PCI_DEADLOCK_BUG
#endif

A_STATUS
ar5513SetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip)
{
    A_STATUS status = A_HARDWARE;
    
    switch (powerRequest) {

    case NETWORK_SLEEP:
        /*
         * Notify Power Mgt is enabled in self-generated frames.
         * If requested, set Power Mode of chip to auto/normal.
         * Duration in units of 128us (1/8 TU)
         */
#if defined(PCI_INTERFACE)
        /* PCI thick driver */
        A_REG_WR(pDev, RST_AMBACLK_CTL, AMBACLK_REF_CLK);
#endif
        A_REG_SET_BIT(pDev, MAC_STA_ID1, PWR_SAV);

#if defined(PCI_INTERFACE)
        if (setChip) {
#if defined(FALCON_PCI_DEADLOCK_BUG)
            ATH_OSSYNCINTR(pDev, ar5513SetPowerPS, pDev);
#else  /* !FALCON_PCI_DEADLOCK_BUG */
            A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_NORMAL);
#endif /* !FALCON_PCI_DEADLOCK_BUG */
        }
        status = A_OK;  // DF_MERGE, temp restore here for debug on VF
#endif /* PCI_INTERFACE */
        // DF_MERGE moved here for DF, temp disable for debug on VF
        //status = A_OK;
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
            int         i;
#if defined(FALCON_PCI_DEADLOCK_BUG)
            A_UINT32    ulAddressLow;
            A_UINT32    reg;

            ATH_OSSYNCINTR(pDev, ar5513SetPowerAwake, pDev);
            udelay(10);  // Give chip the chance to awake

            if (pDev->pHalInfo->halInit) {
            
                ulAddressLow  = cpu2le32(pDev->staConfig.macAddr.st.word);
                ASSERT(ulAddressLow);

                for (i = 0; i < POWER_UP_TIME / 200; i++) {
                    reg = readPlatformReg(pDev, MAC_STA_ID0);
                    if ( reg ==  ulAddressLow) {
                        status = A_OK;
                        break;
                    }
                    udelay(200);
                }
            } else if (!pDev->pHalInfo->macReset) {
                ulAddressLow  = 0xA5A55A5A;

                writePlatformReg(pDev, MAC_STA_ID0, ulAddressLow);
                for (i = 0; i < POWER_UP_TIME / 200; i++) {
                    reg = readPlatformReg(pDev, MAC_STA_ID0);
                    if ( reg ==  ulAddressLow) {
                        status = A_OK;
                        break;
                    }
                    udelay(200);
                    writePlatformReg(pDev, MAC_STA_ID0, ulAddressLow);
                }
            } else {
                udelay(1000);  // Give chip the chance to awake
                status = A_OK;
            }
            ASSERT(status == A_OK);
            if (status != A_OK) {
                uiPrintf("ar5513SetPowerModeAwake: Failed to leave sleep\n");
            } 
#else /* !FALCON_PCI_DEADLOCK_BUG */
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
                uiPrintf("ar5513SetPowerModeAwake: Failed to leave sleep\n");
            }
#endif /* !FALCON_PCI_DEADLOCK_BUG */
        } else {
            status = A_OK;
        }

        if (status == A_OK) {
            A_REG_CLR_BIT(pDev, MAC_STA_ID1, PWR_SAV);
        }
        /* PCI thick driver */
        A_REG_WR(pDev, RST_AMBACLK_CTL, AMBACLK_PLLC_DIV3_CLK);
#else /* !PCI_INTERFACE */
        status = A_OK;
#endif  /* !PCI_INTERFACE */
        break;

    case FULL_SLEEP:
        /*
         * Notify Power Mgt is enabled in self-generated frames.
         * If requested, force chip to sleep.
         */
#if defined(PCI_INTERFACE)
        /* PCI thick driver */
        A_REG_WR(pDev, RST_AMBACLK_CTL, AMBACLK_REF_CLK);
#endif /* PCI_INTERFACE */
        A_REG_SET_BIT(pDev, MAC_STA_ID1, PWR_SAV);

#if defined(PCI_INTERFACE)
#if defined(FALCON_PCI_DEADLOCK_BUG)
        if (setChip) {
            ATH_OSSYNCINTR(pDev, ar5513SetPowerSleep, pDev);
        }
#else /* !FALCON_PCI_DEADLOCK_BUG */
        if (setChip) {
            A_REG_RMW_FIELD(pDev, MAC_SCR, SLMODE, MAC_SLMODE_FSLEEP);
        }
#endif /* !FALCON_PCI_DEADLOCK_BUG */
#endif /* PCI_INTERFACE */
        status = A_OK;
        break;

    default:
        status = A_ENOTSUP;
        break;
    }

    return status;
}

/**************************************************************
 * ar5513GetPowerMode
 *
 * Return the current sleep mode of the chip
 */
A_UINT32
ar5513GetPowerMode(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
    ASSERT(0);
    // Just so happens the h/w maps directly to the abstracted value
    return A_REG_RD_FIELD(pDev, MAC_SCR, SLMODE);
#else
    return AWAKE;
#endif
}

/**************************************************************
 * ar5513GetPowerStatus
 *
 * Return the current sleep state of the chip
 * TRUE = sleeping
 */
A_BOOL
ar5513GetPowerStatus(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
#ifdef BUILD_AR5513
    A_UINT32 ulAddressLow;
    ulAddressLow  = cpu2le32(pDev->staConfig.macAddr.st.word);
    ASSERT(ulAddressLow);
    return (readPlatformReg(pDev, MAC_STA_ID0) ==  ulAddressLow);
#else
    return A_REG_IS_BIT_SET(pDev, MAC_PCICFG, SPWR_DN);
#endif
#else
    return FALSE;
#endif
}

/**************************************************************
 * ar5513SetupPSPollDesc
 *
 * Initialize for PS-Polls
 */
void
ar5513SetupPsPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc)
{
    VPORT_BSS *pVportBaseBss = GET_BASE_BSS(pDev);
    AR5513_TX_CONTROL *pTxControl = TX_CONTROL(pDesc);
    WIRELESS_MODE mode;
    A_UINT8 rateIndex;

    mode = wlanFindModeFromRateTable(pDev, pVportBaseBss);
    if (mode == WIRELESS_MODE_XR) {
        rateIndex = XR_PSPOLL_RATE_INDEX;
    } else {
        rateIndex = PSPOLL_RATE_INDEX;
    }

    // Send PS-Polls at 6mbps.
    pTxControl->TXRate0 =
        pVportBaseBss->bss.pRateTable->info[rateIndex].rateCode;
    pTxControl->TXDataTries0 = 1 + pDev->staConfig.hwTxRetries;

    // PS-Poll descriptor points to itself with the VEOL bit set.
    pTxControl->noAck         = 0;
    pTxControl->clearDestMask = 1;
    pTxControl->PktType       = HAL_DESC_PKT_TYPE_PSPOLL;
    pTxControl->VEOL          = 1;
    pDesc->nextPhysPtr        = pDesc->thisPhysPtr;

}

#endif // #ifdef BUILD_AR5513
