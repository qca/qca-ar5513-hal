/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * Routines used to initialize and generated beacons for the AR5212/AR531X.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Beacon.c#2 $
 */

#ifdef BUILD_AR5212

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanPhy.h"
#include "wlanchannel.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "vport.h"

/* Headers for HW private items */
#include "ar5212Reg.h"
#include "ar5212Beacon.h"
#include "ar5212Interrupts.h"
#include "ar5212Misc.h"
#include "ar5212.h"

#include "wlanbeacon.h"     /* BEACON_INFO */

/**************************************************************
 * ar5212SetupBeaconDesc
 *
 * Fills in the control fields of the beacon descriptor
 *
 * Assumes: frameLen does not include the FCS
 */
void
ar5212SetupBeaconDesc(WLAN_DEV_INFO* pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp)
{
    A_UINT32 antMode;
    AR5212_TX_CONTROL *pTxControl  = TX_CONTROL(pDesc);
    BEACON_INFO       *pBeaconInfo = pDesc->pVportBss->bss.pBeaconInfo;
    const RATE_TABLE  *pRateTable  = pDesc->pVportBss->bss.pRateTable;
    A_UINT16           rateIndex   = pDesc->pVportBss->bss.defaultRateIndex;

    ASSERT(pBeaconInfo);

    /*
    **  Switch Antennas every 4 beacons
    */

    if (updtAntOnly) {
	antMode = (beaconCnt & 4) ? 2 : 1;
    }
    else {
	antMode = 0;
    }

    /* clear the status fields */
    pDesc->hw.word[4] = pDesc->hw.word[5] = 0;

    /* TURBO_PRIME */
    if ((pDev->staConfig.abolt & ABOLT_TURBO_PRIME) && isAp) {
        pTxControl->interruptReq = 1;
    }

    if ((pDesc->pVportBss == GET_BASE_BSS(pDev)) && 
	IS_CHAN_G(pDev->staConfig.pChannel->channelFlags)) {
        rateIndex = pDev->staConfig.gBeaconRate;
    }
    pTxControl->TXRate0 = pRateTable->info[rateIndex].rateCode;

    /* exit early if this is a AP beacon desc antenna update */
    if (isAp && updtAntOnly) { 
        if (pDev->pHalInfo->swSwapDesc) {
            pDesc->hw.word[0] = cpu2le32(pDesc->hw.word[0]);
            pDesc->hw.word[1] = cpu2le32(pDesc->hw.word[1]);
        }
        /* set tx antenna */
        pTxControl->antModeXmit = antMode;

        /* set buffer and frame length */
        pTxControl->bufferLength = pBeaconInfo->frameLen;
        pTxControl->frameLength  = pBeaconInfo->frameLen + FCS_FIELD_SIZE;

        if (pDev->pHalInfo->swSwapDesc) {
            pDesc->hw.word[0] = cpu2le32(pDesc->hw.word[0]);
            pDesc->hw.word[1] = cpu2le32(pDesc->hw.word[1]);
        }

        return;
    } 

    if (pRateTable->info[rateIndex].phy == WLAN_PHY_XR) {
        pTxControl->CTSEnable   = 1;
        pTxControl->RTSCTSRate  = XR_CTS_RATE(pDev);
        pTxControl->RTSCTSDur   = 
			PHY_COMPUTE_TX_TIME(pRateTable, 
					    pTxControl->frameLength, 
					    rateIndex, 
					    FALSE);
    }

    pTxControl->TXDataTries0    = 1;
    pTxControl->noAck           = 1;
    pTxControl->transmitPwrCtrl = MAX_RATE_POWER;
    pTxControl->antModeXmit     = antMode;
    pTxControl->PktType         = HAL_DESC_PKT_TYPE_BEACON;
    pTxControl->compProc        = 3;

    /* set buffer and frame length */
    pTxControl->bufferLength    = pBeaconInfo->frameLen;
    pTxControl->frameLength     = pBeaconInfo->frameLen + FCS_FIELD_SIZE;
    pTxControl->more            = 0;

    /* Fill in the link pointers. */
    if (!isAp) {
        /*
         * If we're a client, link the beacon descriptor back onto itself
         * with the VEOL bit set.
         *
         * NOTE WELL: Do NOT link the virtual pointer back to itself as this
         *            will cause a hang when resetting the queue.
         */
        pDesc->nextPhysPtr  = pDesc->thisPhysPtr;
        pTxControl->VEOL    = 1;
        ASSERT(pDesc->pNextVirtPtr != pDesc);
    }

    if (pDev->pHalInfo->swSwapDesc) {
        pDesc->bufferPhysPtr = cpu2le32(pDesc->bufferPhysPtr);
        pDesc->hw.word[0]    = cpu2le32(pDesc->hw.word[0]);
        pDesc->hw.word[1]    = cpu2le32(pDesc->hw.word[1]);
        pDesc->hw.word[2]    = cpu2le32(pDesc->hw.word[2]);
        pDesc->hw.word[3]    = cpu2le32(pDesc->hw.word[3]);
    }
}

/**************************************************************
 * ar5212BeaconInit
 *
 * Initializes all of the hardware registers used to send beacons.
 *
 */
void
ar5212BeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp)
{
    A_UINT32 beaconInterval = pDev->bssDescr->beaconInterval;
    A_UINT32 value = 0;
    A_UINT32 nextTbtt;

    if (tsf) {
        WLAN_TIMESTAMP timestamp;
        A_UINT32 now;

        halGetTsf(pDev, &timestamp);
        now = TSF_TO_TU(timestamp);

        while (tsf < now) {
            tsf += beaconInterval;
        }
    }
    nextTbtt = tsf + beaconInterval;

    /* Next TBTT should always be a multiple of the beacon period */
    nextTbtt = A_ROUNDUP(nextTbtt, beaconInterval);

    /* Set the next beacon time register */
    writePlatformReg(pDev, MAC_TIMER0, nextTbtt);

    /* Set DMA Beacon Alert (DBA) timer (in 1/8 TU = 128 us) */
    value = (nextTbtt * 8) - (DMA_BEACON_RESPONSE_TIME / 128);
    writePlatformReg(pDev, MAC_TIMER1, value);

    /* Set Software Beacon Alert (SWBA) timer (in 1/8 TU = 128 us) */
    value = (nextTbtt * 8) - (SW_BEACON_RESPONSE_TIME / 128);
    writePlatformReg(pDev, MAC_TIMER2, value);

    if (!isAp) {

        /* Set the ATIM window 
         * Our hardware does not support an ATIM window of 0 (beacons will not work).
         * If the ATIM windows is 0, force it to 1.
         */
        value = nextTbtt + (pDev->staConfig.atimWindow ? pDev->staConfig.atimWindow : 1);
        writePlatformReg(pDev, MAC_TIMER3, value);
        uiPrintf("\nBeaconInit -- t3: %#x = nextTbtt %#x +  atim %#x, tsf: %#x\n",
                 value, nextTbtt, pDev->staConfig.atimWindow, tsf);
    } else {
        /* Enable SWBA interrupt. */
        ar5212EnableInterrupts(pDev, HAL_INT_SWBA);
    }

    /* Set the Beacon Control register */
    value = beaconInterval | MAC_BEACON_EN;
    if (tsf == 0) {
        value |= MAC_BEACON_RESET_TSF;
        /*
         * workaround for hw bug! when resetting the TSF, write twice to the
         * corresponding register; each write to the RESET_TSF bit toggles
         * the internal signal to cause a reset of the TSF - but if the signal
         * is left high, it will reset the TSF on the next chip reset also!
         * writing the bit an even number of times fixes this issue
         */
        writePlatformReg(pDev, MAC_BEACON, MAC_BEACON_RESET_TSF);
    }
    writePlatformReg(pDev, MAC_BEACON, value);

    /*
     * Quiet Time configuration for TGh
     * For now: hw test mode only
     */
    if (pDev->staConfig.quietDuration) {
        uiPrintf("Quiet Period start @ %d for %d (every %d)\n",
                 nextTbtt + pDev->staConfig.quietOffset,
                 pDev->staConfig.quietDuration,
                 beaconInterval);
        A_REG_WR_FIELD(pDev, MAC_QUIET2, QUIET_PERIOD, beaconInterval);
        A_REG_RMW_FIELD(pDev, MAC_QUIET2, QUIET_DURATION, pDev->staConfig.quietDuration);

        A_REG_WR_FIELD(pDev, MAC_QUIET1, NEXT_QUIET, nextTbtt + pDev->staConfig.quietOffset);
        if (pDev->staConfig.quietAckCtsAllow) {
            A_REG_SET_BIT(pDev, MAC_QUIET1, QUIET_ACK_CTS_ENABLE);
        }
        A_REG_SET_BIT(pDev, MAC_QUIET1, QUIET_ENABLE);
    }
}

/**************************************************************
 * ar5212SetupGrpPollChain
 *
 * Fills in the control fields of the group poll desc chain
 *
 * Assumes: frameLen does not include the FCS
 */
void
ar5212SetupGrpPollChain(WLAN_DEV_INFO* pDev, ATHEROS_DESC *pHead, 
                        ATHEROS_DESC *pTail, A_UINT32 hwIndex)
{
    VPORT_BSS *pVportXrBss = GET_XR_BSS(pDev);
    OP_BSS *pOpBss; 
    AR5212_TX_CONTROL *pTxControl;
    ATHEROS_DESC *pDesc;
    
    if (!isXrAp(pDev)) {
	return;
    }

    ASSERT(pVportXrBss);

    pOpBss = &pVportXrBss->bss;

    for (pDesc = pHead; pDesc != pTail; pDesc = pDesc->pNextVirtPtr)  {
        pTxControl = TX_CONTROL(pDesc);
        pTxControl->transmitPwrCtrl = MAX_RATE_POWER;
        pTxControl->clearDestMask   = (pDesc == pHead);
        pTxControl->destIdxValid    = (hwIndex != HWINDEX_INVALID);
        pTxControl->destIdx         = hwIndex;
        pTxControl->antModeXmit     = 0;
        pTxControl->TXRate0         = pOpBss->pRateTable->info[pOpBss->defaultRateIndex].rateCode;
        pTxControl->TXDataTries0    = 1;
        pTxControl->PktType         = HAL_DESC_PKT_TYPE_GRP_POLL;
        pTxControl->CTSEnable       = 1;
        pTxControl->RTSCTSRate      = XR_CTS_RATE(pDev);
        pTxControl->RTSCTSDur       = XR_UPLINK_TRANSACTION_TIME(pDev);
    }

    pTxControl = TX_CONTROL(pTail);
    pTxControl->transmitPwrCtrl = MAX_RATE_POWER;
    pTxControl->clearDestMask   = 1;
    pTxControl->destIdxValid    = (hwIndex != HWINDEX_INVALID);
    pTxControl->destIdx         = hwIndex;
    pTxControl->antModeXmit     = 0;
    pTxControl->PktType         = HAL_DESC_PKT_TYPE_NORMAL;
    pTxControl->TXRate0         = XR_CTS_RATE(pDev);
    pTxControl->TXDataTries0    = 1;
    pTxControl->noAck           = 1;
    pTxControl->VEOL            = 1;

    pTail->nextPhysPtr = pHead->thisPhysPtr;

    /* 
     * when compression is enabled, swSwapDesc is set (due to hardware
     * bug 7740). This is why the group polls also need to swap the
     * descriptions in software and not rely on hardware 
     */
    if (pDev->pHalInfo->swSwapDesc) {
        ar5212SwapHwDesc(pDev, pHead, pTail, 0);
    }
}

/**************************************************************
 * ar5212GetMacTimer3
 *
 * Read MAC Timer3 Register
 */
A_UINT32
ar5212GetMacTimer3(WLAN_DEV_INFO *pDev)
{
    A_UINT32 v;
    
    v = readPlatformReg(pDev, MAC_TIMER3);
    return v;
}

#endif /* BUILD_AR5212 */
