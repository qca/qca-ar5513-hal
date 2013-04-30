/*
 * Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 * Routines used to initialize and generated beacons for the AR5513/AR531X.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Beacon.c#7 $
 */

#ifdef BUILD_AR5513

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
#include "ar5513MacReg.h"
#include "ar5513Beacon.h"
#include "ar5513Interrupts.h"
#include "ar5513Misc.h"
#include "ar5513Mac.h"

#include "wlanbeacon.h"     /* BEACON_INFO */

/**************************************************************
 * ar5513SetupBeaconDesc
 *
 * Fills in the control fields of the beacon descriptor
 *
 * Assumes: frameLen does not include the FCS
 */
void
ar5513SetupBeaconDesc(WLAN_DEV_INFO* pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp)
{
    A_UINT16 antKeyCacheIdx;
    A_UINT8 antCnt, antWinCnt, antIdx;
    AR5513_TX_CONTROL *pTxControl  = TX_CONTROL(pDesc);
    BEACON_INFO       *pBeaconInfo = pDesc->pVportBss->bss.pBeaconInfo;
    const RATE_TABLE  *pRateTable  = pDesc->pVportBss->bss.pRateTable;
    A_UINT16           rateIndex   = pDesc->pVportBss->bss.defaultRateIndex;

    ASSERT(pBeaconInfo);

    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN) {

	/*
	** The dual chain antenna algorithm uses
	** a sliding window scheme. One beacon is 
	** transmitted on each antenna. The window
	** slides forward by one antenna after each
	** iteration. For example, if we have antennas
	** numbered as 1,2,3, and 4 then the sequence
	** will look as follows:
	** 1 2 3 4   2 3 4 1  3 4 1 2  4 1 2 3
	*/

	antCnt = pDev->pHalInfo->beaconAntCtrl.halAntCnt;
	antWinCnt = pDev->pHalInfo->beaconAntCtrl.halAntWinCnt;

	if (antWinCnt + antCnt >= 4) {
	    antIdx = (antWinCnt + antCnt) - 4;
	}
	else {
	    antIdx = antWinCnt + antCnt;
	}

	antKeyCacheIdx = pDev->pHalInfo->beaconAntCtrl.halKeyCacheTbl[antIdx];

	if (++antCnt >= 4) {
	    if (++antWinCnt >= 4) {
		antWinCnt = 0;
	    }
	    antCnt = 0;
	}

	pDev->pHalInfo->beaconAntCtrl.halAntCnt = antCnt;
	pDev->pHalInfo->beaconAntCtrl.halAntWinCnt = antWinCnt;
    }
    else {
	/*
	**  Single Chain: Switch Antennas every 4 beacons
	*/

	if (beaconCnt & 4) {
	    antKeyCacheIdx = pDev->pHalInfo->beaconAntCtrl.halKeyCacheTbl[1];
	}
	else {
	    antKeyCacheIdx = pDev->pHalInfo->beaconAntCtrl.halKeyCacheTbl[0];
	}
    }

    /* clear the status fields */
    A_MEM_ZERO((void *) TX_STATUS(pDesc),sizeof(AR5513_TX_STATUS));

    /* TURBO_PRIME */
    if ((pDev->staConfig.abolt & ABOLT_TURBO_PRIME) && isAp) {
        pTxControl->interruptReq = 1;
    }

#ifdef AR5513_CCC
    /* Need to know when beacon is transmitted
     * for Coordinated channel change */
    if (isAp) {
        pTxControl->interruptReq = 1;
    }
#endif
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
            pDesc->hw.word[2] = cpu2le32(pDesc->hw.word[2]);
            pDesc->hw.word[3] = cpu2le32(pDesc->hw.word[3]);
        }

	/* Set TX Antenna */
	pTxControl->bf_enable = 0;
	pTxControl->destIdx = antKeyCacheIdx;
	pTxControl->destIdxValid = TRUE;

        /* set buffer and frame length */
        pTxControl->bufferLength = pBeaconInfo->frameLen;
        pTxControl->frameLength  = pBeaconInfo->frameLen + FCS_FIELD_SIZE;

        if (pDev->pHalInfo->swSwapDesc) {
            pDesc->hw.word[0] = cpu2le32(pDesc->hw.word[0]);
            pDesc->hw.word[1] = cpu2le32(pDesc->hw.word[1]);
            pDesc->hw.word[2] = cpu2le32(pDesc->hw.word[2]);
            pDesc->hw.word[3] = cpu2le32(pDesc->hw.word[3]);
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

    /* Set TX Antenna */
    pTxControl->bf_enable = 0;
    pTxControl->destIdx = antKeyCacheIdx;
    pTxControl->destIdxValid = TRUE;

    pTxControl->PktType         = HAL_DESC_PKT_TYPE_BEACON;

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
        pTxControl->clearDestMask = 1;
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
 * ar5513BeaconInit
 *
 * Initializes all of the hardware registers used to send beacons.
 *
 */
void
ar5513BeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp)
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
        ar5513EnableInterrupts(pDev, HAL_INT_SWBA);
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
 * ar5513SetupGrpPollChain
 *
 * Fills in the control fields of the group poll desc chain
 *
 * Assumes: frameLen does not include the FCS
 */
void
ar5513SetupGrpPollChain(WLAN_DEV_INFO* pDev, ATHEROS_DESC *pHead, 
                        ATHEROS_DESC *pTail, A_UINT32 hwIndex)
{
    VPORT_BSS *pVportXrBss = GET_XR_BSS(pDev);
    OP_BSS *pOpBss; 
    AR5513_TX_CONTROL *pTxControl;
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
        ar5513SwapHwDesc(pDev, pHead, pTail, 0);
    }
}

/**************************************************************
 * ar5513GetMacTimer3
 *
 * Read MAC Timer3 Register
 */
A_UINT32
ar5513GetMacTimer3(WLAN_DEV_INFO *pDev)
{
    A_UINT32 v;
    
    v = readPlatformReg(pDev, MAC_TIMER3);
    return v;
}

#endif /* BUILD_AR5513 */
