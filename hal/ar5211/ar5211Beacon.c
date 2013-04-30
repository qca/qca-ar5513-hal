/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * Routines used to initialize and generated beacons for the AR5211/AR531X.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Beacon.c#2 $
 */

#ifdef BUILD_AR5211

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "vport.h"
#include "wlanPhy.h"
#include "wlanchannel.h"
#include "halApi.h"
#include "hal.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Beacon.h"
#include "ar5211.h"

#include "wlanbeacon.h"     /* BEACON_INFO */

/**************************************************************
 * ar5211SetupBeaconDesc
 *
 * Fills in the control fields of the beacon descriptor
 *
 * Assumes: frameLen does not include the FCS
 */
void
ar5211SetupBeaconDesc(WLAN_DEV_INFO* pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp)
{
    A_UINT32 antMode;
    VPORT_BSS *pVportBaseBss = GET_BASE_BSS(pDev);
    AR5211_TX_CONTROL *pTxControl = TX_CONTROL(pDesc);
    BEACON_INFO       *pBeaconInfo = pDesc->pVportBss->bss.pBeaconInfo;

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
    pDesc->hw.word[2] = pDesc->hw.word[3] = 0;

    /* set tx antenna mode */
    pTxControl->antModeXmit = antMode;

    /* set buffer and frame length */
    pTxControl->bufferLength    = pBeaconInfo->frameLen;
    pTxControl->frameLength     = pBeaconInfo->frameLen + FCS_FIELD_SIZE;
    pTxControl->more            = 0;

    /* exit early if this is a AP beacon desc update */
    if (isAp && updtAntOnly) {
        return;
    } 

    /* 11b beacons always go out in long preamble - so don't enable short preamble */
    pTxControl->transmitRate = pVportBaseBss->bss.pRateTable->info[LOWEST_RATE_INDEX].rateCode;

    /* Fill in the link pointers. */
    if (!isAp) {
        /*
         * If we're a client, link the beacon descriptor back onto itself
         * with the VEOL bit set
         */
        pDesc->nextPhysPtr = pDesc->thisPhysPtr;
        pTxControl->VEOL   = 1;
    }

    pTxControl->PktType = HAL_DESC_PKT_TYPE_BEACON;
    pTxControl->noAck   = 1;
    pTxControl->more    = 0;
}

/**************************************************************
 * ar5211BeaconInit
 *
 * Initializes all of the hardware registers used to send beacons.
 *
 */
void
ar5211BeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp)
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

    } else {
        /* Enable SWBA interrupt. */
        writePlatformReg(pDev, MAC_IER, 0);
        pDev->MaskReg |= MAC_IMR_SWBA;
        writePlatformReg(pDev, MAC_IMR, pDev->MaskReg);
        writePlatformReg(pDev, MAC_IER, MAC_IER_ENABLE);

        /* Need to enable TXOK and TXERR ISRs for the CAB queue! do it here by
         * generally enabling interrupts for queues based on the txInterruptMask
         */
        A_REG_RMW_FIELD(pDev, MAC_IMR_S0, QCU_TXOK,  pDev->pHalInfo->txNormalIntMask);
        A_REG_RMW_FIELD(pDev, MAC_IMR_S1, QCU_TXERR, pDev->pHalInfo->txNormalIntMask);
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
}

/**************************************************************
 * ar5211GetMacTimer3
 *
 * Read MAC Timer3 Register
 */
A_UINT32
ar5211GetMacTimer3(WLAN_DEV_INFO *pDev)
{
    A_UINT32 v;
    
    v = readPlatformReg(pDev, MAC_TIMER3);
    return v;
}

#endif /* BUILD_AR5211 */
