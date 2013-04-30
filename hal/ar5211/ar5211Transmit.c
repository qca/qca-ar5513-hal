/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Transmit HAL functions
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Transmit.c#2 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "vport.h"
#include "wlanPhy.h"
#include "wlanchannel.h"
#include "wlanframe.h"
#include "wlanSend.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "halDevId.h"
#include "ui.h"
#include "display.h"
#include "ratectrl.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Beacon.h"
#include "ar5211Reset.h"
#include "ar5211KeyCache.h"
#include "ar5211Transmit.h"
#include "ar5211.h"

#include "pktlog.h"

#ifdef BUILD_AP
#include "ar5hwc.h"  /* for some debug flags only - should get rid */
#endif

/**************************************************************
 * ar5211UpdateTxTrigLevel
 *
 * Update Tx FIFO trigger level.
 *
 * Set bIncTrigLevel to TRUE to increase the trigger level.
 * Set bIncTrigLevel to FALSE to decrease the trigger level.
 *
 * Returns TRUE if the trigger level was updated
 */
A_BOOL
ar5211UpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel)
{
    UINT32 regValue, curTrigLevel;

    ASSERT(pDev);

    /* Disable chip interrupts. This is because halUpdateTrigLevel
     * is called from both ISR and non-ISR contexts.
     */
    halDisableInterrupts(pDev, HAL_INT_GLOBAL);

    regValue     = readPlatformReg(pDev, MAC_TXCFG);
    curTrigLevel = (regValue & MAC_FTRIG_M) >> MAC_FTRIG_S;

    if (bIncTrigLevel) {
        /* Increase the trigger level if not already at the maximum */
        if (curTrigLevel < MAX_TX_FIFO_THRESHOLD) {
            /* increase the trigger level */
            curTrigLevel++;
        } else {
            /* no update to the trigger level */

            /* re-enable chip interrupts */
            halEnableInterrupts(pDev, HAL_INT_GLOBAL);

            return FALSE;
        }
    } else {

        /* decrease the trigger level if not already at the minimum */
        if (curTrigLevel > MIN_TX_FIFO_THRESHOLD) {
            /* decrease the trigger level */
            curTrigLevel--;
        } else {
            /* no update to the trigger level */

            /* re-enable chip interrupts */
            halEnableInterrupts(pDev, HAL_INT_GLOBAL);

            return FALSE;
        }
    }

    /* Update the trigger level */
    writePlatformReg(pDev, MAC_TXCFG, (regValue & (~MAC_FTRIG_M)) |
                     ((curTrigLevel << MAC_FTRIG_S) & MAC_FTRIG_M));

    /* re-enable chip interrupts */
    halEnableInterrupts(pDev, HAL_INT_GLOBAL);

    return TRUE;
}

/**************************************************************************
 * ar5211ResetTxQueue
 *
 * Set the retry, aifs, cwmin/max, readyTime regs for specified queue
 */
static INLINE void
ar5211ResetTxQueue(WLAN_DEV_INFO *pDev, int queueNum, HAL_TX_QUEUE_INFO *queueInfo)
{
    A_UINT32 queueOffset = queueNum * sizeof(A_UINT32);
    A_UINT32 retryReg, value;

    if (queueInfo->mode == TXQ_MODE_INACTIVE) {
        return;
    }

    /* set cwMin/Max and AIFS values */
    writePlatformReg(pDev, MAC_D0_LCL_IFS + queueOffset,
                     A_FIELD_VALUE(MAC_D_LCL_IFS, CWMIN, LOG_TO_CW(queueInfo->logCwMin)) |
                     A_FIELD_VALUE(MAC_D_LCL_IFS, CWMAX, LOG_TO_CW(queueInfo->logCwMax)) |
                     A_FIELD_VALUE(MAC_D_LCL_IFS, AIFS, queueInfo->aifs));

    retryReg = A_FIELD_VALUE(MAC_D_RETRY_LIMIT, STA_SH, INIT_SSH_RETRY) |
               A_FIELD_VALUE(MAC_D_RETRY_LIMIT, STA_LG, INIT_SLG_RETRY);
    if (pDev->staConfig.swretryEnabled) {
        value = pDev->staConfig.hwTxRetries;
        if (value > MAC_D_RETRY_LIMIT_FR_SH_M) {
            value = MAC_D_RETRY_LIMIT_FR_SH_M;
        }
        retryReg |= A_FIELD_VALUE(MAC_D_RETRY_LIMIT, FR_LG, value) |
                    A_FIELD_VALUE(MAC_D_RETRY_LIMIT, FR_SH, value);
    } else {
        retryReg |= A_FIELD_VALUE(MAC_D_RETRY_LIMIT, FR_LG, INIT_LG_RETRY) |
                    A_FIELD_VALUE(MAC_D_RETRY_LIMIT, FR_SH, INIT_SH_RETRY);
    }

    /* Set retry limit values */
    writePlatformReg(pDev, MAC_D0_RETRY_LIMIT + queueOffset, retryReg);

    /* enable early termination on the QCU */
    writePlatformReg(pDev, MAC_Q0_MISC + queueOffset, MAC_Q_MISC_DCU_EARLY_TERM_REQ);

    if (pDev->macVersion < MAC_SREV_VERSION_OAHU) {
        /* Configure DCU to use the global sequence count */
        writePlatformReg(pDev, MAC_D0_MISC + queueOffset, MAC_531X_D_MISC_SEQ_NUM_CONTROL);
    }
    
    if (queueInfo->qFlags & TXQ_FLAG_BACKOFF_DISABLE) {
        writePlatformReg(pDev, MAC_D0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_D0_MISC + queueOffset) |
                MAC_D_MISC_POST_FR_BKOFF_DIS);
    }

    if (queueInfo->mode == TXQ_MODE_BEACON) {
        /* Configure QCU for beacons */
        writePlatformReg(pDev, MAC_Q0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_Q0_MISC + queueOffset) |
                MAC_Q_MISC_FSP_DBA_GATED | MAC_Q_MISC_BEACON_USE | MAC_Q_MISC_CBR_INCR_DIS1);

        /* Configure DCU for beacons */
        writePlatformReg(pDev, MAC_D0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_D0_MISC + queueOffset) |
                A_FIELD_VALUE(MAC_D_MISC, ARB_LOCKOUT_CNTRL, MAC_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL) |
                MAC_D_MISC_BEACON_USE | MAC_D_MISC_POST_FR_BKOFF_DIS);
    }

    if (queueInfo->mode == TXQ_MODE_CAB) {
        /* Configure QCU for CAB (Crap After Beacon) frames */

        /*
         * No longer Enable MAC_Q_MISC_RDYTIME_EXP_POLICY,
         * bug #6079.  There is an issue with the CAB Queue
         * not properly refreshing the Tx descriptor if
         * the TXE clear setting is used.
         */
        writePlatformReg(pDev, MAC_Q0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_Q0_MISC + queueOffset) |
                MAC_Q_MISC_FSP_DBA_GATED | MAC_Q_MISC_CBR_INCR_DIS1 |
                MAC_Q_MISC_CBR_INCR_DIS0);

        ASSERT(queueInfo->readyTime);
        writePlatformReg(pDev, MAC_Q0_RDYTIMECFG + queueOffset,
                A_FIELD_VALUE(MAC_Q_RDYTIMECFG, DURATION, queueInfo->readyTime) |
                MAC_Q_RDYTIMECFG_EN);

        /* Configure DCU for CAB */
        writePlatformReg(pDev, MAC_D0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_D0_MISC + queueOffset) |
                A_FIELD_VALUE(MAC_D_MISC, ARB_LOCKOUT_CNTRL, MAC_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL));
    }

    if (queueInfo->mode == TXQ_MODE_PSPOLL) {
        /*
         * We may configure psPoll QCU to be TIM-gated in the
         * future; TIM_GATED bit is not enabled currently because
         * of a hardware problem in Oahu that overshoots the TIM
         * bitmap in beacon and may find matching associd bit in
         * non-TIM elements and send PS-poll PS poll processing
         * will be done in software
         */
        writePlatformReg(pDev, MAC_Q0_MISC + queueOffset,
                readPlatformReg(pDev, MAC_Q0_MISC + queueOffset)
                | MAC_Q_MISC_CBR_INCR_DIS1);
    }
}



/**************************************************************
 * ar5211SetupTxQueue
 *
 * Allocates and initializes a DCU/QCU combination for tx
 *
 */
int
ar5211SetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo)
{
    int      queueNum  = queueInfo->priority;
    A_UINT32 queueMask = (1 << queueNum);

    ASSERT(pDev->pciInfo.DeviceID != AR5211_FPGA11B);

    ASSERT(queueNum < (A_INT32)pDev->pHalInfo->halCapabilities.halTotalQueues);
    ASSERT(!(pDev->pHalInfo->txQueueAllocMask & queueMask));

    pDev->pHalInfo->txQueueAllocMask |= queueMask;
    if (queueInfo->qFlags & TXQ_FLAG_TXINT_ENABLE) {
        pDev->pHalInfo->txNormalIntMask |= queueMask;
    }
    if (queueInfo->qFlags & TXQ_FLAG_TXDESCINT_ENABLE) {
        pDev->pHalInfo->txDescIntMask |= queueMask;
    }

    ar5211ResetTxQueue(pDev, queueNum, queueInfo);

    return queueNum;
}

/**************************************************************
 * ar5211ReleaseTxQueue
 *
 * Frees up a DCU/QCU combination
 *
 */
void
ar5211ReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum)
{
    A_UINT32 queueMask = (1 << queueNum);

    ASSERT(pDev->pHalInfo->txQueueAllocMask & queueMask);

    pDev->pHalInfo->txQueueAllocMask &= ~queueMask;
    pDev->pHalInfo->txNormalIntMask  &= ~queueMask;
    pDev->pHalInfo->txDescIntMask    &= ~queueMask;
}

/**************************************************************
 * ar5211GetTxDP
 *
 * Get the TXDP for the specified queue
 */
A_UINT32
ar5211GetTxDP(WLAN_DEV_INFO *pDev, int queueNum)
{
    A_UINT32 txdpReg;

    ASSERT(pDev->pHalInfo->txQueueAllocMask & (1 << queueNum));

    txdpReg = MAC_Q0_TXDP + (queueNum * sizeof(A_UINT32));
    return readPlatformReg(pDev, txdpReg);
}

/**************************************************************
 * ar5211SetTxDP
 *
 * Set the TxDP for the specified queue
 */
void
ar5211SetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp)
{
    A_UINT32 txdpReg;

    ASSERT(pDev->pHalInfo->txQueueAllocMask & (1 << queueNum));

    /*
     * Make sure that TXE is deasserted before setting the TXDP.  If TXE
     * is still asserted, setting TXDP will have no effect.
     */
    ASSERT(!(readPlatformReg(pDev, MAC_Q_TXE) & (1 << queueNum)));

    txdpReg = MAC_Q0_TXDP + (queueNum * sizeof(A_UINT32));
    writePlatformReg(pDev, txdpReg, txdp);
}

/**************************************************************
 * ar5211StartTxDma
 *
 * Set Transmit Enable bits for the specified queue
 */
void
ar5211StartTxDma(WLAN_DEV_INFO *pDev, int queueNum)
{
    ASSERT(pDev->pHalInfo->txQueueAllocMask & (1 << queueNum));

    // Check to be sure we're not enabling a queue that has its TXD bit set.
    ASSERT( !(readPlatformReg(pDev, MAC_Q_TXD) & (1 << queueNum)) );

    writePlatformReg(pDev, MAC_Q_TXE, (1 << queueNum));
}


/**************************************************************
 * ar5211NumTxPending
 *
 * Returns:
 *   0      if specified queue is stopped
 *   num of pending frames otherwise
 */
A_UINT32
ar5211NumTxPending(WLAN_DEV_INFO *pDev, int queueNum)
{
    A_UINT32 queueMask = (1 << queueNum);
    A_UINT32 pfcReg    = MAC_Q0_STS + (queueNum * sizeof(A_UINT32));
    A_UINT32 numPending;

    if (!(pDev->pHalInfo->txQueueAllocMask & queueMask)) {
        return 0;
    }

    numPending = readPlatformReg(pDev, pfcReg) & MAC_Q_STS_PEND_FR_CNT_M;
    if (!numPending) {
        /*
         * Pending frame count (PFC) can momentarily go to zero while TXE
         * remains asserted.  In other words a PFC of zero is not sufficient
         * to say that the queue has stopped.
         */
        if (readPlatformReg(pDev, MAC_Q_TXE) & queueMask) {
            /* arbitrarily return 1 */
            numPending = 1;
        }
    }

    return numPending;
}


/**************************************************************
 * ar5211StopTxDma
 *
 * Stop transmit on the specified queue
 */
void
ar5211StopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec)
{
    A_UINT32 queueMask = (1 << queueNum);
    int      wait      = msec * 100;

    ASSERT(pDev->pHalInfo->txQueueAllocMask & queueMask);

    writePlatformReg(pDev, MAC_Q_TXD, queueMask);

    while (ar5211NumTxPending(pDev, queueNum)) {
        if ((--wait) < 0) {
#ifdef DEBUG
            uiPrintf("ar5211StopTxDma: failed to stop Tx DMA in %d msec\n", msec);
#endif
            break;
        }
        udelay(10);
    }

    writePlatformReg(pDev, MAC_Q_TXD, 0);
}

/**************************************************************
 * ar5211PauseTx
 *
 * Pause/unpause transmission on the specified queue
 */
void
ar5211PauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause)
{
    if (pause) {
        queueMask &= MAC_5211_D_TXPSE_CTRL_M;         /* only least significant 10 bits */

        ASSERT(pDev->pHalInfo->txQueueAllocMask & queueMask);

        writePlatformReg(pDev, MAC_5211_D_TXPSE, queueMask);
        /* wait until pause request has been served */
        while (!(readPlatformReg(pDev, MAC_5211_D_TXPSE) & MAC_5211_D_TXPSE_STATUS))
            ;
    } else {
        writePlatformReg(pDev, MAC_5211_D_TXPSE, 0);
    }
}

/* Descriptor Access Functions */

void
ar5211SetupTxDesc(WLAN_DEV_INFO *pdevInfo, ATHEROS_DESC *pTxDesc, A_UINT32 hwIndex)
{
    AR5211_TX_CONTROL    *pTxControl   = TX_CONTROL(pTxDesc);
    WLAN_DATA_MAC_HEADER *pHdr         = pTxDesc->pBufferVirtPtr.header;
    SIB_ENTRY            *pSib         = pTxDesc->pDestSibEntry;
    const RATE_TABLE     *pRateTable   = pTxDesc->pVportBss->bss.pRateTable;
    A_BOOL               shortPreamble = USE_SHORT_PREAMBLE(pdevInfo, pSib, pHdr);
    DURATION             nav;
    A_UINT16             rateIndex, ctrlRateIndex;

    /*
     * this frame is being submitted to the hw - clear up the
     * status bits! need to clear up the status bits for the
     * last descriptor only
     */
    pTxDesc->pTxLastDesc->hw.word[2] = 0;
    pTxDesc->pTxLastDesc->hw.word[3] = 0;

    /* look up the rate index */
    rateIndex = rcRateFind(pdevInfo, pTxDesc);
    ctrlRateIndex = pRateTable->info[rateIndex].controlRate;

    ASSERT(!pdevInfo->staConfig.rateCtrlEnable ||
            (pdevInfo->staConfig.rateCtrlEnable && pRateTable->info[rateIndex].valid));
    ASSERT(!pdevInfo->staConfig.rateCtrlEnable ||
            (pdevInfo->staConfig.rateCtrlEnable && pRateTable->info[ctrlRateIndex].valid));

    /* Set transmit rate */
    pTxControl->transmitRate = pRateTable->info[rateIndex].rateCode
        | (shortPreamble ? pRateTable->info[rateIndex].shortPreamble : 0);

    /* update transmit rate stats */
    if (pSib) {
        pSib->stats.txRateKb = A_RATE_LPF(pSib->stats.txRateKb, pRateTable->info[rateIndex].rateKbps);
        pdevInfo->localSta->stats.txRateKb = pSib->stats.txRateKb;
    }

    /* Use auto antenna select mode instead of pSib->antTx */
    pTxControl->antModeXmit = 0;

    pTxControl->PktType =
        ((pHdr->frameControl.fType == FRAME_MGT) && (pHdr->frameControl.fSubtype == SUBT_PROBE_RESP))
        ? HAL_DESC_PKT_TYPE_PROBE_RESP
        : HAL_DESC_PKT_TYPE_NORMAL;

    if (hwIndex != HWINDEX_INVALID) {
        pTxControl->encryptKeyIndex = hwIndex;
        pTxControl->encryptKeyValid = 1;
    }

    if (pdevInfo->staConfig.swretryEnabled && pSib) {
        pTxControl->clearDestMask = pSib->needClearDest ? 1 : 0;
        pSib->needClearDest = FALSE;
    } else {
        /*  Non-head fragments should have a clearDestMask always off as
         *  we should never send a frag if the preceding frag got dropped
         */
        pTxControl->clearDestMask = (WLAN_GET_FRAGNUM(pHdr->seqControl) == 0) ? 1 : 0;
    }

    /* If not unicast, set duration to be zero */
    if (isGrp(&pHdr->address1)) {
        pTxControl->noAck = 1;
        WLAN_SET_DURATION_NAV(pHdr->durationNav, 0);

        return;
    }

    /*
     * Set RTSCTS if MPDU is above the threshold; for fragmented frames
     * the hardware will automatically ignore the values except for the
     * 1st frag in a (new or retried) burst - checked with hw guys (see
     * bug #5914)
     */
    pTxControl->RTSCTSEnable = (pTxControl->frameLength > RTS_THRESHOLD(pdevInfo)) ? 1 : 0;

    /* Set the frame control duration field */
    nav = PHY_COMPUTE_TX_TIME(pRateTable, WLAN_CTRL_FRAME_SIZE, ctrlRateIndex, shortPreamble);
    if (pHdr->frameControl.moreFrag) {
        A_UINT32 nextFragLen = pTxDesc->pTxLastDesc->pNextVirtPtr->hw.txControl.frameLength;

        /* add another '+ sifs + ack' + sifs + time for next frag */
        nav <<= 1;
        nav += PHY_COMPUTE_TX_TIME(pRateTable, nextFragLen, rateIndex, shortPreamble);
    }
    WLAN_SET_DURATION_NAV(pHdr->durationNav, nav);

    return;
}

/*
 * If we do not have a SibEntry we operate on a dummy stats.  Since
 * we do not care much about this data it is safe for multiple contexts.
 */
LOCAL WLAN_STATS dummySibStats;

/*
 * Processing of HW TX descriptor.
 */
A_STATUS
ar5211ProcessTxDesc(WLAN_DEV_INFO *pdevInfo, ATHEROS_DESC *pTxDesc)
{
    ATHEROS_DESC        *pFirst      = pTxDesc->pTxFirstDesc;
    AR5211_TX_STATUS    *pTxStatus   = TX_STATUS(pTxDesc);
    AR5211_TX_CONTROL   *pTxControl  = TX_CONTROL(pFirst);
    SIB_ENTRY           *pSib        = pFirst->pDestSibEntry;
    WLAN_FRAME_HEADER   *pWlanHdr    = pFirst->pBufferVirtPtr.header;
    WLAN_STATS          *pLocalStats = &pdevInfo->localSta->stats;
    WLAN_STATS          *pSibStats   = pSib ? &pSib->stats : &dummySibStats;
    int                 txRate       = pTxControl->transmitRate;
    A_UINT16            shortRetryCount, longRetryCount;
    A_RSSI              rssi;

    ASSERT(pTxDesc->status.tx.status == NOT_DONE);

    /* ensure we have the status correctly */
    A_DESC_CACHE_INVAL(pTxDesc);
    if (!pTxStatus->done) {
        return A_EBUSY;
    }

    if (pTxStatus->pktTransmitOK) {
        pTxDesc->status.tx.status = TRANSMIT_OK;
    } else if (pTxStatus->excessiveRetries) {
        ASSERT(!pTxStatus->filtered);
        pTxDesc->status.tx.status = EXCESSIVE_RETRIES;
    } else {
        ASSERT(pTxStatus->filtered);
        pTxDesc->status.tx.status = FILTERED;
    }

    /* 
     * Account for Oahu HW bug where the number of retries is 
     * one less than it should be.  Also, 0 retries and 1 retry
     * are both reported as 0 retries.
     * It is important to report the correct number of retries
     * as rate control uses it for antenna diversity etc.
     */
    shortRetryCount = (A_UINT16) pTxStatus->shortRetryCount;
    longRetryCount  = (A_UINT16) pTxStatus->longRetryCount;

    if (shortRetryCount > 0) {
        shortRetryCount++;
    }
    if (longRetryCount > 0) {
        longRetryCount++;
    }

    /* Update software copies of the HW status */
    pTxDesc->status.tx.seqNum           = (A_UINT16)pTxStatus->seqNum;
    pTxDesc->status.tx.timestamp        = (A_UINT16)pTxStatus->sendTimestamp;
    pTxDesc->status.tx.retryCount       = shortRetryCount + longRetryCount;

    /* Update with final rate at which frame was sent. */
    pTxDesc->status.tx.rate = pFirst->pVportBss->bss.pRateTable->
                                  rateCodeToIndex[txRate];

    rssi = (A_RSSI) pTxStatus->ackSigStrength;

    PKTLOG_TX_PKT(pdevInfo, pTxControl->frameLength,
            pTxControl->transmitRate,
            rssi,
            pTxDesc->status.tx.retryCount,
            pTxStatus->filtered         << 3 |
            pTxStatus->fifoUnderrun     << 2 |
            pTxStatus->excessiveRetries << 1 |
            pTxStatus->pktTransmitOK    << 0,
            *(A_UINT16*)&pWlanHdr->frameControl,
            (A_UINT16)pTxStatus->seqNum,
            readPlatformReg(pdevInfo, MAC_DEF_ANTENNA) & 0x7,
            (A_UINT16)pTxStatus->sendTimestamp);

    /* Update statistics */
    if (pTxStatus->pktTransmitOK) {
        pLocalStats->ackRssi = A_RSSI_LPF(pLocalStats->ackRssi, rssi);
        pSibStats->ackRssi   = A_RSSI_LPF(pSibStats->ackRssi, rssi);

        /*
         * Update stats if there were any retries for the good frame.
         *
         * If the frame saw excessive retries, do not change the antenna.
         * This matches Oahu, and is ok for Crete unless the HW retry count
         * is set to less than three.
         *
         * Toggle antenna setting in SIB.  The hardware switches the
         * antennas every two transmit failures.  This is for the 2nd retry
         * (3rd transmit),  3th retry (4th transmit), etc.
         *
         * For excessive retries keep the antenna the same.
         */
        if (shortRetryCount || longRetryCount) {
            pLocalStats->TotalRetries++;
            pSibStats->TotalRetries++;
            if (pTxStatus->shortRetryCount) {
                pLocalStats->shortFrameRetryBins[shortRetryCount]++;
                pSibStats->shortFrameRetryBins[shortRetryCount]++;
            } else {
                pLocalStats->RetryBins[longRetryCount]++;
                pSibStats->RetryBins[longRetryCount]++;
            }
        }
    } else {
        pLocalStats->TransmitErrors++;
        pSibStats->TransmitErrors++;

        if (pTxStatus->excessiveRetries) {
            DRV_LOG(DRV_DEBUG_INT, ("Transmit excessive retries\n"));
            pLocalStats->TxExcessiveRetries++;
            pSibStats->TxExcessiveRetries++;
        } else if (pTxStatus->filtered) {
            DRV_LOG(DRV_DEBUG_INT,("Transmit filtered\n"));
            pLocalStats->TxFiltered++;
            pSibStats->TxFiltered++;
        } else if (pTxStatus->fifoUnderrun) {
            DRV_LOG(DRV_DEBUG_INT,("Transmit fifo underrun\n"));
            pLocalStats->TxDmaUnderrun++;
            pSibStats->TxDmaUnderrun++;
        } else {
            apPanic("Unknown transmit error.");
        }
    }

    /*
     * Don't bother counting the broadcast/multicast frames
     * because the frame won't send ACK back; oahu doesn't
     * send acks in promiscuous mode leading to excessive
     * retries for all frames - so don't update ratectrl in
     * such a situation
     */
    ASSERT(txRate);
    if (!pTxStatus->filtered &&
        (pdevInfo->rxFilterReg & HAL_RX_UCAST) &&
        !isGrp(&pWlanHdr->address1))
    {
        rcUpdate(pdevInfo, 
                 pSib,
                 pFirst->pVportBss->bss.pRateTable->rateCodeToIndex[txRate],
                 pTxControl->frameLength,
                 (A_BOOL)pTxStatus->excessiveRetries,
                 pTxDesc->status.tx.retryCount,
                 rssi,
                 0);
    }

    return A_OK;
}

A_BOOL
ar5211GetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_BOOL swap)
{
    AR5211_TX_STATUS *pTxStatus = TX_STATUS(pTxDesc);

    return (A_BOOL)pTxStatus->done;
}


#if defined(DEBUG) || defined(_DEBUG)

void
ar5211DebugPrintTxDesc(WLAN_DEV_INFO *pdevInfo, ATHEROS_DESC *pDesc, A_BOOL verbose)
{
    AR5211_TX_CONTROL *pTxControl = TX_CONTROL(pDesc);
    AR5211_TX_STATUS  *pTxStatus  = TX_STATUS(pDesc);

    A_DESC_CACHE_INVAL(pDesc);

    if (verbose) {
        uiPrintf(CONTROL_1      "%08x  ",        pDesc->hw.word[0]);
        uiPrintf(CONTROL_2      "%08x\n",        pDesc->hw.word[1]);
        uiPrintf(FRAME_LEN      "%03x       ",   pTxControl->frameLength);
        uiPrintf(XMIT_RATE      "%01x\n",        pTxControl->transmitRate);
        uiPrintf(RTS_CTS_EN     "%01x         ", pTxControl->RTSCTSEnable);
        uiPrintf(CLR_DEST_MSK   "%01x\n",        pTxControl->clearDestMask);
        uiPrintf(ANT_MODE_TX    "%01x         ", pTxControl->antModeXmit);
        uiPrintf(ATIM           "%01x         ", pTxControl->PktType);
        uiPrintf(INT_REQ        "%01x\n",        pTxControl->interruptReq);
        uiPrintf(ENCRYPT_KEY_VLD"%01x         ", pTxControl->encryptKeyValid);
        uiPrintf(BUFFER_LEN     "%03x       ",   pTxControl->bufferLength);
        uiPrintf(MORE           "%01x\n",        pTxControl->more);
        uiPrintf(ENCRYPT_KEY_IDX"%02x\n",        pTxControl->encryptKeyIndex);

        return;
    }

    uiPrintf("%p:", pDesc);
    if (!pTxStatus->done) {
        uiPrintf(" .");
    } else {
        if (pTxStatus->pktTransmitOK) {
            uiPrintf(" ok");
        } else if (pTxStatus->excessiveRetries) {
            uiPrintf("  x");
        } else if (pTxStatus->filtered) {
            uiPrintf("  f");
        } else {
            uiPrintf("  ?");
        }
    }
    if (pDesc->staleFlag) {
        uiPrintf(" [STALE]");
    }
    if (pDesc->pDestSibEntry) {
        uiPrintf(" sta%d", pDesc->pDestSibEntry->assocId & 0x3fff);
    }
    if (pTxControl->encryptKeyValid) {
        uiPrintf(" slot%d%s[%s]", pTxControl->encryptKeyIndex,
                 pTxControl->clearDestMask ? "+" : "",
                 ar5211DebugGetKeyType(pdevInfo, pTxControl->encryptKeyIndex));
    }
    if ((pDesc->pTxFirstDesc == pDesc) && pDesc->pBufferVirtPtr.byte) {
        FRAME_CONTROL *fc = &pDesc->pBufferVirtPtr.header->frameControl;
        A_UINT16       rateKbps;

        uiPrintf(" %s[%s%s%s]",
                 halFrameTypeToName[(fc->fType << 4) + fc->fSubtype],
                 fc->wep ? "Encrypt" : "",
                 fc->retry ? "Retry" : "",
                 fc->moreFrag ? "Morefrag" : "");
        uiPrintf(" len:%d/%d", pTxControl->bufferLength, pTxControl->frameLength);
        if (pDesc->pVportBss && pDesc->pVportBss->bss.pRateTable) {
            rateKbps = pDesc->pVportBss->bss.pRateTable->info[
                pDesc->pVportBss->bss.pRateTable->rateCodeToIndex[pTxControl->transmitRate]
                ].rateKbps;
            uiPrintf(" @%d", rateKbps/1000);
        }
    } else {
        uiPrintf(" ---- len:%d", pTxControl->bufferLength);
    }
    if (pDesc->swRetryFlag) {
        uiPrintf(" requeued");
    }
    uiPrintf(" %s", pDesc->hw.txControl.more ? "->" : "|");
    if (pDesc->pNextVirtPtr) {
        if (pDesc->nextPhysPtr != pDesc->pNextVirtPtr->thisPhysPtr) {
            uiPrintf(" {0x%x != 0x%x}", pDesc->nextPhysPtr, pDesc->pNextVirtPtr->thisPhysPtr);
        }
    } else if (pDesc->nextPhysPtr) {
        uiPrintf(" {0x%x != NULL}", pDesc->nextPhysPtr);
    }
    uiPrintf("\n");
}

#endif

#endif // #ifdef BUILD_AR5211
