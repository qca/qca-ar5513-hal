/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Transmit HAL functions
 */

#ifdef BUILD_AR5212

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Receive.c#2 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanPhy.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "vport.h"

/* Headers for HW private items */
#include "ar5212Reg.h"
#include "ar5212Transmit.h"
#include "ar5212Misc.h"
#include "ar5212.h"

#include "pktlog.h"


/**************************************************************
 * ar5212GetRxDP
 *
 * Get the RXDP.
 */
A_UINT32
ar5212GetRxDP(WLAN_DEV_INFO *pDev)
{
    return readPlatformReg(pDev, MAC_RXDP);
}

/**************************************************************
 * ar5212SetRxDP
 *
 * Set the RxDP.
 */
void
ar5212SetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp)
{
    ASSERT(!A_REG_IS_BIT_SET(pDev, MAC_CR, RXE));
    writePlatformReg(pDev, MAC_RXDP, rxdp);
    ASSERT(readPlatformReg(pDev, MAC_RXDP) == rxdp);
}

/**************************************************************
 * ar5212EnableReceive
 *
 * Set Receive Enable bits.
 */
void
ar5212EnableReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_CR, MAC_CR_RXE);
}

/**************************************************************
 * ar5212StopDmaReceive
 *
 * Stop Receive at the DMA engine
 */
A_STATUS
ar5212StopDmaReceive(WLAN_DEV_INFO *pDev)
{
    A_UINT32    regVal, i;

    /* Set receive disable bit */
    writePlatformReg(pDev, MAC_CR, MAC_CR_RXD);

    /* Wait up to 10 ms. Was 100 ms for FPGA. */
    for (i=0; i<1000; i++) {

        regVal = readPlatformReg(pDev, MAC_CR);

        if (!(regVal & MAC_CR_RXE)) {
            return A_OK;
        }

        udelay(10);
    }

    ASSERT(0);
    return A_HARDWARE;
}

/**************************************************************
 * ar5212StartPcuReceive
 *
 * Start Transmit at the PCU engine (unpause receive)
 */
void
ar5212StartPcuReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_DIAG_SW,
                     readPlatformReg(pDev, MAC_DIAG_SW) & ~(MAC_DIAG_RX_DIS));
}

/**************************************************************
 * ar5212StopPcuReceive
 *
 * Stop Transmit at the PCU engine (pause receive)
 */
void
ar5212StopPcuReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_DIAG_SW, readPlatformReg(pDev, MAC_DIAG_SW) | MAC_DIAG_RX_DIS);
}

/**************************************************************
 * ar5212SetMulticastFilter
 *
 * Set multicast filter 0 (lower 32-bits)
 *               filter 1 (upper 32-bits)
 */
void
ar5212SetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1)
{
    writePlatformReg(pDev, MAC_MCAST_FIL0, filter0);
    writePlatformReg(pDev, MAC_MCAST_FIL1, filter1);
}

/**************************************************************
 * ar5212MulticastFilterIndex
 *
 * Clear/Set multicast filter by index
 */
void
ar5212MulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet)
{
    A_UINT32 filterReg;
    A_UINT32 filterIndex;
    A_UINT32 value;

    ASSERT(index < 64);

    if (index < 32) {
        filterReg   = MAC_MCAST_FIL0;
        filterIndex = index;
    } else {
        filterReg   = MAC_MCAST_FIL1;
        filterIndex = index - 32;
    }

    value = readPlatformReg(pDev, filterReg);

    if (bSet) {
        value |= (1 << filterIndex);
    } else {
        value &= ~(1 << filterIndex);
    }

    writePlatformReg(pDev, filterReg, value);
}

/**************************************************************************
 * ar5212RxFilter
 *
 * Turn off/on bits in the receive packet filter.  Save it for resets, etc.
 *
 * Bits can be MAC_RX_UCAST, MAC_RX_MCAST, MAC_RX_BCAST, MAC_RX_BEACON,
 * MAC_RX_PROM, MAC_RX_CONTROL
 *
 * oper can be HAL_RX_FILTER_INIT, HAL_RX_FILTER_SET, HAL_RX_FILTER_CLEAR,
 * HAL_RX_FILTER_TEMP, HAL_RX_FILTER_RESTORE.
 *
 * NOTE: the bits parameter is not used for operation type
 * HAL_RX_FILTER_RESTORE.
 *
 */
void
ar5212RxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper)
{

/*
 * Helpful macros used in the context of this routine only!
 */
#define MAC_BITS(_bits) (                                                     \
    ((_bits) & HAL_RX_COMMON) |                                               \
    (((_bits) & HAL_RX_XR_POLL) ? MAC_RX_XR_POLL : 0) |                       \
    (((_bits) & HAL_RX_PROBE_REQ) ? MAC_RX_PROBE_REQ : 0)                     \
)
#define PHY_BITS(_bits) (                                                     \
    (((_bits) & HAL_RX_PHY_RADAR) ? MAC_PHY_ERR_RADAR : 0)       |            \
    (((_bits) & HAL_RX_PHY_ERR) ?                                             \
         (MAC_PHY_ERR_OFDM_TIMING | MAC_PHY_ERR_CCK_TIMING) : 0) |            \
    (((_bits) & HAL_RX_DOUBLE_CHIRP) ? (MAC_PHY_ERR_DCHIRP) : 0)              \
)
#define ASSERT_BITS(_pDev) {                                                  \
    A_UINT32 value = readPlatformReg(_pDev, MAC_RX_FILTER);                   \
    ASSERT(MAC_BITS((_pDev)->rxFilterReg) == value);                          \
    value = readPlatformReg(_pDev, MAC_PHY_ERR);                              \
    ASSERT(PHY_BITS((_pDev)->rxFilterReg) == value);                          \
}
#define SET_BITS(_pDev, _bits) {                                              \
    writePlatformReg(_pDev, MAC_RX_FILTER, MAC_BITS(_bits));                  \
    writePlatformReg(_pDev, MAC_PHY_ERR, PHY_BITS(_bits));                    \
    if (PHY_BITS(_bits)) {                                                    \
        A_REG_SET_BIT(_pDev, MAC_RXCFG, ZLFDMA);                              \
    } else {                                                                  \
        A_REG_CLR_BIT(_pDev, MAC_RXCFG, ZLFDMA);                              \
    }                                                                         \
}

    switch (oper) {

    case HAL_RX_FILTER_INIT:

        /* Init hardware register and software copy */

        pDev->rxFilterReg = bits;
        SET_BITS(pDev, pDev->rxFilterReg);
        break;

    case HAL_RX_FILTER_SET:

        /* Set frame type bits */

        ASSERT_BITS(pDev);

        pDev->rxFilterReg |= bits;
        SET_BITS(pDev, pDev->rxFilterReg);
        break;

    case HAL_RX_FILTER_CLEAR:

        /* Clear frame type bits */

        ASSERT_BITS(pDev);

        pDev->rxFilterReg &= ~bits;
        SET_BITS(pDev, pDev->rxFilterReg);
        break;

    case HAL_RX_FILTER_TEMP:

        /* Temporarily set hardware register ONLY. */
        SET_BITS(pDev, bits);
        break;

    case HAL_RX_FILTER_RESTORE:

        /* Restore hardware register with software copy */
        SET_BITS(pDev, pDev->rxFilterReg);
        break;

    default:
        // Unsupported operation
        ASSERT(0);
    }
}

/**************************************************************
 * ar5212ProcessRxDesc
 *
 * Process an RX descriptor, and return the status to the caller.
 * Copy some hardware specific items into the software portion
 * of the descriptor.
 */
A_STATUS
ar5212ProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc)
{
    WLAN_STATS       *pLocalStats = &pDev->localSta->stats;
    AR5212_RX_STATUS *pNextStatus;
    AR5212_RX_STATUS *pRxStatus;
    SIB_ENTRY        *pSib;
    A_RSSI           rssi;

    A_DESC_CACHE_INVAL(pDesc);

    pRxStatus = RX_STATUS(pDesc);

    if (!pRxStatus->done) {
        return A_EBUSY;
    }

    /*
     * Given the use of a self-linked tail be very sure that the hw is
     * done with this descriptor; the hw may have done this descriptor
     * once and picked it up again...make sure the hw has moved on.
     */
    A_DESC_CACHE_INVAL(pDesc->pNextVirtPtr);
    pNextStatus = RX_STATUS(pDesc->pNextVirtPtr);
    if ((!pNextStatus->done) &&
        (readPlatformReg(pDev, MAC_RXDP) == pDesc->thisPhysPtr))
    {
        return A_EBUSY;
    }

    pDesc->status.rx.decryptError = FALSE;      /* assume frame is ok */

    /*
     * Drop multiple-descriptor frames.  This is not WECA-compliant if our
     * buffer size (sans wlan header size) is larger than 1514 bytes.
     */
    if (pRxStatus->more || pDev->partialRxDesc) {
        /* capture if the next one has to be discarded */
        pDev->partialRxDesc = pRxStatus->more ? TRUE : FALSE;
        return A_ERROR;
    }

    pDesc->status.rx.timestamp = (A_UINT16)pRxStatus->rxTimestamp;

    rssi = (A_RSSI) pRxStatus->rxSigStrength;

    /*
     * Fill in software versions of information that rest of RX processing
     * requires. Some are valid even for errored frames.
     */
    pDesc->status.rx.dataLength = (A_UINT16)pRxStatus->dataLength;
    pDesc->status.rx.hwIndex    = (A_UINT8)pRxStatus->keyIndex;
    /* Redirect TKIP rx to the tx keyentry */
    if (pDesc->status.rx.hwIndex >= 32 &&
        pDev->keyCache[pDesc->status.rx.hwIndex] == NULL) {
        pDesc->status.rx.hwIndex -= 32;
    }
    pDesc->status.rx.rssi       = rssi;
    pDesc->status.rx.antenna    = (A_UINT8)pRxStatus->rxAntenna;

    /*
     * Log receive packet info
     */
    PKTLOG_RX_PKT (pDev, pRxStatus->dataLength, pRxStatus->rxRate,
               rssi,
               *(A_UINT16*)&pDesc->pBufferVirtPtr.header->frameControl,
               *(A_UINT16*)&pDesc->pBufferVirtPtr.header->seqControl,
               PHY_ERROR_CODE(pRxStatus)  << 4 |
               pRxStatus->decryptCRCError << 3 |
               pRxStatus->CRCError        << 1 |
               pRxStatus->pktReceivedOK   << 0,
               pRxStatus->rxAntenna,
               (A_UINT16)pRxStatus->rxTimestamp);

    if (!pRxStatus->pktReceivedOK) {
        pLocalStats->ReceiveErrors++;

        /*
         * which of all these counters do we ever use? these
         * and other stats counters should be macroized
         */
        if (pRxStatus->CRCError) {
            pLocalStats->RcvCrcErrors++;
            return A_ERROR;
        }
        if (pRxStatus->phyErrorOccured) {
            pLocalStats->RcvPhyErrors++;
            pDesc->status.rx.phyError = (A_UINT8)PHY_ERROR_CODE(pRxStatus);
            return A_PHY_ERROR;
        }
    }

    /* looking at address2 only - don't pay the byte swap cost */
    pSib = sibEntryFind(pDev, &pDesc->pBufferVirtPtr.header->address2);

    /* demux the recd frame to the right vdev */
    if (isXrAp(pDev)) {
        pDesc->pVportBss = isXrBssRxFrame(pDev, pDesc->pBufferVirtPtr.header) ?
                           GET_XR_BSS(pDev) : GET_BASE_BSS(pDev);
    }
    else {
        pDesc->pVportBss = GET_BASE_BSS(pDev);
    }

    pDesc->status.rx.rate = (A_UINT8)pDesc->pVportBss->bss.pRateTable->
                                         rateCodeToIndex[pRxStatus->rxRate];

    if (pRxStatus->keyCacheMiss) {
        A_BOOL faulted = FALSE;

        pLocalStats->RcvKeyCacheMisses++;
        if ((pSib) && (pSib->pPriv)) {
            pSib->stats.RcvKeyCacheMisses++;
            (void)wlanKeyCacheFault(pDev, pSib, &faulted);
        }
        if (faulted == TRUE) {
            /* A real key cache miss.  This frame should be dropped. */
            return A_ERROR;
        }
    }

    if (!pRxStatus->pktReceivedOK) {
        if (pRxStatus->decryptCRCError) {
            ASSERT(!pRxStatus->michaelError);
            pDesc->status.rx.decryptError |= DECRYPTERROR_CRC; /* CRC Error */
        }
        if (pRxStatus->michaelError) {
            uiPrintf("halProcessRxDesc:  Michael error found\n");
            pDesc->status.rx.decryptError |= DECRYPTERROR_MIC; /* MIC Error */
        }
        if (pRxStatus->decompCRCError) {
            /*
             * According to Mr. Compression (aka jsk) we shouldn't
             * get any compression errors unless a valid key cache entry
             * was found
             */
            ASSERT(pRxStatus->keyCacheMiss == 0);
            pLocalStats->RcvDecompCrcErrors++;
            if (pSib) {
                pSib->stats.RcvDecompCrcErrors++;
            }
            return A_ERROR;
        }
        /* An actual decrypt error is returned as ok with the decryptError flag */
        pLocalStats->RcvDecryptCrcErrors++;
        if (pSib) {
            pSib->stats.RcvDecryptCrcErrors++;
        }
    }
    pDesc->status.rx.rate = (A_UINT8) pDesc->pVportBss->bss.pRateTable->rateCodeToIndex[pRxStatus->rxRate];

    /*
     * If using fast diversity; change default antenna if fast diversity
     * chooses other antenna 3 times in a row.
     */
    if (pDev->useFastDiversity) {
#ifdef BUILD_AP
        /* AP needs protection from beacon interrupts. */
        int lock = intLock();
#endif
        if(pDev->cachedDefAnt != pDesc->status.rx.antenna) {
            if (++pDev->countOtherRxAnt >= RX_FLIP_THRESHOLD) {
                ar5212SetDefAntenna(pDev, pDesc->status.rx.antenna);
            }
        } else {
            pDev->countOtherRxAnt = 0;
        }
#ifdef BUILD_AP
        /* AP needs protection from beacon interrupts. */
        intUnlock(lock);
#endif
    }   /* end if fast diversity */

    return A_OK;
}

/**************************************************************
 * ar5212SetupRxDesc
 *
 * Initialize RX descriptor, by clearing the status and clearing
 * the size.  This is not strictly HW dependent, but we want the
 * control and status words to be opaque above the hal.
 */
void
ar5212SetupRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size)
{
    AR5212_RX_CONTROL *pRxControl = RX_CONTROL(pDesc);

    pDesc->hw.word[0] = 0;
    pDesc->hw.word[1] = 0;
    pDesc->hw.word[2] = 0;
    pDesc->hw.word[3] = 0;
    pRxControl->bufferLength = size;
}
#if (defined(UPSD) && defined(BUILD_AP))
/*
 * Upsd Receive
 *  traverse completed receive descriptors.
 *  If we've received a uplink DATA frame and the pSib has anything
 *  in the downlink upsd queue, then move everything in the pSib holding queue
 *  to the hw upsd queue for transmission.  Receive descriptors and their payloads
 *  are untouched and travel up the normal Rx processing chain.
 *  This hw ISR routine is the only code that appends to the hw upsdq, thus
 *  the intLock() in queueSwTxDescriptor() in wlanSend.c is the only synchronization
 *  needed.  Also, the processTxDesc codes can safely reclaim Tx descriptors
 *  for the same reason.
 */
void
ar5212UpsdResponse(WLAN_DEV_INFO *pDev)
{
    VPORT_BSS *pVportBaseBss = GET_BASE_BSS(pDev);
    ATHEROS_DESC *prx, *prxnext;
    QUEUE_DETAILS *sibq, *upsdq;
    AR5212_RX_STATUS *pRxStatus, *pNextStatus;
    SIB_ENTRY *pSib;
    unsigned char ftype;
    unsigned char fsubtype;

    /* The hw upsd downlink queue */
    upsdq = &pVportBaseBss->bss.wmequeues[TXQ_ID_FOR_UPSD];
    pSib = NULL;

    /* For each FRAME_DATA receive desc */
    for(prx=pDev->rxQueue.pDescQueueHead; prx; prx=prx->pNextVirtPtr) {

        /* exit if descriptor is not done */
        A_DESC_CACHE_INVAL(prx);
        pRxStatus = RX_STATUS(prx);
        if (!pRxStatus->done) {
            return;
        }

        /* exit if current descriptor is self-linked */
        prxnext = prx->pNextVirtPtr;
        if (prxnext == prx) {
            return;
        }
        /* exit if next descriptor is not done and hw RXDP is still on this desc */
        A_DESC_CACHE_INVAL(prx->pNextVirtPtr);
        pNextStatus = RX_STATUS(prxnext);
        if ((!pNextStatus->done) && 
            (readPlatformReg(pDev, MAC_RXDP) == prx->thisPhysPtr)) {
            return;
        }

        /* Skip if status is not OK */
        if (!pRxStatus->pktReceivedOK) {
            continue;
        }

        ftype = prx->pBufferVirtPtr.header->frameControl.fType;
        fsubtype = prx->pBufferVirtPtr.header->frameControl.fSubtype;

        pSib = sibEntryFindLocked(pDev, &prx->pBufferVirtPtr.header->address2);

        /* If Sib has a non-empty upsdq, move it to hw TXQ_ID_FOR_UPSD and transmit */
        if (pSib==NULL) {
            continue;
        }
        sibq = &pSib->upsdQueue;
#ifdef DEBUGUPSDISR
        if (A_SEM_VALID(sibq->qSem))
            isrPrintf("%x.%x:%d\n", ftype, fsubtype, sibq->qFrameCount);
#endif
        if (A_SEM_VALID(sibq->qSem) && sibq->qFrameCount && ftype==FRAME_DATA) {
            ATHEROS_DESC *pHead, *pTail;

            pHead = sibq->pDescQueueHead;
            pTail = sibq->pDescQueueTail;

            sibq->pDescQueueHead = sibq->pDescQueueTail = NULL;

            if (upsdq->pDescQueueTail == NULL) {
                upsdq->pDescQueueHead = pHead;
                ar5212SetTxDP(pDev, TXQ_ID_FOR_UPSD, pHead->thisPhysPtr);
            } else {
                upsdq->pDescQueueTail->pNextVirtPtr = pHead;
                upsdq->pDescQueueTail->nextPhysPtr = pHead->thisPhysPtr;
                A_DESC_CACHE_FLUSH(upsdq->pDescQueueTail);
            }
            upsdq->pDescQueueTail = pTail;
            A_PIPEFLUSH();

            writePlatformReg(pDev, MAC_Q_TXE, (1 << TXQ_ID_FOR_UPSD));

            upsdq->qFrameCount += sibq->qFrameCount;
            pSib->numTxPending += sibq->qFrameCount;
            sibq->qFrameCount = 0;
        }

    }
}
#endif

#endif // #ifdef BUILD_AR5212
