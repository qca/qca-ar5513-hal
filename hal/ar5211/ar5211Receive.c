/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Transmit HAL functions
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Receive.c#2 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "vport.h"
#include "wlanPhy.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Transmit.h"
#include "ar5211.h"
#include "pktlog.h"

/**************************************************************
 * ar5211GetRxDP
 *
 * Get the RXDP.
 */
A_UINT32
ar5211GetRxDP(WLAN_DEV_INFO *pDev)
{
    return readPlatformReg(pDev, MAC_RXDP);
}

/**************************************************************
 * ar5211SetRxDP
 *
 * Set the RxDP.
 */
void
ar5211SetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp)
{
    ASSERT(!A_REG_IS_BIT_SET(pDev, MAC_CR, RXE));
    writePlatformReg(pDev, MAC_RXDP, rxdp);
    ASSERT(readPlatformReg(pDev, MAC_RXDP) == rxdp);
}


/**************************************************************
 * ar5211EnableReceive
 *
 * Set Receive Enable bits.
 */
void
ar5211EnableReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_CR, MAC_CR_RXE);
}

/**************************************************************
 * ar5211StopDmaReceive
 *
 * Stop Receive at the DMA engine
 */
A_STATUS
ar5211StopDmaReceive(WLAN_DEV_INFO *pDev)
{
    A_UINT32    regVal, i;

    /* Set receive disable bit */
    writePlatformReg(pDev, MAC_CR, MAC_CR_RXD);

    /* Wait up to 10ms */
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
 * ar5211StartPcuReceive
 *
 * Start Transmit at the PCU engine (unpause receive)
 */
void
ar5211StartPcuReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_DIAG_SW,
        readPlatformReg(pDev, MAC_DIAG_SW) & ~(MAC_DIAG_RX_DIS));
}

/**************************************************************
 * ar5211StopPcuReceive
 *
 * Stop Transmit at the PCU engine (pause receive)
 */
void
ar5211StopPcuReceive(WLAN_DEV_INFO *pDev)
{
    writePlatformReg(pDev, MAC_DIAG_SW, readPlatformReg(pDev, MAC_DIAG_SW) | MAC_DIAG_RX_DIS);
}

/**************************************************************
 * ar5211SetMulticastFilter
 *
 * Set multicast filter 0 (lower 32-bits)
 *               filter 1 (upper 32-bits)
 */
void
ar5211SetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1)
{
    writePlatformReg(pDev, MAC_MCAST_FIL0, filter0);
    writePlatformReg(pDev, MAC_MCAST_FIL1, filter1);
}

/**************************************************************
 * ar5211MulticastFilterIndex
 *
 * Clear/Set multicast filter by index
 */
void
ar5211MulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet)
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
 * ar5211RxFilter
 *
 * Turn off/on bits in the receive packet filter.  Save it for resets, etc.
 *
 * Bits can be MAC_RX_UCAST, MAC_RX_MCAST, MAC_RX_BCAST, MAC_RX_BEACON,
 * MAC_RX_PROM, MAC_RX_CONTROL
 *
 * oper can be HAL_RX_FILTER_INIT, HAL_RX_FILTER_SET, HAL_RX_FILTER_CLEAR,
 * HAL_RX_FILTER_TEMP, HAL_RX_FILTER_RESTORE.
 *
 * NOTE: the bits parameter is not used for operation type HAL_RX_FILTER_RESTORE.
 *
 */
void
ar5211RxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper)
{
    A_UINT32 value;

    /*
     * reduce bits to only those that are supported here; there is
     * a 1:1 mapping for phy error and phy radar bits from the hal
     * definitions to the Oahu MAC definitions
     */
    bits &= (HAL_RX_COMMON | HAL_RX_PHY_ERR | HAL_RX_PHY_RADAR);

    /* don't support phy error reporting for adaptive noise immunity in Oahu */
    bits &= ~HAL_RX_PHY_ERR;

    switch (oper) {

    case HAL_RX_FILTER_INIT:

        /* Init hardware register and software copy */

        pDev->rxFilterReg = bits;
        writePlatformReg(pDev, MAC_RX_FILTER, pDev->rxFilterReg);
        break;


    case HAL_RX_FILTER_SET:

        /* Set frame type bits */

        value = readPlatformReg(pDev, MAC_RX_FILTER);
        ASSERT(pDev->rxFilterReg == value);

        /*
         * Phy errors are dma'ed up only if zero length
         * dma is enabled
         */
        if (bits & (MAC_RX_PHY_RADAR | MAC_RX_PHY_ERR)) {
            A_REG_SET_BIT(pDev, MAC_RXCFG, ZLFDMA);
        }

        pDev->rxFilterReg |= bits;
        writePlatformReg(pDev, MAC_RX_FILTER, pDev->rxFilterReg);
        break;

    case HAL_RX_FILTER_CLEAR:

        /* Clear frame type bits */

        value = readPlatformReg(pDev, MAC_RX_FILTER);
        ASSERT(pDev->rxFilterReg == value);

        pDev->rxFilterReg &= ~bits;
        writePlatformReg(pDev, MAC_RX_FILTER, pDev->rxFilterReg);
        break;

    case HAL_RX_FILTER_TEMP:

        /* Temporarily set hardware register ONLY. */
        writePlatformReg(pDev, MAC_RX_FILTER, bits);
        break;

    case HAL_RX_FILTER_RESTORE:

        /* Restore hardware register with software copy */
        writePlatformReg(pDev, MAC_RX_FILTER, pDev->rxFilterReg);
        break;

    default:
        // Unsupported operation
        ASSERT(0);
    }
}

/**************************************************************
 * ar5211ProcessRxDesc
 *
 * Process an RX descriptor, and return the status to the caller.
 * Copy some hardware specific items into the software portion
 * of the descriptor.
 */
A_STATUS
ar5211ProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc)
{
    WLAN_STATS       *pLocalStats = &pDev->localSta->stats;
    AR5211_RX_STATUS *pNextStatus;
    AR5211_RX_STATUS *pRxStatus;
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
     * Log receive packet info
     */
     PKTLOG_RX_PKT (pDev, pRxStatus->dataLength, pRxStatus->rxRate,
            rssi,
            *(A_UINT16*)&pDesc->pBufferVirtPtr.header->frameControl,
            *(A_UINT16*)&pDesc->pBufferVirtPtr.header->seqControl,
            pRxStatus->phyError        << 4 |
            pRxStatus->decryptCRCError << 3 |
            pRxStatus->fifoOverrun     << 2 |
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
        if (pRxStatus->fifoOverrun) {
            pLocalStats->RcvDmaOverrunErrors++;
            return A_ERROR;
        }
        if (pRxStatus->phyError) {
            pLocalStats->RcvPhyErrors++;
            pDesc->status.rx.phyError = (A_UINT8)pRxStatus->phyError;
            return A_PHY_ERROR;
        }
        if (pRxStatus->CRCError) {
            pLocalStats->RcvCrcErrors++;
            return A_ERROR;
        }
    }

    /* Oahu just supports single (base) BSS */
    pDesc->pVportBss = GET_BASE_BSS(pDev);

    /* looking at address2 only - don't pay the byte swap cost */
    pSib = sibEntryFind(pDev, &pDesc->pBufferVirtPtr.header->address2);

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

    if (pRxStatus->decryptCRCError) {
        ASSERT(!pRxStatus->pktReceivedOK);

        /* An actual decrypt error is returned as ok with the decryptError flag */
        pLocalStats->RcvDecryptCrcErrors++;
        if (pSib) {
            pSib->stats.RcvDecryptCrcErrors++;
        }
        /*
         * Decrypt errors still get sent to the mac layer
         * to deal with appropriately.
         */
        pDesc->status.rx.decryptError = DECRYPTERROR_CRC;   /* frame is corrupt */
    }

    /*
     * Fill in software versions of information that rest of RX processing
     * requires for good frames.
     */
    pDesc->status.rx.dataLength = (A_UINT16)pRxStatus->dataLength;
    pDesc->status.rx.hwIndex    = (A_UINT8)pRxStatus->keyIndex;
    pDesc->status.rx.rssi       = rssi;
    pDesc->status.rx.rate       = (A_UINT8)pDesc->pVportBss->bss.pRateTable->rateCodeToIndex[pRxStatus->rxRate];
    pDesc->status.rx.antenna    = (A_UINT8)pRxStatus->rxAntenna;

    return A_OK;
}

/**************************************************************
 * ar5211SetupRxDesc
 *
 * Initialize RX descriptor, by clearing the status and clearing
 * the size.  This is not strictly HW dependent, but we want the
 * control and status words to be opaque above the hal.
 */
void
ar5211SetupRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size)
{
    AR5211_RX_CONTROL *pRxControl = RX_CONTROL(pDesc);

    pDesc->hw.word[0] = 0;
    pDesc->hw.word[1] = 0;
    pDesc->hw.word[2] = 0;
    pDesc->hw.word[3] = 0;
    pRxControl->bufferLength = size;
}

#endif // #ifdef BUILD_AR5211
