/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halTransmit.c#4 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "display.h"

/**************************************************************
 * halUpdateTxTrigLevel
 *
 * Update Tx FIFO trigger level.
 *
 * Set bIncTrigLevel to TRUE to increase the trigger level.
 * Set bIncTrigLevel to FALSE to decrease the trigger level.
 *
 * Returns TRUE if the trigger level was updated
 */
A_BOOL
halUpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwUpdateTxTrigLevel(pDev, bIncTrigLevel);

}

/**************************************************************
 * halSetupTxQueue
 *
 * Allocates and initializes a DCU/QCU combination for tx
 *
 */
int
halSetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwSetupTxQueue(pDev, queueInfo);
}

/**************************************************************
 * halReleaseTxQueue
 *
 * Frees up a DCU/QCU combination
 *
 */
void
halReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwReleaseTxQueue(pDev, queueNum);
}

/**************************************************************
 * halGetTxDP
 *
 * Get the TXDP for the specified queue
 */
A_UINT32
halGetTxDP(WLAN_DEV_INFO *pDev, int queueNum)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwGetTxDP(pDev, queueNum);
}

/**************************************************************
 * halSetTxDP
 *
 * Set the TxDP for the specified transmit queue
 */
void
halSetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetTxDP(pDev, queueNum, txdp);
}

/**************************************************************
 * halStartTxDma
 *
 * Set Transmit Enable bits for the specified queue
 */
void
halStartTxDma(WLAN_DEV_INFO *pDev, int queueNum)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwStartTxDma(pDev, queueNum);
}


/**************************************************************
 * halIsTxQueueStopped
 *
 * See if the given queue is active or not
 */
A_UINT32
halNumTxPending(WLAN_DEV_INFO *pDev, int queueNum)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwNumTxPending(pDev, queueNum);
}

/**************************************************************
 * halStopTxDma
 *
 * Stop the specified queue
 */
void
halStopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwStopTxDma(pDev, queueNum, msec);
}

/**************************************************************
 * halPauseTx
 *
 * Pause/unpause transmission on the specified queue
 */
void
halPauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwPauseTx(pDev, queueMask, pause);
}

/**************************************************************
 * halGetTxFilter
 *
 * Get tx filter for given queue/index
 */
int
halGetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwGetTxFilter(pDev, queueNum, index);
}

/**************************************************************
 * halSetTxFilter
 *
 * Set tx filter (128 bit) for given queue
 */
void
halSetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index, int value)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetTxFilter(pDev, queueNum, index, value);
}

/**************************************************************
 * halGetTxFilterArraySize
 *
 * Return hardware tx filter array size (number of 32 bit entries)  
 */
int
halGetTxFilterArraySize(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwGetTxFilterArraySize(pDev);
}


/**************************************************************
 * halGetTxFilterArray
 *
 * Read MMRs into given array  
 */
void
halGetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwGetTxFilterArray(pDev, pMmrArray, count);
}

/**************************************************************
 * halSetTxFilterArray
 *
 * Write array of MMRs into tx filter registers
 */
void
halSetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetTxFilterArray(pDev, pMmrArray, count);
}



/**************************************************************
 * halSetupTxDesc
 *
 * Sets the control fields of the transmit descriptor
 * Has more intelligence than it needs - i.e. some of it
 * may be better belong to the wlan layer in the future
 *
 */
void
halSetupTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_UINT32 hwIndex)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetupTxDesc(pDev, pTxDesc, hwIndex);
}

/**************************************************************
 * halProcessTxDescStatus
 *
 * Process TX descriptor status.  Handles completed chain
 * termination, swretry look ahead decision and wlan stats
 * collection.
 *
 */
A_STATUS
halProcessTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwProcessTxDesc(pDev, pTxDesc);
}

A_BOOL
halGetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_BOOL swap)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwGetTxDescDone(pDev, pTxDesc, swap);
}




/**************************************************************
 * DEBUG related routines - defined for DEBUG only
 *
 */
#if defined(DEBUG) || defined(_DEBUG)

const char *halFrameTypeToName[] = {
    "ASSOC_REQ",    "ASSOC_RESP",   "REASSOC_REQ",  "REASSOC_RESP",
    "PROBE_REQ",    "PROBE_RESP",   "MGT:???",      "MGT:???",
    "BEACON",       "ATIM",         "DISASSOC",     "AUTH",
    "DEAUTH",       "MGT:???",      "MGT:???",      "MGT:???",

    "CTRL:???",     "CTRL:???",     "CTRL:???",     "CTRL:???",
    "CTRL:???",     "CTRL:???",     "CTRL:???",     "CTRL:???",
    "CTRL:???",     "CTRL:???",     "PSPOLL",       "RTS",
    "CTS",          "ACK",          "CFEND",        "CFENDACK",

    "DATA",         "DATA_CFACK",   "DATA_CFPOLL",  "DATA_CFACK_CFPOLL",
    "NODATA",       "NODATA_CFACK", "NODATA_CFPOLL","NODATA_CFACK_CFPOLL",
    "DATA:???",     "DATA:???",     "DATA:???",     "DATA:???",
    "DATA:???",     "DATA:???",     "DATA:???",     "DATA:???",

    "???", "???", "???", "???",
    "???", "???", "???", "???",
    "???", "???", "???", "???",
    "???", "???", "???", "???"
};

void
halDebugPrintTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose)
{
    ASSERT(pDev);
    ASSERT(pDesc);

    /* Display all 6 words of the descriptor */
    if (verbose) {
        uiPrintf(DESC_ADDR        "%08x  ", pDesc->thisPhysPtr);
        uiPrintf(NEXT_DESC        "%08x  ", pDesc->nextPhysPtr);
        uiPrintf(BUFFER_PTR       "%08x\n", pDesc->bufferPhysPtr);
        uiPrintf(ORIG_BUFVIRT_PTR "%p\n",   pDesc->pOrigBufferVirtPtr);
    }

    /* Display the hardware specific stuff */
    pDev->pHwFunc->hwDebugPrintTxDesc(pDev, pDesc, verbose);
}

void
halMonitorRxTxActivity(WLAN_DEV_INFO *pDev, int index, int display)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    if (pDev->pHwFunc->hwMonitorRxTxActivity) {
        pDev->pHwFunc->hwMonitorRxTxActivity(pDev, index, display);
    }
}

#endif
#ifdef DEBUGWME
void
halDumpIFS(WLAN_DEV_INFO *pDev)
{
	ar52xxDumpIFS(pDev);
}
#endif
