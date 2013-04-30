/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212Transmit.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Transmit.h#2 $
 */

#ifndef _AR5212_Transmit_H_
#define _AR5212_Transmit_H_

#ifdef _cplusplus
extern "C" {
#endif

A_BOOL
ar5212UpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel);

int
ar5212SetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo);

void
ar5212ReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5212GetTxDP(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5212SetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp);

void
ar5212StartTxDma(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5212NumTxPending(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5212PauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause);

void
ar5212StopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec);


/* Descriptor Access Functions */

void
ar5212SetupTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_UINT32 hwIndex);

A_STATUS
ar5212ProcessTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

A_BOOL
ar5212GetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL swap);

#if defined(DEBUG) || defined(_DEBUG)
void
ar5212DebugPrintTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose);
#endif

#ifdef _cplusplus
}
#endif

#endif /* _ar5212_Transmit_H_ */
