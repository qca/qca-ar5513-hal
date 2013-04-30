/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5211Transmit.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Transmit.h#2 $
 */

#ifndef _AR5211_Transmit_H_
#define _AR5211_Transmit_H_

#ifdef _cplusplus
extern "C" {
#endif

A_BOOL
ar5211UpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel);

int
ar5211SetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo);

void
ar5211ReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5211GetTxDP(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5211SetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp);

void
ar5211StartTxDma(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5211NumTxPending(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5211StopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec);

void
ar5211PauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause);


/* Descriptor Access Functions */

void
ar5211SetupTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_UINT32 hwIndex);

A_STATUS
ar5211ProcessTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

A_BOOL
ar5211GetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL swap);

#if defined(DEBUG) || defined(_DEBUG)
void
ar5211DebugPrintTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose);
#endif

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_Transmit_H_ */
