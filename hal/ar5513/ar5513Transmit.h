/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513Transmit.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Transmit.h#5 $
 */

#ifndef _AR5513_Transmit_H_
#define _AR5513_Transmit_H_

#ifdef _cplusplus
extern "C" {
#endif

A_BOOL
ar5513UpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel);

int
ar5513SetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo);

void
ar5513ReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5513GetTxDP(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5513SetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp);

void
ar5513StartTxDma(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
ar5513NumTxPending(WLAN_DEV_INFO *pDev, int queueNum);

void
ar5513StopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec);

int
ar5513GetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index);

void
ar5513SetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index, int value);

int
ar5513GetTxFilterArraySize(WLAN_DEV_INFO *pDev);

void
ar5513GetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT count);

void
ar5513SetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT count);

void
ar5513PauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause);

/* Descriptor Access Functions */

void
ar5513SetupTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_UINT32 hwIndex);

A_STATUS
ar5513ProcessTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

A_BOOL
ar5513GetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL swap);

#if defined(DEBUG) || defined(_DEBUG)
void
ar5513DebugPrintTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose);

void
ar5513MonitorRxTxActivity(WLAN_DEV_INFO *pDev, int index, int display);

#endif

#ifdef _cplusplus
}
#endif

#endif /* _ar5513_Transmit_H_ */
