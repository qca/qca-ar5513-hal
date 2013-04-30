/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212Receive.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Receive.h#1 $
 */

#ifndef _AR5212_Receive_H_
#define _AR5212_Receive_H_

#ifdef _cplusplus
extern "C" {
#endif

A_UINT32
ar5212GetRxDP(WLAN_DEV_INFO *pDev);

void
ar5212SetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp);

void
ar5212EnableReceive(WLAN_DEV_INFO *pDev);

A_STATUS
ar5212StopDmaReceive(WLAN_DEV_INFO *pDev);

void
ar5212StartPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5212StopPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5212SetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1);

void
ar5212MulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet);

void
ar5212RxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper);

A_STATUS
ar5212ProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

void
ar5212SetupRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size);

#ifdef UPSD
void
ar5212UpsdResponse(WLAN_DEV_INFO *pDev);
void
ar5212PProcessRxDesc(WLAN_DEV_INFO);
#endif

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_Receive_H_ */
