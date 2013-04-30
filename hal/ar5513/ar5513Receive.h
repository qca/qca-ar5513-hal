/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513Receive.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Receive.h#2 $
 */

#ifndef _AR5513_Receive_H_
#define _AR5513_Receive_H_

#ifdef _cplusplus
extern "C" {
#endif

A_UINT32
ar5513GetRxDP(WLAN_DEV_INFO *pDev);

void
ar5513SetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp);

void
ar5513EnableReceive(WLAN_DEV_INFO *pDev);

A_STATUS
ar5513StopDmaReceive(WLAN_DEV_INFO *pDev);

void
ar5513StartPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5513StopPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5513SetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1);

void
ar5513MulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet);

void
ar5513RxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper);

A_STATUS
ar5513ProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

void
ar5513SetupRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size);

#ifdef UPSD
void
ar5513UpsdResponse(WLAN_DEV_INFO *pDev);
void
ar5513PProcessRxDesc(WLAN_DEV_INFO);
#endif

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_Receive_H_ */
