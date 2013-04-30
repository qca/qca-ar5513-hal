/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5211Receive.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Receive.h#1 $
 */

#ifndef _AR5211_Receive_H_
#define _AR5211_Receive_H_

#ifdef _cplusplus
extern "C" {
#endif

A_UINT32
ar5211GetRxDP(WLAN_DEV_INFO *pDev);

void
ar5211SetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp);

void
ar5211EnableReceive(WLAN_DEV_INFO *pDev);

A_STATUS
ar5211StopDmaReceive(WLAN_DEV_INFO *pDev);

void
ar5211StartPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5211StopPcuReceive(WLAN_DEV_INFO *pDev);

void
ar5211SetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1);

void
ar5211MulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet);

void
ar5211RxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper);

A_STATUS
ar5211ProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

void
ar5211SetupRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size);

void
ar5211InitRxDesc(ATHEROS_DESC *pDesc, A_UINT32 size);

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_Receive_H_ */
