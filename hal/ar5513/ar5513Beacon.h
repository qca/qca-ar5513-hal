/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Beacon.h#3 $
 */

#ifndef _AR5513_BEACON_H_
#define _AR5513_BEACON_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO */

#ifdef _cplusplus
extern "C" {
#endif

void
ar5513SetupBeaconDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp);

void
ar5513BeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp);

void
ar5513SetupGrpPollChain(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_UINT32 hwIndex);

A_UINT32
ar5513GetMacTimer3(WLAN_DEV_INFO *);
    
#ifdef _cplusplus
}
#endif

#endif /* _AR5513_BEACON_H_ */
