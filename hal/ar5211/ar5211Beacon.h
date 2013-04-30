/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Beacon.h#2 $
 */

#ifndef _AR5211_BEACON_H_
#define _AR5211_BEACON_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO */

#ifdef _cplusplus
extern "C" {
#endif

void
ar5211SetupBeaconDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp);

void
ar5211BeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp);

A_UINT32
ar5211GetMacTimer3(WLAN_DEV_INFO *);
    
#ifdef _cplusplus
}
#endif

#endif /* _AR5211_BEACON_H_ */
