/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Interrupts.h#1 $
 */

#ifndef _AR5211_INTERRUPTS_H_
#define _AR5211_INTERRUPTS_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO */

#ifdef _cplusplus
extern "C" {
#endif


A_BOOL
ar5211IsInterruptPending(WLAN_DEV_INFO *pDev);

HAL_INT_TYPE
ar5211GetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pQueueBitMask);

void
ar5211EnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

void
ar5211DisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

A_UINT32
ar5211GetInterruptQueue(WLAN_DEV_INFO *pDev,HAL_INT_TYPE intType);



#ifdef _cplusplus
}
#endif

#endif /* _AR5211_INTERRUPTS_H_ */
