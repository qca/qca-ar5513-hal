/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Interrupts.h#2 $
 */

#ifndef _AR5513_INTERRUPTS_H_
#define _AR5513_INTERRUPTS_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO */

#ifdef _cplusplus
extern "C" {
#endif


A_BOOL
ar5513IsInterruptPending(WLAN_DEV_INFO *pDev);

HAL_INT_TYPE
ar5513GetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pDescQueueBitMask);

void
ar5513EnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

void
ar5513DisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_INTERRUPTS_H_ */
