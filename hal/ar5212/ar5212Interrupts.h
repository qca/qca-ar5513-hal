/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Interrupts.h#1 $
 */

#ifndef _AR5212_INTERRUPTS_H_
#define _AR5212_INTERRUPTS_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO */

#ifdef _cplusplus
extern "C" {
#endif


A_BOOL
ar5212IsInterruptPending(WLAN_DEV_INFO *pDev);

HAL_INT_TYPE
ar5212GetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pDescQueueBitMask);

void
ar5212EnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

void
ar5212DisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_INTERRUPTS_H_ */
