/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212Power.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Power.h#1 $
 */

#ifndef _AR5212_POWER_H_
#define _AR5212_POWER_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5212SetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip);

A_UINT32
ar5212GetPowerMode(WLAN_DEV_INFO *pDev);

A_BOOL
ar5212GetPowerStatus(WLAN_DEV_INFO *pDev);

void
ar5212SetupPsPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_POWER_H_ */
