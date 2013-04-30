/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513Power.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Power.h#2 $
 */

#ifndef _AR5513_POWER_H_
#define _AR5513_POWER_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5513SetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip);

A_UINT32
ar5513GetPowerMode(WLAN_DEV_INFO *pDev);

A_BOOL
ar5513GetPowerStatus(WLAN_DEV_INFO *pDev);

void
ar5513SetupPsPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_POWER_H_ */
