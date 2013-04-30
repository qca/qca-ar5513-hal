/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5211Reset.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Reset.h#1 $
 */

#ifndef _AR5211_RESET_H_
#define _AR5211_RESET_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5211ChipReset(WLAN_DEV_INFO *pDev, A_UINT16 channelFlags);

A_STATUS
ar5211Reset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange);

A_STATUS
ar5211PerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

A_STATUS
ar5211SleepExit(WLAN_DEV_INFO *pDev);

A_STATUS
ar5211Disable(WLAN_DEV_INFO *pDev);

A_STATUS
ar5211PhyDisable(WLAN_DEV_INFO *pDev);

RFGAIN_STATES
ar5211GetRfgain(WLAN_DEV_INFO *pDevInfo);

void
ar5211InitializeGainValues(struct gainValues *pGainValues);

void
ar5211SetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit);

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_RESET_H_ */
