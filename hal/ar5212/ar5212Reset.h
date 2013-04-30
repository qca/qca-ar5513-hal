/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212Reset.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Reset.h#2 $
 */

#ifndef _AR5212_RESET_H_
#define _AR5212_RESET_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5212ChipReset(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

A_STATUS
ar5212Reset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange);

A_STATUS
ar5212PerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

A_STATUS
ar5212SleepExit(WLAN_DEV_INFO *pDev);

A_STATUS
ar5212Disable(WLAN_DEV_INFO *pDev);

A_STATUS
ar5212PhyDisable(WLAN_DEV_INFO *pDev);

RFGAIN_STATES
ar5212GetRfgain(WLAN_DEV_INFO *pDevInfo);

void
ar5212InitializeGainValues(WLAN_DEV_INFO *pDev, struct gainValues *pGainValues);

A_BOOL
ar5212AllocateRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo);

void
ar5212FreeRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo);

void
ar5212SetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit);

A_STATUS
ar5212MacStop(WLAN_DEV_INFO *pDev);

extern const struct RfHalFuncs ar5111Funcs;
extern const struct RfHalFuncs ar5112Funcs;
extern const struct RfHalFuncs ar2413Funcs;

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_RESET_H_ */
