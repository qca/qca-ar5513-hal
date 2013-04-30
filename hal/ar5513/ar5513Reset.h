/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513Reset.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Reset.h#3 $
 */

#ifndef _AR5513_RESET_H_
#define _AR5513_RESET_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5513ChipReset(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

A_STATUS
ar5513Reset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange);

A_STATUS
ar5513PerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

A_STATUS
ar5513SleepExit(WLAN_DEV_INFO *pDev);

A_STATUS
ar5513Disable(WLAN_DEV_INFO *pDev);

A_STATUS
ar5513PhyDisable(WLAN_DEV_INFO *pDev);

RFGAIN_STATES
ar5513GetRfgain(WLAN_DEV_INFO *pDevInfo);

void
ar5513InitializeGainValues(WLAN_DEV_INFO *pDev, struct gainValues *pGainValues);

A_BOOL
ar5513AllocateRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo);

void
ar5513FreeRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo);

void
ar5513SetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit);

A_STATUS
ar5513MacStop(WLAN_DEV_INFO *pDev);

void
ar5513UpdateSynth2_4War(WLAN_DEV_INFO *pDev);

#if defined(DEADCODE)
extern const struct RfHalFuncs ar5111Funcs;
#endif /* DEADCODE */

extern const struct RfHalFuncs ar5112Funcs;

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_RESET_H_ */
