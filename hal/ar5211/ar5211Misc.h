/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5211Misc.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Misc.h#2 $
 */

#ifndef _AR5211_MISC_H_
#define _AR5211_MISC_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5211EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data);

A_STATUS
ar5211SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain);

A_STATUS
ar5211GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac);

void
ar5211SetLedState(WLAN_DEV_INFO	*pDev, A_BOOL bConnected);

void
ar5211EnableRfKill(WLAN_DEV_INFO *pDev);

void
ar5211GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5211GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

A_UINT32
ar5211GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5211GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val);

void
ar5211GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel);

void
ar5211WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId,
                   A_UINT16 timOffset);

void
ar5211SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers);

void
ar5211GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf);

void
ar5211ResetTsf(WLAN_DEV_INFO *pDev);

void
ar5211SetAdhocMode(WLAN_DEV_INFO *pDev);

A_BOOL
ar5211GetRfKill(WLAN_DEV_INFO *pDev);

void
ar5211SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *);

A_UINT32
ar5211GetRandomSeed(WLAN_DEV_INFO *pDev);

A_BOOL
ar5211DetectCardPresent(WLAN_DEV_INFO *pDev);

A_UINT32
ar5211MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext);

void
ar5211GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                     HAL_CHANNEL_DATA *pData);

void
ar5211ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList);

void
ar5211EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams);

void
ar5211DumpRegisters(WLAN_DEV_INFO *pDev);

A_RSSI32
ar5211GetCurRssi(WLAN_DEV_INFO *pDev);

A_UINT32
ar5211GetDefAntenna(WLAN_DEV_INFO *pDev);

void
ar5211SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna);

void
ar5211SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                       CHAN_VALUES *pChval);

void
ar5211UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt);

A_BOOL
ar5211UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev);

void
ar5211DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose);

A_UINT32
ar5211GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord);

void
ar5211GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
ar5211SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_MISC_H_ */
