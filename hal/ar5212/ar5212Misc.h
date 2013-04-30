/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212Misc.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Misc.h#3 $
 */

#ifndef _AR5212_MISC_H_
#define _AR5212_MISC_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5212EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data);

A_STATUS
ar5212SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain);

A_STATUS
ar5212GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac);

void
ar5212SetLedState(WLAN_DEV_INFO	*pDev, A_BOOL bConnected);

void
ar5212EnableRfKill(WLAN_DEV_INFO *pDev);

void
ar5212GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5212GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

A_UINT32
ar5212GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5212GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val);

void
ar5212GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel);

void
ar5212WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset);

void
ar5212SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers);

void
ar5212GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf);

void
ar5212ResetTsf(WLAN_DEV_INFO *pDev);

void
ar5212SetAdhocMode(WLAN_DEV_INFO *pDev);

A_BOOL
ar5212GetRfKill(WLAN_DEV_INFO *pDev);

void
ar5212SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *);

A_UINT32
ar5212GetRandomSeed(WLAN_DEV_INFO *pDev);

A_BOOL
ar5212DetectCardPresent(WLAN_DEV_INFO *pDev);

A_UINT32
ar5212MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext);

void
ar5212GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                     HAL_CHANNEL_DATA *pData);

void
ar5212ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList);

void
ar5212EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams);

void
ar5212EnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr);

void
ar5212DisableFriendlyDetection(WLAN_DEV_INFO *pDev);

void
ar5212AniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param);

A_INT32
ar5212AniGetListenTime(WLAN_DEV_INFO *pDev);

void
ar5212DumpRegisters(WLAN_DEV_INFO *pDev);

A_RSSI32
ar5212GetCurRssi(WLAN_DEV_INFO *pDev);

A_UINT32
ar5212GetDefAntenna(WLAN_DEV_INFO *pDev);

void
ar5212SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna);

void
ar5212SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                       CHAN_VALUES *pChval);

void
ar5212UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt);

A_BOOL
ar5212UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev);

void
ar5212DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose);

void
ar5212SetSleepRegisters(WLAN_DEV_INFO *pDev, A_BOOL enSlowClockSleep);

void
ar5212SwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete);

A_UINT32
ar5212GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord);

void
ar5212GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
ar5212SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
ar5212SendXrChirp(WLAN_DEV_INFO *pDev);

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_MISC_H_ */
