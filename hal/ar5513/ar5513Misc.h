/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513Misc.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Misc.h#4 $
 */

#ifndef _AR5513_MISC_H_
#define _AR5513_MISC_H_

#ifdef _cplusplus
extern "C" {
#endif

A_STATUS
ar5513InitFrmTypeCapTable(WLAN_DEV_INFO *pDev);

A_STATUS
ar5513EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data);

A_STATUS
ar5513EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data);

A_STATUS
ar5513SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain);

A_STATUS
ar5513GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac);

void
ar5513SetLedState(WLAN_DEV_INFO	*pDev, A_BOOL bConnected);

void
ar5513EnableRfKill(WLAN_DEV_INFO *pDev);

void
ar5513GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5513GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

A_UINT32
ar5513GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
ar5513GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val);

void
ar5513GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel);

void
ar5513WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset);

void
ar5513SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers);

void
ar5513GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf);

void
ar5513ResetTsf(WLAN_DEV_INFO *pDev);

void
ar5513SetAdhocMode(WLAN_DEV_INFO *pDev);

A_BOOL
ar5513GetRfKill(WLAN_DEV_INFO *pDev);

void
ar5513SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *);

A_UINT32
ar5513GetRandomSeed(WLAN_DEV_INFO *pDev);

A_BOOL
ar5513DetectCardPresent(WLAN_DEV_INFO *pDev);

A_UINT32
ar5513MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext);

void
ar5513GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                     HAL_CHANNEL_DATA *pData);

void
ar5513ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList);

void
ar5513EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams);

void
ar5513EnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr);

void
ar5513DisableFriendlyDetection(WLAN_DEV_INFO *pDev);

void
ar5513AniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param);

A_INT32
ar5513AniGetListenTime(WLAN_DEV_INFO *pDev);

void
ar5513DumpRegisters(WLAN_DEV_INFO *pDev);

A_RSSI32
ar5513GetCurRssi(WLAN_DEV_INFO *pDev);

A_UINT32
ar5513GetDefAntenna(WLAN_DEV_INFO *pDev);

void
ar5513SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna);

void
ar5513SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                       CHAN_VALUES *pChval);

void
ar5513UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt);

A_BOOL
ar5513UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev);

void
ar5513DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose);

void
ar5513SetupSleepRegisters(WLAN_DEV_INFO *pDev);

void
ar5513RestoreSleepRegisters(WLAN_DEV_INFO *pDev);

void
ar5513SwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete);

A_UINT32
ar5513GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord);

void
ar5513GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
ar5513SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
ar5513SendXrChirp(WLAN_DEV_INFO *pDev);

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_MISC_H_ */
