/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5513KeyCache.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513KeyCache.h#5 $
 */

#ifndef _AR5513_KEYCACHE_H_
#define _AR5513_KEYCACHE_H_

#ifdef _cplusplus
extern "C" {
#endif

A_UINT32
ar5513GetKeyCacheSize(WLAN_DEV_INFO *pDev);

void
ar5513ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev);

void
ar5513ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);

void
ar5513SetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                       WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                    A_BOOL bXorKey);

A_UINT16
ar5513KeyCacheAlloc(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex, SIB_ENTRY *pSib,
                    A_UINT16 keyType, A_BOOL *pHwEncrypt);

void
ar5513KeyCacheFree(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex);

A_UINT16
ar5513SetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIdx, 
                    WLAN_PRIV_RECORD *pKey, A_BOOL en);

#if defined(PT_2_PT_ANT_DIV) || !defined(BUILD_AP)
void
ar5513ProcAntennaData(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib,
		      ATHEROS_DESC *pDesc, A_UINT8 descType);
#else /* ! PT_2_PT_ANT_DIV */
void
ar5513ProcAckAntennaData(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, 
			 ATHEROS_DESC *pTxDesc);

void
ar5513ProcRxAntennaData(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, 
			 ATHEROS_DESC *pRxDesc);
#endif /* ! PT_2_PT_ANT_DIV */

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5513DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 keyCacheIndex);

#endif


#ifdef _cplusplus
}
#endif

#endif /* _AR5513_KEYCACHE_H_ */
