/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5212KeyCache.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212KeyCache.h#3 $
 */

#ifndef _AR5212_KEYCACHE_H_
#define _AR5212_KEYCACHE_H_

#ifdef _cplusplus
extern "C" {
#endif

void
ar5212ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev);

void
ar5212ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);

void
ar5212SetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                       WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                    A_BOOL bXorKey);

A_UINT16
ar5212KeyCacheAlloc(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex, SIB_ENTRY *pSib,
                    A_UINT16 keyType, A_BOOL *pHwEncrypt);

void
ar5212KeyCacheFree(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex);

A_UINT16
ar5212SetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIdx, 
                    WLAN_PRIV_RECORD *pKey, A_BOOL en);

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5212DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 keyCacheIndex);

#endif


#ifdef _cplusplus
}
#endif

#endif /* _AR5212_KEYCACHE_H_ */
