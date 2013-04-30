/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines exported routines from ar5211KeyCache.c
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211KeyCache.h#3 $
 */

#ifndef _AR5211_KEYCACHE_H_
#define _AR5211_KEYCACHE_H_

#ifdef _cplusplus
extern "C" {
#endif

void
ar5211ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev);

void
ar5211ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);

void
ar5211SetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                       WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                    A_BOOL bXorKey);

A_UINT16
ar5211KeyCacheAlloc(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex, SIB_ENTRY *pSib,
                    A_UINT16 keyType, A_BOOL *pHwEncrypt);

void
ar5211KeyCacheFree(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex);

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5211DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 idx);

#endif


#ifdef _cplusplus
}
#endif

#endif /* _AR5211_KEYCACHE_H_ */
