/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halKeyCache.c#3 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

/**************************************************************
 * halReserveHalKeyCacheEntries
 *
 * Pre-allocates key cache entries needed by HAL
 */
void
halReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev && pDev->pHwFunc && 
	   pDev->pHwFunc->hwReserveHalKeyCacheEntries);

    pDev->pHwFunc->hwReserveHalKeyCacheEntries(pDev);

    return;
}

/**************************************************************
 * halResetKeyCache
 *
 * Clears ALL key cache entries
 */
void
halResetKeyCache(WLAN_DEV_INFO *pDev)
{
    A_UINT16 keyCacheIndex;

    ASSERT(pDev);

    for (keyCacheIndex = 0; keyCacheIndex < halGetCapability(pDev, HAL_GET_KEY_CACHE_SIZE, 0); keyCacheIndex++) {
        halResetKeyCacheEntry(pDev, keyCacheIndex);
    }

}

/**************************************************************
 * halResetKeyCacheEntry
 *
 * Clears the specified key cache entry
 */
void
halResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwResetKeyCacheEntry);

    pDev->pHwFunc->hwResetKeyCacheEntry(pDev, keyCacheIndex);
}

/**************************************************************
 * halSetKeyCacheEntry
 *
 * Sets the specified key cache entry
 */
void
halSetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
	                WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                    A_BOOL bXorKey)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwSetKeyCacheEntry);

    pDev->pHwFunc->hwSetKeyCacheEntry(pDev, keyCacheIndex, pMacAddr, pWlanPrivRecord, bXorKey);
}


/**************************************************************
 * halKeyCacheAlloc
 *
 * Allocate key cache index, including handling over-subscription.
 *
 * First try to find an unused key cache entry.
 *
 * If we have to steal a key cache entry, we take the oldest SIB that
 * does not have any pending tx frames, with preference to SIBs that
 * are not associated.
 *
 * If all key cache entries are in use by the tx queue, we
 * fail by returning HWINDEX_INVALID.
 *
 * NOTE: caller must hold the keySem.
 */
A_UINT16
halKeyCacheAlloc(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex, SIB_ENTRY *pSib,
                 A_UINT16 keyType, A_BOOL *pHwEncrypt)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwKeyCacheAlloc);

    return (pDev->pHwFunc->hwKeyCacheAlloc(pDev, hwIndex, pSib, keyType,
                                           pHwEncrypt));
}

/*
 * Free the key cache entry referenced by hwIndex
 */
void
halKeyCacheFree(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwKeyCacheFree);

    pDev->pHwFunc->hwKeyCacheFree(pDev, hwIndex);
}

/**************************************************************
 * halSetDecompMask
 *
 * update the decompression mask based on key cache index
 */
A_UINT16
halSetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                 WLAN_PRIV_RECORD *pKey, A_BOOL en)
{

    ASSERT(pDev && pDev->pHwFunc);

    if (pDev->pHwFunc->hwSetDecompMask) {
        return pDev->pHwFunc->hwSetDecompMask(pDev, keyCacheIndex, pKey, en);
    }

    return DECOMPINDEX_INVALID;
}


