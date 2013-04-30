/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific key cache routines.
 *
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211KeyCache.c#3 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211KeyCache.h"
#include "ar5211.h"


#ifdef DEBUG
extern int keyDebugLevel;
#endif /* DEBUG */

/**************************************************************
 * ar5211ReserveHalKeyCacheEntries
 *
 * Pre-allocates key cache entries needed by HAL
 */
void
ar5211ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev)
{
    return;
}

/**************************************************************
 * ar5211ResetKeyCacheEntry
 *
 * Clears the specified key cache entry
 */
void
ar5211ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex)
{
    A_UINT32 keyCacheOffset;

    ASSERT(keyCacheIndex < pDev->pHalInfo->halCapabilities.halKeyCacheSize);

    keyCacheOffset = MAC_KEY_CACHE +
                     (keyCacheIndex * sizeof(AR5211_KEY_CACHE_ENTRY));

    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyVal0), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyVal1), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyVal2), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyVal3), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyVal4), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyType), MAC_KEY_TYPE_CLEAR);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, macAddrLo), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, macAddrHi), 0);
}


// Base addresses of key cache in h/w
#define KEYREGS_INDEX(field) \
    (FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, field) / sizeof (A_UINT32))

// Set up key values
#define bytesToInt(str,idx) \
    ((str[idx]) | (str[idx+1] << 8) | (str[idx+2] << 16) | (str[idx+3] << 24))

/**************************************************************
 * ar5211SetKeyCacheEntry
 *
 * Sets the specified key cache entry
 *
 * Note: WEP key cache hardware requires that each double-word pair be
 * written in even/odd order (since the destination is a 64-bit register).
 * So we compile all the values for an entry into an array, then write the
 * array.  Reads can be done in any order.
 */
void
ar5211SetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                       WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                       A_BOOL bXorKey)
{
    A_UINT32    keyField;
    A_UINT32    macHi = 0, macLo = 0;
    int         i;
    A_UINT32    xorMask = bXorKey ? (KEY_XOR << 24 | KEY_XOR << 16 | KEY_XOR << 8 | KEY_XOR) : 0;
    A_UINT8     *pKeyVal;
    A_UINT16    keyLength;

    REGISTER_VAL keyRegs[] = {
            {MAC_KEY_CACHE, 0}, {MAC_KEY_CACHE+0x4, 0}, {MAC_KEY_CACHE+0x8, 0}, {MAC_KEY_CACHE+0xc, 0},
            {MAC_KEY_CACHE+0x10, 0}, {MAC_KEY_CACHE+0x14, MAC_KEY_TYPE_CLEAR}, {MAC_KEY_CACHE+0x18, 0}, 
            {MAC_KEY_CACHE+0x1c, 0} };

    ASSERT(keyCacheIndex < pDev->pHalInfo->halCapabilities.halKeyCacheSize);

    /* Update keyRegs to be relative to keyCacheIndex. */
    for (i = 0; i < (sizeof(keyRegs) / sizeof(REGISTER_VAL)); i++) {
        keyRegs[i].Offset += keyCacheIndex * sizeof(AR5211_KEY_CACHE_ENTRY);
    }

    if (pWlanPrivRecord != NULL) {
        /* Set Key length and key value array according to key type */

        switch (pWlanPrivRecord->keyType) {
        case PRIV_KEY_TYPE_AES_OCB:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_AES;
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_AES_CCM:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_CLEAR;
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_CKIP:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_CLEAR;
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            break;

        case PRIV_KEY_TYPE_TKIP:
        case PRIV_KEY_TYPE_TKIP_SW:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_CLEAR;
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            break;

        case PRIV_KEY_TYPE_WEP:
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            switch (keyLength) {
            case 128:
                keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_WEP_128;
                break;

            case 104:
                keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_WEP_104;
                break;

            case 40:
                keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_WEP_40;
                break;
            }
            break;
        default:
            ASSERT(0);
            return;
        }

        switch (keyLength) {
        case 128:
        case 104:
            if (keyLength == 104) {
                // Only one byte (13th) in this register
                keyRegs[KEYREGS_INDEX(keyVal4)].Value = (bytesToInt(pKeyVal,12)  ^ xorMask) & 0xff;
            } else {
                keyRegs[KEYREGS_INDEX(keyVal4)].Value = (bytesToInt(pKeyVal,12) ^ xorMask);
            }
            // fall-through

            keyRegs[KEYREGS_INDEX(keyVal3)].Value = (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value = (bytesToInt(pKeyVal,6) ^ xorMask);
            // fall-through

        case 40:
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (bytesToInt(pKeyVal,4) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = (bytesToInt(pKeyVal,0) ^ xorMask);
            break;

        default:
            return;     /* EINVAL */
        }
    }

    /* Set MAC address -- shifted right by 1.  MacLo is
     * the 4 MSBs, and MacHi is the 2 LSBs.
     */
    if (pMacAddr != NULL) {
        macHi = (pMacAddr->octets[5] << 8) | pMacAddr->octets[4];
        macLo = (pMacAddr->octets[3] << 24) | (pMacAddr->octets[2] << 16) |
                (pMacAddr->octets[1] << 8)  | pMacAddr->octets[0];

        macLo >>= 1;
        macLo |= (macHi & 1) << 31; /* carry */
        macHi >>= 1;
    }
    keyRegs[KEYREGS_INDEX(macAddrLo)].Value = macLo;
    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = macHi;

    // Set "key is valid" bit
    keyField = keyRegs[KEYREGS_INDEX(macAddrHi)].Value;
    ((MAC_ADDR_HI_KEY_VALID *)&keyField)->keyValid = TRUE;
    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = keyField;

    // Write them to hardware from the array, thereby ensuring that
    // they are written in order (even/odd).  Remove clear text value
    // of key from prying eyes.
    for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
        writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
        keyRegs[i].Value = 0;
    }
}

A_UINT16
ar5211KeyCacheAlloc(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex,
                    SIB_ENTRY *pSib, A_UINT16 keyType,
                    A_BOOL *pHwEncrypt)
{
    SIB_ENTRY        *pVictimSib = NULL;
    SIB_ENTRY        *pAssocSib  = NULL;
    SIB_ENTRY        *pFreeSib   = NULL;
    A_UINT32         tsAssoc     = 0xffffffff;
    A_UINT32         tsFree      = 0xffffffff;
    SIB_ENTRY        *pSibtmp;
    SIB_ENTRY        **keyCacheSib;

    if (pHwEncrypt) {
       *pHwEncrypt = TRUE;
    }

    keyCacheSib = pdevInfo->keyCacheSib;

    if (hwIndex != HWINDEX_INVALID) {
        /*
         * We are attempting to reuse a previously allocated index.
         */
        if  (keyCacheSib[hwIndex] != NULL && keyCacheSib[hwIndex] != pSib) {
            ar5211ResetKeyCacheEntry(pdevInfo, hwIndex);
        }

        keyCacheSib[hwIndex] = pSib;
        return (hwIndex);
    }

    for (hwIndex = MAX_SHARED_KEYS; hwIndex < pdevInfo->keyCacheSize;
         hwIndex++)
    {
        pSibtmp = keyCacheSib[hwIndex];
        if (pSibtmp == NULL) {
            break;
        } else if (pSibtmp->numTxPending <= 0 &&
                   pSibtmp != pdevInfo->localSta)
        {
            /* Is there a race condition here?  The sib may wake up? */
            if (pSibtmp->staState & (STATE_AUTH|STATE_ASSOC)) {
                if (pSibtmp->staLastActivityTime < tsAssoc) {
                    tsAssoc   = pSibtmp->staLastActivityTime;
                    pAssocSib = pSibtmp;
                }
            } else {
                if (pSibtmp->staLastActivityTime < tsFree) {
                    tsFree   = pSibtmp->staLastActivityTime;
                    pFreeSib = pSibtmp;
                }
            }
        }
    }

    if (hwIndex < pdevInfo->keyCacheSize) {
        /*
         * We found an available entry
         * Update entry to reflect new owner and return
         * found entry.
         */
        keyCacheSib[hwIndex] = pSib;
        return (hwIndex);
    }

    /*
     * No easy key cache entry was found. We will steal an entry
     */
    pVictimSib = pFreeSib ? pFreeSib : pAssocSib;

    if (pVictimSib == NULL) {
#ifdef DEBUG
        if (keyDebugLevel > 0) {
            uiPrintf("ar5211KeyCacheAlloc: didn't find a victim Sib\n");
        }
#endif
        return HWINDEX_INVALID;
    }

#ifdef DEBUG
    if (keyDebugLevel > 0) {
        uiPrintf("ar5211KeyCacheAlloc: victim Sib = 0x%x index=%d\n",
                 (A_UINT32)pVictimSib, pVictimSib->hwIndex);
    }
#endif

    /* Steal pSib's key cache entry */
    if (pVictimSib != pSib) {
        A_SIB_ENTRY_LOCK(pVictimSib);
    }
    hwIndex = pVictimSib->hwIndex;
    ASSERT(keyCacheSib[hwIndex] == pVictimSib);

    ar5211KeyCacheFree(pdevInfo, hwIndex);

    pdevInfo->keyCache[hwIndex]  = NULL;
    pVictimSib->hwIndex          = HWINDEX_INVALID;

    /*
     * Make sure that first frame will clear up any pending
     * retries from previous pSib
     */
    pSib->needClearDest = TRUE;

    /* Let swretry know we're pulling the rug out from under it */
    swretryStateReset(pdevInfo, pVictimSib, TRUE, FALSE);

    if (pVictimSib != pSib) {
        A_SIB_ENTRY_UNLOCK(pVictimSib);
    }

    /*
     * Mark the stolen entry as belonging to the new Sib
     */
    keyCacheSib[hwIndex] = pSib;

    return hwIndex;
}

void
ar5211KeyCacheFree(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex)
{
    SIB_ENTRY *pSib;

    pSib = pdevInfo->keyCacheSib[hwIndex];
    ASSERT(pSib != NULL);

    ar5211ResetKeyCacheEntry(pdevInfo, hwIndex);
    pdevInfo->keyCacheSib[hwIndex] = NULL;

    /*
     * shouldn't be freeing hwIndex if frames are pending! swretry
     * depends on the hwIndex
     */
    ASSERT(!pSib->numTxPending);
    ASSERT(!pSib->numFilteredPending);
}

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5211DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 idx)
{
    static const char *keyTypeToName[] = {
        "wep40",
        "wep104",
        "invalid",
        "wep128",
        "invalid",
        "ocb",
        "invalid",
        "clear"
    };
    A_UINT32 offset;
    A_UINT32 keyType;

    offset = MAC_KEY_CACHE + idx * sizeof(AR5211_KEY_CACHE_ENTRY);
    keyType = readPlatformReg(pDev, offset + FIELD_OFFSET(AR5211_KEY_CACHE_ENTRY, keyType));
    return keyTypeToName[keyType & 0x07];
}

#endif
#endif // #ifdef BUILD_AR5211

