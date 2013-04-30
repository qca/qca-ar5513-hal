/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific key cache routines.
 *
 */

#ifdef BUILD_AR5212

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212KeyCache.c#3 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"

/* Headers for HW private items */
#include "ar5212Reg.h"
#include "ar5212KeyCache.h"
#include "ar5212Misc.h"
#include "ar5212.h"

#ifdef DEBUG
extern int keyDebugLevel;
#endif /* DEBUG */

static void
ar5212KeyCacheInvalidate(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex);

/**************************************************************
 * ar5212ReserveHalKeyCacheEntries
 *
 * Pre-allocates key cache entries needed by HAL
 */
void
ar5212ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev)
{
    return;
}

/**************************************************************
 * ar5212ResetKeyCacheEntry
 *
 * Clears the specified key cache entry
 */
void
ar5212ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex)
{
    A_UINT32 keyCacheOffset;

    keyCacheOffset = MAC_KEY_CACHE +
        (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY));

    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyVal0), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyVal1), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyVal2), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyVal3), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyVal4), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyType),
                     MAC_KEY_TYPE_CLEAR);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, macAddrLo), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, macAddrHi), 0);
}


// Base addresses of key cache in h/w
#define KEYREGS_INDEX(field) \
    (FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, field) / sizeof (A_UINT32))

// Set up key values
#define bytesToInt(str,idx) \
    ((str[idx]) | (str[idx+1] << 8) | (str[idx+2] << 16) | (str[idx+3] << 24))

/**************************************************************
 * ar5212SetKeyCacheEntry
 *
 * Sets the specified key cache entry
 *
 * Note: WEP key cache hardware requires that each double-word pair be
 * written in even/odd order (since the destination is a 64-bit register).
 * So we compile all the values for an entry into an array, then write the
 * array.  Reads can be done in any order, but the CACHE_PROTECT bit in
 * the eeprom must allow it for STA configs.
 *
 * AR5212 Supports WEP, TKIP and AES-OCB encryption in all macRevs.
 *        Supports AES-CCM only in macRev > 1.3
 *        Supports TKIP-Michael only macRev > 1.2
 */
void
ar5212SetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                       WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                       A_BOOL bXorKey)
{
    A_UINT32     keyField, savedKey[2];
    A_UINT16     tkipIndex;
    A_UINT32     macHi = 0, macLo = 0, savedMacLo = 0, savedMacHi = 0;
    int          i;
    A_UINT32     xorMask = bXorKey ? (KEY_XOR << 24 | KEY_XOR << 16 |
                                      KEY_XOR << 8 | KEY_XOR) : 0;
    A_UINT8      *pKeyVal;
    A_UINT16     keyLength;
    A_BOOL       isAP = (pDev->localSta->serviceType == WLAN_AP_SERVICE);
    REGISTER_VAL keyRegs[8];

    ASSERT(keyCacheIndex < pDev->pHalInfo->halCapabilities.halKeyCacheSize);

    /*
     *  We have to sequence the writes to prevent MIC errors of pending rx frames.
     *  When we install either the encryption or decryption keys, we install
     *  the ones complement of the real key so that the encrypt/decrypt data path
     *  will fail with decryption errors whilst we are mucking with the MIC
     *  keys. After all of the potentially 4 key entries are set up,
     *  we must then go back and correct the corrupted key info.
     */

    /*
     * starting with clear key type, compression needs the keyIndex
     * associated with mac address
     */
    keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_CLEAR;
    savedKey[0] = savedKey[1] = 0;

    if (pWlanPrivRecord) {
        /* Set Key length and key value array according to key type */
        switch (pWlanPrivRecord->keyType) {
        case PRIV_KEY_TYPE_AES_OCB:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_AES;
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_AES_CCM:
            keyRegs[KEYREGS_INDEX(keyType)].Value
                = halGetCapability(pDev, HAL_GET_CIPHER_SUPPORT,
                                   PRIV_KEY_TYPE_AES_CCM)
                ? MAC_KEY_TYPE_CCM
                : MAC_KEY_TYPE_CLEAR;
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_TKIP:
            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_TKIP;
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            break;

        case PRIV_KEY_TYPE_CKIP:
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
                keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                    (bytesToInt(pKeyVal,12) ^ xorMask) & 0xff;
            } else {
                keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                    (bytesToInt(pKeyVal,12) ^ xorMask);
            }
            // fall-through

            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            // fall-through

        case 40:

            /* intentionally corrupt the first 2 words at keyIndex */

            savedKey[1] =  (bytesToInt(pKeyVal,4) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (~savedKey[1] & 0xffff);

            savedKey[0] = (bytesToInt(pKeyVal,0) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = ~savedKey[0];

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
    /* Don't put the macAddrs into this entry; save them for the decrypt entry */
    savedMacLo = macLo;
    savedMacHi = macHi;
    /* Set valid bit in savedMacHi */
    ((MAC_ADDR_HI_KEY_VALID *)&savedMacHi)->keyValid = TRUE;

    keyRegs[KEYREGS_INDEX(macAddrLo)].Value = 0;
    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = 0;

    // Set "key is valid" bit
    keyField = keyRegs[KEYREGS_INDEX(macAddrHi)].Value;
    ((MAC_ADDR_HI_KEY_VALID *)&keyField)->keyValid = TRUE;
    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = keyField;

    if (!pWlanPrivRecord || pWlanPrivRecord->keyType != PRIV_KEY_TYPE_TKIP ||
        !halGetCapability(pDev, HAL_GET_MIC_SUPPORT, PRIV_KEY_TYPE_TKIP))
    {
        // Write them to hardware from the array, thereby ensuring that
        // they are written in order (even/odd).
        for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
            keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
            writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
        }

        /* Write the correct keyval0 now */
        writePlatformReg(pDev, MAC_KEY_CACHE +
                         (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                         savedKey[0]);
        writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                         (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                         savedKey[1]);

        writePlatformReg(pDev, MAC_KEY_CACHE + (24) +
                         (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                         savedMacLo);
        writePlatformReg(pDev, MAC_KEY_CACHE + (28) +
                         (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                         savedMacHi);
        return;
    }

    /*
     *  Encrypt key = i (already done above)
     *  Enmic key   = i + 64
     *  Decrypt key = i + 32
     *  Demic key   = i + 32 + 64
     */

    /* Now do the TKIP sub-keys */
    if (isAP) {
        /* Write out the encrypt key */
        for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
            keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
            writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
        }

        if (keyCacheIndex < MAX_SHARED_KEYS) {
            /* Need an enmic key */

            tkipIndex = keyCacheIndex + 64; // enmic key
            ASSERT(tkipIndex < pDev->pHalInfo->halCapabilities.halKeyCacheSize);
            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            /* patch up the encrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);

        } else {
            /* Adding a unique key */

            tkipIndex = keyCacheIndex + 32; // decrypt key
            pKeyVal = pWlanPrivRecord->keyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_TKIP;
            keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                (bytesToInt(pKeyVal,12) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (~savedKey[1] & 0xffff);
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = ~savedKey[0];

            keyRegs[KEYREGS_INDEX(macAddrLo)].Value = savedMacLo;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = savedMacHi;

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 32 + 64; // demic key
            ASSERT(tkipIndex < pDev->pHalInfo->halCapabilities.halKeyCacheSize);
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; // enmic key 
            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* Now write out the correct values */
            /* encrypt */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            /* then decrypt */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            /* savedMac{Lo,Hi} already written out above */
        }
    } else {

        /* STA */
        if (pMacAddr != NULL) {

            tkipIndex = keyCacheIndex; // encrypt key
            pKeyVal = pWlanPrivRecord->keyVal;
            /* Do not clear this entry! */

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_TKIP;
            keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                (bytesToInt(pKeyVal,12) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (~savedKey[1] & 0xffff);
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = ~savedKey[0];

            keyRegs[KEYREGS_INDEX(macAddrLo)].Value = 0;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = 0;

            // Set "key is valid" bit
            keyField = keyRegs[KEYREGS_INDEX(macAddrHi)].Value;
            ((MAC_ADDR_HI_KEY_VALID *)&keyField)->keyValid = TRUE;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = keyField;

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; // enmic key 

            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* patch up the encrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            tkipIndex = keyCacheIndex + 32; // decrypt key
            pKeyVal = pWlanPrivRecord->keyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_TKIP;
            keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                (bytesToInt(pKeyVal,12) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (~savedKey[1] & 0xffff);
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = ~savedKey[0];

            keyRegs[KEYREGS_INDEX(macAddrLo)].Value = savedMacLo;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = savedMacHi;

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64 + 32; // demic key
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            /* patch up the decrypt key */
            writePlatformReg(pDev,MAC_KEY_CACHE +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);
        } else {

            /* Group Key installation */

            tkipIndex = keyCacheIndex; // decrypt key
            pKeyVal = pWlanPrivRecord->keyVal;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_TKIP;
            keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                (bytesToInt(pKeyVal,12) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            keyRegs[KEYREGS_INDEX(keyVal1)].Value = (~savedKey[1] & 0xffff);
            keyRegs[KEYREGS_INDEX(keyVal0)].Value = ~savedKey[0];

            keyRegs[KEYREGS_INDEX(macAddrLo)].Value = savedMacLo;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = savedMacHi;

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; // demic key
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);
            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5212_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            /* patch up the decrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5212_KEY_CACHE_ENTRY)),
                             savedKey[1]);
        }
    }

    return;
}

A_UINT16
ar5212KeyCacheAlloc(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex,
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
    A_UINT16         indexLimit, singleEntry = 0;

    if (pHwEncrypt) {
       *pHwEncrypt = TRUE;
    }

    keyCacheSib = pdevInfo->keyCacheSib;

    if (hwIndex != HWINDEX_INVALID) {
        /*
         * We are attempting to reuse a previously allocated index
         * or we really want a particular index.
         * If the new key is TKIP make sure all four entries are either
         * free or already allocated to this Sib.
         *
         * In theory we should be able to assert that either
         * the keyCacheSib entry is free (ie. NULL) or it is already
         * owned by us.  Unfortunately, the callers of the key cache
         * routines are not obeying the rules of freeing up key cache
         * entries once they are done.  They just keep reusing them
         * as they please.  Most of the cases are in the Station mode.
         * and non wpa.
         */

        if  (keyCacheSib[hwIndex] != NULL && keyCacheSib[hwIndex] != pSib) {
            ar5212KeyCacheInvalidate(pdevInfo, hwIndex);
        }

        if (keyType == PRIV_KEY_TYPE_TKIP) {
            WLAN_PRIV_RECORD  keyInfo;
            /*
             * We Re-use the KeyCache Index, which was  added as WEP,  for TKIP.
             * Roll Back the Compression State of the previous index.
             */
            keyInfo.keyType = PRIV_KEY_TYPE_WEP;
            ar5212SetDecompMask(pdevInfo, hwIndex, &keyInfo, FALSE);

            if (hwIndex < (pdevInfo->keyCacheSize >> 2) &&
                (!keyCacheSib[hwIndex + 32] ||
                 keyCacheSib[hwIndex + 32] == pSib) &&
                (!keyCacheSib[hwIndex + 64] ||
                 keyCacheSib[hwIndex + 64] == pSib) &&
                (!keyCacheSib[hwIndex + 32 + 64] ||
                keyCacheSib[hwIndex + 32 + 64] == pSib) )
            {
                keyCacheSib[hwIndex + 32] = pSib;
                keyCacheSib[hwIndex + 64] = pSib;
                keyCacheSib[hwIndex + 32 + 64] = pSib;
            } else if (pHwEncrypt) {
               *pHwEncrypt = FALSE;
            }
        }
        keyCacheSib[hwIndex] = pSib;
        return (hwIndex);
    }
    if (pdevInfo->staConfig.encryptionAlg == ENCRYPTION_TKIP ||
        pdevInfo->staConfig.encryptionAlg == ENCRYPTION_AUTO)
    {
        /*
         * If we are supporting TKIP then limit the size of the key
         * cache to the max number of tkip entries we can have
         */
        indexLimit = (A_UINT16)A_MIN(pdevInfo->keyCacheSize,
                         pdevInfo->pHalInfo->halCapabilities.halKeyCacheSize / 4);
    } else {
        indexLimit = (A_UINT16)pdevInfo->keyCacheSize;
    }


    for (hwIndex = MAX_SHARED_KEYS; hwIndex < indexLimit; hwIndex++)
    {
        if (((hwIndex & 0x1F) < MAX_SHARED_KEYS) &&
            pdevInfo->localSta->serviceType == WLAN_STA_SERVICE)
        {
            /*
             * Do not allocate an index which would be required by a
             * broadcast key using TKIP
             */
            continue;
        }
        pSibtmp = keyCacheSib[hwIndex];
        if (pSibtmp == NULL) {
            if (keyType != PRIV_KEY_TYPE_TKIP) {
                break;
            } else {
                singleEntry = hwIndex;
                if (hwIndex < (pdevInfo->keyCacheSize >> 2) &&
                    keyCacheSib[hwIndex + 32] == NULL &&
                    keyCacheSib[hwIndex + 64] == NULL &&
                    keyCacheSib[hwIndex + 32 + 64] == NULL)
                {
                     break;
                }
            }
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

    if (hwIndex < indexLimit) {
        /*
         * We found an available entry (or 4 for tkip)
         * Update entry to reflect new owner and return
         * found entry.
         */
        if (keyType == PRIV_KEY_TYPE_TKIP) {
                ASSERT(hwIndex < (pdevInfo->keyCacheSize >> 2));
                keyCacheSib[hwIndex + 32] = pSib;
                keyCacheSib[hwIndex + 64] = pSib;
                keyCacheSib[hwIndex + 32 + 64] = pSib;
        }
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
            uiPrintf("ar5212KeyCacheAlloc: didn't find a victim Sib\n");
        }
#endif
        return HWINDEX_INVALID;
    }

#ifdef DEBUG
    if (keyDebugLevel > 0) {
        uiPrintf("ar5212KeyCacheAlloc: victim Sib = 0x%x index=%d\n",
                 (A_UINT32)pVictimSib, pVictimSib->hwIndex);
    }
#endif

    /* Steal pSib's key cache entry */
    if (pVictimSib != pSib) {
        A_SIB_ENTRY_LOCK(pVictimSib);
    }
    hwIndex = pVictimSib->hwIndex;
    ASSERT(keyCacheSib[hwIndex] == pVictimSib);

    ar5212KeyCacheFree(pdevInfo, hwIndex);

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
    if (keyType == PRIV_KEY_TYPE_TKIP) {
        ASSERT(hwIndex < (pdevInfo->keyCacheSize >> 2));
        ASSERT(keyCacheSib[hwIndex + 32] == NULL);
        ASSERT(keyCacheSib[hwIndex + 64] == NULL);
        ASSERT(keyCacheSib[hwIndex + 32 + 64] == NULL);
        keyCacheSib[hwIndex + 32] = pSib;
        keyCacheSib[hwIndex + 64] = pSib;
        keyCacheSib[hwIndex + 32 + 64] = pSib;
    }
    keyCacheSib[hwIndex] = pSib;

    return hwIndex;
}

void
ar5212KeyCacheFree(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex)
{
    SIB_ENTRY *pSib;

    ar5212KeyCacheInvalidate(pdevInfo, hwIndex);
    pSib = pdevInfo->keyCacheSib[hwIndex];
    ASSERT(pSib != NULL);
    pdevInfo->keyCacheSib[hwIndex] = NULL;
    if (pSib && pSib->decompMaskIndex == hwIndex) {
        pSib->decompMaskIndex = ar5212SetDecompMask(pdevInfo, hwIndex,
                                                    pdevInfo->keyCache[hwIndex],
                                                    FALSE);
    }

    /*
     * shouldn't be freeing hwIndex if frames are pending! swretry
     * depends on the hwIndex of the tx key only
     */
    ASSERT(!pSib->numTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->numFilteredPending || pSib->hwIndex != hwIndex);
}

static void
ar5212KeyCacheInvalidate(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex)
{
    SIB_ENTRY *pSib;

    pSib = pdevInfo->keyCacheSib[hwIndex];
    ASSERT(pSib != NULL);
    /*
     * shouldn't be freeing hwIndex if frames are pending! swretry
     * depends on the hwIndex of the tx key only
     */
    ASSERT(!pSib->numTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->numFilteredPending || pSib->hwIndex != hwIndex);

    /*
     * If the key is TKIP we actually need to reset 4 key cache entries.
     * Unfortunately we don't know the type of key we are trying to free
     * and we can't read the key cache, so we use the keyCacheSib.
     * If the respective offsets in the keycacheSib correspond to the
     * Sib of the passed index, we reset them.  The only potential side
     * effect of this is that we might free up some really old entries
     * that didn't use to be TKIP but still point to this pSib. This is safe
     * because the Psib doesn't have any outstanding frames and actually good
     * because it increases the chances a new TKIP allocation will get the
     * 4 necessary slots for hardware encryption.
     */
    if (hwIndex < (pdevInfo->keyCacheSize >> 2)) {
        if (pdevInfo->keyCacheSib[hwIndex + 32] == pSib) {
            pdevInfo->keyCacheSib[hwIndex + 32] = NULL;
            pdevInfo->keyCache[hwIndex + 32]    = NULL;
            ar5212ResetKeyCacheEntry(pdevInfo, hwIndex + 32);
        }
        if (pdevInfo->keyCacheSib[hwIndex + 64] == pSib) {
            pdevInfo->keyCacheSib[hwIndex + 64] = NULL;
            pdevInfo->keyCache[hwIndex + 64]    = NULL;
            ar5212ResetKeyCacheEntry(pdevInfo, hwIndex + 64);
        }
        if (pdevInfo->keyCacheSib[hwIndex + 32 + 64] == pSib) {
            pdevInfo->keyCacheSib[hwIndex + 32 + 64] = NULL;
            pdevInfo->keyCache[hwIndex + 32 + 64]    = NULL;
            ar5212ResetKeyCacheEntry(pdevInfo, hwIndex + 32 + 64);
        }
    }

    ar5212ResetKeyCacheEntry(pdevInfo, hwIndex);
}

/**************************************************************
 * ar5212SetDecompMask
 *
 * update decompression mask based on keyCache index
 */
A_UINT16
ar5212SetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIdx,
                    WLAN_PRIV_RECORD *pKey, A_BOOL en)
{
    if (keyCacheIdx != HWINDEX_INVALID &&
         pDev->pHalInfo->halCapabilities.halCompressSupport)
    {
        A_UINT16 decompMaskIdx = keyCacheIdx;

        /* TKIP decrypt index */
        if (pKey && pKey->keyType == PRIV_KEY_TYPE_TKIP) {
            decompMaskIdx += 32;
        }
        ASSERT(decompMaskIdx < DECOMP_MASK_SIZE);
        A_REG_WR(pDev, MAC_DCM_A, decompMaskIdx);
        A_REG_WR(pDev, MAC_DCM_D, en ? MAC_DCM_D_EN : 0);
        pDev->decompMaskArray[decompMaskIdx] = en;

        return (en ? keyCacheIdx : DECOMPINDEX_INVALID);
    }

    return DECOMPINDEX_INVALID;
}

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5212DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 idx)
{
    static const char *keyTypeToName[] = {
        "wep40",
        "wep104",
        "invalid",
        "wep128",
        "tkip",
        "ocb",
        "ccm",
        "clear"
    };
    A_UINT32 offset;
    A_UINT32 keyType;

    offset = MAC_KEY_CACHE + idx * sizeof(AR5212_KEY_CACHE_ENTRY);
    keyType = readPlatformReg(pDev, offset +
                              FIELD_OFFSET(AR5212_KEY_CACHE_ENTRY, keyType));
    return keyTypeToName[keyType & 0x07];
}

#endif
#endif // #ifdef BUILD_AR5212

