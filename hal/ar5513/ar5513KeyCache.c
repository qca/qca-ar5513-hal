/*
 * Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific key cache routines.
 *
 */

#ifdef BUILD_AR5513

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513KeyCache.c#10 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanext.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"

/* Headers for HW private items */
#include "ar5513MacReg.h"
#include "ar5513KeyCache.h"
#include "ar5513Misc.h"
#include "ar5513Mac.h"

// Base addresses of key cache in h/w
#define KEYREGS_INDEX(field) \
    (FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, field) / sizeof (A_UINT32))

// Set up key values
#define bytesToInt(str,idx) \
    ((str[idx]) | (str[idx+1] << 8) | (str[idx+2] << 16) | (str[idx+3] << 24))


#ifdef DEBUG
extern int keyDebugLevel;
int kcDebugLevel = 0;
#endif /* DEBUG */

#define KC_DEBUG_LOG    0x0001
#define KC_DEBUG_PLUMB  0x0002

static void
ar5513KeyCacheInvalidate(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex);

/**************************************************************
 * ar5513GetKeyCacheSize
 *
 * Returns size of hardware key cache.
 */
A_UINT32
ar5513GetKeyCacheSize(WLAN_DEV_INFO *pDev)
{
    return MAC_KEY_CACHE_SIZE;
}

/**************************************************************
 * ar5513ReserveHalKeyCacheEntries
 *
 * This routine allocates the last 2 or 4 Key Cache entries
 * to be used for the beacon antenna control. The chain_sel,
 * chain0_sel, and chain1_sel bits are configured appropriately
 * for each entry. Entry 124 is Chain 0 Antenna 0. Entry 125 is
 * Chain 0 Antenna 1. Entry 126 is Chain 1 Antenna 0. Finally,
 * entry 127 is Chain 1 Antenna 1.
 *
 * Returns None.
 */
void
ar5513ReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev)
{
    A_UINT16 i, j, nentries;
    A_UINT16 hwIndex;
    A_INT8 chainSel, antSel;
    A_UINT32 keyCacheOffset;
    A_UINT32 keyField;
    REGISTER_VAL keyRegs[8];

//  Update Key Cache txChainSel field
#define SET_TX_CHAIN_SEL(value)                                 \
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;           \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->txChainSel = value;  \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;

//  Update Key Cache txChain0Sel field
#define SET_TX_CHAIN0_ANT(value)                                \
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;           \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->txChain0Sel = value; \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;

//  Update Key Cache txChain1Sel field
#define SET_TX_CHAIN1_ANT(value)                                \
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;           \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->txChain1Sel = value; \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;

    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN) {
        nentries = 4;
    }
    else {
        nentries = 2;
    }

    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = 0;

    chainSel = antSel = 0;

    for (i = MAC_KEY_CACHE_SIZE - nentries, j = 0;
         i < MAC_KEY_CACHE_SIZE;
         i++, j++)
    {

        /*
        ** Reserve Key Cache entries by passing localSta as the 
        ** SIB pointer. There is no key type, hence PRIV_KEY_TYPE_NULL
        */

        hwIndex = ar5513KeyCacheAlloc(pDev, i, pDev->localSta, 
                          PRIV_KEY_TYPE_NULL, NULL);
        pDev->pHalInfo->beaconAntCtrl.halKeyCacheTbl[j] = hwIndex;

        keyRegs[KEYREGS_INDEX(keyType)].Value = 0;

        SET_TX_CHAIN_SEL(chainSel);

        if (chainSel) {
            /* Set Chain 1 Antenna select bit */
            SET_TX_CHAIN1_ANT(antSel);

            /* 
            ** Set Chain 0 Antenna select bit with the opposite
            ** just to make it different from chain 1
            */
            SET_TX_CHAIN0_ANT(antSel ^ 1);
        } else {
            SET_TX_CHAIN0_ANT(antSel);
            SET_TX_CHAIN1_ANT(antSel ^ 1);
        }

        keyCacheOffset = MAC_KEY_CACHE + (hwIndex * 
                          sizeof(AR5513_KEY_CACHE_ENTRY));
        writePlatformReg(pDev, keyCacheOffset +
                         FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),
                 keyRegs[KEYREGS_INDEX(keyType)].Value);

        if (antSel) {
            chainSel ^= 1;
        }

        antSel ^= 1;
    }

    return;
}

/**************************************************************
 * ar5513ResetKeyCacheEntry
 *
 * Clears the specified key cache entry
 */
void
ar5513ResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex)
{
    A_UINT32 keyCacheOffset;

    keyCacheOffset = MAC_KEY_CACHE +
        (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyVal0), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyVal1), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyVal2), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyVal3), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyVal4), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),
                     MAC_KEY_TYPE_CLEAR);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, macAddrLo), 0);
    writePlatformReg(pDev, keyCacheOffset +
                     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, macAddrHi), 0);
}


/**************************************************************
 * ar5513SetKeyCacheEntry
 *
 * Sets the specified key cache entry
 *
 * Note: WEP key cache hardware requires that each double-word pair be
 * written in even/odd order (since the destination is a 64-bit register).
 * So we compile all the values for an entry into an array, then write the
 * array.  Reads can be done in any order, but the CACHE_PROTECT bit in
 * the eeprom must allow it for STA configs.
 *
 * AR5513 Supports WEP, TKIP and AES-OCB encryption in all macRevs.
 *        Supports AES-CCM only in macRev > 1.3
 *        Supports TKIP-Michael only macRev > 1.2
 */
void
ar5513SetKeyCacheEntry(WLAN_DEV_INFO *pDev,    A_UINT16 keyCacheIndex,
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
    A_UINT32     chainCtrl;

    chainCtrl = pDev->staConfig.txChainCtrl;

//  Update Key Cache keyType field
#define SET_KEY_TYPE(value)                                         \
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;               \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->keyTypeBitfld = value;   \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;

// Set "Update Beam Forming Cache" bit
#define SET_UPDT_BF_CACHE_BIT(chainControl)                         \
    if (chainControl == DUAL_CHAIN) {                               \
        keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;           \
        ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->updtBfCache = TRUE;  \
        keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;           \
    }

    ASSERT(keyCacheIndex < MAC_KEY_CACHE_SIZE);

    /*
     *  We have to sequence the writes to prevent MIC errors of pending rx
     *  frames.  When we install either the encryption or decryption keys,
     *  we install the ones complement of the real key so that the
     *  encrypt/decrypt data path will fail with decryption errors whilst
     *  we are mucking with the MIC keys.  After all of the potentially 
     *  4 key entries are set up, we must then go back and correct the
     *  corrupted key info.
     */

    /*
     * starting with clear key type, compression needs the keyIndex
     * associated with mac address.
     */
    
    /* 
     * Also for AR5513, clears updt_bf_cache bit 
     * that is used to enable beam forming in dual chain mode.
     */

    keyRegs[KEYREGS_INDEX(keyType)].Value = MAC_KEY_TYPE_CLEAR;

    savedKey[0] = savedKey[1] = 0;

    if (pWlanPrivRecord) {
        /* Set Key length and key value array according to key type */
        switch (pWlanPrivRecord->keyType) {
        case PRIV_KEY_TYPE_AES_OCB:
            SET_KEY_TYPE(MAC_KEY_TYPE_AES);
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_AES_CCM:
            SET_KEY_TYPE(halGetCapability(pDev, HAL_GET_CIPHER_SUPPORT,
                PRIV_KEY_TYPE_AES_CCM) ? MAC_KEY_TYPE_CCM : MAC_KEY_TYPE_CLEAR);
            keyLength = pWlanPrivRecord->aesKeyLength;
            pKeyVal = pWlanPrivRecord->aesKeyVal;
            break;

        case PRIV_KEY_TYPE_TKIP:
            SET_KEY_TYPE(MAC_KEY_TYPE_TKIP);
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            break;

        case PRIV_KEY_TYPE_CKIP:
        case PRIV_KEY_TYPE_TKIP_SW:
            SET_KEY_TYPE(MAC_KEY_TYPE_CLEAR);
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;
            break;

        case PRIV_KEY_TYPE_WEP:
            keyLength = pWlanPrivRecord->keyLength;
            pKeyVal = pWlanPrivRecord->keyVal;

            switch (keyLength) {
            case 128:
                SET_KEY_TYPE(MAC_KEY_TYPE_WEP_128);
                break;
            case 104:
                SET_KEY_TYPE(MAC_KEY_TYPE_WEP_104);
                break;
            case 40:
                SET_KEY_TYPE(MAC_KEY_TYPE_WEP_40);
                break;
            }
            break;

        default:
            /* unknown keytype */
            ASSERT(0);
            return;     /* EINVAL */
        }

        switch (keyLength) {
        case 128:
        case 104:
            if (keyLength == 104) {
                /* Only one byte (13th) in this register */
                keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                    (bytesToInt(pKeyVal,12) ^ xorMask) & 0xff;
            } else {
                keyRegs[KEYREGS_INDEX(keyVal4)].Value =
                    (bytesToInt(pKeyVal,12) ^ xorMask);
            }
            /* fall-through */

            keyRegs[KEYREGS_INDEX(keyVal3)].Value =
                (bytesToInt(pKeyVal,10) ^ xorMask) & 0xffff;
            keyRegs[KEYREGS_INDEX(keyVal2)].Value =
                (bytesToInt(pKeyVal,6) ^ xorMask);
            /* fall-through */

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

#ifdef DEBUG
        if (kcDebugLevel & KC_DEBUG_PLUMB){
            if (pMacAddr) {
                uiPrintf("Plumbing Connection key at index %d\n", keyCacheIndex);
                uiPrintf("BSSID used: ");
                for (i = 0; i < 6; i++) {
                    uiPrintf("%02x%s", pMacAddr->octets[i], i < (6-1) ? ":" : "\n");
                }
            } else {
                uiPrintf("Plumbing BSS key at index %d\n", keyCacheIndex);
            }
            uiPrintf("Plumbed key value: ");
            for (i = 0; i < (keyLength/8); i++) {
                uiPrintf("%02x", pKeyVal[i] & 0xff);
            }
            uiPrintf("\n");
        }
#endif
    }

    /* Set MAC address -- shifted right by 1.  MacLo is
     * the 4 MSBs, and MacHi is the 2 LSBs.
     */
    if (pMacAddr != NULL) {
        macHi = (pMacAddr->octets[5] <<  8) |  pMacAddr->octets[4];
        macLo = (pMacAddr->octets[3] << 24) | (pMacAddr->octets[2] << 16) |
                (pMacAddr->octets[1] <<  8) |  pMacAddr->octets[0];

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

    /* Set "key is valid" bit */
    keyField = keyRegs[KEYREGS_INDEX(macAddrHi)].Value;
    ((MAC_ADDR_HI_KEY_VALID *)&keyField)->keyValid = TRUE;
    keyRegs[KEYREGS_INDEX(macAddrHi)].Value = keyField;

    SET_UPDT_BF_CACHE_BIT(chainCtrl);

    if (!pWlanPrivRecord ||
         pWlanPrivRecord->keyType != PRIV_KEY_TYPE_TKIP ||
        !halGetCapability(pDev, HAL_GET_MIC_SUPPORT, PRIV_KEY_TYPE_TKIP))
    {
        /*
         * Write them to hardware from the array, thereby ensuring that
         * they are written in order (even/odd).
         */

        for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
            keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
            writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
        }

        /* Write the correct keyval0 now */
        writePlatformReg(pDev, MAC_KEY_CACHE +
                         (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                         savedKey[0]);
        writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                         (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                         savedKey[1]);

        writePlatformReg(pDev, MAC_KEY_CACHE + (24) +
                         (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                         savedMacLo);
        writePlatformReg(pDev, MAC_KEY_CACHE + (28) +
                         (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                         savedMacHi);

#ifdef DEBUG
        if (kcDebugLevel & KC_DEBUG_LOG) {
            void logKC(int);
            sysWbFlush();
            logKC(keyCacheIndex);
        }
#endif
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
                (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
            writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
        }

        if (keyCacheIndex < MAX_SHARED_KEYS) {

            /* Need an enmic key */

            tkipIndex = keyCacheIndex + 64; /* enmic key */
            ASSERT(tkipIndex < MAC_KEY_CACHE_SIZE);
            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* patch up the encrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);

        } else {

            /* Adding a unique key */

            tkipIndex = keyCacheIndex + 32; /* decrypt key */
            pKeyVal = pWlanPrivRecord->keyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            SET_KEY_TYPE(MAC_KEY_TYPE_TKIP);
            SET_UPDT_BF_CACHE_BIT(chainCtrl);

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
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 32 + 64; /* demic key */
            ASSERT(tkipIndex < MAC_KEY_CACHE_SIZE);
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset,
                                 keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; /* enmic key */
            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* Now write out the correct values */
            /* encrypt */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            /* then decrypt */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            /* savedMac{Lo,Hi} already written out above */
        }

    } else {

        /* STA */
        if (pMacAddr != NULL) {

            tkipIndex = keyCacheIndex; /* encrypt key */
            pKeyVal = pWlanPrivRecord->keyVal;
            /* Do not clear this entry! */

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            SET_KEY_TYPE(MAC_KEY_TYPE_TKIP);
            SET_UPDT_BF_CACHE_BIT(chainCtrl);

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

            /* Set "key is valid" bit */
            keyField = keyRegs[KEYREGS_INDEX(macAddrHi)].Value;
            ((MAC_ADDR_HI_KEY_VALID *)&keyField)->keyValid = TRUE;
            keyRegs[KEYREGS_INDEX(macAddrHi)].Value = keyField;

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; /* enmic key */

            pKeyVal = pWlanPrivRecord->micTxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* patch up the encrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);

            tkipIndex = keyCacheIndex + 32; /* decrypt key */
            pKeyVal = pWlanPrivRecord->keyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            SET_KEY_TYPE(MAC_KEY_TYPE_TKIP);
            SET_UPDT_BF_CACHE_BIT(chainCtrl);

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
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64 + 32; /* demic key */
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* patch up the decrypt key */
            writePlatformReg(pDev,MAC_KEY_CACHE +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             ((keyCacheIndex + 32) *
                              sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);
        } else {

            /* Group Key installation */

            tkipIndex = keyCacheIndex; /* decrypt key */
            pKeyVal = pWlanPrivRecord->keyVal;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            SET_KEY_TYPE(MAC_KEY_TYPE_TKIP);
            SET_UPDT_BF_CACHE_BIT(chainCtrl);

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
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            tkipIndex = keyCacheIndex + 64; /* demic key */
            pKeyVal = pWlanPrivRecord->micRxKeyVal;
            pDev->keyCache[tkipIndex] = NULL;

            A_MEM_ZERO(keyRegs, sizeof(keyRegs));

            keyRegs[0].Value = bytesToInt(pKeyVal, 0);
            keyRegs[2].Value = bytesToInt(pKeyVal, 4);

            /* Valid bit is zero */

            for (i = 0; i < (sizeof (keyRegs) / sizeof (REGISTER_VAL)); i++) {
                keyRegs[i].Offset = MAC_KEY_CACHE + (i*4) +
                    (tkipIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
                writePlatformReg(pDev, keyRegs[i].Offset, keyRegs[i].Value);
            }

            /* patch up the decrypt key */
            writePlatformReg(pDev, MAC_KEY_CACHE +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[0]);
            writePlatformReg(pDev, MAC_KEY_CACHE + 4 +
                             (keyCacheIndex * sizeof(AR5513_KEY_CACHE_ENTRY)),
                             savedKey[1]);
        }
    }

    return;
}

A_UINT16
ar5513KeyCacheAlloc(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex,
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
#ifdef AR5513_QOS
    int i;
#endif

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
            ar5513KeyCacheInvalidate(pdevInfo, hwIndex);
        }

        if (keyType == PRIV_KEY_TYPE_TKIP) {
            WLAN_PRIV_RECORD  keyInfo;
            /*
             * We Re-use the KeyCache Index, which was  added as WEP,  for TKIP.
             * Roll Back the Compression State of the previous index.
             */
            keyInfo.keyType = PRIV_KEY_TYPE_WEP;

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
         *
         * NOTE:  Subtract 1 for Beacon Antenna control entries.
         */
        indexLimit = A_MIN((A_UINT16)pdevInfo->keyCacheSize,
                         MAC_KEY_CACHE_SIZE / 4) - 1;
    } else {
        /*
         * Subtract 4 or 2 based on chainCtrl for 
         * Beacon Antenna control entries.
         */

        /* assumes txChainCtrl == rxChainCtrl */
        if (pdevInfo->staConfig.txChainCtrl == DUAL_CHAIN) {
            indexLimit = (A_UINT16)pdevInfo->keyCacheSize - 4;
        } else {
            indexLimit = (A_UINT16)pdevInfo->keyCacheSize - 2;	    
        }
    }

    for (hwIndex = MAX_SHARED_KEYS; hwIndex < indexLimit; hwIndex++) {
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
#ifdef AR5513_QOS
        } else if (pSibtmp->totalTxPending <= 0 &&
                   pSibtmp != pdevInfo->localSta)
#else
        } else if (pSibtmp->numTxPending <= 0 &&
                   pSibtmp != pdevInfo->localSta)
#endif
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
            uiPrintf("ar5513KeyCacheAlloc: didn't find a victim Sib\n");
        }
#endif
        return HWINDEX_INVALID;
    }

#ifdef DEBUG
    if (keyDebugLevel > 0) {
        uiPrintf("ar5513KeyCacheAlloc: victim Sib = 0x%x index=%d\n",
                 (A_UINT32)pVictimSib, pVictimSib->hwIndex);
    }
#endif

    /* Steal pSib's key cache entry */
    if (pVictimSib != pSib) {
        A_SIB_ENTRY_LOCK(pVictimSib);
    }
    hwIndex = pVictimSib->hwIndex;
    ASSERT(keyCacheSib[hwIndex] == pVictimSib);

    ar5513KeyCacheFree(pdevInfo, hwIndex);

    pdevInfo->keyCache[hwIndex]  = NULL;
    pVictimSib->hwIndex          = HWINDEX_INVALID;

    /*
     * Make sure that first frame will clear up any pending
     * retries from previous pSib
     */
#ifdef AR5513_QOS
    for (i=0; i < QUEUE_INDEX_MAX; i++) {
        pSib->swRetryInfo[i].needClearDest = TRUE;
    }
#else
    pSib->needClearDest = TRUE;
#endif

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
ar5513KeyCacheFree(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex)
{
    SIB_ENTRY *pSib;

    ar5513KeyCacheInvalidate(pdevInfo, hwIndex);
    pSib = pdevInfo->keyCacheSib[hwIndex];
    ASSERT(pSib != NULL);
    pdevInfo->keyCacheSib[hwIndex] = NULL;

    /*
     * shouldn't be freeing hwIndex if frames are pending! swretry
     * depends on the hwIndex of the tx key only
     */
#ifdef AR5513_QOS
    ASSERT(!pSib->totalTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->totalFilteredPending || pSib->hwIndex != hwIndex);
#else
    ASSERT(!pSib->numTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->numFilteredPending || pSib->hwIndex != hwIndex);
#endif
}

static void
ar5513KeyCacheInvalidate(WLAN_DEV_INFO *pdevInfo, A_UINT16 hwIndex)
{
    SIB_ENTRY *pSib;

    pSib = pdevInfo->keyCacheSib[hwIndex];
    ASSERT(pSib != NULL);
    /*
     * shouldn't be freeing hwIndex if frames are pending! swretry
     * depends on the hwIndex of the tx key only
     */
#ifdef AR5513_QOS
    ASSERT(!pSib->totalTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->totalFilteredPending || pSib->hwIndex != hwIndex);
#else
    ASSERT(!pSib->numTxPending || pSib->hwIndex != hwIndex);
    ASSERT(!pSib->numFilteredPending || pSib->hwIndex != hwIndex);
#endif

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
            ar5513ResetKeyCacheEntry(pdevInfo, hwIndex + 32);
        }
        if (pdevInfo->keyCacheSib[hwIndex + 64] == pSib) {
            pdevInfo->keyCacheSib[hwIndex + 64] = NULL;
            pdevInfo->keyCache[hwIndex + 64]    = NULL;
            ar5513ResetKeyCacheEntry(pdevInfo, hwIndex + 64);
        }
        if (pdevInfo->keyCacheSib[hwIndex + 32 + 64] == pSib) {
            pdevInfo->keyCacheSib[hwIndex + 32 + 64] = NULL;
            pdevInfo->keyCache[hwIndex + 32 + 64]    = NULL;
            ar5513ResetKeyCacheEntry(pdevInfo, hwIndex + 32 + 64);
        }
    }

    ar5513ResetKeyCacheEntry(pdevInfo, hwIndex);
}

/**************************************************************
 * ar5513SetDecompMask
 *
 * update decompression mask based on keyCache index
 */
A_UINT16
ar5513SetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIdx,
                    WLAN_PRIV_RECORD *pKey, A_BOOL en)
{
    return DECOMPINDEX_INVALID;
}


/**************************************************************
 * Diversity Routines
 *
 */

#if defined (LOG_ANT_DIVERSITY)
/* Log antenna processing */
A_UINT32    log_desc_rssi;
A_UINT32    log_reg_defant_in;
A_UINT32    log_reg_defant_out;
A_UINT32    log_reg_kc_in;
A_UINT32    log_reg_kc_out;
A_UINT32    log_var_quad_in;
A_UINT32    log_var_quad_out;
A_UINT32    log_var_daw_in;
A_UINT32    log_var_daw_out;

extern A_UINT32 log_defant_writes;
extern A_UINT32 log_war_count;

#include "apdefs.h"
#include "apcfg.h"
#include "wlanframe.h"

#endif /* LOG_ANT_DIVERSITY */

#if defined (LOG_ANT_DIVERSITY) || defined (E1_WAR_12564)
A_UINT32    log_desc_comb;
#endif

/*
 * Update Key Cache rxChain0Ack field.
 * HW updates txChain0Sel from this.
 */
static void
setRxChain0Ant(WLAN_DEV_INFO *pDev, A_UINT32 keyCacheOffset, A_UINT8 value)
{
    A_UINT32        savMaskReg;
    A_UINT32        keyField;

    savMaskReg = readPlatformReg(pDev, MAC_KC_MASK);
    keyField   = readPlatformReg(pDev, 
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));

    /* Mask unwanted writes */
    writePlatformReg(pDev, MAC_KC_MASK, ~MAC_KC_MASK_RX_CHAIN0_ACK);

    /* Update Key Cache rxChain0Ack field */
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->rxChain0Ack = value;
    writePlatformReg(pDev,
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),
            keyField);

    /* Restore mask */
    writePlatformReg(pDev, MAC_KC_MASK, savMaskReg);
}

/*
 * Update Key Cache rxChain1Ack field.
 * HW updates txChain1Sel from this.
 */
static void
setRxChain1Ant(WLAN_DEV_INFO *pDev, A_UINT32 keyCacheOffset, A_UINT8 value)
{
    A_UINT32        savMaskReg;
    A_UINT32        keyField;

    savMaskReg = readPlatformReg(pDev, MAC_KC_MASK);
    keyField   = readPlatformReg(pDev, 
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));

    /* Mask unwanted writes */
    writePlatformReg(pDev, MAC_KC_MASK, ~MAC_KC_MASK_RX_CHAIN1_ACK);

    /* Update Key Cache rxChain0Ack field */
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->rxChain1Ack = value;
    writePlatformReg(pDev,
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),
            keyField);

    /* Restore mask */
    writePlatformReg(pDev, MAC_KC_MASK, savMaskReg);
}


/*
 * Update Key Cache setStrongChain field.
 * This should in theory not be needed in for E2 silicon.
 */
static void
setStrongChain(WLAN_DEV_INFO *pDev, A_UINT32 keyCacheOffset, A_UINT8 value)
{
    A_UINT32        savMaskReg;
    A_UINT32        keyField;

    savMaskReg = readPlatformReg(pDev, MAC_KC_MASK);
    keyField   = readPlatformReg(pDev, 
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));

    /* Mask unwanted writes */
    writePlatformReg(pDev, MAC_KC_MASK, ~MAC_KC_MASK_CHAIN_SEL);

    /* Update Key Cache rxChain0Ack field */
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->txChainSel = value;
    writePlatformReg(pDev,
            keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),
            keyField);

    /* Restore mask */
    writePlatformReg(pDev, MAC_KC_MASK, savMaskReg);
}


#define CHAIN1REQ_M    1
#define CHAIN0REQ_M    2
#define CHAIN0REQ_S    1
#define CHAINSTRONG_M 16
#define CHAINSTRONG_S  4

#define NOT_EQ(_a,_b)   (MAKE_BOOL((!MAKE_BOOL((_a)) || !MAKE_BOOL((_b))) && (MAKE_BOOL((_a)) || MAKE_BOOL((_b)))))

/* Workaround for E1 silicon bugs that affect KC antenna updates */
#ifdef E1_WAR_12564
static int
forceWarUpdate(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, A_UINT32 req)
{
    A_UINT32        curDefAnt      = 0;
    A_UINT32        savDefAnt      = 0;
    A_UINT32        keyCacheOffset = 0xffffffff;    /* mark as not valid */

    A_UINT32        warFlags;
    int             retval         = 0;
    int             forceUpdate    = 0;
    WLAN_STA_CONFIG *pCfg          = &pDev->staConfig;

    warFlags = apCfgWarOptionGet();

    if (!(warFlags & WAR_DIVCHECK_LOG)) {
        return retval;
    }

    if (warFlags & WAR_DIVCHECK_DEFANT) {
        curDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
        savDefAnt = curDefAnt;
    }

    if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {
        keyCacheOffset = MAC_KEY_CACHE + (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));
    }

    if (pCfg->txChainCtrl == DUAL_CHAIN || pCfg->txChainCtrl == CHAIN_FIXED_B) {
        A_UINT8 chain1AntReq = (req & CHAIN1REQ_M);

        if (NOT_EQ(chain1AntReq, pDev->quadAnt.chain1Ant)) {

            /* force */
            forceUpdate |= 2;

            if (pDev->quadAnt.chain1Ant) {
                curDefAnt |= MAC_DEF_ANT_CHN1_ANT;
            } else {
                curDefAnt &= ~MAC_DEF_ANT_CHN1_ANT;
            }

            if (warFlags & WAR_DIVCHECK_KC && keyCacheOffset != 0xffffffff) {
                setRxChain1Ant(pDev, keyCacheOffset, pDev->quadAnt.chain1Ant);
                retval |= 2;
            }

        }
    }

    if (pCfg->txChainCtrl == DUAL_CHAIN || pCfg->txChainCtrl == CHAIN_FIXED_A) {
        A_UINT8 chain0AntReq = (req & CHAIN0REQ_M) >> CHAIN0REQ_S;

        if (NOT_EQ(chain0AntReq, pDev->quadAnt.chain0Ant)) {

            /* force */
            forceUpdate |= 1;

            if (pDev->quadAnt.chain0Ant) {
                curDefAnt |= MAC_DEF_ANT_CHN0_ANT;
            } else {
                curDefAnt &= ~MAC_DEF_ANT_CHN0_ANT;
            }

            if (warFlags & WAR_DIVCHECK_KC && keyCacheOffset != 0xffffffff) {
                setRxChain0Ant(pDev, keyCacheOffset, pDev->quadAnt.chain0Ant);
                retval |= 1;
            }
        }
    }

    /*
     * TODO: this is being removed for E1 and if it turns out to
     * be not needed for E2 than this blockl of code can be deleted.
     */
#ifdef EVALUATE_FOR_E2
    if (pCfg->chainCtrl == DUAL_CHAIN) {
        A_UINT8 chainStrong = (req & CHAINSTRONG_M) >> CHAINSTRONG_S;

        if (NOT_EQ(chainStrong, pDev->quadAnt.chainStrong)) {

            /* force */
            forceUpdate |= 1;

            if (pDev->quadAnt.chain0Ant) {
                curDefAnt |= MAC_DEF_ANT_CHN_SEL;
            } else {
                curDefAnt &= ~MAC_DEF_ANT_CHN_SEL;
            }

            if (warFlags & WAR_DIVCHECK_KC && keyCacheOffset != 0xffffffff) {
                setStrongChain(pDev, keyCacheOffset, pDev->quadAnt.chainStrong);
                retval |= 8;
            }
        }
    }
#endif

    if (forceUpdate) {
        if (warFlags & WAR_DIVCHECK_DEFANT) {
            ar5513SetDefAntenna(pDev, curDefAnt);
            if ((3 & savDefAnt) != (3 & curDefAnt)) {
                retval |= 4;
            }
            if ((4 & savDefAnt) != (4 & curDefAnt)) {
                retval |= 0x10;
            }
        }
        if (log_war_count < 0xffff) {
            log_war_count++;
        }
    }

    return retval;
}
#endif

#if defined(PT_2_PT_ANT_DIV) || !defined(BUILD_AP)
/* NDIS BUILD is always forced to be PT_2_PT */
#define  DB_SENSITIVITY    0    /* delta in rssi db to consider switch */
#define  DIV_SX_THRESHOLD  4    /* number of consecutive frames on other ant to switch */

/******************************************************************
 * ar5513ProcAntennaData
 *
 * Process Antenna data for TX Acks and RX data, i.e. best antennas
 * This routine is specific to a Point to Point Antenna Diversity
 * algorithm.
 */

void
ar5513ProcAntennaData (WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, 
                       ATHEROS_DESC *pDesc, A_UINT8 descType)
{
#if defined (LOG_ANT_DIVERSITY)
    WLAN_FRAME_HEADER   *pWlanHdr    = pDesc->pBufferVirtPtr.header;
#endif
#if defined (E1_WAR_12564)
    int                 warstat;    /* For E1 WAR */
#endif
    AR5513_TX_STATUS    *pTxStat;
    AR5513_RX_STATUS    *pRxStat;
    A_UINT32            keyCacheOffset;
    char                rssi0, rssi1, rssi2, rssi3;
    int                 rssiDiff;
    A_UINT32             newDefAnt;
    A_UINT8             strongChain;
    WLAN_STA_CONFIG     *pCfg = &pDev->staConfig;

#if defined (LOG_ANT_DIVERSITY)
    log_desc_rssi      = 0xffffffff;
    log_var_daw_in     = log_defant_writes;
    log_reg_defant_in  = readPlatformReg(pDev, MAC_DEF_ANTENNA);
    log_reg_defant_out = (((pDev->quadAnt.chainStrong & 1) << 7) | (pDev->quadAnt.divSxChnStrong & 0x0f)) << 16;

#endif

    if (descType == 0) {
#if defined (LOG_ANT_DIVERSITY)
        AR5513_TX_CONTROL *pTxControl = TX_CONTROL(pDesc->pTxFirstDesc);
#endif
        pTxStat = TX_STATUS(pDesc);

        if (pTxStat->pktTransmitOK == 1) {
            rssi0 = (char)pTxStat->ackRssiAnt0Chain0;
            rssi1 = (char)pTxStat->ackRssiAnt1Chain0;
            rssi2 = (char)pTxStat->ackRssiAnt0Chain1;
            rssi3 = (char)pTxStat->ackRssiAnt1Chain1;
#if defined (LOG_ANT_DIVERSITY)
            log_desc_rssi = pDesc->hw.word[6];
            log_reg_defant_in |= (pTxStat->txSeqNum & 0xfff) << 16;
#endif
#if defined (LOG_ANT_DIVERSITY) || defined (E1_WAR_12564)
            log_desc_comb = pTxStat->ackChain1AntReq |
                            pTxStat->ackChain0AntReq << 1 |
                            pTxStat->ackChain1AntSel << 2 |
                            pTxStat->ackChain0AntSel << 3 |
                            pTxStat->ackChainStrong << 4 |
                            pTxStat->beamFormEnabled << 5 |
                            pTxControl->bf_enable << 6 |
                            (pTxControl->TXRate0 & 0x1f) << 8 |
                            pTxStat->sendTimestamp << 16;
#endif
            strongChain = (char)pTxStat->ackChainStrong;
        } else {
            rssi0 = rssi1 = rssi2 = rssi3 = strongChain = 0;
            return;
        }
    } else {
        pRxStat = RX_STATUS(pDesc);

        if (pRxStat->pktReceivedOK == 1) {
            rssi0 = (char)pRxStat->rssiAnt0Chain0;
            rssi1 = (char)pRxStat->rssiAnt1Chain0;
            rssi2 = (char)pRxStat->rssiAnt0Chain1;
            rssi3 = (char)pRxStat->rssiAnt1Chain1;
#if defined (LOG_ANT_DIVERSITY)
            log_desc_rssi = pDesc->hw.word[2];
#endif
#if defined (LOG_ANT_DIVERSITY) || defined (E1_WAR_12564)
            log_desc_comb = pRxStat->chain1AntReq |
                            pRxStat->chain0AntReq << 1 |
                            pRxStat->chain1AntSel << 2 |
                            pRxStat->chain0AntSel << 3 |
                            pRxStat->chainStrong << 4 |
                            (pRxStat->rxRate & 0x1f) << 8 |
                            pRxStat->rxTimestamp << 16;
#endif
            strongChain = (char)pRxStat->chainStrong;
        } else {
            rssi0 = rssi1 = rssi2 = rssi3 = strongChain = 0;
            return;
        }
    }

#if defined (LOG_ANT_DIVERSITY)
    log_reg_kc_in = 0x55555555;
    if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {
        keyCacheOffset = MAC_KEY_CACHE + (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

        log_reg_kc_in = (0x8000 & readPlatformReg(pDev, keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, macAddrHi))) << 16;
        if (log_reg_kc_in) {
            log_reg_kc_in |= (0x00007e00 & readPlatformReg(pDev, keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType)));
        }
    }
    log_var_quad_in = ((((pDev->quadAnt.chain1Ant & 1) << 7) | (pDev->quadAnt.divSxChn1 & 0x0f)) << 8) |
                      (((pDev->quadAnt.chain0Ant & 1) << 7) | (pDev->quadAnt.divSxChn0 & 0x0f));
#endif

    /* TODO: For E1 testing, watch for KeyCache to get out of sync */
#if defined (E1_WAR_12564)
    warstat = forceWarUpdate(pDev, pSib, log_desc_comb);
#endif

    /* assumes rxChainCtrl == txChainCtrl */
    switch (pCfg->rxChainCtrl) {
    case DUAL_CHAIN:
    case CHAIN_FIXED_B:
        /* Do not update if rssi is reported as invalid */
        if ((unsigned char)rssi3 != 0x80 && (unsigned char)rssi2 != 0x80) {

            /*
             *  Compute difference between Antenna 1 and 0 on Chain 1
             */
            rssiDiff = rssi3 - rssi2;

            /*
             * If less than relative dB sensitivity, meaning Antenna 0 is
             * stronger
             */
            if (rssiDiff < -DB_SENSITIVITY) {
                /* Is RX Ack Antenna for this station currently set to Ant 1 */
                if (pDev->quadAnt.chain1Ant == 1) {
                    /* Increment diversity switch count for Chain 1 */
                    if (++pDev->quadAnt.divSxChn1 >= DIV_SX_THRESHOLD) {

                        newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                        newDefAnt &= ~MAC_DEF_ANT_CHN1_ANT;
                        ar5513SetDefAntenna(pDev, newDefAnt);

                        /*
                         * Update RX ack antenna in Key Cache for this
                         * station
                         */
                        pDev->quadAnt.chain1Ant = 0;

                        if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {

                            keyCacheOffset = MAC_KEY_CACHE + 
                                (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

                            /* Update Key Cache Entry */
                            setRxChain1Ant(pDev, keyCacheOffset, pDev->quadAnt.chain1Ant);
                        }

                        pDev->quadAnt.divSxChn1 = 0;
                    }
                } else {
                    pDev->quadAnt.divSxChn1 = 0;
                }
            } else {
                /*
                 *  If greater than relative dB sensitivity, meaning 
                 *  Antenna 1 is stronger than Antenna 0
                 */
                if (rssiDiff > DB_SENSITIVITY) {
                /* Is RX Ack Antenna for this Station set to Ant 0 */
                    if (pDev->quadAnt.chain1Ant == 0) {
                        if (++pDev->quadAnt.divSxChn1 >= DIV_SX_THRESHOLD) {

                            newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                            newDefAnt |= MAC_DEF_ANT_CHN1_ANT;
                            ar5513SetDefAntenna(pDev, newDefAnt);

                            pDev->quadAnt.chain1Ant = 1;

                            if (pSib != NULL &&
                                pSib->hwIndex != HWINDEX_INVALID)
                            {
                                keyCacheOffset = MAC_KEY_CACHE + 
                                        (pSib->hwIndex * 
                                        sizeof(AR5513_KEY_CACHE_ENTRY));

                                /* Update Key Cache Entry */
                                setRxChain1Ant(pDev, keyCacheOffset, pDev->quadAnt.chain1Ant);
                            }

                            pDev->quadAnt.divSxChn1 = 0;
                        }
                    } else {
                        pDev->quadAnt.divSxChn1 = 0;
                    }
                }
            }
            if (pCfg->rxChainCtrl == CHAIN_FIXED_B) {
                break;
#if 0           // DF_MERGE old loc, moved due to split chain ctrl
            } else {
                /* 
                 *  Find strongest chain
                 */
                if (strongChain != pDev->quadAnt.chainStrong) {
                    if (++pDev->quadAnt.divSxChnStrong >= DIV_SX_THRESHOLD) {

                        newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                        if (strongChain == 1) {
                            newDefAnt |= MAC_DEF_ANT_CHN_SEL;
                        } else {
                            newDefAnt &= ~MAC_DEF_ANT_CHN_SEL;
                        }
                        ar5513SetDefAntenna(pDev, newDefAnt);

                        pDev->quadAnt.chainStrong = strongChain;

                        /*
                         * Update strong chain in Key Cache for this 
                         * station
                         */
                        if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {

                            keyCacheOffset = MAC_KEY_CACHE + 
                                (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

                            /* Update Key Cache Entry */
                            setStrongChain(pDev, keyCacheOffset, pDev->quadAnt.chainStrong);
                        }

                        pDev->quadAnt.divSxChnStrong = 0;
                    
                    } else {
                        pDev->quadAnt.divSxChnStrong = 0;
                    }
                }
#endif // DF_MERGE old loc
            }
        }


    case CHAIN_FIXED_A:
    default:
        /* Do not update if rssi is reported as invalid */
        if ((unsigned char)rssi1 != 0x80 && (unsigned char)rssi0 != 0x80) {

            /*
             *  Compute difference between Antenna 1 and 0 on Chain 0
             */
            rssiDiff = rssi1 - rssi0;

            /*
             * If less than relative dB sensitivity, meaning Antenna 0 is 
             * stronger than antenna 1
             */
            if (rssiDiff < -DB_SENSITIVITY) {
                /* Is RX Ack Antenna for this station currently set to Ant 1 */
                if (pDev->quadAnt.chain0Ant == 1) {
                    /* Increment diversity switch count for Chain 0 */
                    if (++pDev->quadAnt.divSxChn0 >= DIV_SX_THRESHOLD) {

                        newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                        newDefAnt &= ~MAC_DEF_ANT_CHN0_ANT;
                        ar5513SetDefAntenna(pDev, newDefAnt);

                        /*
                        ** Update RX ack antenna in Key Cache for this 
                        ** station
                        */
                        pDev->quadAnt.chain0Ant = 0;

                        if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {
                            keyCacheOffset = MAC_KEY_CACHE + 
                                            (pSib->hwIndex * 
                                            sizeof(AR5513_KEY_CACHE_ENTRY));

                            /* Update Key Cache Entry */
                            setRxChain0Ant(pDev, keyCacheOffset, pDev->quadAnt.chain0Ant);
                        }

                        pDev->quadAnt.divSxChn0 = 0;
                    }
                } else {
                    pDev->quadAnt.divSxChn0 = 0;
                }
            } else {
                /*
                 *  If greater than relative dB sensitivity, meaning 
                 *  Antenna 1 is stronger than Antenna 0
                 */
                if (rssiDiff > DB_SENSITIVITY) {
                    /* Is RX Ack Antenna for this Station set to Ant 0 */
                    if (pDev->quadAnt.chain0Ant == 0) {
                        if (++pDev->quadAnt.divSxChn0 >= DIV_SX_THRESHOLD) {
                            newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                            newDefAnt |= MAC_DEF_ANT_CHN0_ANT;
                            ar5513SetDefAntenna(pDev, newDefAnt);

                            pDev->quadAnt.chain0Ant = 1;

                            if (pSib != NULL &&
                                pSib->hwIndex != HWINDEX_INVALID)
                            {
                                keyCacheOffset = MAC_KEY_CACHE + 
                                                (pSib->hwIndex * 
                                                sizeof(AR5513_KEY_CACHE_ENTRY));

                                /* Update Key Cache Entry */
                                setRxChain0Ant(pDev, keyCacheOffset, pDev->quadAnt.chain0Ant);
                            }

                            pDev->quadAnt.divSxChn0 = 0;
                        }
                    } else {
                        pDev->quadAnt.divSxChn0 = 0;
                    }
                }
            }
        }

        break;
    }
#if 1 // DF_MERGE new loc, depends on txChainCtrl
    if (pCfg->txChainCtrl == DUAL_CHAIN ) {
        /* 
         *  Find strongest chain
         */
        if (strongChain != pDev->quadAnt.chainStrong) {
            if (++pDev->quadAnt.divSxChnStrong >= DIV_SX_THRESHOLD) {

                newDefAnt = readPlatformReg(pDev, MAC_DEF_ANTENNA);
                if (strongChain == 1) {
                    newDefAnt |= MAC_DEF_ANT_CHN_SEL;
                } else {
                    newDefAnt &= ~MAC_DEF_ANT_CHN_SEL;
                }
                ar5513SetDefAntenna(pDev, newDefAnt);

                pDev->quadAnt.chainStrong = strongChain;

                /*
                 * Update strong chain in Key Cache for this 
                 * station
                 */
                if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {

                    keyCacheOffset = MAC_KEY_CACHE + 
                        (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

                    /* Update Key Cache Entry */
                    setStrongChain(pDev, keyCacheOffset, pDev->quadAnt.chainStrong);
                }
                pDev->quadAnt.divSxChnStrong = 0;
            } else {
                pDev->quadAnt.divSxChnStrong = 0;
            }
        }
    }
#endif // DF_MERGE new loc

#if defined (LOG_ANT_DIVERSITY)
    log_reg_defant_out |= 0x07 & readPlatformReg(pDev, MAC_DEF_ANTENNA);
    log_reg_defant_out |= ((((pDev->quadAnt.chainStrong & 1) << 7) | (pDev->quadAnt.divSxChnStrong & 0x0f)) << 8);
    log_reg_kc_out = 0xaaaaaaaa;
    if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID) {
        keyCacheOffset = MAC_KEY_CACHE + (pSib->hwIndex * sizeof(AR5513_KEY_CACHE_ENTRY));

        log_reg_kc_out = (0x8000 & readPlatformReg(pDev, keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, macAddrHi))) << 16;
        if (log_reg_kc_out) {
            log_reg_kc_out |= (0x00007e00 & readPlatformReg(pDev, keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType)));
        }
    }
    log_var_quad_out = ((((pDev->quadAnt.chain1Ant & 1) << 7) | (pDev->quadAnt.divSxChn1 & 0x0f)) << 8) |
                        (((pDev->quadAnt.chain0Ant & 1) << 7) | (pDev->quadAnt.divSxChn0 & 0x0f));

    log_var_daw_out = ((log_var_daw_in & 0x0f) << 4) | (log_defant_writes & 0x0f);
    log_defant_writes = 0;

    ar5513DivLogAdd(descType |
                        (isGrp(&pWlanHdr->address1) ? 2 : 0) |
                        (pSib == 0 ? 4 : 0) |
                        log_var_daw_out << 8 |
                        (log_war_count & 0xffff) << 16,
                    log_desc_rssi,
                    log_desc_comb,
                    log_reg_defant_in,
#if defined (E1_WAR_12564)
                    log_reg_defant_out | (warstat << 28),
#else
                    log_reg_defant_out,
#endif
                    log_reg_kc_in,
                    log_reg_kc_out,
                    (log_var_quad_in << 16) | log_var_quad_out);
#endif /* LOG_ANT_DIVERSITY */

    return;
}

#else /* ! PT_2_PT_ANT_DIV */
/* This routine is not tested or used yet */
/******************************************************************
 * ar5513ProcAckAntennaData
 *
 * Process Antenna data for TX Acks, i.e. best antennas
 */

void
ar5513ProcAckAntennaData (WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, 
			  ATHEROS_DESC *pTxDesc )
{
    A_UINT32 keyField;
    A_UINT32 keyCacheOffset;
    REGISTER_VAL keyRegs[8];
    A_INT32  rssiDiff;
    AR5513_TX_STATUS *pStat = TX_STATUS(pTxDesc);
    WLAN_STA_CONFIG *pCfg = &pDev->staConfig;

    /* LW
     * The following code will corrupt keycache, temporary disable it
     */
    return;

#define  DB_SENSITIVITY  0
#define  DIV_SX_THRESHOLD  2

//  Update Key Cache rxChain0Ack field
#define SET_RX_CHAIN0_ANT(value)                                        \
    keyRegs[KEYREGS_INDEX(keyType)].Value = readPlatformReg(pDev,       \
	keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));\
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;                   \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->rxChain0Ack = value;         \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;                   \
    writePlatformReg(pDev, keyCacheOffset +                             \
		     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),     \
		     keyRegs[KEYREGS_INDEX(keyType)].Value);

//  Update Key Cache rxChain1Ack field
#define SET_RX_CHAIN1_ANT(value)                                        \
    keyRegs[KEYREGS_INDEX(keyType)].Value = readPlatformReg(pDev,       \
	keyCacheOffset + FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));\
    keyField = keyRegs[KEYREGS_INDEX(keyType)].Value;                   \
    ((DUAL_CHAIN_BF_KEY_TYPE *)&keyField)->rxChain1Ack = value;         \
    keyRegs[KEYREGS_INDEX(keyType)].Value = keyField;                   \
    writePlatformReg(pDev, keyCacheOffset +                             \
		     FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType),     \
		     keyRegs[KEYREGS_INDEX(keyType)].Value);

    if ( pStat->pktTransmitOK ) {
	if (pSib != NULL && pSib->hwIndex != HWINDEX_INVALID ) {
	    switch (pCfg->rxChainCtrl) {
	    case DUAL_CHAIN:
	    case CHAIN_FIXED_B:
		/*
		**  Compute difference between Antenna 1 and 0 on Chain 1
		*/
		rssiDiff = pStat->ackRssiAnt1Chain1 - pStat->ackRssiAnt0Chain1;

		/*
		** If less than relative dB sensitivity, meaning Antenna 0 is 
		** stronger
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    /* Is RX Ack Antenna for this station currently set to Ant 1 */
		    if (pSib->quadAnt.antChain1 == 1) {
			/* Increment diversity switch count for Chain 1 */
			if (++pSib->quadAnt.divSxChn1 >= DIV_SX_THRESHOLD) {
			    /*
			    ** Update RX ack antenna in Key Cache for this 
			    ** station
			    */
			    keyCacheOffset = MAC_KEY_CACHE + (pSib->hwIndex * 
				      sizeof(AR5513_KEY_CACHE_ENTRY));
			    pSib->quadAnt.antChain1 = 0;
			    /* Update Key Cache Entry */
			    SET_RX_CHAIN1_ANT(pSib->quadAnt.antChain1);
			    pSib->quadAnt.divSxChn1 = 0;
			}
		    }
		    else {
			pSib->quadAnt.divSxChn1 = 0;
		    }
		}
		else {
		    /*
		    **  If greater than relative dB sensitivity, meaning 
		    **  Antenna 1 is stronger than Antenna 0
		    */
		    if (rssiDiff > DB_SENSITIVITY) {
			/* Is RX Ack Antenna for this Station set to Ant 0 */
			if (pSib->quadAnt.antChain1 == 0) {
			    if (++pSib->quadAnt.divSxChn1 >= 
				                      DIV_SX_THRESHOLD) {
				keyCacheOffset = MAC_KEY_CACHE + 
				    (pSib->hwIndex * 
				     sizeof(AR5513_KEY_CACHE_ENTRY));
				pSib->quadAnt.antChain1 = 1;
				/* Update Key Cache Entry */
				SET_RX_CHAIN1_ANT(pSib->quadAnt.antChain1);
				pSib->quadAnt.divSxChn1 = 0;
			    }
			}
			else {
			    pSib->quadAnt.divSxChn1 = 0;
			}
		    }
		}

		if (pCfg->rxChainCtrl == CHAIN_FIXED_B) {
		    break;
		}

	    case CHAIN_FIXED_A:
	    default:
		/*
		**  Compute difference between Antenna 1 and 0 on Chain 0
		*/
		rssiDiff = pStat->ackRssiAnt1Chain0 - pStat->ackRssiAnt0Chain0;

		/*
		** If less than relative dB sensitivity, meaning Antenna 0 is 
		** stronger than antenna 1
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    /* Is RX Ack Antenna for this station currently set to Ant 1 */
		    if (pSib->quadAnt.antChain0 == 1) {
			/* Increment diversity switch count for Chain 0 */
			if (++pSib->quadAnt.divSxChn0 >= DIV_SX_THRESHOLD) {
			    /*
			    ** Update RX ack antenna in Key Cache for this 
			    ** station
			    */
			    keyCacheOffset = MAC_KEY_CACHE + (pSib->hwIndex * 
				      sizeof(AR5513_KEY_CACHE_ENTRY));
			    pSib->quadAnt.antChain0 = 0;
			    /* Update Key Cache Entry */
			    SET_RX_CHAIN0_ANT(pSib->quadAnt.antChain0);
			    pSib->quadAnt.divSxChn0 = 0;
			}
		    }
		    else {
			pSib->quadAnt.divSxChn0 = 0;
		    }
		}
		else {
		    /*
		    **  If greater than relative dB sensitivity, meaning 
		    **  Antenna 1 is stronger than Antenna 0
		    */
		    if (rssiDiff > DB_SENSITIVITY) {
			/* Is RX Ack Antenna for this Station set to Ant 0 */
			if (pSib->quadAnt.antChain0 == 0) {
			    if (++pSib->quadAnt.divSxChn0 >= 
				                          DIV_SX_THRESHOLD) {
				keyCacheOffset = MAC_KEY_CACHE + 
				    (pSib->hwIndex * 
				      sizeof(AR5513_KEY_CACHE_ENTRY));
				pSib->quadAnt.antChain0 = 1;
				/* Update Key Cache Entry */
				SET_RX_CHAIN0_ANT(pSib->quadAnt.antChain0);
				pSib->quadAnt.divSxChn0 = 0;
			    }
			}
			else {
			    pSib->quadAnt.divSxChn0 = 0;
			}
		    }
		}
		break;
	    }
	}
    }

    return;
}

/* This routine is not tested or used yet */
/******************************************************************
 * ar5513ProcRxAntennaData
 *
 * Process Antenna data for RX data, i.e. best antennas
 */

void
ar5513ProcRxAntennaData (WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, 
			  ATHEROS_DESC *pRxDesc )
{
    int i, j, sum, avg[4];
    A_INT32 rssiDiff;
    A_UINT32 newDefAnt;
    AR5513_RX_STATUS *pStat = RX_STATUS(pRxDesc);
    WLAN_STA_CONFIG *pCfg = &pDev->staConfig;
    struct dualChainAntCtrl *pAnt = &pDev->staConfig.quadAnt;

    /* LW
     * The following code will corrupt keycache, temporary disable it
     */
    return;

#define  DB_SENSITIVITY  0
  
    if (pStat->pktReceivedOK == 1) {
	/*
	** Collect RX data RSSI samples until buffer is full
	*/
	if (pAnt->idxCnt < CHAIN_ANT_ENTRIES) {
	    switch (pCfg->rxChainCtrl) {
	    case DUAL_CHAIN:
		pAnt->chain0[0][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt0Chain0;
		pAnt->chain0[1][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt1Chain0;
		pAnt->chain1[0][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt0Chain1;
		pAnt->chain1[1][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt1Chain1;
		break;
	    case CHAIN_FIXED_B:
		pAnt->chain1[0][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt0Chain1;
		pAnt->chain1[1][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt1Chain1;
		break;
	    case CHAIN_FIXED_A:
		pAnt->chain0[0][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt0Chain0;
		pAnt->chain0[1][pAnt->idxCnt] = (A_UINT8) pStat->rssiAnt1Chain0;
		break;
	    }

	    ++pAnt->idxCnt;
	}

	/*
	**  If log buffer is full, then analyze RSSI data
	**  in order to determine best Default Antenna
	*/

	if (pAnt->idxCnt >= CHAIN_ANT_ENTRIES) {

	    pAnt->idxCnt = 0;

	    newDefAnt = readPlatformReg(pDev,MAC_DEF_ANTENNA);

	    switch (pCfg->rxChainCtrl) {

	    case DUAL_CHAIN:

		for (i=0, sum=0; i < 4; i++) {
		    for (j=0, sum=0; j < CHAIN_ANT_ENTRIES; j++) {
			switch (i) {
			case 0:
			    sum += pAnt->chain0[0][j];
			    break;
			case 1:
			    sum += pAnt->chain0[1][j];
			    break;
			case 2:
			    sum += pAnt->chain1[0][j];
			    break;
			case 3:
			    sum += pAnt->chain1[1][j];
			    break;
			}

			avg[i] = sum >> CHAIN_ANT_ENT_SHIFT;
		    }
		}

		/*
		**  Antenna 1 minus Antenna 0 for Chain 0
		*/
		rssiDiff = avg[1] - avg[0]; 

		/*
		** Is Antenna 0 stronger than Ant 1
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    if (pAnt->chain0Ant == 1) {
			pAnt->chain0Ant = 0;
			newDefAnt &= ~MAC_DEF_ANT_CHN0_ANT;
		    }
		}
		else {  /* Ant 1 maybe stronger */
		    if (rssiDiff > DB_SENSITIVITY) {
			if (pAnt->chain0Ant == 0) {
			    pAnt->chain0Ant = 1;
			    newDefAnt |= MAC_DEF_ANT_CHN0_ANT;
			}
		    }
		}

		/*
		** Antenna 1 minus Ant 0 for Chain 1
		*/

		rssiDiff = avg[3] - avg[2];

		/*
		** Is Antenna 0 stronger than Ant 1
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    if (pAnt->chain1Ant == 1) {
			pAnt->chain1Ant = 0;
			newDefAnt &= ~MAC_DEF_ANT_CHN1_ANT;
		    }
		}
		else {  /* Ant 1 maybe stronger */
		    if (rssiDiff > DB_SENSITIVITY) {
			if (pAnt->chain1Ant == 0) {
			    pAnt->chain1Ant = 1;
			    newDefAnt |= MAC_DEF_ANT_CHN1_ANT;
			}
		    }
		}

		/*
		**  Find strongest chain
		*/
		for (i=0, sum=0; i < CHAIN_ANT_ENTRIES; i++) {
		    sum += pAnt->chainStrg[i];
		}

		/*
		** If sum is greater than entries divided by 2
		*/

		if (sum > (CHAIN_ANT_ENTRIES >> 1)) {
		    newDefAnt |= MAC_DEF_ANT_CHN_SEL;
		}
		else {
		    newDefAnt &= ~MAC_DEF_ANT_CHN_SEL;
		}
		break;

	    case CHAIN_FIXED_B:

		for (i=0, sum=0; i < 2; i++) {
		    for (j=0, sum=0; j < CHAIN_ANT_ENTRIES; j++) {
			switch (i) {
			case 0:
			    sum += pAnt->chain1[0][j];
			    break;
			case 1:
			    sum += pAnt->chain1[1][j];
			    break;
			}

			avg[i] = sum >> CHAIN_ANT_ENT_SHIFT;
		    }
		}

		/*
		** Antenna 1 minus Ant 0 for Chain 1
		*/

		rssiDiff = avg[1] - avg[0];

		/*
		** Is Antenna 0 stronger than Ant 1
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    if (pAnt->chain1Ant == 1) {
			pAnt->chain1Ant = 0;
			newDefAnt &= ~MAC_DEF_ANT_CHN1_ANT;
		    }
		}
		else {  /* Ant 1 maybe stronger */
		    if (rssiDiff > DB_SENSITIVITY) {
			if (pAnt->chain1Ant == 0) {
			    pAnt->chain1Ant = 1;
			    newDefAnt |= MAC_DEF_ANT_CHN1_ANT;
			}
		    }
		}
		break;

	    case CHAIN_FIXED_A:

		for (i=0, sum=0; i < 2; i++) {
		    for (j=0, sum=0; j < CHAIN_ANT_ENTRIES; j++) {
			switch (i) {
			case 0:
			    sum += pAnt->chain0[0][j];
			    break;
			case 1:
			    sum += pAnt->chain0[1][j];
			    break;
			}

			avg[i] = sum >> CHAIN_ANT_ENT_SHIFT;
		    }
		}

		/*
		**  Antenna 1 minus Antenna 0 for Chain 0
		*/
		rssiDiff = avg[1] - avg[0]; 

		/*
		** Is Antenna 0 stronger than Ant 1
		*/
		if (rssiDiff < -DB_SENSITIVITY) {
		    if (pAnt->chain0Ant == 1) {
			pAnt->chain0Ant = 0;
			newDefAnt &= ~MAC_DEF_ANT_CHN0_ANT;
		    }
		}
		else {  /* Ant 1 maybe stronger */
		    if (rssiDiff > DB_SENSITIVITY) {
			if (pAnt->chain0Ant == 0) {
			    pAnt->chain0Ant = 1;
			    newDefAnt |= MAC_DEF_ANT_CHN0_ANT;
			}
		    }
		}
		break;
	    }

	    writePlatformReg(pDev,MAC_DEF_ANTENNA,newDefAnt);	    
	}
    }

    return;
}

#endif /* ! PT_2_PT_ANT_DIV */

#if defined(DEBUG) || defined(_DEBUG)

const char *
ar5513DebugGetKeyType(WLAN_DEV_INFO *pDev, A_UINT32 idx)
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

    offset = MAC_KEY_CACHE + idx * sizeof(AR5513_KEY_CACHE_ENTRY);
    keyType = readPlatformReg(pDev, offset +
                              FIELD_OFFSET(AR5513_KEY_CACHE_ENTRY, keyType));
    return keyTypeToName[keyType & 0x07];
}

/*
 * Temporary code to debug Keycache issues
 */
#define MAX_DUMP_KC 8
unsigned int dumpKCarray[8 * MAX_DUMP_KC];

void dumpkc(void)
{
    int ii;
    WLAN_DEV_INFO *pDevWLan;
    int lock;
    unsigned int x;
    int carry;

    pDevWLan = gDrvInfo.pDev[0];

    lock = intLock();
    for (ii = 0; ii < 8 * MAX_DUMP_KC; ii++) {
        dumpKCarray[ii] = readPlatformReg(pDevWLan, 0x8800 + 4 * ii);
    }
    intUnlock(lock);

    for (ii = 0; ii < MAX_DUMP_KC; ii++) {

        uiPrintf("key: ");
        x = dumpKCarray[8 * ii + 0];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        x = dumpKCarray[8 * ii + 1];
        uiPrintf("%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff);
        x = dumpKCarray[8 * ii + 2];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        x = dumpKCarray[8 * ii + 3];
        uiPrintf("%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff);
        x = dumpKCarray[8 * ii + 4];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        uiPrintf("\n");

        x = dumpKCarray[8 * ii + 5];
        uiPrintf("Loc: %3d, type %2d, UBFC %1.1d\n",
            ii, x & 7, x & 0x200 ? 1 : 0);

        x = dumpKCarray[8 * ii + 6];
        carry = x & 0x80000000;
        uiPrintf("mac: %2.2x:%2.2x:%2.2x:%2.2x:",
            (x << 1) & 0xff,
            (x >> 7) & 0xff,
            (x >> 15) & 0xff,
            (x >> 23) & 0xff);
        x = dumpKCarray[8 * ii + 7];
        uiPrintf("%2.2x:%2.2x, VAL %1.1d\n",
            ((x << 1) | carry) & 0xff,
            (x >> 7) & 0xff,
            (x & 0x8000) ? 1 : 0);
    }
}

#define MAX_LOG_KC 20
int logkcentry = 0;
unsigned int logKcArray[MAX_LOG_KC][8];

void
logKC(int idx)
{
    int ii;
    WLAN_DEV_INFO *pDevWLan;
    int lock;

    if (logkcentry >= MAX_LOG_KC) {
        return;
    }

    pDevWLan = gDrvInfo.pDev[0];

    lock = intLock();
    for (ii = 0; ii < 8; ii++) {
        logKcArray[logkcentry][ii] = 
            readPlatformReg(pDevWLan, 0x8800 + 32 * idx + 4 * ii);
    }
    logKcArray[logkcentry][1] &= 0x0000ffff;
    logKcArray[logkcentry][1] |= (idx & 0xff) << 24;
    logkcentry++;
    intUnlock(lock);
}

void
dumplogkc(void)
{
    int jj;
    unsigned int x;
    int carry;
    int loc;

    uiPrintf("Logged %d\n", logkcentry);
    for (jj = 0; jj < logkcentry; jj++) {
        uiPrintf("%d\n", jj);

        uiPrintf("key: ");
        x = logKcArray[jj][0];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        x = logKcArray[jj][1];
        uiPrintf("%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff);
        loc = (x >> 24) & 0xff;
        x = logKcArray[jj][2];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        x = logKcArray[jj][3];
        uiPrintf("%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff);
        x = logKcArray[jj][4];
        uiPrintf("%2.2x%2.2x%2.2x%2.2x",
            x & 0xff,
            (x >> 8) & 0xff,
            (x >> 16) & 0xff,
            (x >> 24) & 0xff);
        uiPrintf("\n");

        x = logKcArray[jj][5];
        uiPrintf("Loc: %3d, type %2d, UBFC %1.1d\n",
            loc, x & 7, x & 0x200 ? 1 : 0);

        x = logKcArray[jj][6];
        carry = x & 0x80000000;
        uiPrintf("mac: %2.2x:%2.2x:%2.2x:%2.2x:",
            (x << 1) & 0xff,
            (x >> 7) & 0xff,
            (x >> 15) & 0xff,
            (x >> 23) & 0xff);
        x = logKcArray[jj][7];
        uiPrintf("%2.2x:%2.2x, VAL %1.1d\n",
            ((x << 1) | carry) & 0xff,
            (x >> 7) & 0xff,
            (x & 0x8000) ? 1 : 0);
    }
}

#endif
#endif // #ifdef BUILD_AR5513

