/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips specific device attachment and device info collection
 *  Connects Init Reg Vectors, EEPROM Data, and device Functions to pDev
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Attach.c#5 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanPhy.h"
#include "wlanchannel.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"
#include "halDevId.h"

/* Headers for HW private items */
#include "ar5211/ar5211Reg.h"
#include "ar5211/ar5211Attach.h"
#include "ar5211/ar5211Phy.h"
#include "ar5211/ar5211.h"

/* Header files exporting vector mapped functions */
#include "ar5211/ar5211Reset.h"
#include "ar5211/ar5211Misc.h"
#include "ar5211/ar5211KeyCache.h"
#include "ar5211/ar5211Power.h"
#include "ar5211/ar5211Beacon.h"
#include "ar5211/ar5211Transmit.h"
#include "ar5211/ar5211Receive.h"
#include "ar5211/ar5211Interrupts.h"

#if defined(AR531X)
#include "ar531xreg.h"
#endif

/* Forward Declarations */
static A_STATUS
ar5211Detach(WLAN_DEV_INFO *pDev);

static A_STATUS
ar5211FillCapabilityInfo(WLAN_DEV_INFO *pDev);

A_STATUS
ar5211VerifyEepromChecksum(WLAN_DEV_INFO *pDev);

static A_STATUS
ar5211AllocateProm(EEP_MAP *pEepMap);

static A_STATUS
ar5211ReadEepromIntoDataset(WLAN_DEV_INFO *pDev);

static A_STATUS
ar5211ReadHeaderInfo(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo);

static void
getPcdacInterceptsFromPcdacMinMax(A_UINT16 version, A_UINT16 pcdacMin,
    A_UINT16 pcdacMax, A_UINT16 *pPcdacValues);

static A_UINT16
fbin2freq(A_UINT16 version, A_UINT16 fbin);

static A_UINT16
fbin2freq_2p4(A_UINT16 version, A_UINT16 fbin);

/* Constants */
static const HW_FUNCS ar5211Funcs = {
    ar5211Detach,
    ar5211FillCapabilityInfo,

    /* Reset Functions */
    ar5211Reset,
    ar5211PhyDisable,
    ar5211Disable,
    ar5211PerCalibration,
    ar5211GetRfgain,
    ar5211SetTxPowerLimit,

    /* TX Functions */
    ar5211UpdateTxTrigLevel,
    ar5211SetupTxQueue,
    ar5211ReleaseTxQueue,
    ar5211GetTxDP,
    ar5211SetTxDP,
    ar5211StartTxDma,
    ar5211NumTxPending,
    ar5211StopTxDma,
    ar5211PauseTx,

    ar5211SetupTxDesc,
    ar5211ProcessTxDesc,
    ar5211GetTxDescDone,
#if defined(DEBUG) || defined(_DEBUG)
    ar5211DebugPrintTxDesc,
#endif

    /* RX Functions */
    ar5211GetRxDP,
    ar5211SetRxDP,
    ar5211EnableReceive,
    ar5211StopDmaReceive,
    ar5211StartPcuReceive,
    ar5211StopPcuReceive,
    ar5211SetMulticastFilter,
    ar5211MulticastFilterIndex,
    ar5211RxFilter,
    ar5211ProcessRxDesc,
#ifdef UPSD
    NULL,
#endif
    ar5211SetupRxDesc,

    /* Misc Functions */
    ar5211SetRegulatoryDomain,
    ar5211SetLedState,
    ar5211WriteAssocid,
    ar5211GpioCfgInput,
    ar5211GpioCfgOutput,
    ar5211GpioGet,
    ar5211GpioSet,
    ar5211GpioSetIntr,
    ar5211SetStaBeaconTimers,
    ar5211GetTsf,
    ar5211ResetTsf,
    ar5211SetAdhocMode,
    ar5211SetBasicRate,    
    ar5211GetRandomSeed,
    ar5211DetectCardPresent,
    ar5211MibControl,
    ar5211GetChannelData,
    ar5211ProcessNoiseFloor,
    ar5211EnableRadarDetection,
    NULL,    /* placeholder for ANI function hwGetListenTime */
    NULL,
    NULL,
    NULL,
    ar5211GetCurRssi,
    ar5211GetDefAntenna,
    ar5211SetDefAntenna,
    ar5211SetAntennaSwitch,
    ar5211UpdateAntenna,
    ar5211UseShortSlotTime,
    ar5211DmaDebugDump,
    ar5211GetMacTimer3,
    NULL,
    NULL,
    ar5211GetHwDescWord,
    ar5211GetApSwitchHelper,
    ar5211SetApSwitchHelper,
    NULL,			/* hwSendXrChirp */
    NULL,                       /* hwMonitorRxTxActivity */

    /* Key Cache Functions */
    ar5211ReserveHalKeyCacheEntries,
    ar5211ResetKeyCacheEntry,
    ar5211SetKeyCacheEntry,
    ar5211KeyCacheAlloc,
    ar5211KeyCacheFree,

    /* Power Management Functions */
    ar5211SetPowerMode,
    ar5211GetPowerMode,
    ar5211GetPowerStatus,
    ar5211SetupPsPollDesc,

    /* Beacon Functions */
    ar5211SetupBeaconDesc,
    ar5211BeaconInit,
    NULL,

    /* Interrupt Functions */
    ar5211IsInterruptPending,
    ar5211GetInterrupts,
    ar5211EnableInterrupts,
    ar5211DisableInterrupts,
#ifdef UPSD
    NULL,
#endif
};

static const A_UINT16 channels2_4[] = {2412, 2447, 2484};


/**************************************************************
 * ar5211Attach
 *
 * ar5211Attach performs the device specific functions for halAttach()
 */
A_STATUS
ar5211Attach(WLAN_DEV_INFO *pDev, A_UINT16 deviceID)
{
    struct gainValues *pGainValues;
    EEP_MAP  *pEMap;
    A_UINT32 revId, promSize;
    A_STATUS status;
    A_UINT16 eepVersion;
    A_UINT32 macRev;
    A_UINT32 addr, loop, wrData, rdData, pattern;
    A_UINT32 regAddr[2] = {MAC_STA_ID0, PHY_BASE+(8 << 2)};
    A_UINT32 regHold[2];
    A_UINT32 patternData[4] = {0x55555555, 0xaaaaaaaa, 0x66666666, 0x99999999};
    A_UINT16 i;

    ASSERT(pDev);

    /* Pull Chip out of Reset */
    status = ar5211ChipReset(pDev, 0);
    if (status != A_OK) {
        return status;
    }

    /* Read Revisions from Chips */
#if defined(PCI_INTERFACE)
    macRev           = readPlatformReg(pDev, MAC_SREV) & MAC_SREV_ID_M;
    pDev->macVersion = (A_UINT16) (macRev >> MAC_SREV_ID_S);
    pDev->macRev     = (A_UINT16) (macRev & MAC_SREV_REVISION_M);
#elif defined(AR531X)
    macRev           = sysRegRead(AR531X_REV);
    pDev->macVersion = (macRev & REV_WMAC_MAJ) >> REV_WMAC_MAJ_S;
    pDev->macRev     = (macRev & REV_WMAC_MIN) >> REV_WMAC_MIN_S;
#endif

    if ((pDev->macVersion < MAC_SREV_VERSION_MAUI_2) ||
        (pDev->macVersion > MAC_SREV_VERSION_OAHU))
    {
        uiPrintf("ar5211Attach: Mac Chip Rev 0x%02X is not supported by this driver\n",
                 pDev->macVersion);
        return A_HARDWARE;
    }

    pDev->phyRev = (A_UINT16)readPlatformReg(pDev, PHY_CHIP_ID);

    // Test PHY & MAC registers
    for (i = 0; i < 2; i++) {
        addr       = regAddr[i];
        regHold[i] = readPlatformReg(pDev, addr);

        for (loop = 0; loop < 0x100; loop++) {

            wrData = (loop << 16) | loop;
            writePlatformReg(pDev, addr, wrData);
            rdData = readPlatformReg(pDev, addr);
            if (rdData != wrData) {
                uiPrintf("ar5211Attach: address test failed addr: 0x%08x - wr:0x%08x != rd:0x%08x\n",
                         addr, wrData, rdData);
                return A_HARDWARE;
            }
        }
        for (pattern = 0; pattern < 4; pattern++) {
            wrData = patternData[pattern];
            writePlatformReg(pDev, addr, wrData);
            rdData = readPlatformReg(pDev, addr);
            if (wrData != rdData) {
                uiPrintf("ar5211Attach: address test failed addr: 0x%08x - wr:0x%08x != rd:0x%08x\n",
                         addr, wrData, rdData);
                return A_HARDWARE;
            }
        }
        writePlatformReg(pDev, regAddr[i], regHold[i]);
    }
    udelay(100);

    /* Set correct Baseband to analog shift setting to access analog chips. */
    if (pDev->macVersion >= MAC_SREV_VERSION_OAHU) {
        writePlatformReg(pDev, PHY_BASE, 0x00000007);
    } else {
        writePlatformReg(pDev, PHY_BASE, 0x00000047);
    }

    /* Read Radio Chip Rev Extract */
    writePlatformReg(pDev, (PHY_BASE + (0x34 << 2)), 0x00001c16);
    for (i = 0; i < 8; i++) {
        writePlatformReg(pDev, (PHY_BASE + (0x20 << 2)), 0x00010000);
    }
    revId = (readPlatformReg(pDev, PHY_BASE + (256 << 2)) >> 24) & 0xff;
    revId = ((revId & 0xf0) >> 4) | ((revId & 0x0f) << 4);
    pDev->analog5GhzRev = (A_UINT16)reverseBits(revId, 8);
    if ((pDev->analog5GhzRev & 0xF0) != RAD5_SREV_MAJOR) {
        uiPrintf("ar5211Attach: 5G Radio Chip Rev 0x%02X is not supported by this driver\n",
                 pDev->analog5GhzRev);
        return A_HARDWARE;
    }

    /* EEPROM version and size checking */
    status = ar5211EepromRead(pDev, ATHEROS_EEPROM_OFFSET + 1, &eepVersion);
    if (status != A_OK) {
        goto ee_error;
    }

#if defined(PCI_INTERFACE)
    promSize = (readPlatformReg(pDev, MAC_PCICFG) & MAC_PCICFG_EEPROM_SIZE_M) >>
               MAC_PCICFG_EEPROM_SIZE_S;
#else
    promSize = MAC_PCICFG_EEPROM_SIZE_16K;
#endif

    // Allocate EEPROM Map
    pEMap = (struct eepMap *)A_DRIVER_MALLOC(sizeof(struct eepMap));
    if (!pEMap) {
        uiPrintf("ar5211Attach: Could not allocate memory for EEPROM\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEMap, sizeof(struct eepMap));
    pDev->pHalInfo->pEepData = pEMap;

    if ( (eepVersion >= EEPROM_VER3) && (promSize == MAC_PCICFG_EEPROM_SIZE_16K) ) {
        status = ar5211VerifyEepromChecksum(pDev);
        if (status != A_OK) {
            goto ee_error;
        }
        status = ar5211AllocateProm(pEMap);
        if (status != A_OK) {
            goto ee_error;
        }

        /* Set Version So EEPROM Header can be correctly interpreted */
        pEMap->version = eepVersion;

        status = ar5211ReadEepromIntoDataset(pDev);
        if (status != A_OK) {
            goto ee_error;
        }

        status = ar5211EepromRead(pDev, EEPROM_PROTECT_OFFSET, &pEMap->protect);
        if (status != A_OK) {
            goto ee_error;
        }

        status = ar5211FillCapabilityInfo(pDev);
        if (status != A_OK) {
            goto ee_error;
        }

        /* If Bmode and AR5211, verify 2.4 analog exists */
        if ((pDev->macVersion >= MAC_SREV_VERSION_OAHU) &&
            pEMap->pEepHeader->Bmode)
        {
            /* Set correct Baseband to analog shift setting to access analog chips. */
            writePlatformReg(pDev, PHY_BASE, 0x00004007);
            udelay(2000);
            writePlatformReg(pDev, (PHY_BASE + (0x34 << 2)), 0x00001c16);
            for (i = 0; i < 8; i++) {
                writePlatformReg(pDev, (PHY_BASE + (0x20 << 2)), 0x00010000);
            }
            revId = (readPlatformReg(pDev, PHY_BASE + (256 << 2)) >> 24) & 0xff;
            revId = ((revId & 0xf0) >> 4) | ((revId & 0x0f) << 4);
            pDev->analog2GhzRev = (A_UINT16)reverseBits(revId, 8);

            /* Set baseband for 5GHz chip */
            writePlatformReg(pDev, PHY_BASE, 0x00000007);
            udelay(2000);

            {
                if ((pDev->analog2GhzRev & 0xF0) != RAD2_SREV_MAJOR) {
                    uiPrintf("ar5211Attach: 2G Radio Chip Rev 0x%02X is not supported by this driver\n",
                             pDev->analog2GhzRev);
                    goto ee_error;
                }
            }
        } else {
            pEMap->pEepHeader->Bmode = 0;
        }

    } else {
        uiPrintf("ar5211Attach: A Rev 3 16k calibrated EEPROM is required for correct operation\n");
        goto ee_error;
    }

    /* Allocate thermal gain adjustment structure */
    pGainValues = (struct gainValues *)A_DRIVER_MALLOC(sizeof(struct gainValues));
    if (!pGainValues) {
        uiPrintf("ar5211Attach: Could not allocate memory for gain optimization values\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pGainValues, sizeof(struct gainValues));
    pDev->pHalInfo->pGainValues = pGainValues;
    /* Initialize gain ladder thermal calibration structure */
    ar5211InitializeGainValues(pGainValues);

    /*
     * It's okay for the address to be assigned by software, so we ignore the
     * following error value if returned.
     */
    (void) ar5211GetMacAddr(pDev, &pDev->staConfig.macPermAddr);

#ifdef DEBUG
    uiPrintf("wlan revisions: mac %d.%d phy %d.%d analog %d.%d\n",
             pDev->macVersion, pDev->macRev,
             pDev->phyRev >> 4, pDev->phyRev & 0xf,
             pDev->analog5GhzRev >> 4, pDev->analog5GhzRev & 0xf);
#endif

    /* Attach rate table for various wireless modes to the hal */
    ar5211AttachRateTables(pDev);

    /* Attach device specific functions to the hal */
    pDev->pHwFunc = &ar5211Funcs;

    return A_OK;

ee_error:
    uiPrintf("ar5211Attach: EEPROM-related failure\n");
    ar5211Detach(pDev);
    return A_HARDWARE;
}

/**************************************************************
 * ar5211Detach
 *
 * Detach removes all hwLayer allocated data structures and places the
 * device into reset.
 */
A_STATUS
ar5211Detach(WLAN_DEV_INFO *pDev)
{
    HAL_INFO       *pInfo;
    struct eepMap  *pData;

    ASSERT(pDev && pDev->pHalInfo);

    pInfo = pDev->pHalInfo;
    pData = pInfo->pEepData;

    if (pData) {
        ASSERT(pData->version >= EEPROM_VER3);

        if (pData->pEepHeader) {
            A_DRIVER_FREE(pData->pEepHeader, sizeof(EEP_HEADER_INFO));
        }

        if (pData->pPcdacInfo) {
            A_DRIVER_FREE(pData->pPcdacInfo, sizeof(PCDACS_ALL_MODES));
        }

        if (pData->pTrgtPowerInfo) {
            A_DRIVER_FREE(pData->pTrgtPowerInfo, sizeof(TRGT_POWER_ALL_MODES));
        }

        if (pData->pRdEdgesPower) {
            A_DRIVER_FREE(pData->pRdEdgesPower, 
                          sizeof(RD_EDGES_POWER) * NUM_EDGES * NUM_CTLS_3_3);
        }

        A_DRIVER_FREE(pData, sizeof(struct eepMap));
        pInfo->pEepData = NULL;

        A_DRIVER_FREE(pInfo->pGainValues, sizeof(struct gainValues));
        pInfo->pGainValues = NULL;
    }

    return A_OK;
}

/**************************************************************
 * ar5211FillCapabilityInfo
 *
 * Fill all software cached or static hardware state information
 *
 * Returns failure for various EEPROM reading failure cases
 */
static A_STATUS
ar5211FillCapabilityInfo(WLAN_DEV_INFO *pDev)
{
    A_STATUS         status = A_OK;
    HAL_CAPABILITIES *pCap = &(pDev->pHalInfo->halCapabilities);
    EEP_HEADER_INFO  *pEepHeader = pDev->pHalInfo->pEepData->pEepHeader;
    A_UINT16         regDmn;
    A_UINT32         wMode = 0;
    
    /* Read and save EEPROM regDomain */
    status = ar5211EepromRead(pDev, REGULATORY_DOMAIN_OFFSET, &regDmn);
    if (status != A_OK) {
        return status;
    }
    pCap->halRegDmn = regDmn;

    /* Get wireless mode */
    if (pEepHeader->Amode) {
        wMode |= MODE_SELECT_11A;
        if (!pEepHeader->turboDisable) {
            wMode |= MODE_SELECT_TURBO;
        }
    }
    if (pEepHeader->Bmode) {
        wMode |= MODE_SELECT_11B;
    }
    pCap->halWirelessModes      = wMode;
   
    pCap->halLow2GhzChan        = 2312;
    pCap->halHigh2GhzChan       = 2732;
    pCap->halLow5GhzChan        = 4920;
    pCap->halHigh5GhzChan       = 6100;
   
    pCap->halCipherCkipSupport  = FALSE;
    pCap->halCipherTkipSupport  = FALSE;
    pCap->halCipherAesCcmSupport = FALSE;

    pCap->halMicCkipSupport     = FALSE;
    pCap->halMicTkipSupport     = FALSE;
    pCap->halMicAesCcmSupport   = FALSE;

    pCap->halChanSpreadSupport  = FALSE;
    pCap->halSleepAfterBeaconBroken = FALSE;
    
    pCap->halCompressSupport    = FALSE;
    pCap->halBurstSupport       = FALSE;
    pCap->halFastFramesSupport  = FALSE;
    pCap->halChapTuningSupport  = FALSE;
    pCap->halTurboGSupport      = FALSE;
    pCap->halTurboPrimeSupport  = FALSE;
    pCap->halXrSupport          = FALSE;

    pCap->halDeviceType = pEepHeader->deviceType;
    pCap->halTotalQueues = HAL_NUM_TX_QUEUES;
    pCap->halKeyCacheSize = MAC_KEY_CACHE_SIZE;

    return status;
}

/************** EEPROM REV 3 ATTACH FUNCTIONS *****************/

/**************************************************************
 * ar5211VerifyEepromChecksum
 *
 * Allocates memory for the EEPROM structures
 */
A_STATUS
ar5211VerifyEepromChecksum(WLAN_DEV_INFO *pDev)
{
    A_STATUS    status = A_OK;
    A_UINT16    data;
    A_UINT32    addr;
    A_UINT16    chkSum = 0;

    /* EEPROM version and size checking */

    for (addr = ATHEROS_EEPROM_OFFSET; addr < 0x400; addr++) {
        status = ar5211EepromRead(pDev, addr, &data);
        if (status != A_OK) {
            goto ee_error;
        }
        chkSum ^= (data & 0xffff);
    }

    chkSum &= 0xffff;
    if (chkSum != 0xffff) {
        uiPrintf("ar5211VerifyEepromChecksum: Checksum %04x != 0xffff\n", chkSum);
        return A_HARDWARE;
    }

    return status;

ee_error:
    uiPrintf("ar5211Attach: EEPROM read failed\n");
    return A_HARDWARE;
}

/**************************************************************
 * ar5211AllocateProm
 *
 * Allocates memory for the EEPROM structures
 */
A_STATUS
ar5211AllocateProm(EEP_MAP *pEepMap)
{
    /* TODO: only allocate 2.4 info if configured */
    int i;

    /* allocate the struct to hold the header info */
    pEepMap->pEepHeader = (EEP_HEADER_INFO *)A_DRIVER_MALLOC(sizeof(EEP_HEADER_INFO));
    if (!pEepMap->pEepHeader) {
        uiPrintf("Unable to allocate eeprom structure for header info\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEepMap->pEepHeader, sizeof(EEP_HEADER_INFO));

    /* allocate the struct to hold the pcdac/power info */
    pEepMap->pPcdacInfo = (PCDACS_ALL_MODES *)A_DRIVER_MALLOC(sizeof(PCDACS_ALL_MODES));
    if (!pEepMap->pPcdacInfo) {
        uiPrintf("Unable to allocate eeprom structure for pcdac/power info\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEepMap->pPcdacInfo, sizeof(PCDACS_ALL_MODES));

    pEepMap->pPcdacInfo->numChannels11a = NUM_11A_EEPROM_CHANNELS;
    pEepMap->pPcdacInfo->numChannels2_4 = NUM_2_4_EEPROM_CHANNELS;

    for (i = 0; i < NUM_11A_EEPROM_CHANNELS; i ++) {
        pEepMap->pPcdacInfo->DataPerChannel11a[i].numPcdacValues = NUM_PCDAC_VALUES;
    }

    /* the channel list for 2.4 is fixed, fill this in here */
    for (i = 0; i < NUM_2_4_EEPROM_CHANNELS; i++) {
        pEepMap->pPcdacInfo->Channels2_4[i] = channels2_4[i];
        pEepMap->pPcdacInfo->DataPerChannel11b[i].numPcdacValues = NUM_PCDAC_VALUES;
        pEepMap->pPcdacInfo->DataPerChannel11g[i].numPcdacValues = NUM_PCDAC_VALUES;
    }

    /* allocate the structure to hold target power info */
    pEepMap->pTrgtPowerInfo = (TRGT_POWER_ALL_MODES *)A_DRIVER_MALLOC(sizeof(TRGT_POWER_ALL_MODES));
    if (!pEepMap->pTrgtPowerInfo) {
        uiPrintf("Unable to allocate eeprom structure for target power info\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEepMap->pTrgtPowerInfo, sizeof(TRGT_POWER_ALL_MODES));

    /* allocate structure for RD edges */
    pEepMap->pRdEdgesPower = (RD_EDGES_POWER *)A_DRIVER_MALLOC(sizeof(RD_EDGES_POWER) *
        NUM_EDGES * NUM_CTLS_3_3);
    if (!pEepMap->pRdEdgesPower) {
        uiPrintf("Unable to allocate eeprom structure for RD edges info\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEepMap->pRdEdgesPower, sizeof(RD_EDGES_POWER) * NUM_EDGES * NUM_CTLS_3_3);
    return A_OK;
}

/**************************************************************
 * ar5211ReadEepromFreqPierInfo
 *
 * Now copy EEPROM frequency pier contents into the allocated space
 */
INLINE A_STATUS
ar5211ReadEepromFreqPierInfo(WLAN_DEV_INFO *pDev)
{
    
    A_STATUS             status = A_OK;
    A_UINT16             tempValue;
    A_UINT32             i;
    A_UINT16             offset;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap           = pDev->pHalInfo->pEepData;
    pEepromData    = pMap->pPcdacInfo;
 
    if (pMap->version >= EEPROM_VER3_3) {
        offset = EEPROM_GROUPS_OFFSET3_3 + GROUP1_OFFSET;

        for (i = 0; i < pEepromData->numChannels11a; i += 2) {
            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_fp_error;
            }
            pEepromData->Channels11a[i] = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK_3_3);
            pEepromData->Channels11a[i + 1] = (A_UINT16)( tempValue & FREQ_MASK_3_3);
        } 
    } else {
        offset = EEPROM_GROUPS_OFFSET3_2 + GROUP1_OFFSET;

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
            goto ee_fp_error;
        }

        pEepromData->Channels11a[0] = (A_UINT16)(( tempValue >> 9 ) & FREQ_MASK);
        pEepromData->Channels11a[1] = (A_UINT16)(( tempValue >> 2 ) & FREQ_MASK);
        pEepromData->Channels11a[2] = (A_UINT16)(( tempValue << 5 ) & FREQ_MASK);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
            goto ee_fp_error;
        }
        pEepromData->Channels11a[2] = (A_UINT16)((( tempValue >> 11 ) & 0x1f) | pEepromData->Channels11a[2]);
        pEepromData->Channels11a[3] = (A_UINT16)(( tempValue >> 4 ) & FREQ_MASK);
        pEepromData->Channels11a[4] = (A_UINT16)(( tempValue << 3 ) & FREQ_MASK);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
            goto ee_fp_error;
        }
        pEepromData->Channels11a[4] = (A_UINT16)((( tempValue >> 13 ) & 0x7) | pEepromData->Channels11a[4]);
        pEepromData->Channels11a[5] = (A_UINT16)(( tempValue >> 6 ) & FREQ_MASK);
        pEepromData->Channels11a[6] =  (A_UINT16)(( tempValue << 1 ) & FREQ_MASK);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
            goto ee_fp_error;
        }
        pEepromData->Channels11a[6] = (A_UINT16)((( tempValue >> 15 ) & 0x1) | pEepromData->Channels11a[6]);
        pEepromData->Channels11a[7] = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK);
        pEepromData->Channels11a[8] =  (A_UINT16)(( tempValue >> 1 ) & FREQ_MASK);
        pEepromData->Channels11a[9] =  (A_UINT16)(( tempValue << 6 ) & FREQ_MASK);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
            goto ee_fp_error;
        }
        pEepromData->Channels11a[9] = (A_UINT16)((( tempValue >> 10 ) & 0x3f) | pEepromData->Channels11a[9]);
    }

    for (i = 0; i < pEepromData->numChannels11a; i++ ) {
        pEepromData->Channels11a[i] = fbin2freq(pMap->version, pEepromData->Channels11a[i]);
    }

ee_fp_error:    
    return status;
}

/**************************************************************
 * ar5211ReadEepromRawPowerCalInfo
 *
 * Now copy EEPROM Raw Power Calibration per frequency contents 
 * into the allocated space
 */
INLINE A_STATUS
ar5211ReadEepromRawPowerCalInfo(WLAN_DEV_INFO *pDev)
{
    A_STATUS             status = A_OK;
    A_UINT16             tempValue;
    A_UINT32             i, j;
    A_UINT32             offset = 0;
    A_UINT16             mode;
    A_UINT16             enable24;
    A_UINT16             numChannels = 0;
    DATA_PER_CHANNEL     *pChannelData = NULL;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap            = pDev->pHalInfo->pEepData;
    pEepromData     = pMap->pPcdacInfo;

    enable24        = pMap->pEepHeader->Bmode;
    /*
     * Group 2:  read raw power data for all frequency piers
     *
     * NOTE: Group 2 contains the raw power calibration information 
     * for each of the channels that we recorded in ar5211ReadEepromFreqPiersInfo()
     */
    for (mode = 0; mode <= MAX_MODE; mode++) {
        A_UINT16 *pChannels = NULL;
        offset          = (pMap->version >= EEPROM_VER3_3) ? 
                           EEPROM_GROUPS_OFFSET3_3 : EEPROM_GROUPS_OFFSET3_2;

        switch (mode) {
        case MODE_11A:
            offset      += GROUP2_OFFSET;
            numChannels  = pEepromData->numChannels11a;
            pChannelData = pEepromData->DataPerChannel11a;
            pChannels    = pEepromData->Channels11a;
            break;

        case MODE_11B:
            if (!enable24) {
                continue;
            }
            offset      += GROUP3_OFFSET;
            numChannels  = pEepromData->numChannels2_4;
            pChannelData = pEepromData->DataPerChannel11b;
            pChannels    = pEepromData->Channels2_4;

            break;

        case MODE_OFDM_24:
            if (!enable24) {
                continue;
            }
            offset      += GROUP4_OFFSET;
            numChannels  = pEepromData->numChannels2_4;
            pChannelData = pEepromData->DataPerChannel11g;
            pChannels    = pEepromData->Channels2_4;
            break;

        default:
            ASSERT(0);
        }

        for (i = 0; i < numChannels; i++) {
            pChannelData->channelValue = pChannels[i];

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_rp_error;
            }
            pChannelData->pcdacMax     = (A_UINT16)(( tempValue >> 10 ) & PCDAC_MASK);
            pChannelData->pcdacMin     = (A_UINT16)(( tempValue >> 4  ) & PCDAC_MASK);
            pChannelData->PwrValues[0] = (A_UINT16)(( tempValue << 2  ) & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_rp_error;
            }
            pChannelData->PwrValues[0] = (A_UINT16)(((tempValue >> 14 ) & 0x3 ) | pChannelData->PwrValues[0]);
            pChannelData->PwrValues[1] = (A_UINT16)((tempValue >> 8 ) & POWER_MASK);
            pChannelData->PwrValues[2] = (A_UINT16)((tempValue >> 2 ) & POWER_MASK);
            pChannelData->PwrValues[3] = (A_UINT16)((tempValue << 4 ) & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_rp_error;
            }
            pChannelData->PwrValues[3] = (A_UINT16)(( ( tempValue >> 12 ) & 0xf ) | pChannelData->PwrValues[3]);
            pChannelData->PwrValues[4] = (A_UINT16)(( tempValue >> 6 ) & POWER_MASK);
            pChannelData->PwrValues[5] = (A_UINT16)(tempValue  & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_rp_error;
            }
            pChannelData->PwrValues[6] = (A_UINT16)(( tempValue >> 10 ) & POWER_MASK);
            pChannelData->PwrValues[7] = (A_UINT16)(( tempValue >> 4  ) & POWER_MASK);
            pChannelData->PwrValues[8] = (A_UINT16)(( tempValue << 2  ) & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_rp_error;
            }
            pChannelData->PwrValues[8]  = (A_UINT16)(( ( tempValue >> 14 ) & 0x3 ) | pChannelData->PwrValues[8]);
            pChannelData->PwrValues[9]  = (A_UINT16)(( tempValue >> 8 ) & POWER_MASK);
            pChannelData->PwrValues[10] = (A_UINT16)(( tempValue >> 2 ) & POWER_MASK);

            getPcdacInterceptsFromPcdacMinMax(pMap->version, pChannelData->pcdacMin,
                        pChannelData->pcdacMax, pChannelData->PcdacValues) ;

            for (j = 0; j < pChannelData->numPcdacValues; j++) {
                pChannelData->PwrValues[j] = (A_UINT16)(PWR_STEP * pChannelData->PwrValues[j]);
                /* Note these values are scaled up. */
            }
            pChannelData++;
        }
    }

ee_rp_error:
    return status;
}

/**************************************************************
 * ar5211ReadEepromTargetPowerCalInfo
 *
 * Copy EEPROM Target Power Calbration per rate contents 
 * into the allocated space
 */
INLINE A_STATUS
ar5211ReadEepromTargetPowerCalInfo(WLAN_DEV_INFO *pDev)
{

    A_STATUS             status = A_OK;
    A_UINT16             tempValue;
    A_UINT32             i;
    A_UINT32             offset = 0;
    A_UINT16             enable24;
    A_UINT16             mode;
    A_UINT16             numChannels = 0; 
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap            = pDev->pHalInfo->pEepData;
    enable24        = pMap->pEepHeader->Bmode;

    for (mode = MODE_11A; mode <= MAX_MODE; mode++) {
        TRGT_POWER_INFO     *pPowerInfo = NULL;
        A_UINT16            *pNumTrgtChannels = NULL;
        offset          = (pMap->version >= EEPROM_VER3_3) ? 
                            EEPROM_GROUPS_OFFSET3_3 : EEPROM_GROUPS_OFFSET3_2;

        switch (mode) {
        case MODE_11A:
            offset          += GROUP5_OFFSET;
            numChannels      = NUM_TEST_FREQUENCIES;
            pPowerInfo       = pMap->pTrgtPowerInfo->trgtPwr_11a;
            pNumTrgtChannels = &pMap->pTrgtPowerInfo->numTargetPwr_11a;
            break;

        case MODE_11B:
            if (!enable24) {
                continue;
            }
            offset          += GROUP6_OFFSET;
            numChannels      = 2;
            pPowerInfo       = pMap->pTrgtPowerInfo->trgtPwr_11b;
            pNumTrgtChannels = &pMap->pTrgtPowerInfo->numTargetPwr_11b;
            break;

        case MODE_OFDM_24:
            if (!enable24) {
                continue;
            }
            offset          += GROUP7_OFFSET;
            numChannels      = 3;
            pPowerInfo       = pMap->pTrgtPowerInfo->trgtPwr_11g;
            pNumTrgtChannels = &pMap->pTrgtPowerInfo->numTargetPwr_11g;
            break;

        default:
            ASSERT(0);
        } //end mode switch

        *pNumTrgtChannels = 0;
        for (i = 0; i < numChannels; i++) {
            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_tp_error;
            }
            if (pMap->version >= EEPROM_VER3_3) {
                pPowerInfo->testChannel = (A_UINT16)(( tempValue >> 8 ) & 0xff);
            } else {
                pPowerInfo->testChannel = (A_UINT16)(( tempValue >> 9 ) & 0x7f);
            }

            if (pPowerInfo->testChannel != 0) {
                /* get the channel value and read rest of info */
                if (mode == MODE_11A) {
                    pPowerInfo->testChannel = fbin2freq(pMap->version, pPowerInfo->testChannel);
                } else {
                    pPowerInfo->testChannel = fbin2freq_2p4(pMap->version, pPowerInfo->testChannel);
                }

                if (pMap->version >= EEPROM_VER3_3) {
                    pPowerInfo->twicePwr6_24 = (A_UINT16)(( tempValue >> 2 ) & POWER_MASK);
                    pPowerInfo->twicePwr36   = (A_UINT16)(( tempValue << 4 ) & POWER_MASK);
                } else {
                    pPowerInfo->twicePwr6_24 = (A_UINT16)(( tempValue >> 3 ) & POWER_MASK);
                    pPowerInfo->twicePwr36   = (A_UINT16)(( tempValue << 3 ) & POWER_MASK);
                }

                status = ar5211EepromRead(pDev, offset++, &tempValue);
                if (status != A_OK) {
                   goto ee_tp_error;
                }
                if (pMap->version >= EEPROM_VER3_3) {
                    pPowerInfo->twicePwr36 =
                        (A_UINT16)(( ( tempValue >> 12 ) & 0xf ) | pPowerInfo->twicePwr36);
                    pPowerInfo->twicePwr48 = (A_UINT16)(( tempValue >> 6 ) & POWER_MASK);
                    pPowerInfo->twicePwr54 = (A_UINT16)( tempValue & POWER_MASK);
                } else {
                    pPowerInfo->twicePwr36 =
                        (A_UINT16)(( ( tempValue >> 13 ) & 0x7 ) | pPowerInfo->twicePwr36);
                    pPowerInfo->twicePwr48 = (A_UINT16)(( tempValue >> 7 ) & POWER_MASK);
                    pPowerInfo->twicePwr54 = (A_UINT16)(( tempValue >> 1 ) & POWER_MASK);
                }
                (*pNumTrgtChannels)++;
            }
            pPowerInfo++;
        }
    }

ee_tp_error:
    return status;
}

/**************************************************************
 * ar5211ReadEepromCTLInfo
 *
 * Now copy EEPROM Coformance Testing Limits contents 
 * into the allocated space
 */
INLINE A_STATUS
ar5211ReadEepromCTLInfo(WLAN_DEV_INFO *pDev)
{

    A_STATUS             status = A_OK;
    A_UINT16             tempValue;
    A_UINT32             i, j;
    A_UINT32             offset = 0;
    RD_EDGES_POWER       *pRdEdgePwrInfo;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap           = pDev->pHalInfo->pEepData;
    pEepromData    = pMap->pPcdacInfo;
    pRdEdgePwrInfo = pMap->pRdEdgesPower;

    offset  = GROUP8_OFFSET + ((pMap->version >= EEPROM_VER3_3) ?
                              EEPROM_GROUPS_OFFSET3_3 : EEPROM_GROUPS_OFFSET3_2);

    for (i = 0; i < pMap->pEepHeader->numCtls; i++) {
        if (pMap->pEepHeader->ctl[i] == 0) {
            /* Move offset and edges */
            offset += ((pMap->version >= EEPROM_VER3_3) ? 8 : 7);
            pRdEdgePwrInfo += NUM_EDGES;
            continue;
        }

        if (pMap->version >= EEPROM_VER3_3) {
            for (j = 0; j < NUM_EDGES; j += 2) {
                status = ar5211EepromRead(pDev, offset++, &tempValue);
                if (status != A_OK) {
                    goto ee_ctl_error;
                }
                pRdEdgePwrInfo[j].rdEdge = (A_UINT16)((tempValue >> 8) & FREQ_MASK_3_3);
                pRdEdgePwrInfo[j + 1].rdEdge = (A_UINT16)(tempValue & FREQ_MASK_3_3);
            }
            for (j = 0; j < NUM_EDGES; j += 2) {
                status = ar5211EepromRead(pDev, offset++, &tempValue);
                if (status != A_OK) {
                    goto ee_ctl_error;
                }
                pRdEdgePwrInfo[j].twice_rdEdgePower = 
                                (A_UINT16)((tempValue >> 8) & POWER_MASK);
                pRdEdgePwrInfo[j].flag = (tempValue >> 14) & 1;
                pRdEdgePwrInfo[j + 1].twice_rdEdgePower = 
                                (A_UINT16)(tempValue & POWER_MASK);
                pRdEdgePwrInfo[j + 1].flag = (tempValue >> 6) & 1;
            }
        } else { 
            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[0].rdEdge = (A_UINT16)(( tempValue >> 9 ) & FREQ_MASK);
            pRdEdgePwrInfo[1].rdEdge = (A_UINT16)(( tempValue >> 2 ) & FREQ_MASK);
            pRdEdgePwrInfo[2].rdEdge = (A_UINT16)(( tempValue << 5 ) & FREQ_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[2].rdEdge = (A_UINT16)((( tempValue >> 11 ) & 0x1f) | pRdEdgePwrInfo[2].rdEdge);
            pRdEdgePwrInfo[3].rdEdge = (A_UINT16)(( tempValue >> 4 ) & FREQ_MASK);
            pRdEdgePwrInfo[4].rdEdge = (A_UINT16)(( tempValue << 3 ) & FREQ_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[4].rdEdge = (A_UINT16)((( tempValue >> 13 ) & 0x7) | pRdEdgePwrInfo[4].rdEdge);
            pRdEdgePwrInfo[5].rdEdge = (A_UINT16)(( tempValue >> 6 ) & FREQ_MASK);
            pRdEdgePwrInfo[6].rdEdge = (A_UINT16)(( tempValue << 1 ) & FREQ_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[6].rdEdge = (A_UINT16)((( tempValue >> 15 ) & 0x1) | pRdEdgePwrInfo[6].rdEdge);
            pRdEdgePwrInfo[7].rdEdge = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK);

            pRdEdgePwrInfo[0].twice_rdEdgePower = (A_UINT16)(( tempValue >> 2 ) & POWER_MASK);
            pRdEdgePwrInfo[1].twice_rdEdgePower = (A_UINT16)(( tempValue << 4 ) & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[1].twice_rdEdgePower = (A_UINT16)((( tempValue >> 12 ) & 0xf) | pRdEdgePwrInfo[1].twice_rdEdgePower);
            pRdEdgePwrInfo[2].twice_rdEdgePower = (A_UINT16)(( tempValue >> 6 ) & POWER_MASK);
            pRdEdgePwrInfo[3].twice_rdEdgePower = (A_UINT16)(tempValue & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[4].twice_rdEdgePower = (A_UINT16)(( tempValue >> 10 ) & POWER_MASK);
            pRdEdgePwrInfo[5].twice_rdEdgePower = (A_UINT16)(( tempValue >> 4 ) & POWER_MASK);
            pRdEdgePwrInfo[6].twice_rdEdgePower = (A_UINT16)(( tempValue << 2 ) & POWER_MASK);

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
                goto ee_ctl_error;
            }
            pRdEdgePwrInfo[6].twice_rdEdgePower = (A_UINT16)((( tempValue >> 14 ) & 0x3) | pRdEdgePwrInfo[6].twice_rdEdgePower);
            pRdEdgePwrInfo[7].twice_rdEdgePower = (A_UINT16)(( tempValue >> 8 ) & POWER_MASK);
        }

        for (j = 0; j < NUM_EDGES; j++ ) {
            if (pRdEdgePwrInfo[j].rdEdge != 0 || pRdEdgePwrInfo[j].twice_rdEdgePower != 0) {
                if (((pMap->pEepHeader->ctl[i] & 0x3) == 0) ||
                    ((pMap->pEepHeader->ctl[i] & 0x3) == 3))
                {
                    pRdEdgePwrInfo[j].rdEdge = fbin2freq(pMap->version, pRdEdgePwrInfo[j].rdEdge);
                } else {
                    pRdEdgePwrInfo[j].rdEdge = fbin2freq_2p4(pMap->version, pRdEdgePwrInfo[j].rdEdge);
                }
            }
        }
        pRdEdgePwrInfo += NUM_EDGES;
    }

ee_ctl_error:
    return status;
}

/**************************************************************
 * ar5211ReadEepromIntoDataset
 *
 * Now verify and copy EEPROM contents into the allocated space
 */
A_STATUS
ar5211ReadEepromIntoDataset(WLAN_DEV_INFO *pDev)
{
    A_STATUS             status = A_OK;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    pMap           = pDev->pHalInfo->pEepData;
    
    /* Read the header information here */
    ar5211ReadHeaderInfo(pDev, pMap->pEepHeader);

    /*
     * Group 1: frequency pier locations readback
     * check that the structure has been populated with enough space to hold the channels
     *
     * NOTE: Group 1 contains the 5 GHz channel numbers that have dBm->pcdac calibrated information
     */
    status = ar5211ReadEepromFreqPierInfo(pDev);
    if (status != A_OK) {
        goto ee_error;
    }

    /*
     * Group 2:  readback data for all frequency piers
     *
     * NOTE: Group 2 contains the raw power calibration information for each of the channels
     * that we recorded above
     */
    status = ar5211ReadEepromRawPowerCalInfo(pDev);
    if (status != A_OK) {
        goto ee_error;
    }
 
    /*
     * Group 5: target power values per rate
     *
     * NOTE: Group 5 contains the recorded maximum power in dB that can be attained
     * for the given rate.
     */
    /* Read the power per rate info for test channels */
    status = ar5211ReadEepromTargetPowerCalInfo(pDev);
    if (status != A_OK) {
        goto ee_error;
    }

    /*
     * Group 8: Conformance Test Limits information
     *
     * NOTE: Group 8 contains the values to limit the maximum transmit power
     * value based on any band edge violations.
     */
    /* Read the RD edge power limits */
    status = ar5211ReadEepromCTLInfo(pDev);
    if (status != A_OK) {
        goto ee_error;
    }

    return status;

ee_error:
    uiPrintf("ar5211Attach: EEPROM read failed\n");
    return A_HARDWARE;
}

/**************************************************************
 * ar5211ReadHeaderInfo
 *
 * Read the individual header fields for a Rev 3 EEPROM
 */
A_STATUS
ar5211ReadHeaderInfo(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo)
{
    A_STATUS status = A_OK;
    A_UINT16 tempValue;
    A_UINT32 offset;
    A_UINT16 i;
    A_UINT32 *pHeadOff;
    A_UINT16 version = pDev->pHalInfo->pEepData->version;

    enum HeaderInfoMode {
        headerInfo11A,          
        headerInfo11B,
        headerInfo11G
    };

    static A_UINT32 headerOffset3_0[] = {
        0x00C2, /* 0 - Mode bits, device type, max turbo power */
        0x00C4, /* 1 - 2.4 and 5 antenna gain */
        0x00C5, /* 2 - Begin 11A modal section */
        0x00D0, /* 3 - Begin 11B modal section */
        0x00DA, /* 4 - Begin 11G modal section */
        0x00E4  /* 5 - Begin CTL section */
    };
    static A_UINT32 headerOffset3_3[] = {
        0x00C2, /* 0 - Mode bits, device type, max turbo power */
        0x00C3, /* 1 - 2.4 and 5 antenna gain */
        0x00D4, /* 2 - Begin 11A modal section */
        0x00F2, /* 3 - Begin 11B modal section */
        0x010D, /* 4 - Begin 11G modal section */
        0x0128  /* 5 - Begin CTL section */
    };

    if (version >= EEPROM_VER3_3) {
        pHeadOff = headerOffset3_3;
        pHeaderInfo->numCtls = NUM_CTLS_3_3;
    } else {
        pHeadOff = headerOffset3_0;
        pHeaderInfo->numCtls = NUM_CTLS;
    }

    status = ar5211EepromRead(pDev, pHeadOff[0], &tempValue);
    if (status != A_OK) {
       goto ee_error;
    }
    pHeaderInfo->turboDisable = (A_UINT8)((tempValue >> 15) & 0x01);
    pHeaderInfo->rfKill = (A_UINT8)((tempValue >> 14) & 0x01);
    pHeaderInfo->deviceType = (A_UINT8)((tempValue >> 11) & 0x07);
    pHeaderInfo->turbo2WMaxPower = (A_UINT8)((tempValue >> 4) & 0x7F);
    pHeaderInfo->xtnd5GSupport = (A_UINT8)((tempValue >> 3) & 0x01);
    pHeaderInfo->Gmode = (A_UINT8)((tempValue >> 2) & 0x01);
    pHeaderInfo->Bmode = (A_UINT8)((tempValue >> 1) & 0x01);
    pHeaderInfo->Amode = (A_UINT8)(tempValue & 0x01);

    status = ar5211EepromRead(pDev, pHeadOff[1], &tempValue);
    if (status != A_OK) {
       goto ee_error;
    }
    pHeaderInfo->antennaGainMax[0] = (A_INT8)((tempValue >> 8) & 0xFF);
    pHeaderInfo->antennaGainMax[1] = (A_INT8)(tempValue & 0xFF);

    /* Read the moded sections of the EEPROM header in the order A, B, G */
    for (i = headerInfo11A; i <= headerInfo11G; i++) {
        /* Set the offset via the index */
        offset = pHeadOff[2 + i];

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->switchSettling[i] = (A_UINT16)((tempValue >> 8) & 0x7f);
        pHeaderInfo->txrxAtten[i] = (A_UINT16)((tempValue >> 2) & 0x3f);
        pHeaderInfo->antennaControl[0][i] = (A_UINT16)((tempValue << 4) & 0x3f);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->antennaControl[0][i] = (A_UINT16)(((tempValue >> 12) & 0x0f) | pHeaderInfo->antennaControl[0][i]);
        pHeaderInfo->antennaControl[1][i] = (A_UINT16)((tempValue >> 6) & 0x3f);
        pHeaderInfo->antennaControl[2][i] = (A_UINT16)(tempValue  & 0x3f);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->antennaControl[3][i] = (A_UINT16)((tempValue >> 10)  & 0x3f);
        pHeaderInfo->antennaControl[4][i] = (A_UINT16)((tempValue >> 4)  & 0x3f);
        pHeaderInfo->antennaControl[5][i] = (A_UINT16)((tempValue << 2)  & 0x3f);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->antennaControl[5][i] = (A_UINT16)(((tempValue >> 14)  & 0x03) | pHeaderInfo->antennaControl[5][i]);
        pHeaderInfo->antennaControl[6][i] = (A_UINT16)((tempValue >> 8)  & 0x3f);
        pHeaderInfo->antennaControl[7][i] = (A_UINT16)((tempValue >> 2)  & 0x3f);
        pHeaderInfo->antennaControl[8][i] = (A_UINT16)((tempValue << 4)  & 0x3f);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->antennaControl[8][i] = (A_UINT16)(((tempValue >> 12)  & 0x0f) | pHeaderInfo->antennaControl[8][i]);
        pHeaderInfo->antennaControl[9][i] = (A_UINT16)((tempValue >> 6)  & 0x3f);
        pHeaderInfo->antennaControl[10][i] = (A_UINT16)(tempValue & 0x3f);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->adcDesiredSize[i] = (A_INT8)((tempValue >> 8)  & 0xff);
        switch (i) {
        case headerInfo11A:
            pHeaderInfo->ob4 = (A_UINT16)((tempValue >> 5)  & 0x07);
            pHeaderInfo->db4 = (A_UINT16)((tempValue >> 2)  & 0x07);
            pHeaderInfo->ob3 = (A_UINT16)((tempValue << 1)  & 0x07);
            break;
        case headerInfo11B:
            pHeaderInfo->obFor24 = (A_UINT16)((tempValue >> 4)  & 0x07);
            pHeaderInfo->dbFor24 = (A_UINT16)(tempValue & 0x07);
            break;
        case headerInfo11G:
            pHeaderInfo->obFor24g = (A_UINT16)((tempValue >> 4)  & 0x07);
            pHeaderInfo->dbFor24g = (A_UINT16)(tempValue & 0x07);
            break;
        }

        if (i == headerInfo11A) {
            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_error;
            }
            pHeaderInfo->ob3 = (A_UINT16)(((tempValue >> 15)  & 0x01) | pHeaderInfo->ob3);
            pHeaderInfo->db3 = (A_UINT16)((tempValue >> 12)  & 0x07);
            pHeaderInfo->ob2 = (A_UINT16)((tempValue >> 9)  & 0x07);
            pHeaderInfo->db2 = (A_UINT16)((tempValue >> 6)  & 0x07);
            pHeaderInfo->ob1 = (A_UINT16)((tempValue >> 3)  & 0x07);
            pHeaderInfo->db1 = (A_UINT16)(tempValue & 0x07);
        }

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->txEndToXLNAOn[i] = (A_UINT16)((tempValue >> 8)  & 0xff);
        pHeaderInfo->thresh62[i] = (A_UINT16)(tempValue  & 0xff);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->txEndToXPAOff[i] = (A_UINT16)((tempValue >> 8)  & 0xff);
        pHeaderInfo->txFrameToXPAOn[i] = (A_UINT16)(tempValue  & 0xff);

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->pgaDesiredSize[i] = (A_INT8)((tempValue >> 8)  & 0xff);
        pHeaderInfo->noiseFloorThresh[i] = (tempValue  & 0xff);
        if (pHeaderInfo->noiseFloorThresh[i] & 0x80) {
            pHeaderInfo->noiseFloorThresh[i] = 0 - ((pHeaderInfo->noiseFloorThresh[i] ^ 0xff) + 1);
        }

        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->xlnaGain[i] = (A_UINT16)((tempValue >> 5)  & 0xff);
        pHeaderInfo->xgain[i] = (A_UINT16)((tempValue >> 1)  & 0x0f);
        pHeaderInfo->xpd[i] = (A_UINT8)(tempValue  & 0x01);

        if (version >= EEPROM_VER3_3) {
            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_error;
            }
            pHeaderInfo->falseDetectBackoff[i] = (tempValue >> 6) & 0x7F;
            switch (i) {
            case headerInfo11B:
                pHeaderInfo->ob2GHz[0] = tempValue & 0x7;
                pHeaderInfo->db2GHz[0] = (tempValue >> 3) & 0x7;
                break;
            case headerInfo11G:
                pHeaderInfo->ob2GHz[1] = tempValue & 0x7;
                pHeaderInfo->db2GHz[1] = (tempValue >> 3) & 0x7;
                break;
            }
        }
        if (version >= EEPROM_VER3_4) {
            pHeaderInfo->gainI[i] = (tempValue >> 13) & 0x07;

            status = ar5211EepromRead(pDev, offset++, &tempValue);
            if (status != A_OK) {
               goto ee_error;
            }
            pHeaderInfo->gainI[i] |= (tempValue << 3) & 0x38;
        } else {
            pHeaderInfo->gainI[i] = 10;
        }
    }
    if (version < EEPROM_VER3_3) {
        /* Version 3.1+ specific parameters */
        status = ar5211EepromRead(pDev, 0xec, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->ob2GHz[0] = tempValue & 0x7;
        pHeaderInfo->db2GHz[0] = (tempValue >> 3) & 0x7;

        status = ar5211EepromRead(pDev, 0xed, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->ob2GHz[1] = tempValue & 0x7;
        pHeaderInfo->db2GHz[1] = (tempValue >> 3) & 0x7;

    }
    
    /* Initialize corner cal (thermal tx gain adjust parameters) */
    pHeaderInfo->cornerCal.clip = 4;
    pHeaderInfo->cornerCal.pd90 = 1;
    pHeaderInfo->cornerCal.pd84 = 1;
    pHeaderInfo->cornerCal.gSel = 0;

    /*
     * Read the conformance test limit identifiers
     * These are used to match regulatory domain testing needs with
     * the RD-specific tests that have been calibrated in the EEPROM.
     */
    offset = pHeadOff[5];
    for (i = 0; i < pHeaderInfo->numCtls; i += 2) {
        status = ar5211EepromRead(pDev, offset++, &tempValue);
        if (status != A_OK) {
           goto ee_error;
        }
        pHeaderInfo->ctl[i] = (A_UINT16)((tempValue >> 8) & 0xff);
        pHeaderInfo->ctl[i+1] = (A_UINT16)(tempValue & 0xff);
    }

    /* WAR for recent changes to NF scale */
    if (version <= EEPROM_VER3_2) {
        pHeaderInfo->noiseFloorThresh[headerInfo11A] = -54;
        pHeaderInfo->noiseFloorThresh[headerInfo11B] = -1;
        pHeaderInfo->noiseFloorThresh[headerInfo11G] = -1;
    }

    /* WAR to override thresh62 for better 2.4 and 5 operation */
    if (version <= EEPROM_VER3_2) {
        pHeaderInfo->thresh62[headerInfo11A] = 15; // 11A
        pHeaderInfo->thresh62[headerInfo11B] = 28; // 11B
        pHeaderInfo->thresh62[headerInfo11G] = 28; // 11G
    }

    return A_OK;

ee_error:
    uiPrintf("ar5211Attach: EEPROM read failed\n");
    return A_HARDWARE;
}

void
getPcdacInterceptsFromPcdacMinMax(A_UINT16 version, A_UINT16 pcdacMin,
    A_UINT16 pcdacMax, A_UINT16 *pPcdacValues)
{
    const static A_UINT16 intercepts3[] = {0,5,10,20,30,50,70,85,90,95,100};
    const static A_UINT16 intercepts3_2[] = {0,10,20,30,40,50,60,70,80,90,100};

    A_UINT16 i;
    const A_UINT16 *pIntercepts;

    if (version >= EEPROM_VER3_2) {
        pIntercepts = intercepts3_2;
    } else {
        pIntercepts = intercepts3;
    }

    /* loop for the percentages in steps or 5 */
    for (i = 0; i < NUM_INTERCEPTS; i++ ) {
        *pPcdacValues =  (A_UINT16)((pIntercepts[i] * pcdacMax + (100 - pIntercepts[i]) * pcdacMin) / 100);
        pPcdacValues++;
    }
}

/**************************************************************************
 * fbin2freq - Get channel value from binary representation held in eeprom
 *
 * RETURNS: the frequency in MHz
 */
A_UINT16
fbin2freq(A_UINT16 version, A_UINT16 fbin)
{
    A_UINT16 returnValue; 

    if (version <= EEPROM_VER3_2) {
        returnValue = (fbin>62) ? (A_UINT16)(5100 + 10*62 + 5*(fbin-62)) : (A_UINT16)(5100 + 10*fbin);
    } else {
        returnValue = (A_UINT16)(4800 + 5*fbin);
    }
    return returnValue;
}

A_UINT16
fbin2freq_2p4(A_UINT16 version, A_UINT16 fbin)
{
    A_UINT16 returnValue; 

    if (version <= EEPROM_VER3_2) {
        returnValue = (A_UINT16)(2400 + fbin);
    } else {
        returnValue = (A_UINT16)(2300 + fbin);
    }
    return returnValue;
}

/************** EEPROM REV 3 DEBUG PRINT FUNCTIONS *****************/
#if defined(DEBUG) && defined(BUILD_AP)
static void
printHeaderInfo(WLAN_DEV_INFO *pDev, EEP_MAP *pEepMap, A_UINT32 mode)
{
    A_UINT16 i, j, modeArray;
    EEP_HEADER_INFO *pHeaderInfo = pEepMap->pEepHeader;
    A_UINT16 version = pEepMap->version;
    A_UINT32 regDmn = pDev->pHalInfo->halCapabilities.halRegDmn;

    switch(mode) {
    case MODE_11A:
        modeArray = 0;
        break;

    case MODE_OFDM_24:
        modeArray = 2;
        break;

    case MODE_11B:
        modeArray = 1;
        break;

    default:
        uiPrintf("Illegal mode passed to printHeaderInfo\n");
        return;
    } //end switch

    uiPrintf("\n");
    uiPrintf(" =======================Header Information======================\n");

    uiPrintf(" |  Major Version           %2d  ", version >> 12);
    uiPrintf("|  Minor Version           %2d  |\n", version & 0xFFF);
    uiPrintf(" |-------------------------------------------------------------|\n");

    if(version >= EEPROM_VER3_1) {
        uiPrintf(" |  A Mode         %1d  ", pHeaderInfo->Amode);
        uiPrintf("|  B Mode         %1d  ", pHeaderInfo->Bmode);
        uiPrintf("|  G Mode        %1d  |\n", pHeaderInfo->Gmode);
    }
    else {
        uiPrintf(" |  A Mode                   %1d  ", pHeaderInfo->Amode);
        uiPrintf("|  B Mode                   %1d  |\n", pHeaderInfo->Bmode);
    }

    if(regDmn >> 15) {
        uiPrintf(" |  Country Code %03x  ", regDmn >> 15);
    } else {
        uiPrintf(" |  Reg. Domain  %03x  ", regDmn & 0xFFF);
    }

    uiPrintf("|  turbo Disable  %1d  ", pHeaderInfo->turboDisable);
    uiPrintf("|  RF Silent     %1d  |\n", pEepMap->pEepHeader->rfKill);
    uiPrintf(" |-------------------------------------------------------------|\n");
    if(version >= EEPROM_VER3_3) {
        uiPrintf(" |  global mode              %1x  ", (regDmn & 0xF0) == 0x60);
        uiPrintf("|  False detect backoff  0x%02x  |\n", pHeaderInfo->falseDetectBackoff[modeArray]);
    }
    uiPrintf(" |  device type              %1x  ", pHeaderInfo->deviceType);

    uiPrintf("|  Switch Settling Time  0x%02x  |\n", pHeaderInfo->switchSettling[modeArray]);
    uiPrintf(" |  ADC Desired size       %2d  ", pHeaderInfo->adcDesiredSize[modeArray]);
    uiPrintf("|  XLNA Gain             0x%02x  |\n", pHeaderInfo->xlnaGain[modeArray]);

    uiPrintf(" |  tx end to XLNA on     0x%02x  ", pHeaderInfo->txEndToXLNAOn[modeArray]);
    uiPrintf("|  Threashold 62         0x%02x  |\n", pHeaderInfo->thresh62[modeArray]);

    uiPrintf(" |  tx end to XPA off     0x%02x  ", pHeaderInfo->txEndToXPAOff[modeArray]);
    uiPrintf("|  tx end to XPA on      0x%02x  |\n", pHeaderInfo->txFrameToXPAOn[modeArray]);

    uiPrintf(" |  PGA Desired size       %2d  ", pHeaderInfo->pgaDesiredSize[modeArray]);
    uiPrintf("|  Noise Threshold        %2d  |\n", pHeaderInfo->noiseFloorThresh[modeArray]);

    uiPrintf(" |  XPD Gain              0x%02x  ", pHeaderInfo->xgain[modeArray]);
    uiPrintf("|  XPD                      %1d  |\n", pHeaderInfo->xpd[modeArray]);

    uiPrintf(" |  txrx Attenuation      0x%02x  ", pHeaderInfo->txrxAtten[modeArray]);
    
    
    uiPrintf("|  Antenna control    0  0x%02X  |\n", pHeaderInfo->antennaControl[0][modeArray]);
    for(j = 1; j <= 10; j+=2) {
        uiPrintf(" |  Antenna control   %2d  0x%02X  ", j, pHeaderInfo->antennaControl[j][modeArray]);
        uiPrintf("|  Antenna control   %2d  0x%02X  |\n", j + 1, pHeaderInfo->antennaControl[j + 1][modeArray]);
    }

    uiPrintf(" |-------------------------------------------------------------|\n");
    if (mode == MODE_11A) {
        uiPrintf(" |   OB_1   %1d   ", pHeaderInfo->ob1);
        uiPrintf("|   OB_2    %1d   ", pHeaderInfo->ob2);
        uiPrintf("|   OB_3   %1d  ", pHeaderInfo->ob3);
        uiPrintf("|   OB_4     %1d   |\n", pHeaderInfo->ob4);
        uiPrintf(" |   DB_1   %1d   ", pHeaderInfo->db1);
        uiPrintf("|   DB_2    %1d   ", pHeaderInfo->db2);
        uiPrintf("|   DB_3   %1d  ", pHeaderInfo->db3);
        uiPrintf("|   DB_4     %1d   |\n", pHeaderInfo->db4);
    } else {
        if(version >= EEPROM_VER3_1) {
            if (mode == MODE_11B) {
                uiPrintf(" |   OB_1   %1d   ", pHeaderInfo->obFor24);
                uiPrintf("|   B_OB    %1d   ", pHeaderInfo->ob2GHz[0]);
                uiPrintf("|   DB_1   %1d  ", pHeaderInfo->dbFor24);
                uiPrintf("|   B_DB     %1d   |\n", pHeaderInfo->db2GHz[0]);
            } else {
                uiPrintf(" |   OB_1   %1d   ", pHeaderInfo->obFor24g);
                uiPrintf("|   B_OB    %1d   ", pHeaderInfo->ob2GHz[1]);
                uiPrintf("|   DB_1   %1d  ", pHeaderInfo->dbFor24g);
                uiPrintf("|   B_DB     %1d   |\n", pHeaderInfo->db2GHz[1]);
            }
        } else {
            if (mode == MODE_11B) {
                uiPrintf(" |  OB_1                     %1d  ", pHeaderInfo->obFor24);
                uiPrintf("|  DB_1                     %1d  |\n", pHeaderInfo->dbFor24);
            } else {
                uiPrintf(" |  OB_1                     %1d  ", pHeaderInfo->obFor24g);
                uiPrintf("|  DB_1                     %1d  |\n", pHeaderInfo->dbFor24g);
            }

        }
        
    }
    if ((mode == MODE_11A) && (version == EEPROM_VER3_2)) {
        uiPrintf(" |-------------------------------------------------------------|\n");
        for(i = 0; i < 4; i++) {
            uiPrintf(" |  Gsel_%1d  %1d   ", i, pHeaderInfo->cornerCal.gSel);
            uiPrintf("|  Pd84_%1d   %1d   ", i, pHeaderInfo->cornerCal.pd84);
            uiPrintf("|  Pd90_%1d  %1d  ", i, pHeaderInfo->cornerCal.pd90);
            uiPrintf("|  clip_%1d    %1d   |\n", i, pHeaderInfo->cornerCal.clip);
        }
    }
    uiPrintf(" ===============================================================\n");
    return;

}

static void
printChannelInfo(PCDACS_ALL_MODES *pEepromData, A_UINT32 mode)
{
    A_UINT16            i, j, k = 0;
    DATA_PER_CHANNEL    *pDataPerChannel;

    switch(mode) {
    case MODE_11A:
        pDataPerChannel = pEepromData->DataPerChannel11a;
        break;

    case MODE_OFDM_24:
        pDataPerChannel = pEepromData->DataPerChannel11g;
        break;

    case MODE_11B:
        pDataPerChannel = pEepromData->DataPerChannel11b;
        break;

    default:
        uiPrintf("ar5211Attach: Illegal mode passed to printChannelInfo\n");
        return;
    }
    
    uiPrintf("\n");
    if (mode == MODE_11A) {
        uiPrintf("=========================Calibration Information============================\n");
        
        for (k = 0; k < 10; k+=5) {
            for (i = k; i < k + 5; i++) {
                uiPrintf("|     %04d     ",
                    pDataPerChannel[i].channelValue);
            }
            uiPrintf("|\n");
            
            uiPrintf("|==============|==============|==============|==============|==============|\n");
            for (i = k; i < k + 5; i++) {
                uiPrintf("|pcdac pwr(dBm)");
            }
            uiPrintf("|\n");

            for (j = 0; j < pDataPerChannel[0].numPcdacValues; j++) {
                for (i = k; i < k + 5; i++) {
                    uiPrintf("|  %02d    %2d.%02d ",
                        pDataPerChannel[i].PcdacValues[j],
                        pDataPerChannel[i].PwrValues[j] / EEP_SCALE,
                        pDataPerChannel[i].PwrValues[j] % EEP_SCALE);
                }
                uiPrintf("|\n");
            }
            
            uiPrintf("|              |              |              |              |              |\n"); 
            for (i = k; i < k + 5; i++) {
                uiPrintf("| pcdac min %02d ", pDataPerChannel[i].pcdacMin);
            }
            uiPrintf("|\n");
            for (i = k; i < k + 5; i++) {
                uiPrintf("| pcdac max %02d ", pDataPerChannel[i].pcdacMax);
            }
            uiPrintf("|\n");
            uiPrintf("|==============|==============|==============|==============|==============|\n");
        }
    } else {
        uiPrintf("               ==========Calibration Information=============\n");
        
        for (i = 0; i < 3; i++) {
            if (0 == i) {
                uiPrintf("               ");
            }
            uiPrintf("|     %04d     ",
                pDataPerChannel[i].channelValue);
        }
        uiPrintf("|\n");
        
        uiPrintf("               |==============|==============|==============|\n");
        for (i = 0; i < 3; i++) {
            if (0 == i) {
                uiPrintf("               ");
            }
            uiPrintf("|pcdac pwr(dBm)");
        }
        uiPrintf("|\n");

        for (j = 0; j < pDataPerChannel[0].numPcdacValues; j++) {
            for (i = 0; i < 3; i++) {
                if (0 == i) {
                    uiPrintf("               ");
                }
                uiPrintf("|  %02d    %2d.%02d ",
                    pDataPerChannel[i].PcdacValues[j],
                    pDataPerChannel[i].PwrValues[j] / EEP_SCALE,
                    pDataPerChannel[i].PwrValues[j] % EEP_SCALE);

            }
            uiPrintf("|\n");
        }
        
        uiPrintf("               |              |              |              |\n");    
        uiPrintf("               ");
        for (i = 0; i < 3; i++) {
            uiPrintf("| pcdac min %02d ", pDataPerChannel[i].pcdacMin);
        }
        uiPrintf("|\n");
        uiPrintf("               ");
        for (i = 0; i < 3; i++) {
            uiPrintf("| pcdac max %02d ", pDataPerChannel[i].pcdacMax);
        }
        uiPrintf("|\n");
        uiPrintf("               |==============|==============|==============|\n");

    }
}

static void
printTargetPowerInfo(TRGT_POWER_ALL_MODES *pPowerInfoAllModes, A_UINT32 mode)
{
    A_UINT16 i, k;
    TRGT_POWER_INFO     *pPowerInfo;

    uiPrintf("\n");
    if (mode == MODE_11A) {
        pPowerInfo = pPowerInfoAllModes->trgtPwr_11a;
        uiPrintf("============================Target Power Info===============================\n");
    
        for (k = 0; k < 8; k+=4) {
            uiPrintf("|     rate     ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %04d     ", pPowerInfo[i].testChannel);
            }
            uiPrintf("|\n");
            
            uiPrintf("|==============|==============|==============|==============|==============|\n");

            uiPrintf("|     6-24     ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr6_24)/2);
            }
            uiPrintf("|\n");

            uiPrintf("|      36      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr36)/2);
            }
            uiPrintf("|\n");

            uiPrintf("|      48      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr48)/2);
            }
            uiPrintf("|\n");

            uiPrintf("|      54      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr54)/2);
            }

            uiPrintf("|\n");
            uiPrintf("|==============|==============|==============|==============|==============|\n");
        }
    } else {
        if (mode == MODE_11B) {
            pPowerInfo = pPowerInfoAllModes->trgtPwr_11b;
        } else {
            pPowerInfo = pPowerInfoAllModes->trgtPwr_11g;
        }
        uiPrintf("=============Target Power Info================\n");

        uiPrintf("|     rate     ");
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %04d     ",
                pPowerInfo[i].testChannel);
        }
        uiPrintf("|\n");
        
        uiPrintf("|==============|==============|==============|\n");

        if (mode == MODE_11B) {
            uiPrintf("|      1       ");
        } else {
            uiPrintf("|     6-24     ");
        }

        for (i = 0; i < 2; i++) {
            uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr6_24)/2);
        }
        uiPrintf("|\n");

        if (mode == MODE_11B) {
            uiPrintf("|      2       ");
        } else {
            uiPrintf("|      36      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr36)/2);
        }
        uiPrintf("|\n");

        if (mode == MODE_11B) {
            uiPrintf("|      5.5     ");
        } else {
            uiPrintf("|      48      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr48)/2);
        }
        uiPrintf("|\n");

        if (mode == MODE_11B) {
            uiPrintf("|      11      ");
        } else {
            uiPrintf("|      54      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %4.1f     ", (float)(pPowerInfo[i].twicePwr54)/2);
        }

        uiPrintf("|\n");
        uiPrintf("|==============|==============|==============|\n");

    }
}

static void
printRDEdges(RD_EDGES_POWER *pRdEdgePwrInfo, A_UINT16 *pTestGroups,
             A_UINT32 mode, A_UINT16 maxNumCtl, A_UINT16 version)
{
    A_UINT16    i=0, j;
    A_UINT16    ctlMode = 0;
    const static char *ctlType[4] = {
        "11a base mode ] ",
        "11b mode ]      ",
        "11g mode ]      ",
        "11a TURBO mode ]"
    };

    uiPrintf("\n");
    uiPrintf("=======================Test Group Band Edge Power========================\n");
    while ((pTestGroups[i] != 0) && (i < maxNumCtl)) {
        switch(pTestGroups[i] & 0x3) {
        case 0: 
            ctlMode = MODE_11A;
            break;
        case 3:
            ctlMode = MODE_11A;
            break;
        case 1: 
            ctlMode = MODE_11B;
            break;
        case 2: 
            ctlMode = MODE_OFDM_24;
            break;
        }
        if (mode != ctlMode) {
            i++;
            pRdEdgePwrInfo += NUM_EDGES;
            continue;
        }
        uiPrintf("|                                                                       |\n");
        uiPrintf("| CTL: 0x%02x   [ 0x%x %s", pTestGroups[i] & 0xff, pTestGroups[i], ctlType[pTestGroups[i] & 0x3]);
        
        uiPrintf("                                   |\n");
        uiPrintf("|=======|=======|=======|=======|=======|=======|=======|=======|=======|\n");

        uiPrintf("| edge  ");
        for (j = 0; j < NUM_EDGES; j++) {
            if (pRdEdgePwrInfo[j].rdEdge == 0) {
                uiPrintf("|  --   ");
            } else {
                uiPrintf("| %04d  ", pRdEdgePwrInfo[j].rdEdge);
            }
        }

        uiPrintf("|\n");
        uiPrintf("|=======|=======|=======|=======|=======|=======|=======|=======|=======|\n");
        uiPrintf("| power ");
        for (j = 0; j < NUM_EDGES; j++) {
            if (pRdEdgePwrInfo[j].rdEdge == 0) {
                uiPrintf("|  --   ");
            } else {
                uiPrintf("| %4.1f  ", (float)(pRdEdgePwrInfo[j].twice_rdEdgePower)/2);
            }
        }

        uiPrintf("|\n");
        if (version >= EEPROM_VER3_3) {
            uiPrintf("|=======|=======|=======|=======|=======|=======|=======|=======|=======|\n");
            uiPrintf("| flag  ");
            for (j = 0; j < NUM_EDGES; j++) {
                if (pRdEdgePwrInfo[j].rdEdge == 0) {
                    uiPrintf("|  --   ");
                } else {
                    uiPrintf("|   %1d   ", pRdEdgePwrInfo[j].flag);
                }
            }

            uiPrintf("|\n");
        }
        uiPrintf("=========================================================================\n");
        i++;
        pRdEdgePwrInfo += NUM_EDGES;
    }
}

static void
printEepromStruct(WLAN_DEV_INFO *pDev, EEP_MAP *pEepMap, A_UINT32 mode)
{
    printHeaderInfo(pDev, pEepMap, mode);
    printChannelInfo(pEepMap->pPcdacInfo, mode);
    printTargetPowerInfo(pEepMap->pTrgtPowerInfo, mode);
    printRDEdges(pEepMap->pRdEdgesPower, pEepMap->pEepHeader->ctl, mode, pEepMap->pEepHeader->numCtls,
        pEepMap->version);
    return;
}

void
dumpEeprom(int unit)
{
    WLAN_DEV_INFO *pDev = gDrvInfo.pDev[unit];

    EEP_MAP *pEepMap;
    ASSERT(pDev->pHalInfo->pEepData);

    pEepMap = pDev->pHalInfo->pEepData;
    printEepromStruct(pDev, pEepMap, MODE_11A);
    printEepromStruct(pDev, pEepMap, MODE_11B);
    printEepromStruct(pDev, pEepMap, MODE_OFDM_24);
    return;
}
#endif


#endif /* #ifdef BUILD_AR5211 */
