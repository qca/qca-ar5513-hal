/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * Chips specific device attachment and device info collection
 * Connects Init Reg Vectors, EEPROM Data, and device Functions to pDev
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Reset.c#2 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "wlanchannel.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"
#include "halDevId.h"
#include "vport.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Misc.h"
#include "ar5211Reset.h"
#include "ar5211Power.h"
#include "ar5211Receive.h"
#include "ar5211.h"

#if defined(AR531X)
#include "ar531xreg.h"
#endif

/* Add static register initialization vectors */
#include "ar5211/boss.ini"

/*
 * Structure to hold 11b tuning information for Beanie/Sombrero
 * 16 MHz mode, divider ratio = 198 = NP+S. N=16, S=4 or 6, P=12
 */
typedef struct ChanInfo2Ghz {
    A_UINT32    refClkSel;      // reference clock select, 1 for 16 MHz
    A_UINT32    channelSelect;  // P[7:4]S[3:0] bits
    A_UINT16    channel5111;    // 11a channel for 5111
} CHAN_INFO_2GHZ;

#define CI_2GHZ_INDEX_CORRECTION 19
const static CHAN_INFO_2GHZ chan2GHzData[] = {
    { 1, 0x46, 96  }, // 2312 -19
    { 1, 0x46, 97  }, // 2317 -18
    { 1, 0x46, 98  }, // 2322 -17
    { 1, 0x46, 99  }, // 2327 -16
    { 1, 0x46, 100 }, // 2332 -15
    { 1, 0x46, 101 }, // 2337 -14
    { 1, 0x46, 102 }, // 2342 -13
    { 1, 0x46, 103 }, // 2347 -12
    { 1, 0x46, 104 }, // 2352 -11
    { 1, 0x46, 105 }, // 2357 -10
    { 1, 0x46, 106 }, // 2362  -9
    { 1, 0x46, 107 }, // 2367  -8
    { 1, 0x46, 108 }, // 2372  -7
    /* index -6 to 0 are pad to make this a nolookup table */
    { 1, 0x46, 116 }, //       -6
    { 1, 0x46, 116 }, //       -5
    { 1, 0x46, 116 }, //       -4
    { 1, 0x46, 116 }, //       -3
    { 1, 0x46, 116 }, //       -2
    { 1, 0x46, 116 }, //       -1
    { 1, 0x46, 116 }, //        0
    { 1, 0x46, 116 }, // 2412   1
    { 1, 0x46, 117 }, // 2417   2
    { 1, 0x46, 118 }, // 2422   3
    { 1, 0x46, 119 }, // 2427   4
    { 1, 0x46, 120 }, // 2432   5
    { 1, 0x46, 121 }, // 2437   6
    { 1, 0x46, 122 }, // 2442   7
    { 1, 0x46, 123 }, // 2447   8
    { 1, 0x46, 124 }, // 2452   9
    { 1, 0x46, 125 }, // 2457  10
    { 1, 0x46, 126 }, // 2462  11
    { 1, 0x46, 127 }, // 2467  12
    { 1, 0x46, 128 }, // 2472  13
    { 1, 0x44, 124 }, // 2484  14
    { 1, 0x46, 136 }, // 2512  15
    { 1, 0x46, 140 }, // 2532  16
    { 1, 0x46, 144 }, // 2552  17
    { 1, 0x46, 148 }, // 2572  18
    { 1, 0x46, 152 }, // 2592  19
    { 1, 0x46, 156 }, // 2612  20
    { 1, 0x46, 160 }, // 2632  21
    { 1, 0x46, 164 }, // 2652  22
    { 1, 0x46, 168 }, // 2672  23
    { 1, 0x46, 172 }, // 2692  24
    { 1, 0x46, 176 }, // 2712  25
    { 1, 0x46, 180 }  // 2732  26
};

const static GAIN_OPTIMIZATION_LADDER GainLadder = {
    9, //numStepsInLadder
    4, //defaultStepNum
    { { {4, 1, 1, 1},  6, "FG8"},
      { {4, 0, 1, 1},  4, "FG7"},
      { {3, 1, 1, 1},  3, "FG6"},
      { {4, 0, 0, 1},  1, "FG5"},
      { {4, 1, 1, 0},  0, "FG4"}, // noJack
      { {4, 0, 1, 0}, -2, "FG3"}, // halfJack
      { {3, 1, 1, 0}, -3, "FG2"}, // clip3
      { {4, 0, 0, 0}, -4, "FG1"}, // noJack
      { {2, 1, 1, 0}, -6, "FG0"}  // clip2
    }
};

static A_INT16
ar5211RunNoiseFloor(WLAN_DEV_INFO *pDev, A_UINT8 runTime, A_INT16 startingNF);

static A_INT16
ar5211GetNf(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_STATUS
ar5211CalNoiseFloor(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_STATUS
ar5211SetResetReg(WLAN_DEV_INFO *pDev, A_UINT32 resetMask);

static A_STATUS
ar5211SetChannel(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static void
ar5211SetRf6and7(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval);

static void
ar5211SetBoardValues(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval);

static A_STATUS
ar5211SetTransmitPower(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static void
ar5211SetPowerTable(WLAN_DEV_INFO *pDev, PCDACS_EEPROM *pSrcStruct, A_UINT16 channel);

static void
ar5211SetRateTable(WLAN_DEV_INFO *pDev, RD_EDGES_POWER *pRdEdgesPower,
                   TRGT_POWER_INFO *pPowerInfo, A_UINT16 numChannels,
                   CHAN_VALUES *pChval, A_UINT16 tpcScaleReduction,
                   struct eepMap *pData);

static A_UINT16
ar5211GetScaledPower(A_UINT16 channel, A_UINT16 pcdacValue, PCDACS_EEPROM *pSrcStruct);

static A_BOOL
ar5211FindValueInList(A_UINT16 channel, A_UINT16 pcdacValue, PCDACS_EEPROM *pSrcStruct, A_UINT16 *powerValue);

static A_UINT16
ar5211GetInterpolatedValue(A_UINT16 target, A_UINT16 srcLeft, A_UINT16 srcRight,
                           A_UINT16 targetLeft, A_UINT16 targetRight, A_BOOL scaleUp);

static void
ar5211GetLowerUpperValues(A_UINT16 value, A_UINT16 *pList, A_UINT16 listSize,
                          A_UINT16 *pLowerValue, A_UINT16 *pUpperValue);

static void
ar5211GetLowerUpperPcdacs(A_UINT16 pcdac, A_UINT16 channel, PCDACS_EEPROM *pSrcStruct,
                    A_UINT16 *pLowerPcdac, A_UINT16 *pUpperPcdac);

static void
ar5211SetRfgain(WLAN_DEV_INFO *pDevInfo);

static void
ar5211RequestRfgain(WLAN_DEV_INFO *pDevInfo);

static A_BOOL
ar5211InvalidGainReadback(GAIN_VALUES *pGainValues, A_UINT16 channelFlags);

static A_BOOL
ar5211IsGainAdjustNeeded(GAIN_VALUES *pGainValues);

static A_INT32
ar5211AdjustGain(GAIN_VALUES *pGainValues);

/* Additional Time delay to wait after activiting the Base band */
#define BASE_ACTIVATE_DELAY    100 //100 usec
#define PLL_SETTLE_DELAY       300 //300 usec

/*
 * WAR for bug 6773.  udelay() does a PIO READ on the PCI bus which allows
 * other cards' DMA reads to complete in the middle of our reset.
 */
#define WAR_6773(x)          \
    if ((++(x) % 64) == 0) { \
        udelay(1);           \
    }

/**************************************************************
 * ar5211Reset
 *
 * Places the device in and out of reset and then places sane
 * values in the registers based on EEPROM config, initialization
 * vectors (as determined by the mode), and station configuration
 *
 * bChannelChange is used to preserve DMA/PCU registers across
 * a HW Reset during channel change.
 */
A_STATUS
ar5211Reset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange)
{
    A_UINT32        ulAddressLow, ulAddressHigh;
    A_UINT32        macStaId1;
    A_UINT32        reg, data, synthDelay, i;
    A_STATUS        status;
    A_BOOL          is11b = IS_CHAN_CCK(pChval->channelFlags);
    A_UINT16        arrayEntries;
    A_UINT16        modesIndex = 0, freqIndex = 0;
    A_UINT32        saveFrameSeqCount[MAC_NUM_DCU];
    A_UINT32        saveTsfLow = 0, saveTsfHigh = 0;
    struct eepMap   *pEepData = pDev->pHalInfo->pEepData;
    WLAN_STA_CONFIG *pCfg = &pDev->staConfig;
    VPORT_BSS       *pVportBaseBss = GET_BASE_BSS(pDev);
    A_UINT32        saveDefAntenna;
    int             regWrites = 0;

    ASSERT(IS_CHAN_2GHZ(pChval->channelFlags) ^
           IS_CHAN_5GHZ(pChval->channelFlags));
    ASSERT(IS_CHAN_OFDM(pChval->channelFlags) ^
           IS_CHAN_CCK(pChval->channelFlags));
    ASSERT(pEepData->version >= EEPROM_VER3);

    /* Preserve certain DMA hardware registers on a channel change */
    if (bChannelChange) {
        /* AR5211 WAR
         * Need to save/restore the TSF because of a hardware bug
         * that accelerates the TSF during a chip reset.
         *
         * We could use system timer routines to more
         * accurately restore the TSF, but
         * 1. Timer routines on certain platforms are
         *    not accurate enough (e.g. 1 ms resolution).
         * 2. It would still not be accurate.
         *
         * The most important aspect of this workaround,
         * is that, after reset, the TSF is behind
         * other STAs TSFs (see bug 4127).  This will
         * allow the STA to properly resynchronize its
         * TSF in adhoc mode.
         */
        saveTsfLow  = readPlatformReg(pDev, MAC_TSF_L32);
        saveTsfHigh = readPlatformReg(pDev, MAC_TSF_U32);

        /* Read frame sequence count */
        if (pDev->macVersion >= MAC_SREV_VERSION_OAHU) {
            saveFrameSeqCount[0] = readPlatformReg(pDev, MAC_D0_SEQNUM);
        } else {
            for (i = 0; i < MAC_NUM_DCU; i++) {
                saveFrameSeqCount[i] = readPlatformReg(pDev, MAC_D0_SEQNUM + (i * sizeof(A_UINT32)));
            }
        }
        /* Save off the latest NF calibration */
        ar5211GetNf(pDev, pDev->staConfig.phwChannel);
    }

    /*
     * Preserve the antenna on a channel change
     */
    saveDefAntenna = readPlatformReg(pDev, MAC_DEF_ANTENNA);
    if (saveDefAntenna == 0) {
        saveDefAntenna = 1;
    }

    /* Save hardware flag before chip reset clears the register */
    macStaId1 = readPlatformReg(pDev, MAC_STA_ID1) & MAC_STA_ID1_BASE_RATE_11B;

    /* Adjust gain parameters before reset if there's an outstanding gain update */
    ar5211GetRfgain(pDev);

    status = ar5211ChipReset(pDev, pChval->channelFlags);
    if (status != A_OK) {
        return status;
    }

    /*
     * Initialize the global Interrupt Enable reference count.
     * Assume interrupts were disabled.
     */
    pDev->pHalInfo->globIntRefCount = 1;

    /* Setup the indices for the next set of register array writes */
    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
        modesIndex = 1;
        freqIndex  = 1;
        break;
    case CHANNEL_T:
        modesIndex = 2;
        freqIndex  = 1;
        break;
    case CHANNEL_B:
        modesIndex = 3;
        freqIndex  = 2;
        break;
    case CHANNEL_PUREG:
        modesIndex = 4;
        freqIndex  = 2;
        break;
    default:
        /* Ah, a new wireless mode */
        ASSERT(0);
        break;
    }

    /* Set correct Baseband to analog shift setting to access analog chips. */
    if (pDev->macVersion >= MAC_SREV_VERSION_OAHU) {
        writePlatformReg(pDev, PHY_BASE, 0x00000007);
    } else {
        writePlatformReg(pDev, PHY_BASE, 0x00000047);
    }

    /* Write parameters specific to AR5211 */
    if (pDev->macVersion >= MAC_SREV_VERSION_OAHU) {
        if (IS_CHAN_2GHZ(pChval->channelFlags)) {
            if (pEepData->version >= EEPROM_VER3_1) {
                A_UINT32 ob2GHz, db2GHz;

                if (IS_CHAN_CCK(pChval->channelFlags)) {
                    ob2GHz = pEepData->pEepHeader->ob2GHz[0];
                    db2GHz = pEepData->pEepHeader->db2GHz[0];
                } else {
                    ob2GHz = pEepData->pEepHeader->ob2GHz[1];
                    db2GHz = pEepData->pEepHeader->db2GHz[1];
                }
                ob2GHz = reverseBits(ob2GHz, 3);
                db2GHz = reverseBits(db2GHz, 3);
                ar5211Mode2_4[25][freqIndex] = (ar5211Mode2_4[25][freqIndex] & ~0xC0) |
                    ((ob2GHz << 6) & 0xC0);
                ar5211Mode2_4[26][freqIndex] = (ar5211Mode2_4[26][freqIndex] & ~0x0F) |
                    ( ((ob2GHz >> 2) & 0x1) | ((db2GHz << 1) & 0x0E) );
            }
            arrayEntries = sizeof(ar5211Mode2_4) / (sizeof(A_UINT32) * 3);
            for (i = 0; i < arrayEntries; i++) {
                writePlatformReg(pDev, ar5211Mode2_4[i][0], ar5211Mode2_4[i][freqIndex]);
            }
        }
    }

    /* Write the analog registers 6 and 7 before other config */
    ar5211SetRf6and7(pDev, pEepData->pEepHeader, pChval);

    /* Write registers that vary across all modes */
    arrayEntries = sizeof(ar5211Modes) / (sizeof(A_UINT32) * 5);
    for (i = 0; i < arrayEntries; i++) {
        writePlatformReg(pDev, ar5211Modes[i][0], ar5211Modes[i][modesIndex]);
        WAR_6773(regWrites);
    }

    arrayEntries = sizeof(ar5211BB_RfGain) / (sizeof(A_UINT32) * 3);
    /* Write RFGain Parameters that differ between 2.4 and 5 GHz */
    for (i = 0; i < arrayEntries; i++) {
        writePlatformReg(pDev, ar5211BB_RfGain[i][0], ar5211BB_RfGain[i][freqIndex]);
        WAR_6773(regWrites);
    }

    arrayEntries = sizeof(ar5211Common) / (sizeof(A_UINT32) * 2);
    /* Write Common Array Parameters */
    for (i = 0; i < arrayEntries; i++) {
        reg = ar5211Common[i][0];

        /* On channel change, don't reset the PCU registers */
        if ( !(bChannelChange && (reg >= 0x8000 && reg < 0x9000)) ) {
            writePlatformReg(pDev, reg, ar5211Common[i][1]);
            WAR_6773(regWrites);
        }
    }

    /* Fix pre-AR5211 register values, this includes AR531Xs. */
    if (pDev->macVersion < MAC_SREV_VERSION_OAHU) {
        /*
         * The TX and RX latency values have changed locations within the
         * USEC register in AR5211.  Since they're set via the .ini, for both
         * AR5211 and AR531X, they are written properly here for AR531X.
         */
        data = readPlatformReg(pDev, MAC_USEC);
        ASSERT((data & 0x00700000) == 0); // Must be 0 for proper write in AR531X
        writePlatformReg(pDev, MAC_USEC,
            (data & (MAC_USEC_M | MAC_USEC_32_M | MAC_531X_USEC_TX_LAT_M)) |
            ((29 << MAC_531X_USEC_RX_LAT_S) & MAC_531X_USEC_RX_LAT_M));

        /* The following registers exist only on AR531X. */
        writePlatformReg(pDev, MAC_531X_QDCLKGATE, 0);

        /* Set proper ADC & DAC delays for AR531X. */
        writePlatformReg(pDev, 0x00009878, 0x00000008);

        /* Enable the PCU FIFO corruption ECO on AR531X. */
        writePlatformReg(pDev, MAC_DIAG_SW,
                         readPlatformReg(pDev, MAC_DIAG_SW) | MAC_531X_DIAG_USE_ECO);
    }

    /* Restore certain DMA hardware registers on a channel change */
    if (bChannelChange) {
        /* AR5211 WAR - Restore TSF */
        writePlatformReg(pDev, MAC_TSF_L32, saveTsfLow);
        writePlatformReg(pDev, MAC_TSF_U32, saveTsfHigh);

        if (pDev->macVersion >= MAC_SREV_VERSION_OAHU) {
            writePlatformReg(pDev, MAC_D0_SEQNUM, saveFrameSeqCount[0]);
        } else {
            for (i = 0; i < MAC_NUM_DCU; i++) {
                writePlatformReg(pDev, MAC_D0_SEQNUM + (i * sizeof(A_UINT32)),
                                 saveFrameSeqCount[i]);
            }
        }
    }

    ulAddressLow  = cpu2le32(pCfg->macAddr.st.word),
    ulAddressHigh = cpu2le16(pCfg->macAddr.st.half);
    writePlatformReg(pDev, MAC_STA_ID0, ulAddressLow);

    if (serviceType == WLAN_AP_SERVICE) {
        writePlatformReg(pDev, MAC_STA_ID1, macStaId1 | ulAddressHigh |
                         MAC_STA_ID1_STA_AP |
                         MAC_STA_ID1_RTS_USE_DEF);
        writePlatformReg(pDev, MAC_BSS_ID0, ulAddressLow);
        writePlatformReg(pDev, MAC_BSS_ID1, ulAddressHigh);
    } else {
        /*
         * Set the IBSS mode only if we were configured to be so. If bssType is
         * ANY_BSS then the scan will determine what mode we finally end up in.
         */
        ASSERT((pCfg->bssType == INDEPENDENT_BSS) || (pCfg->bssType == INFRASTRUCTURE_BSS));
        writePlatformReg(pDev, MAC_STA_ID1, macStaId1 | ulAddressHigh |
                         ((pCfg->bssType == INDEPENDENT_BSS) ? MAC_STA_ID1_AD_HOC : 0) |
                         MAC_STA_ID1_RTS_USE_DEF);
        /* then our BSSID */
        ulAddressLow  = cpu2le32(pVportBaseBss->bss.bssId.st.word);
        ulAddressHigh = cpu2le16(pVportBaseBss->bss.bssId.st.half);
        writePlatformReg(pDev, MAC_BSS_ID0, ulAddressLow);
        writePlatformReg(pDev, MAC_BSS_ID1, ulAddressHigh);
    }

    /* Restore previous antenna */
    writePlatformReg(pDev, MAC_DEF_ANTENNA, saveDefAntenna);

    writePlatformReg(pDev, MAC_ISR, 0xFFFFFFFF); // cleared on write

    /* Set CFG, little endian for register access. */
#ifdef BIG_ENDIAN
    writePlatformReg(pDev, MAC_CFG, MAC_CFG_SWTD | MAC_CFG_SWRD);
#else
    writePlatformReg(pDev, MAC_CFG, INIT_CONFIG_STATUS);
#endif

    /* AR5211 WAR - pre-Production Oahu only.
     * Disable clock gating in all DMA blocks. This is a workaround for
     * a hardware problem seen with 11B and AES.
     * This workaround will result in higher power consumption.
     */
    if ((pDev->macVersion == MAC_SREV_VERSION_OAHU) &&
        (pDev->macRev < MAC_SREV_OAHU_PROD))
    {
        writePlatformReg(pDev, MAC_CFG,
            readPlatformReg(pDev, MAC_CFG) | MAC_5211_CFG_CLK_GATE_DIS);
    }

    writePlatformReg(pDev, MAC_RSSI_THR, INIT_RSSI_THR);

    /* Setup the transmit power values. */
    if (ar5211SetTransmitPower(pDev, pChval) != A_OK) {
        return A_ERROR;
    }

    /* Setup board specific options for EEPROM version 3 */
    ar5211SetBoardValues(pDev, pEepData->pEepHeader, pChval);

    /* Write additional initialization registers for overwrite debug functionality */
    if (pDev->pInitRegs) {
        for (i = 0; i < MAX_REG_ADD_COUNT; i++) {
            if (pDev->pInitRegs[i].Offset == 0) {
                break;
            }
            writePlatformReg(pDev, pDev->pInitRegs[i].Offset, pDev->pInitRegs[i].Value);
        }
    }

    status = ar5211SetChannel(pDev, pChval);
    if (status != A_OK) {
        return status;
    }

    /* Activate the PHY */
    if ((pDev->pciInfo.DeviceID == AR5211_FPGA11B) &&
        IS_CHAN_2GHZ(pChval->channelFlags))
    {
        writePlatformReg(pDev, 0xd808, 0x502); // required for FPGA
    }
    writePlatformReg(pDev, PHY_ACTIVE, PHY_ACTIVE_EN);

    /*
     * Wait for the frequency synth to settle (synth goes on via PHY_ACTIVE_EN).
     * Read the phy active delay register. Value is in 100ns increments.
     */
    data = readPlatformReg(pDev, PHY_RX_DELAY) & PHY_RX_DELAY_M;

    if (IS_CHAN_CCK(pChval->channelFlags)) {
        synthDelay = (4 * data) / 22;
    } else {
        synthDelay = data / 10;
    }

    /*
     * There is an issue if the AP starts the calibration before the base
     * band timeout completes, could result in the rx_clear false triggering.
     * Added an extra delay, BASE_ACTIVATE_DELAY, to ensure that this
     * condition will not happen.
     */
    udelay(synthDelay + BASE_ACTIVATE_DELAY);


    /* Calibrate the AGC and poll the bit going to 0 for completion. */
    writePlatformReg(pDev, PHY_AGC_CONTROL,
                     readPlatformReg(pDev, PHY_AGC_CONTROL) | PHY_AGC_CONTROL_CAL);

    for (i = 0; i < 10000; i++) {
        if ( (readPlatformReg(pDev, PHY_AGC_CONTROL) &
              PHY_AGC_CONTROL_CAL) == 0 ) {
            break;
        }
        udelay(50);
    }

    /* Perform noise floor and set status */
    status = ar5211CalNoiseFloor(pDev, pChval);
    if (status != A_OK) {
        pChval->channelFlags |= (is11b ? 0 : CHANNEL_CW_INT);
        return status;
    }

    /*
     * Start IQ calibration w/ 2^(INIT_IQCAL_LOG_COUNT_MAX+1) samples
     * -Cal is not necessary for CCK-only operation
     * -Don't run cal if previous results exist (and no current request)
     */
    if ( (pCfg->calibrationTime != 0) && !IS_CHAN_B(pChval->channelFlags) && 
         !(pDev->pHalInfo->iqCalState == IQ_CAL_DONE) )
    {
        /* Start IQ calibration w/ 2^(INIT_IQCAL_LOG_COUNT_MAX+1) samples */
        A_REG_WR(pDev, PHY_TIMING_CTRL4,
            PHY_TIMING_CTRL4_DO_IQCAL |
            A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCAL_LOG_COUNT_MAX, pCfg->iqLogCountMax));
        pDev->pHalInfo->iqCalState = IQ_CAL_RUNNING;
    } else {
        pDev->pHalInfo->iqCalState = IQ_CAL_INACTIVE;
    }

    /* Set 1:1 QCU to DCU mapping for all queues */
    for (i = 0; i < MAC_NUM_DCU; i++) {
        writePlatformReg(pDev, MAC_D0_QCUMASK + (i * sizeof(A_UINT32)), 1 << i);
    }

    /* Setup QCU0 transmit interrupt masks (TX_ERR, TX_OK, TX_DESC, TX_URN) */
    writePlatformReg(pDev, MAC_IMR_S0,
                     (MAC_IMR_S0_QCU_TXOK_M & MAC_QCU_0) |
                     (MAC_IMR_S0_QCU_TXDESC_M & (MAC_QCU_0<<MAC_IMR_S0_QCU_TXDESC_S)));
    writePlatformReg(pDev, MAC_IMR_S1, (MAC_IMR_S1_QCU_TXERR_M & MAC_QCU_0));
    writePlatformReg(pDev, MAC_IMR_S2, (MAC_IMR_S2_QCU_TXURN_M & MAC_QCU_0));

    /*
     * DPD Added 4/19 from mainline:
     * AR5211 WAR - GBL_EIFS must always be written after writing to any QCUMASK register
     */
    writePlatformReg(pDev, MAC_D_GBL_IFS_EIFS, readPlatformReg(pDev, MAC_D_GBL_IFS_EIFS));

    /* Now set up the Interrupt Mask Register and save it for future use */
    pDev->MaskReg = INIT_INTERRUPT_MASK;
    writePlatformReg(pDev, MAC_IMR, pDev->MaskReg);

    /* Enable bus error interrupts */
    writePlatformReg(pDev, MAC_IMR_S2, readPlatformReg(pDev, MAC_IMR_S2) |
                     MAC_IMR_S2_MCABT | MAC_IMR_S2_SSERR | MAC_IMR_S2_DPERR);

    /* Enable interrupts specific to AP */
    if (serviceType == WLAN_AP_SERVICE) {
        writePlatformReg(pDev, MAC_IMR, readPlatformReg(pDev, MAC_IMR) |
                         MAC_IMR_MIB);
        pDev->MaskReg |= MAC_IMR_MIB;
    }

    if (ar5211GetRfKill(pDev)) {
        ar5211EnableRfKill(pDev);
    }

    /*
     * Writing to MAC_BEACON will start timers. Hence it should be the last
     * register to be written. Do not reset tsf, do not enable beacons at this point,
     * but preserve other values like beaconInterval.
     */
    writePlatformReg(pDev, MAC_BEACON, (readPlatformReg(pDev, MAC_BEACON) &
                                       ~(MAC_BEACON_EN | MAC_BEACON_RESET_TSF)));

    return A_OK;
}

/**************************************************************
 * ar5211PhyDisable
 *
 * Places the PHY and Radio chips into reset.  A full reset
 * must be called to leave this state.  The PCI/MAC/PCU are
 * not placed into reset as we must receive interrupt to
 * re-enable the hardware.
 */
A_STATUS
ar5211PhyDisable(WLAN_DEV_INFO *pDev)
{
    return ar5211SetResetReg(pDev, MAC_RC_BB);
}

/**************************************************************
 * ar5211Disable
 *
 * Places all of hardware into reset
 */
A_STATUS
ar5211Disable(WLAN_DEV_INFO *pDev)
{
    A_STATUS status;

    status = ar5211SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    /* Reset the HW - PCI must be reset after the rest of the device has been reset. */
    status = ar5211SetResetReg(pDev, MAC_RC_MAC | MAC_RC_BB | MAC_RC_PCI);
    if (status != A_OK) {
        return status;
    }

    return status;
}

/**************************************************************
 * ar5211ChipReset
 *
 * Places the hardware into reset and then pulls it out of reset
 *
 * Only write the PLL if we're changing to or from CCK mode
 *
 * Attach calls with channelFlags = 0, as the coldreset should have
 * us in the correct mode and we cannot check the hwchannel flags.
 *
 * WARNING - the PLL/mode/turbo write order is very strict
 * - do not change
 */
A_STATUS
ar5211ChipReset(WLAN_DEV_INFO *pDev, A_UINT16 channelFlags)
{
    A_STATUS status;

    /* Reset the HW - PCI must be reset after the rest of the device has been reset */
    status = ar5211SetResetReg(pDev, MAC_RC_MAC | MAC_RC_BB | MAC_RC_PCI);
    if (status != A_OK) {
        return status;
    }

    /* Bring out of sleep mode */
    status = ar5211SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    /* Clear warm reset register */
    status = ar5211SetResetReg(pDev, 0);

    /*
     * Perform warm reset before the mode/PLL/turbo registers are changed
     * in order to deactivate the radio.  Mode changes with an active radio can
     * result in corrupted shifts to the radio device.
     */

    /* Set CCK and Turbo modes correctly */
    if (IS_CHAN_2GHZ(channelFlags)) {
        if (IS_CHAN_CCK(channelFlags)) {
            /* 2.4 GHZ CCK Mode */
            writePlatformReg(pDev, PHY_TURBO, 0);
            writePlatformReg(pDev, PHY_5211_MODE, PHY_5211_MODE_CCK | PHY_5211_MODE_RF2GHZ);
            writePlatformReg(pDev, PHY_PLL_CTL, PHY_PLL_CTL_44);
            /* Wait for the PLL to settle */
            udelay(PLL_SETTLE_DELAY);
        } else {
            /* 2.4 GHZ OFDM Mode */
            writePlatformReg(pDev, PHY_TURBO, 0);
            if (pDev->pciInfo.DeviceID == AR5211_DEVID) {
                writePlatformReg(pDev, PHY_PLL_CTL, PHY_PLL_CTL_40);
                udelay(PLL_SETTLE_DELAY);
                writePlatformReg(pDev, PHY_5211_MODE, PHY_5211_MODE_OFDM | PHY_5211_MODE_RF2GHZ);
            }
        }
    } else if (IS_CHAN_5GHZ(channelFlags)) {
        if (pDev->pciInfo.DeviceID == AR5211_DEVID) {
            writePlatformReg(pDev, PHY_PLL_CTL, PHY_PLL_CTL_40);
            udelay(PLL_SETTLE_DELAY);
            writePlatformReg(pDev, PHY_5211_MODE, PHY_5211_MODE_OFDM | PHY_5211_MODE_RF5GHZ);
        }
        if (IS_CHAN_TURBO(channelFlags)) {
            /* 5 GHZ OFDM Turbo Mode */
            writePlatformReg(pDev, PHY_TURBO, PHY_FC_TURBO_MODE | PHY_FC_TURBO_SHORT);
        } else {
            /* 5 GHZ OFDM Mode */
            writePlatformReg(pDev, PHY_TURBO, 0);
        }
    }
    /* else no flags set - must be attach calling - do nothing */

    return status;
}


/**************************************************************************
 * ar5211PerCalibration - Periodic calibration of PHY
 *
 * Recalibrate the lower PHY chips to account for temperature/environment
 * changes.
 */
A_STATUS
ar5211PerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_STATUS status = A_OK;
    A_INT8   qCoff;
    A_INT32  qCoffDenom;
    A_INT32  iqCorrMeas;
    A_INT8   iCoff;
    A_INT32  iCoffDenom;
    A_UINT32 powerMeasQ;
    A_UINT32 powerMeasI;
    WLAN_STA_CONFIG *pConfig = &pDev->staConfig;

    /* IQ calibration in progress. Check to see if it has finished. */
    if ((pDev->pHalInfo->iqCalState == IQ_CAL_RUNNING) &&
        !(A_REG_RD(pDev, PHY_TIMING_CTRL4) & PHY_TIMING_CTRL4_DO_IQCAL) &&
        !pConfig->iqOverride)
    {
        /* IQ Calibration has finished.  Set to INACTIVE unless cal is valid */
        pDev->pHalInfo->iqCalState = IQ_CAL_INACTIVE;

        /* Read calibration results. */
        powerMeasI = A_REG_RD(pDev, PHY_IQCAL_RES_PWR_MEAS_I);
        powerMeasQ = A_REG_RD(pDev, PHY_IQCAL_RES_PWR_MEAS_Q);
        iqCorrMeas = A_REG_RD(pDev, PHY_IQCAL_RES_IQ_CORR_MEAS);

        /*
         * Prescale these values to remove 64-bit operation requirement at the loss
         * of a little precision.
         */
        iCoffDenom = (powerMeasI / 2 + powerMeasQ / 2) / 128;
        qCoffDenom = powerMeasQ / 64;

        /* Protect against divide-by-0. */
        if ( (iCoffDenom != 0) && (qCoffDenom != 0) ) {
            iCoff = (A_INT8)((-iqCorrMeas) / iCoffDenom);
            /* IQCORR_Q_I_COFF is a signed 6 bit number */
            iCoff = iCoff & 0x3F;

            qCoff = (A_INT8)(((A_INT32)powerMeasI / qCoffDenom) - 64);
            /* IQCORR_Q_Q_COFF is a signed 5 bit number */
            qCoff = qCoff & 0x1F;

#ifdef CALIBRATION_DEBUG
            uiPrintf("powerMeasI = 0x%08x\n", powerMeasI);
            uiPrintf("powerMeasQ = 0x%08x\n", powerMeasQ);
            uiPrintf("iqCorrMeas = 0x%08x\n", iqCorrMeas);
            uiPrintf("iCoff      = %d\n", iCoff);
            uiPrintf("qCoff      = %d\n", qCoff);
#endif

            /* Write IQ */
            A_REG_WR(pDev, PHY_TIMING_CTRL4, readPlatformReg(pDev, PHY_TIMING_CTRL4) |
                     PHY_TIMING_CTRL4_IQCORR_ENABLE |
                     A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_I_COFF, iCoff) |
                     A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_Q_COFF, qCoff));

            /* Save IQ values for any resets back to this channel */
            pDev->pHalInfo->iqCalState = IQ_CAL_DONE;
            pChval->iqCalValid = TRUE;
            pChval->iCoff = iCoff;
            pChval->qCoff = qCoff;
        } else {
            /* Running Cal values appear to have been cleared */
            ASSERT(0);
        }
    } else if (!IS_CHAN_B(pChval->channelFlags) && 
        (pDev->pHalInfo->iqCalState == IQ_CAL_DONE) &&
        (pChval->iqCalValid == FALSE) &&
        !pConfig->iqOverride)
    {
        /* Restart IQ calibration if configured channel has changed and does not have valid values */
        A_REG_RMW_FIELD(pDev, PHY_TIMING_CTRL4, IQCAL_LOG_COUNT_MAX, pConfig->iqLogCountMax);
        A_REG_SET_BIT(pDev, PHY_TIMING_CTRL4, DO_IQCAL);
        pDev->pHalInfo->iqCalState = IQ_CAL_RUNNING;
    }

    /* Check noise floor results */
    ar5211GetNf(pDev, pChval);
    if (pChval->channelFlags & CHANNEL_CW_INT) {
        return A_EBADCHANNEL;
    }

    status = ar5211CalNoiseFloor(pDev, pChval);
    if (status != A_OK) {
        /*
         * Delay 5ms before retrying the noise floor just to make sure, as
         * we are in an error condition here.
         */
        udelay(5000);
        status = ar5211CalNoiseFloor(pDev, pChval);
    }

    if (status == A_EBADCHANNEL && !IS_CHAN_B(pChval->channelFlags)) {
        pChval->channelFlags |= CHANNEL_CW_INT;
    }
    ar5211RequestRfgain(pDev);

    return status;
}

/**************************************************************
 * ar5211SetResetReg
 *
 * Writes the given reset bit mask into the reset register
 */
A_STATUS
ar5211SetResetReg(WLAN_DEV_INFO *pDev, A_UINT32 resetMask)
{
#if defined(PCI_INTERFACE)
    A_UINT32    mask = resetMask ? resetMask : ~0;
    A_UINT32    i;
#ifndef BUILD_AP
    A_STATUS    status;
    A_UINT32    reg32;
    A_UINT32    slot= pDev->pOSHandle->PciSlotNumber;
    NDIS_HANDLE handle = pDev->pOSHandle->NicAdapterHandle;
    A_UINT32    gpioValue;
    A_UINT32    gpioDO;

    /*
     * To ensure that the driver can reset the
     * MAC, wake up the chip
     */
    status = ar5211SetPowerMode(pDev, AWAKE, TRUE);

    if (status != A_OK) {
        return status;
    }

    /*
     * Save the GPIO State
     * since it is not sticky across a reset
     */
    gpioValue = A_REG_RD(pDev, MAC_GPIOCR);
    gpioDO    = A_REG_RD(pDev, MAC_GPIODO);

    /*
     * Disable interrupts
     */
    A_REG_WR(pDev, MAC_IER, MAC_IER_DISABLE);
    A_REG_RD(pDev, MAC_IER);

    /* need some delay before flush any pending MMR writes */
    udelay(15);
    readPlatformReg(pDev, MAC_RXDP);

    /*
     * Flush the park address of the PCI controller
     * WAR 7428
     */
    if (resetMask & MAC_RC_PCI) {
        for (i = 0; i < 32; i++) {
            NdisReadPciSlotInformation(handle, slot, PCI_STATUS_REGISTER, &reg32, 4);
        }
    }
#else
    /* need some delay before flush any pending MMR writes */
    udelay(15);
    readPlatformReg(pDev, MAC_RXDP);
#endif

    writePlatformReg(pDev, MAC_RC, resetMask);

#ifndef BUILD_AP
   if (resetMask & MAC_RC_PCI) {
        for (i = 0; i < 8; i++) {
            NdisReadPciSlotInformation(handle, slot, PCI_STATUS_REGISTER, &reg32, 4);
        }
    }
#endif

    /* need to wait at least 128 clocks when resetting PCI before read */
    udelay(15);

    resetMask  &= MAC_RC_MAC | MAC_RC_BB;
    mask &= MAC_RC_MAC | MAC_RC_BB;

    for (i = 0; i < 10000; i++) {
        if ((readPlatformReg(pDev, MAC_RC) & mask) == resetMask) {
            if (ar5211SetPowerMode(pDev, AWAKE, TRUE) == A_OK) {
                A_REG_RD(pDev, MAC_ISR_RAC);
            }
#ifndef BUILD_AP
            /*
             * Restore the GPIO State
             */
            A_REG_WR(pDev, MAC_GPIOCR, gpioValue);
            A_REG_WR(pDev, MAC_GPIODO, gpioDO);
            if (pDev->pHalInfo->halCapabilities.halDeviceType == ATH_DEV_TYPE_MINIPCI) {
                A_REG_RMW_FIELD(pDev, MAC_PCICFG, LED_MODE, 0x4);
            }
#endif 
            return A_OK;
        }
        udelay(50);
    }

    uiPrintf("ar5211SetResetReg: resetHW failed set %x != %lx\n", resetMask,
             readPlatformReg(pDev, MAC_RC));

    return A_HARDWARE;
#elif defined(AR531X)
    A_UINT32 resetBB, resetBits, regMask;
    A_UINT32 reg;

#if AR5312
    if (pDev->devno == 0) {
#endif
        resetBB   = RESET_WARM_WLAN0_BB;
        resetBits = RESET_WLAN0 | RESET_WARM_WLAN0_MAC;
        regMask   = ~(RESET_WLAN0 | RESET_WARM_WLAN0_MAC | RESET_WARM_WLAN0_BB);
#if AR5312
    } else {
        resetBB   = RESET_WARM_WLAN1_BB;
        resetBits = RESET_WLAN1 | RESET_WARM_WLAN1_MAC;
        regMask   = ~(RESET_WLAN1 | RESET_WARM_WLAN1_MAC | RESET_WARM_WLAN1_BB);
    }
#endif

    reg = sysRegRead(AR531X_RESET);             /* read before */

    if (resetMask == MAC_RC_BB) {
        /* Put baseband in reset */
        reg |= resetBB;
        sysRegWrite(AR531X_RESET, reg);
        (void)sysRegRead(AR531X_RESET);         /* read after */
        return A_OK;
    }

    if (resetMask) {
        /*
         * Reset the MAC and baseband.  This is a bit different than
         * the PCI version, but holding in reset causes problems.
         */
        reg &= regMask;
        reg |= resetBits;
        sysRegWrite(AR531X_RESET, reg);
        (void)sysRegRead(AR531X_RESET);         /* read after */
        udelay(100);
    }

    /* Bring MAC and baseband out of reset */
    reg &= regMask;
    (void)sysRegRead(AR531X_RESET);             /* read before */
    sysRegWrite(AR531X_RESET, reg);
    (void)sysRegRead(AR531X_RESET);             /* read after */

    return A_OK;
#endif
}

/**************************************************************
 * ar5211SetChannel
 *
 * Takes the MHz channel value and sets the Channel value
 *
 * ASSUMES: Writes enabled to analog bus
 */
A_STATUS
ar5211SetChannel(WLAN_DEV_INFO *pDev,  CHAN_VALUES *pChval)
{
    A_UINT32    refClk, reg32;
    A_UINT32    lowChannel, highChannel;
    A_UINT32    data2111 = 0;
    A_INT16     channel5111, channelIEEE;
    A_INT16     index2Ghz;

    lowChannel = halGetCapability(pDev, HAL_GET_LOW_CHAN_EDGE, pChval->channelFlags);
    highChannel = halGetCapability(pDev, HAL_GET_HIGH_CHAN_EDGE, pChval->channelFlags);
    if ((pChval->channel < lowChannel) || (pChval->channel > highChannel)) {
        uiPrintf("ar5211SetChannel: Invalid Channel Number %4d MHz\n", pChval->channel);
        return A_EINVAL;
    }
    channelIEEE = wlanConvertGHztoCh(pChval->channel, pChval->channelFlags);

    if (IS_CHAN_2GHZ(pChval->channelFlags)) {
        index2Ghz = channelIEEE + CI_2GHZ_INDEX_CORRECTION;
        ASSERT(index2Ghz >= 0);
        channel5111 = chan2GHzData[index2Ghz].channel5111;                // 5111 channel
        data2111  = reverseBits(chan2GHzData[index2Ghz].channelSelect, 8) & 0xff;
        data2111  = (data2111 << 5) | (chan2GHzData[index2Ghz].refClkSel << 4);
    } else {
        channel5111 = channelIEEE;
    }

    /* Rest of the code is common for 5 GHz and 2.4 GHz. */
    if (channel5111 >= 145 || (channel5111 & 0x1)) {
        reg32  = reverseBits((channel5111 - 24), 8) & 0xFF;
        refClk = 1;
    } else {
        reg32  = reverseBits(((channel5111 - 24) / 2), 8) & 0xFF;
        refClk = 0;
    }

    reg32 = (reg32 << 2) | (refClk << 1) | (1 << 10) | 0x1;
    writePlatformReg(pDev, PHY_BASE + (0x27 << 2),
                     (((data2111 & 0xff) << 8) | (reg32 & 0xff)));
    reg32 = (reg32 >> 8) & 0xff;
    writePlatformReg(pDev, PHY_BASE + (0x34 << 2),
                     ((data2111 & 0x0000ff00) | reg32));

    pDev->staConfig.phwChannel = pChval;
    return A_OK;
}

/**************************************************************************
 * Peform the noisefloor calibration for the length of time set
 * in runTime (valid values 1 to 7)
 *
 * Returns: The NF value at the end of the given time (or 0 for failure)
 */
A_INT16
ar5211RunNoiseFloor(WLAN_DEV_INFO *pDev, A_UINT8 runTime, A_INT16 startingNF)
{
    A_INT16 nf, i;
    int     searchTime;

    ASSERT(runTime <= 7);

    /* Setup  noise floor run time and starting value */
    writePlatformReg(pDev, PHY_BASE+(25<<2), (readPlatformReg(pDev, PHY_BASE+(25<<2)) & ~0xFFF) |
                     ((runTime << 9) & 0xE00) | (startingNF & 0x1FF));

    /* Calibrate the noise floor */
    writePlatformReg(pDev, PHY_AGC_CONTROL, readPlatformReg(pDev, PHY_AGC_CONTROL) |
                     PHY_AGC_CONTROL_NF);

    /* Compute the required amount of searchTime needed to finish NF */
    if (runTime == 0) {
        /* 8 search windows * 6.4us each */
        searchTime = 8  * 7;
    } else {
        /* 512 * runtime search windows * 6.4us each */
        searchTime = (runTime * 512)  * 7;
    }

    /* Do not read noise floor until it has been updated */
    /*
     *  As a guesstimate - we may only get 1/60th the time on the air to see search windows
     *  in a heavily congested network (40 us every 2400 us of time)
     */
    for (i = 0; ((i < 60) && (readPlatformReg(pDev, PHY_AGC_CONTROL) & PHY_AGC_CONTROL_NF)); i++) {
        udelay(searchTime);
    }
    if (i >= 60) {
#ifdef DEBUG
        uiPrintf("NF with runTime %d failed to end on channel %d\n", runTime, pDev->staConfig.phwChannel->channel);
        uiPrintf("  PHY NF Reg state:     0x%lx\n", readPlatformReg(pDev, PHY_AGC_CONTROL));
        uiPrintf("  PHY Active Reg state: 0x%lx\n", readPlatformReg(pDev, PHY_ACTIVE));
#endif
        return 0;
    }

    nf = (A_INT16)(readPlatformReg(pDev, PHY_BASE+(25<<2)) >> 19) & 0x1ff;
    if (nf & 0x100) {
        nf = 0 - ((nf ^ 0x1ff) + 1);
    }

    return nf;
}

/**************************************************************************
 * Read the NF and check it against the noise floor threshhold
 *
 * Returns: The signed NF value.
 */
A_INT16
ar5211GetNf(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_INT16 nfThresh;
    A_INT16 nf        = 0;
    int     arrayMode = 0;
    A_BOOL  is5GHz    = FALSE;

    switch (pChval->channelFlags & (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ)) {
    case CHANNEL_A:
        arrayMode = 0;
        is5GHz    = TRUE;
        break;
    case CHANNEL_B:
        arrayMode = 1;
        break;
    case CHANNEL_PUREG:
        arrayMode = 2;
        break;
    default:
        ASSERT(0);
        break;
    }

    if (readPlatformReg(pDev, PHY_AGC_CONTROL) & PHY_AGC_CONTROL_NF) {
#ifdef DEBUG
        uiPrintf("NF failed to complete in calibration window\n");
#endif
    } else {
        /* Finished NF cal, check against threshold */
        nfThresh = pDev->pHalInfo->pEepData->pEepHeader->noiseFloorThresh[arrayMode];
        nf = (A_INT16)(readPlatformReg(pDev, PHY_BASE+(25<<2)) >> 19) & 0x1ff;
        if (nf & 0x100) {
            nf = 0 - ((nf ^ 0x1ff) + 1);
        }
        if (nf > nfThresh) {
            uiPrintf("Noise floor failed detected %d higher than thresh %d\n", nf, nfThresh);
            pChval->channelFlags |= (is5GHz ? CHANNEL_CW_INT : 0);
        }
    }
    return nf;
}

/**************************************************************************
 * ar5211CalNoiseFloor
 *
 * Peform the noisefloor calibration and check for any constant channel
 * interference.
 *
 * NOTE: preAR5211 have a lengthy carrier wave detection process - hence
 * it is if'ed for MKK regulatory domain only.
 *
 * Returns: TRUE for a successful noise floor calibration; else FALSE
 */
A_STATUS
ar5211CalNoiseFloor(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_INT16 nf, arrayMode = 0;
    A_INT16 nfThresh;

    /* Check for Carrier Wave interference in MKK regulatory zone */
    if ((pDev->macVersion < MAC_SREV_VERSION_OAHU) &&
        wlanIsNfCheckRequired(pDev))
    {
        switch (pChval->channelFlags & (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ)) {
        case CHANNEL_A:
            arrayMode = 0;
            break;
        case CHANNEL_B:
            arrayMode = 1;
            break;
        case CHANNEL_PUREG:
            arrayMode = 2;
            break;
        default:
            ASSERT(0);
            break;
        }
        nfThresh = pDev->pHalInfo->pEepData->pEepHeader->noiseFloorThresh[arrayMode];

        /* Run a quick noise floor that will hopefully complete (decrease delay time) */
        nf = ar5211RunNoiseFloor(pDev, 0, 0);
        if (nf > nfThresh) {
            uiPrintf("First run failed with value %d higher than thresh %d\n", nf, nfThresh);
            nf = ar5211RunNoiseFloor(pDev, 2, nf);
            if (nf > nfThresh) {
                uiPrintf("Second run failed with value %d higher than thresh %d\n", nf, nfThresh);
                nf = ar5211RunNoiseFloor(pDev, 7, nf);
                if (nf > nfThresh) {
                    uiPrintf("Full run failed with value %d higher than thresh %d\n", nf, nfThresh);
                    return A_HARDWARE;
                }
            }
        }
    } else {
        /* Calibrate the noise floor */
        writePlatformReg(pDev, PHY_AGC_CONTROL, readPlatformReg(pDev, PHY_AGC_CONTROL) |
                         PHY_AGC_CONTROL_NF);
    }

    return A_OK;
}



/**************************************************************
 * ar5211SetRf6and7
 *
 * Reads EEPROM header info from device structure and programs
 * analog registers 6 and 7
 *
 * REQUIRES: Access to the analog device
 */
static void
ar5211SetRf6and7(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval)
{
    A_UINT16    rfXpdGain, rfPloSel, rfPwdXpd, gainI;
    A_UINT16    tempOB    = 0;
    A_UINT16    tempDB    = 0;
    A_BOOL      arrayMode = 0;
    A_UINT16    freqIndex;
    int         arrayEntries, i;

    freqIndex = IS_CHAN_2GHZ(pChval->channelFlags) ? 2 : 1;

    /*
     * TODO: This array mode correspondes with the index used during the read
     * For readability, this should be changed to an enum or #define
     */
    switch ( pChval->channelFlags &
             (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ) )
    {
    case CHANNEL_A:
        if ((pChval->channel > 4000) && (pChval->channel < 5260)) {
            tempOB = pHeaderInfo->ob1;
            tempDB = pHeaderInfo->db1;
        } else if ((pChval->channel >= 5260) && (pChval->channel < 5500)) {
            tempOB = pHeaderInfo->ob2;
            tempDB = pHeaderInfo->db2;
        } else if ((pChval->channel >= 5500) && (pChval->channel < 5725)) {
            tempOB = pHeaderInfo->ob3;
            tempDB = pHeaderInfo->db3;
        }
        else if (pChval->channel >= 5725) {
            tempOB = pHeaderInfo->ob4;
            tempDB = pHeaderInfo->db4;
        }

        arrayMode = 0;

        ar5211Rf6n7[5][freqIndex]  = (ar5211Rf6n7[5][freqIndex] & ~0x10000000) |
                                     (pHeaderInfo->cornerCal.pd90 << 28);
        ar5211Rf6n7[6][freqIndex]  = (ar5211Rf6n7[6][freqIndex] & ~0x04000000) |
                                     (pHeaderInfo->cornerCal.pd84 << 26);
        ar5211Rf6n7[21][freqIndex] = (ar5211Rf6n7[21][freqIndex] & ~0x08) |
                                     (pHeaderInfo->cornerCal.gSel << 3);
        break;
    case CHANNEL_B:
        tempOB = pHeaderInfo->obFor24;
        tempDB = pHeaderInfo->dbFor24;
        arrayMode = 1;
        break;
    case CHANNEL_PUREG:
        tempOB = pHeaderInfo->obFor24g;
        tempDB = pHeaderInfo->dbFor24g;
        arrayMode = 2;
        break;
    default:
        ASSERT(0);
        break;
    }

    rfXpdGain = pHeaderInfo->xgain[arrayMode];
    rfPloSel  = pHeaderInfo->xpd[arrayMode];
    rfPwdXpd  = !pHeaderInfo->xpd[arrayMode];
    gainI     = pHeaderInfo->gainI[arrayMode];

    if ( (tempOB < 1) || (tempOB > 5) ) {
        uiPrintf("ar5211SetRf6and7: OB should be between 1 and 5 inclusively, not %d", tempOB);
    }
    if ( (tempDB < 1) || (tempDB > 5) ) {
        uiPrintf("ar5211SetRf6and7: DB should be between 1 and 5 inclusively, not %d", tempDB);
    }

    /* Set rfXpdGain and rfPwdXpd */
    ar5211Rf6n7[11][freqIndex] =  (ar5211Rf6n7[11][freqIndex] & ~0xC0) |
        (((reverseBits(rfXpdGain, 4) << 7) | (rfPwdXpd << 6)) & 0xC0);
    ar5211Rf6n7[12][freqIndex] =  (ar5211Rf6n7[12][freqIndex] & ~0x07) |
        ((reverseBits(rfXpdGain, 4) >> 1) & 0x07);

    /* Set OB */
    ar5211Rf6n7[12][freqIndex] =  (ar5211Rf6n7[12][freqIndex] & ~0x80) |
        ((reverseBits(tempOB, 3) << 7) & 0x80);
    ar5211Rf6n7[13][freqIndex] =  (ar5211Rf6n7[13][freqIndex] & ~0x03) |
        ((reverseBits(tempOB, 3) >> 1) & 0x03);

    /* Set DB */
    ar5211Rf6n7[13][freqIndex] =  (ar5211Rf6n7[13][freqIndex] & ~0x1C) |
        ((reverseBits(tempDB, 3) << 2) & 0x1C);

    /* Set rfPloSel */
    ar5211Rf6n7[17][freqIndex] =  (ar5211Rf6n7[17][freqIndex] & ~0x08) |
        ((rfPloSel << 3) & 0x08);

    /* Set GainI */
    ar5211Rf6n7[20][freqIndex] =  (ar5211Rf6n7[20][freqIndex] & ~0xF0) |
        ((reverseBits(gainI, 6) << 4) & 0xF0);
    ar5211Rf6n7[21][freqIndex] =  (ar5211Rf6n7[21][freqIndex] & ~0x03) |
        ((reverseBits(gainI, 6) >> 4) & 0x03);

    /* Write the Rf registers 6 & 7 */
    arrayEntries = sizeof(ar5211Rf6n7) / (sizeof(A_UINT32) * 3);
    for (i = 0; i < arrayEntries; i++) {
        writePlatformReg(pDev, ar5211Rf6n7[i][0], ar5211Rf6n7[i][freqIndex]);
    }

    // Now that we have reprogrammed rfgain value, clear the flag.
    pDev->pHalInfo->rfgainState = RFGAIN_INACTIVE;
}

#define NO_FALSE_DETECT_BACKOFF   2
#define CB22_FALSE_DETECT_BACKOFF 6
/**************************************************************
 * ar5211SetBoardValues
 *
 * Reads EEPROM header info and programs the device for correct operation
 * given the channel value
 */
void
ar5211SetBoardValues(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval)
{
    int arrayMode = 0;
    int falseDectectBackoff;
    int eepVersion = pDev->pHalInfo->pEepData->version;

    switch ( pChval->channelFlags &
             (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ) )
    {
    case CHANNEL_A:
        arrayMode = 0;
        A_REG_RMW_FIELD(pDev, PHY_FRAME_CTL, TX_CLIP, pHeaderInfo->cornerCal.clip);
        break;
    case CHANNEL_B:
        arrayMode = 1;
        break;
    case CHANNEL_PUREG:
        arrayMode = 2;
        break;
    default:
        ASSERT(0);
        break;
    }

    /* Set the antenna register(s) correctly for the chip revision */
    if (pDev->macVersion < MAC_SREV_VERSION_OAHU) {
        writePlatformReg(pDev, PHY_BASE + (68 << 2),
            (readPlatformReg(pDev, PHY_BASE + (68 << 2)) & 0xFFFFFFFC) | 0x3);
    } else {
        writePlatformReg(pDev, PHY_BASE + (68 << 2),
            (readPlatformReg(pDev, PHY_BASE + (68 << 2)) & 0xFFFFFC06) |
            (pHeaderInfo->antennaControl[0][arrayMode] << 4) | 0x1);

         ar5211SetAntennaSwitch(pDev, pDev->staConfig.diversityControl, pChval);

        /* Set the Noise Floor Thresh on ar5211 devices */
        writePlatformReg(pDev, PHY_BASE + (90 << 2), (pHeaderInfo->noiseFloorThresh[arrayMode] & 0x1FF) |
                         (1 << 9));
    }
    writePlatformReg(pDev, PHY_BASE + (17 << 2),
        (readPlatformReg(pDev, PHY_BASE + (17 << 2)) & 0xFFFFC07F) |
        ((pHeaderInfo->switchSettling[arrayMode] << 7) & 0x3F80));
    writePlatformReg(pDev, PHY_BASE + (18 << 2),
        (readPlatformReg(pDev, PHY_BASE + (18 << 2)) & 0xFFFC0FFF) |
        ((pHeaderInfo->txrxAtten[arrayMode] << 12) & 0x3F000));
    writePlatformReg(pDev, PHY_BASE + (20 << 2),
        (readPlatformReg(pDev, PHY_BASE + (20 << 2)) & 0xFFFF0000) |
        ((pHeaderInfo->pgaDesiredSize[arrayMode] << 8) & 0xFF00) |
        (pHeaderInfo->adcDesiredSize[arrayMode] & 0x00FF));
    writePlatformReg(pDev, PHY_BASE + (13 << 2),
        (pHeaderInfo->txEndToXPAOff[arrayMode] << 24) |
        (pHeaderInfo->txEndToXPAOff[arrayMode] << 16) |
        (pHeaderInfo->txFrameToXPAOn[arrayMode] << 8) |
        pHeaderInfo->txFrameToXPAOn[arrayMode]);
    writePlatformReg(pDev, PHY_BASE + (10 << 2),
        (readPlatformReg(pDev, PHY_BASE + (10 << 2)) & 0xFFFF00FF) |
        (pHeaderInfo->txEndToXLNAOn[arrayMode] << 8));
    writePlatformReg(pDev, PHY_BASE + (25 << 2),
        (readPlatformReg(pDev, PHY_BASE + (25 << 2)) & 0xFFF80FFF) |
        ((pHeaderInfo->thresh62[arrayMode] << 12) & 0x7F000));

    /* False detect backoff - suspected 32 MHz spur causes false detects in OFDM,
     * causing Tx Hangs.  Decrease weak signal sensitivity for this card.
     */
    falseDectectBackoff = NO_FALSE_DETECT_BACKOFF;
    if (eepVersion < EEPROM_VER3_3) {
        if(pDev->pciInfo.SubVendorDeviceID == 0x1022 &&
            pChval->channelFlags & CHANNEL_OFDM)
        {
            falseDectectBackoff += CB22_FALSE_DETECT_BACKOFF;
        }
    } else {
        A_UINT32 remainder = pChval->channel % 32;

        if ((remainder != 0) && ((remainder < 10) || (remainder > 22))) {
            falseDectectBackoff += pHeaderInfo->falseDetectBackoff[arrayMode];
        }
    }
    writePlatformReg(pDev, 0x9924, (readPlatformReg(pDev, 0x9924) & 0xFFFFFF01) |
        ((falseDectectBackoff << 1) & 0xF7));

    /* Write previous IQ results or use initial values */
    if (pChval->iqCalValid) {
        A_REG_WR(pDev, PHY_TIMING_CTRL4, readPlatformReg(pDev, PHY_TIMING_CTRL4) |
                 PHY_TIMING_CTRL4_IQCORR_ENABLE |
                 A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_I_COFF, pChval->iCoff) |
                 A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_Q_COFF, pChval->qCoff));
    }
}

/**************************************************************
 * ar5211SetTransmitPower
 *
 * Sets the transmit power in the baseband for the given
 * operating channel and mode.
 */
A_STATUS
ar5211SetTransmitPower(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    static const A_UINT16 tpcScaleReductionTable[5] = {0, 3, 6, 9, 63};

    A_STATUS        status        = A_OK;
    A_UINT16        numChannels   = 0;
    TRGT_POWER_INFO *pPowerInfo   = NULL;
    RD_EDGES_POWER  *pRdEdgePower = NULL;
    PCDACS_EEPROM   eepromPcdacs;
    struct eepMap   *pData;
    A_UINT8         cfgCtl;
    int             i;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pData = pDev->pHalInfo->pEepData;

    /* setup the pcdac struct to point to the correct info, based on mode */
    switch ( pChval->channelFlags &
             (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ) )
    {
    case CHANNEL_A:
        eepromPcdacs.numChannels     = pData->pPcdacInfo->numChannels11a;
        eepromPcdacs.pChannelList    = pData->pPcdacInfo->Channels11a;
        eepromPcdacs.pDataPerChannel = pData->pPcdacInfo->DataPerChannel11a;
        numChannels = pData->pTrgtPowerInfo->numTargetPwr_11a;
        pPowerInfo  = pData->pTrgtPowerInfo->trgtPwr_11a;
        break;

    case CHANNEL_PUREG:
        eepromPcdacs.numChannels     = pData->pPcdacInfo->numChannels2_4;
        eepromPcdacs.pChannelList    = pData->pPcdacInfo->Channels2_4;
        eepromPcdacs.pDataPerChannel = pData->pPcdacInfo->DataPerChannel11g;
        numChannels = pData->pTrgtPowerInfo->numTargetPwr_11g;
        pPowerInfo  = pData->pTrgtPowerInfo->trgtPwr_11g;
        break;

    case CHANNEL_B:
        eepromPcdacs.numChannels     = pData->pPcdacInfo->numChannels2_4;
        eepromPcdacs.pChannelList    = pData->pPcdacInfo->Channels2_4;
        eepromPcdacs.pDataPerChannel = pData->pPcdacInfo->DataPerChannel11b;
        numChannels = pData->pTrgtPowerInfo->numTargetPwr_11b;
        pPowerInfo  = pData->pTrgtPowerInfo->trgtPwr_11b;
        break;
    default:
        ASSERT(0);
        break;
    }

    ar5211SetPowerTable(pDev, &eepromPcdacs, pChval->channel);

    /* Match CTL to EEPROM value */
    cfgCtl = wlanGetCtl(pDev, pChval);

    for (i = 0; i < pData->pEepHeader->numCtls; i++) {
        if (pData->pEepHeader->ctl[i] == 0) {
            continue;
        } else if (pData->pEepHeader->ctl[i] == cfgCtl) {
            pRdEdgePower = &(pData->pRdEdgesPower[i * NUM_EDGES]);
            break;
        }
    }

    ar5211SetRateTable(pDev, pRdEdgePower,
                       pPowerInfo, numChannels, pChval,
                       tpcScaleReductionTable[pDev->staConfig.tpScale],
                       pData);

    return status;
}

/**************************************************************
 * ar5211SetTxPowerLimit
 *
 * Sets a limit on the overall output power.  Used for dynamic
 * transmit power control and the like.
 *
 * NOTE: The power passed in is in units of 0.5 dBm.
 */
void
ar5211SetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit)
{
    /* Make sure we don't exceed the maximum. */
    limit = A_MIN(limit, MAX_RATE_POWER);

    A_REG_WR(pDev, PHY_POWER_TX_RATE_MAX, limit);
}

/**************************************************************
 * ar5211SetPowerTable
 *
 * Read the transmit power levels from the structures taken from EEPROM
 * Interpolate read transmit power values for this channel
 * Organize the transmit power values into a table for writing into the hardware
 */
void
ar5211SetPowerTable(WLAN_DEV_INFO *pDev, PCDACS_EEPROM *pSrcStruct, A_UINT16 channel)
{
    FULL_PCDAC_STRUCT pcdacStruct;
    A_UINT16          pcdacTable[PWR_TABLE_SIZE];

    A_UINT16          i, j;
    A_UINT16          *pPcdacValues;
    A_INT16           *pScaledUpDbm;
    A_INT16           minScaledPwr;
    A_INT16           maxScaledPwr;
    A_INT16           pwr;
    A_UINT16          pcdacMin = 0;
    A_UINT16          pcdacMax = 63;
    A_UINT16          pcdacTableIndex;
    A_UINT16          scaledPcdac;
    A_UINT32          addr;
    A_UINT32          temp32;

    A_MEM_ZERO(&pcdacStruct, sizeof(FULL_PCDAC_STRUCT));
    A_MEM_ZERO(pcdacTable, sizeof(A_UINT16) * PWR_TABLE_SIZE);
    pPcdacValues = pcdacStruct.PcdacValues;
    pScaledUpDbm = pcdacStruct.PwrValues;

    /* Initialize the pcdacs to dBM structs pcdacs to be 1 to 63 */
    for (i = PCDAC_START, j = 0; i <= PCDAC_STOP; i+= PCDAC_STEP, j++) {
        pPcdacValues[j] = i;
    }

    pcdacStruct.numPcdacValues = j;
    pcdacStruct.pcdacMin = PCDAC_START;
    pcdacStruct.pcdacMax = PCDAC_STOP;

    /* Fill out the power values for this channel */
    for (j = 0; j < pcdacStruct.numPcdacValues; j++ ) {
        pScaledUpDbm[j] = ar5211GetScaledPower(channel, pPcdacValues[j], pSrcStruct);
    }

    /* Now scale the pcdac values to fit in the 64 entry power table */
    minScaledPwr = pScaledUpDbm[0];
    maxScaledPwr = pScaledUpDbm[pcdacStruct.numPcdacValues - 1];

    /* find minimum and make monotonic */
    for (j = 0; j < pcdacStruct.numPcdacValues; j++) {
        if (minScaledPwr >= pScaledUpDbm[j]) {
            minScaledPwr = pScaledUpDbm[j];
            pcdacMin = j;
        }
        /*
         * Make the full_hsh monotonically increasing otherwise interpolation algorithm will get fooled
         * gotta start working from the top, hence i = 63 - j.
         */
        i = (A_UINT16)(pcdacStruct.numPcdacValues - 1 - j);
        if (i == 0) {
            break;
        }
        if (pScaledUpDbm[i-1] > pScaledUpDbm[i]) {
            /*
             * It could be a glitch, so make the power for this pcdac
             * the same as the power from the next highest pcdac.
             */
            pScaledUpDbm[i - 1] = pScaledUpDbm[i];
        }
    }

    for (j = 0; j < pcdacStruct.numPcdacValues; j++) {
        if (maxScaledPwr < pScaledUpDbm[j]) {
            maxScaledPwr = pScaledUpDbm[j];
            pcdacMax = j;
        }
    }

    /* Find the first power level with a pcdac */
    pwr = (A_UINT16)(PWR_STEP * ((minScaledPwr - PWR_MIN + PWR_STEP / 2) / PWR_STEP)  + PWR_MIN);

    /* Write all the first pcdac entries based off the pcdacMin */
    pcdacTableIndex = 0;
    for (i = 0; i < (2 * (pwr - PWR_MIN) / EEP_SCALE + 1); i++) {
        pcdacTable[pcdacTableIndex++] = pcdacMin;
    }

    i = 0;
    while ((pwr < pScaledUpDbm[pcdacStruct.numPcdacValues - 1]) &&
        (pcdacTableIndex < PWR_TABLE_SIZE))
    {
        pwr += PWR_STEP;
        /* stop if dbM > max_power_possible */
        while ((pwr < pScaledUpDbm[pcdacStruct.numPcdacValues - 1]) &&
               ((pwr - pScaledUpDbm[i])*(pwr - pScaledUpDbm[i+1]) > 0))
        {
            i++;
        }
        /* scale by 2 and add 1 to enable round up or down as needed */
        scaledPcdac = (A_UINT16)(ar5211GetInterpolatedValue(pwr, pScaledUpDbm[i],
                                 pScaledUpDbm[i + 1], (A_UINT16)(pPcdacValues[i] * 2),
                                 (A_UINT16)(pPcdacValues[i + 1] * 2), 0 ) + 1);

        pcdacTable[pcdacTableIndex] = scaledPcdac / 2;
        if (pcdacTable[pcdacTableIndex] > pcdacMax) {
            pcdacTable[pcdacTableIndex] = pcdacMax;
        }
        pcdacTableIndex++;
    }

    /* Write all the last pcdac entries based off the last valid pcdac */
    while (pcdacTableIndex < PWR_TABLE_SIZE) {
        pcdacTable[pcdacTableIndex] = pcdacTable[pcdacTableIndex - 1];
        pcdacTableIndex++;
    }

    /* Finally, write the power values into the baseband power table */
    addr = PHY_BASE + (608 << 2);
    for (i = 0; i < 32; i++) {
        temp32 = 0xffff & ((pcdacTable[2 * i + 1] << 8) | 0xff);
        temp32 = (temp32 << 16) | (0xffff & ((pcdacTable[2 * i] << 8) | 0xff));
        writePlatformReg(pDev, addr, temp32);
        addr += 4;
    }

}

/**************************************************************
 * ar5211SetRateTable
 *
 * Sets the transmit power in the baseband for the given
 * operating channel and mode.
 */
void
ar5211SetRateTable(WLAN_DEV_INFO *pDev, RD_EDGES_POWER *pRdEdgesPower,
                   TRGT_POWER_INFO *pPowerInfo, A_UINT16 numChannels,
                   CHAN_VALUES *pChval, A_UINT16 tpcScaleReduction,
                   struct eepMap *pData)
{
    A_UINT16    ratesArray[NUM_RATES];

    A_UINT16    *pRatesPower;
    A_UINT16    lowerChannel, lowerIndex=0, lowerPower=0;
    A_UINT16    upperChannel, upperIndex=0, upperPower=0;
    A_UINT16    twiceMaxEdgePower=63;
    A_UINT16    twicePower = 0;
    A_UINT16    i, numEdges;
    A_UINT16    tempChannelList[NUM_EDGES]; /* temp array for holding edge channels */
    A_UINT16    twiceMaxRDPower;
    A_INT16     scaledPower = 0;        /* for gcc -O2 */
    A_UINT16    mask = 0x3f;
    A_UINT32    reg32;
    A_BOOL      paPreDEnable = 0;
    A_INT8      twiceAntennaGain, twiceAntennaReduction = 0;

    pRatesPower     = ratesArray;
    twiceMaxRDPower = wlanGetChannelPower(pDev, pChval) * 2;

    if (IS_CHAN_5GHZ(pChval->channelFlags)) {
        twiceAntennaGain = pData->pEepHeader->antennaGainMax[0];
    } else {
        twiceAntennaGain = pData->pEepHeader->antennaGainMax[1];
    }

    twiceAntennaReduction = wlanGetAntennaReduction(pDev, pChval, twiceAntennaGain);

    if (pRdEdgesPower) {
        /* Get the edge power */
        for (i = 0; i < NUM_EDGES; i++) {
            if (pRdEdgesPower[i].rdEdge == 0) {
                break;
            }
            tempChannelList[i] = pRdEdgesPower[i].rdEdge;
        }
        numEdges = i;

        ar5211GetLowerUpperValues(pChval->channel, tempChannelList, numEdges,
                                  &lowerChannel, &upperChannel);
        /* Get the index for the lower channel */
        for (i = 0; i < numEdges; i++) {
            if (lowerChannel == tempChannelList[i]) {
                break;
            }
        }
        /* Is lower channel ever outside the rdEdge? */
        ASSERT(i != numEdges);

        if (((lowerChannel == upperChannel) && (lowerChannel == pChval->channel)) ||
            (pRdEdgesPower[i].flag))
        {
            /*
             * If there's an exact channel match or an inband flag set
             * on the lower channel use the given rdEdgePower
             */
            twiceMaxEdgePower = pRdEdgesPower[i].twice_rdEdgePower;
            ASSERT(twiceMaxEdgePower > 0);
        }
    }

    /* extrapolate the power values for the test Groups */
    for (i = 0; i < numChannels; i++) {
        tempChannelList[i] = pPowerInfo[i].testChannel;
    }

    ar5211GetLowerUpperValues(pChval->channel, tempChannelList, numChannels,
                              &lowerChannel, &upperChannel);

    /* get the index for the channel */
    for (i = 0; i < numChannels; i++) {
        if (lowerChannel == tempChannelList[i]) {
            lowerIndex = i;
        }

        if (upperChannel == tempChannelList[i]) {
            upperIndex = i;
            break;
        }
    }

    for (i = 0; i < NUM_RATES; i++) {
        if (IS_CHAN_OFDM(pChval->channelFlags)) {
            /* power for rates 6,9,12,18,24 is all the same */
            if (i < 5) {
                lowerPower = pPowerInfo[lowerIndex].twicePwr6_24;
                upperPower = pPowerInfo[upperIndex].twicePwr6_24;
            } else if (i == 5) {
                lowerPower = pPowerInfo[lowerIndex].twicePwr36;
                upperPower = pPowerInfo[upperIndex].twicePwr36;
            } else if (i == 6) {
                lowerPower = pPowerInfo[lowerIndex].twicePwr48;
                upperPower = pPowerInfo[upperIndex].twicePwr48;
            } else if (i == 7) {
                lowerPower = pPowerInfo[lowerIndex].twicePwr54;
                upperPower = pPowerInfo[upperIndex].twicePwr54;
            }
        } else {
            switch (i) {
            case 0:
            case 1:
                lowerPower = pPowerInfo[lowerIndex].twicePwr6_24;
                upperPower = pPowerInfo[upperIndex].twicePwr6_24;
                break;
            case 2:
            case 3:
                lowerPower = pPowerInfo[lowerIndex].twicePwr36;
                upperPower = pPowerInfo[upperIndex].twicePwr36;
                break;
            case 4:
            case 5:
                lowerPower = pPowerInfo[lowerIndex].twicePwr48;
                upperPower = pPowerInfo[upperIndex].twicePwr48;
                break;
            case 6:
            case 7:
                lowerPower = pPowerInfo[lowerIndex].twicePwr54;
                upperPower = pPowerInfo[upperIndex].twicePwr54;
                break;
            }
        }

        twicePower = (A_UINT16)ar5211GetInterpolatedValue(pChval->channel, lowerChannel,
                     upperChannel, lowerPower, upperPower, 0);

        /* Reduce power by band edge restrictions */
        twicePower = A_MIN(twicePower, twiceMaxEdgePower);

#ifndef BUILD_AP
        /* Turbo limitation not required for nonbattery dependent devices */
        /* TODO: get rid of BUILD_AP someday, some nonAP devices don't have a 2W limit */
        if (IS_CHAN_TURBO(pChval->channelFlags)) {
            /* If turbo is set, reduce power to keep power consumption under 2 Watts */
            if (pData->version >= EEPROM_VER3_1) {
                twicePower = A_MIN(twicePower, pData->pEepHeader->turbo2WMaxPower);
            }
        }
#endif

        /* Reduce power by max regulatory domain allowed restrictions */
        pRatesPower[i] = A_MIN(twicePower, twiceMaxRDPower - twiceAntennaReduction);

        /* Use 6 Mb power level for transmit power scaling reduction */
        /* We don't want to reduce higher rates if its not needed */
        if (i == 0) {
            pDev->maxTxPowerAvail = pRatesPower[0];

            scaledPower = pRatesPower[0] - (tpcScaleReduction * 2);
            if (scaledPower < 1) {
                scaledPower = 1;
            }
            if (pDev->staConfig.overRideTxPower) {
                scaledPower = pDev->staConfig.overRideTxPower * 2;
            }
        }

        pRatesPower[i] = A_MIN(pRatesPower[i], scaledPower);
    }


    /* Record txPower at Rate 6 for info gathering */
    pDev->tx6PowerInHalfDbm = pRatesPower[0];

#if 0
    uiPrintf("Final Output Power Setting for channel %d MHz\n", pChval->channel);
    uiPrintf("Final Power 6 Mb %d dBm, MaxRD: %d dBm, MaxEdge %d dBm, - TPC Scale %d dBm - Ant Red %d dBm\n",
        scaledPower / 2, twiceMaxRDPower / 2, twiceMaxEdgePower / 2, tpcScaleReduction, twiceAntennaReduction / 2);
    uiPrintf("Max Turbo %d dBm if turbo and version greater than 3001 : 0x%04X\n", pData->pEepHeader->turbo2WMaxPower,
        pData->version);
    for (i = 0; i < NUM_RATES; i++) {
        uiPrintf("  %d dBm |", pRatesPower[i] / 2);
    }
    uiPrintf("\n");
#endif

    /* Write the power table into the hardware */
    reg32 = (((paPreDEnable & 1)<< 30) | ((pRatesPower[3] & mask) << 24) |
             ((paPreDEnable & 1)<< 22) | ((pRatesPower[2] & mask) << 16) |
             ((paPreDEnable & 1)<< 14) | ((pRatesPower[1] & mask) << 8)  |
             ((paPreDEnable & 1)<< 6 ) |  (pRatesPower[0] & mask));
    writePlatformReg(pDev, PHY_POWER_TX_RATE1, reg32);

    reg32 = ( ((paPreDEnable & 1)<< 30) | ((pRatesPower[7] & mask) << 24) |
         ((paPreDEnable & 1)<< 22) | ((pRatesPower[6] & mask) << 16) |
         ((paPreDEnable & 1)<< 14) | ((pRatesPower[5] & mask) << 8) |
         ((paPreDEnable & 1)<< 6 ) |  (pRatesPower[4] & mask)  );
    writePlatformReg(pDev, PHY_POWER_TX_RATE2, reg32);

    /* set max power to the power value at rate 6 */
    ar5211SetTxPowerLimit(pDev, pRatesPower[0]);
}

/**************************************************************
 * ar5211GetScaledPower
 *
 * Gets or interpolates the pcdac value from the calibrated data
 */
A_UINT16
ar5211GetScaledPower(A_UINT16 channel, A_UINT16 pcdacValue, PCDACS_EEPROM *pSrcStruct)
{
    A_UINT16    powerValue;
    A_UINT16    lFreq, rFreq;           /* left and right frequency values */
    A_UINT16    llPcdac, ulPcdac;       /* lower and upper left pcdac values */
    A_UINT16    lrPcdac, urPcdac;       /* lower and upper right pcdac values */
    A_UINT16    lPwr, uPwr;             /* lower and upper temp pwr values */
    A_UINT16    lScaledPwr, rScaledPwr; /* left and right scaled power */

    if (ar5211FindValueInList(channel, pcdacValue, pSrcStruct, &powerValue)) {
        /* value was copied from srcStruct */
        return powerValue;
    }

    ar5211GetLowerUpperValues(channel, pSrcStruct->pChannelList,
                              pSrcStruct->numChannels, &lFreq, &rFreq);
    ar5211GetLowerUpperPcdacs(pcdacValue, lFreq, pSrcStruct, &llPcdac, &ulPcdac);
    ar5211GetLowerUpperPcdacs(pcdacValue, rFreq, pSrcStruct, &lrPcdac, &urPcdac);

    /* get the power index for the pcdac value */
    ar5211FindValueInList(lFreq, llPcdac, pSrcStruct, &lPwr);
    ar5211FindValueInList(lFreq, ulPcdac, pSrcStruct, &uPwr);
    lScaledPwr = ar5211GetInterpolatedValue(pcdacValue, llPcdac, ulPcdac,
                                            lPwr, uPwr, 0);

    ar5211FindValueInList(rFreq, lrPcdac, pSrcStruct, &lPwr);
    ar5211FindValueInList(rFreq, urPcdac, pSrcStruct, &uPwr);
    rScaledPwr = ar5211GetInterpolatedValue(pcdacValue, lrPcdac, urPcdac,
                                            lPwr, uPwr, 0);

    return ar5211GetInterpolatedValue(channel, lFreq, rFreq,
                                      (A_UINT16)lScaledPwr,
                                      (A_UINT16)rScaledPwr, 0);
}

/**************************************************************
 * ar5211FindValueInList
 *
 * Find the value from the calibrated source data struct
 */
A_BOOL
ar5211FindValueInList(A_UINT16 channel, A_UINT16 pcdacValue, PCDACS_EEPROM *pSrcStruct, A_UINT16 *powerValue)
{
    DATA_PER_CHANNEL    *pChannelData;
    A_UINT16            *pPcdac;
    A_UINT16            i, j;

    pChannelData = pSrcStruct->pDataPerChannel;

    for (i = 0; i < pSrcStruct->numChannels; i++ ) {
        if (pChannelData->channelValue == channel) {
            pPcdac = pChannelData->PcdacValues;

            for (j = 0; j < pChannelData->numPcdacValues; j++ ) {
                if (*pPcdac == pcdacValue) {
                    *powerValue = pChannelData->PwrValues[j];
                    return TRUE;
                }
                pPcdac++;
            }
        }
        pChannelData++;
    }

    return FALSE;
}

/**************************************************************
 * ar5211GetInterpolatedValue
 *
 * Returns interpolated or the scaled up interpolated value
 */
A_UINT16
ar5211GetInterpolatedValue(A_UINT16 target, A_UINT16 srcLeft, A_UINT16 srcRight,
                           A_UINT16 targetLeft, A_UINT16 targetRight, A_BOOL scaleUp)
{
    A_UINT16 returnValue;
    A_INT16 lRatio;
    A_UINT16 scaleValue = EEP_SCALE;

    /*
     * To get an accurate ratio, always scale, If we want to scale, then
     * don't scale back down.
     */
    if ((targetLeft * targetRight) == 0) {
        return 0;
    }

    if (scaleUp) {
        scaleValue = 1;
    }

    if (srcRight != srcLeft) {
        /* note the ratio always need to be scaled, since it will be a fraction */
        lRatio = ((target - srcLeft) * EEP_SCALE / (srcRight - srcLeft));
        if (lRatio < 0) {
            /* Return as Left target if value would be negative */
            returnValue = targetLeft * (scaleUp ? EEP_SCALE : 1);
        } else if (lRatio > EEP_SCALE) {
            /* Return as Right target if Ratio is greater than 100% (SCALE) */
            returnValue = targetRight * (scaleUp ? EEP_SCALE : 1);
        } else {
            returnValue = ((lRatio * targetRight + (EEP_SCALE - lRatio) *
                                  targetLeft) / scaleValue);
        }
    } else {
        returnValue = targetLeft * (scaleUp ? EEP_SCALE : 1);
    }

    return returnValue;
}

/**************************************************************
 * ar5211GetLowerUpperValues
 *
 *  Look for value being within 0.1 of the search values
 *  however, NDIS can't do float calculations, so multiply everything
 *  up by EEP_SCALE so can do integer arithmatic
 *
 * INPUT  value       -value to search for
 * INPUT  pList       -ptr to the list to search
 * INPUT  listSize    -number of entries in list
 * OUTPUT pLowerValue -return the lower value
 * OUTPUT pUpperValue -return the upper value
 *
 * WARNING: This input list should be filled and in ascending order
 */
void
ar5211GetLowerUpperValues(A_UINT16 value, A_UINT16 *pList, A_UINT16 listSize,
                          A_UINT16 *pLowerValue, A_UINT16 *pUpperValue)
{
    A_UINT16    i;
    A_UINT16    listEndValue = *(pList + listSize - 1);
    A_UINT32    target       = value * EEP_SCALE;

    /*
     * See if value is lower than the first value in the list
     * if so return first value
     */
    if (target < (A_UINT32)(*pList * EEP_SCALE - EEP_DELTA)) {
        *pLowerValue = *pList;
        *pUpperValue = *pList;
        return;
    }

    /*
     * See if value is greater than last value in list
     * if so return last value
     */
    if (target > (A_UINT32)(listEndValue * EEP_SCALE + EEP_DELTA)) {
        *pLowerValue = listEndValue;
        *pUpperValue = listEndValue;
        return;
    }

    /* look for value being near or between 2 values in list */
    for (i = 0; i < listSize; i++) {
        /*
         * If value is close to the current value of the list
         * then target is not between values, it is one of the values
         */
        if (A_ABS(pList[i] * EEP_SCALE - (A_INT32)target) < EEP_DELTA) {
            *pLowerValue = pList[i];
            *pUpperValue = pList[i];
            return;
        }

        /*
         * Look for value being between current value and next value
         * if so return these 2 values
         */
        if (target < (A_UINT32)(pList[i + 1] * EEP_SCALE - EEP_DELTA)) {
            *pLowerValue = pList[i];
            *pUpperValue = pList[i + 1];
            return;
        }
    }
}

/**************************************************************
 * ar5211GetLowerUpperPcdacs
 *
 * Get the upper and lower pcdac given the channel and the pcdac
 * used in the search
 */
void
ar5211GetLowerUpperPcdacs(A_UINT16 pcdac, A_UINT16 channel,
                          PCDACS_EEPROM *pSrcStruct, A_UINT16 *pLowerPcdac,
                          A_UINT16 *pUpperPcdac)
{
    DATA_PER_CHANNEL *pChannelData;
    A_UINT16         i;

    /* Find the channel information */
    pChannelData = pSrcStruct->pDataPerChannel;
    for (i = 0; i < pSrcStruct->numChannels; i++) {
        if (pChannelData->channelValue == channel) {
            break;
        }
        pChannelData++;
    }

    ar5211GetLowerUpperValues(pcdac, pChannelData->PcdacValues,
                              pChannelData->numPcdacValues,
                              pLowerPcdac, pUpperPcdac);
}

#ifdef DEBUG
static void
ar5211dumpGainValues(GAIN_VALUES *pGainValues)
{
    uiPrintf("currStepNum %d\n", pGainValues->currStepNum);
    uiPrintf("currGain %d\n", pGainValues->currGain);
    uiPrintf("targetGain %d\n", pGainValues->targetGain);
    uiPrintf("loTrig %d\n", pGainValues->loTrig);
    uiPrintf("hiTrig %d\n", pGainValues->hiTrig);
    uiPrintf("[Cur] StepName: %s StepGain: %2d bb_tx_clip %d rf_pwd_90 %d "
             "rf_pwd_84 %d rf_rfgainsel %d\n",
             pGainValues->currStep->stepName, pGainValues->currStep->stepGain,
             pGainValues->currStep->paramVal[0], pGainValues->currStep->paramVal[1],
             pGainValues->currStep->paramVal[2], pGainValues->currStep->paramVal[3]);
}
#endif

/**************************************************************
 * ar5211InitializeGainValues
 *
 * Initialize the gain structure to good values
 */
void
ar5211InitializeGainValues(GAIN_VALUES *pGainValues)
{
    /* initialize gain optimization values */
    pGainValues->currStepNum = GainLadder.defaultStepNum;
    pGainValues->currStep    = (GAIN_OPTIMIZATION_STEP *)&(GainLadder.optStep[GainLadder.defaultStepNum]);
    pGainValues->active      = TRUE;
    pGainValues->loTrig      = 20;
    pGainValues->hiTrig      = 35;
}

/**************************************************************
 * ar5211InvalidGainReadback
 */
A_BOOL
ar5211InvalidGainReadback(GAIN_VALUES *pGainValues, A_UINT16 channelFlags)
{
    A_UINT32    gStep, g;
    A_UINT32    L1, L2, L3, L4;

    gStep = IS_CHAN_CCK(channelFlags) ? 0x18 : 0x3f;

    L1 = 0;
    L2 = (gStep == 0x3f) ? 50 : gStep + 4;
    L3 = (gStep != 0x3f) ? 0x40 : L1;
    L4 = L3 + 50;

    pGainValues->loTrig = L1 + ((gStep == 0x3f) ? DYN_ADJ_LO_MARGIN : 0);
    pGainValues->hiTrig = L4 - ((gStep == 0x3f) ? DYN_ADJ_UP_MARGIN : -5); // never adjust if != 0x3f

    g = pGainValues->currGain;

    return !( ((g>=L1) && (g<=L2)) || ((g>=L3) && (g<=L4)) );
}

/**************************************************************
 * ar5211RequestRfgain
 *
 * Enable the probe gain check on the next packet
 */
void
ar5211RequestRfgain(WLAN_DEV_INFO *pDev)
{
    A_UINT32 reg;

    /* Enable the gain readback probe */
    reg = ((pDev->tx6PowerInHalfDbm << PHY_PAPD_PROBE_POWERTX_S)  & PHY_PAPD_PROBE_POWERTX_M) |
          PHY_PAPD_PROBE_NEXT_TX;
    A_REG_WR(pDev, PHY_PAPD_PROBE, reg);

    pDev->pHalInfo->rfgainState = RFGAIN_READ_REQUESTED;
}

/**************************************************************
 * ar5211GetRfgain
 *
 * Exported call to check for a recent gain reading and return
 * the current state of the thermal calibration gain engine.
 */
RFGAIN_STATES
ar5211GetRfgain(WLAN_DEV_INFO *pDev)
{
    GAIN_VALUES *pGainValues = pDev->pHalInfo->pGainValues;
    A_UINT32    rddata;

    if (!(pGainValues->active)) {
        return (-1);
    }

    if(pDev->pHalInfo->rfgainState == RFGAIN_READ_REQUESTED) {
        /* Caller had asked to setup a new reading. Check it. */
        rddata = A_REG_RD(pDev, PHY_PAPD_PROBE);
        if (!(rddata & PHY_PAPD_PROBE_NEXT_TX)) {
            /* bit got cleared, we have a new reading. */
            pGainValues->currGain = (rddata >> PHY_PAPD_PROBE_GAINF_S);
            pDev->pHalInfo->rfgainState = RFGAIN_INACTIVE; // inactive by default
            if (!ar5211InvalidGainReadback(pGainValues, pDev->staConfig.pChannel->channelFlags)) {
                if (ar5211IsGainAdjustNeeded(pGainValues)) {
                    if (ar5211AdjustGain(pGainValues) > 0) {
                        /* Change needed. Copy ladder info into eeprom info. */
                        ar5211SetRfgain(pDev);
                        pDev->pHalInfo->rfgainState = RFGAIN_NEED_CHANGE;
                        /* Request IQ recalibration for temperature change */
                        pDev->pHalInfo->iqCalState = IQ_CAL_INACTIVE;
                    }
                }
            }
        }
    }
    return pDev->pHalInfo->rfgainState;
}

/**************************************************************
 * ar5211IsGainAdjustNeeded
 *
 * Check to see if our readback gain level sits within the linear
 * region of our current variable attenuation window
 */
A_BOOL
ar5211IsGainAdjustNeeded(GAIN_VALUES *pGainValues)
{
    return ((pGainValues->currGain <= pGainValues->loTrig) || (pGainValues->currGain >= pGainValues->hiTrig));
}

/**************************************************************
 * ar5211AdjustGain
 *
 * Move the rabbit ears in the correct direction.
 */
A_INT32
ar5211AdjustGain(GAIN_VALUES *pGainValues)
{
    /* return > 0 for valid adjustments. */
    if (!(pGainValues->active)) {
        return (-1);
    }

    pGainValues->currStep = &(GainLadder.optStep[pGainValues->currStepNum]);

    if (pGainValues->currGain >= pGainValues->hiTrig) {
        if (pGainValues->currStepNum == 0) {
#ifdef DEBUG
            if (GainDebug) {
                uiPrintf("Max gain limit.\n");
            }
#endif
            return -1;
        }

#ifdef DEBUG
        if (GainDebug) {
            uiPrintf("Adding gain: currG=%d [%s] --> ",
                pGainValues->currGain, pGainValues->currStep->stepName);
        }
#endif

        pGainValues->targetGain = pGainValues->currGain;
        while ((pGainValues->targetGain >= pGainValues->hiTrig) && (pGainValues->currStepNum > 0)) {
            pGainValues->targetGain -= 2 * (GainLadder.optStep[--(pGainValues->currStepNum)].stepGain - pGainValues->currStep->stepGain);
            pGainValues->currStep = &(GainLadder.optStep[pGainValues->currStepNum]);
        }

#ifdef DEBUG
        uiPrintf("targG=%d [%s]\n", pGainValues->targetGain, pGainValues->currStep->stepName);
#endif
        return 1;
    }

    if (pGainValues->currGain <= pGainValues->loTrig) {
        if (pGainValues->currStepNum == (GainLadder.numStepsInLadder - 1)) {
#ifdef DEBUG
            if (GainDebug) {
                uiPrintf("Min gain limit.\n");
            }
#endif
            return -2;
        }

#ifdef DEBUG
        if (GainDebug) {
            uiPrintf("Deducting gain: currG=%d [%s] --> ", pGainValues->currGain, pGainValues->currStep->stepName);
        }
#endif

        pGainValues->targetGain = pGainValues->currGain;
        while ((pGainValues->targetGain <= pGainValues->loTrig) &&
            (pGainValues->currStepNum < (GainLadder.numStepsInLadder - 1)))
        {
            pGainValues->targetGain -= 2 * (GainLadder.optStep[++(pGainValues->currStepNum)].stepGain -
                pGainValues->currStep->stepGain);
            pGainValues->currStep = &(GainLadder.optStep[pGainValues->currStepNum]);
        }

#ifdef DEBUG
        uiPrintf("targG=%d [%s]\n", pGainValues->targetGain, pGainValues->currStep->stepName);
#endif

        return 2;
    }

    return (0); // should've called needAdjGain before calling this.
}

/**************************************************************
 * ar5211SetRfgain
 *
 * Adjust the 5GHz EEPROM information with the desired calibration values.
 */
void
ar5211SetRfgain(WLAN_DEV_INFO *pDev)
{
    GAIN_VALUES *pGainValues = pDev->pHalInfo->pGainValues;
    EEP_HEADER_INFO   *pHeaderInfo = pDev->pHalInfo->pEepData->pEepHeader;

    if (!(pGainValues->active)) {
        return ;
    }
#ifdef DEBUG
    if (GainDebug) {
        ar5211dumpGainValues(pGainValues);
    }
#endif

    pHeaderInfo->cornerCal.clip = pGainValues->currStep->paramVal[0]; // bb_tx_clip
    pHeaderInfo->cornerCal.pd90 = pGainValues->currStep->paramVal[1]; // rf_pwd_90
    pHeaderInfo->cornerCal.pd84 = pGainValues->currStep->paramVal[2]; // rf_pwd_84
    pHeaderInfo->cornerCal.gSel = pGainValues->currStep->paramVal[3]; // rf_rfgainsel
}

#endif // #ifdef BUILD_AR5211
