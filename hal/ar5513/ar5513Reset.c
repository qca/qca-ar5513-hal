/*
 * Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 * Chips specific device attachment and device info collection
 * Connects Init Reg Vectors, EEPROM Data, and device Functions to pDev
 */

#ifdef BUILD_AR5513

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Reset.c#19 $"

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
#include "ar5513MacReg.h"
#include "ar5513Misc.h"
#include "ar5513Reset.h"
#include "ar5513Power.h"
#include "ar5513Receive.h"
#include "ar5513Mac.h"
#ifndef BUILD_AP
#include "intercept.h"
#endif

#if defined(AR5513)
#include "ar5513reg.h"  /* ar5513 system definitions */
#endif /* AR5513 */

/* Add const static register initialization vectors */
#if defined(FALCON_EMUL)
#include "ar5513/falcon_mac_emul.ini"
#elif defined(AR5513)
#include "ar5513/ar5513.ini"
#endif /* AR5513 */

#ifndef NDIS_HW
#include "apcfg.h"
#endif

static void
ar5513GetNf(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_STATUS
ar5513SetResetReg(WLAN_DEV_INFO *pDev, A_UINT32 resetMask);

static A_STATUS
ar5513SetChannel5112(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static void
ar5513SetRfRegs5112(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo,
                CHAN_VALUES *pChval, A_UINT16 modesIndex, A_UINT16 *rfXpdGain);

static void
ar5513ModifyRfBuffer(A_UINT32 *rfBuf, A_UINT32 reg32, A_UINT32 numBits, A_UINT32 firstBit, A_UINT32 column);

static A_UINT32
ar5513GetRfField(A_UINT32 *rfBuf, A_UINT32 numBits, A_UINT32 firstBit, A_UINT32 column);

static void
ar5513SetBoardValues(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval);

static void
ar5513SetDeltaSlope(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_STATUS
ar5513SetTransmitPower(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval, A_UINT16 *rfXpdGain, A_INT16 powerLimit, A_BOOL calledDuringReset, A_UINT8 chnIdx);

static A_INT16
getPminAndPcdacTableFromPowerTable(A_INT16 *pwrTableT4, A_UINT16 retVals[]);

static A_INT16
getPminAndPcdacTableFromTwoPowerTables(A_INT16 *pwrTableLXpdT4, A_INT16 *pwrTableHXpdT4,
                                       A_UINT16 retVals[], A_INT16 *pMid);

static void
ar5513SetPowerTable5112(WLAN_DEV_INFO *pDev, A_UINT16 *pPcdacTable, A_INT16 *pPowerMin, A_INT16 *pPowerMax, CHAN_VALUES *pChval, A_UINT16 *rfXpdGain, A_UINT8 chnIdx);

static void
ar5513SetRateTable(WLAN_DEV_INFO *pDev, A_UINT16 *pRatesPower,
                   TRGT_POWER_ALL_MODES *pTargetPowers, CHAN_VALUES *pChval,
                   A_INT16 tpcScaleReduction, A_INT16 powerLimit, struct eepMap *pData,
                   A_INT16 *pMinPower, A_INT16 *pMaxPower, A_INT16 *pBandEdge, A_UINT8 chnIdx);

static void
ar5513CorrectGainDelta(WLAN_DEV_INFO *pDev, A_UINT16 *pRatesPower,
                       A_UINT16 *pPowerValues, int twiceOfdmCckDelta);

static void
ar5513GetTargetPowers(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval, TRGT_POWER_INFO *pPowerInfo,
                      A_UINT16 numChannels, TRGT_POWER_INFO *pNewPower);

static A_UINT16
ar5513GetMaxEdgePower(A_UINT16 channel, RD_EDGES_POWER  *pRdEdgesPower);

static A_UINT16
ar5513GetInterpolatedValue(A_UINT16 target, A_UINT16 srcLeft, A_UINT16 srcRight,
                           A_UINT16 targetLeft, A_UINT16 targetRight, A_BOOL scaleUp);

static void
ar5513GetLowerUpperValues(A_UINT16 value, A_UINT16 *pList, A_UINT16 listSize,
                          A_UINT16 *pLowerValue, A_UINT16 *pUpperValue);

static void
ar5513RequestRfgain(WLAN_DEV_INFO *pDev);

static A_BOOL
ar5513InvalidGainReadback(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues, A_UINT16 channelFlags);

static A_BOOL
ar5513IsGainAdjustNeeded(GAIN_VALUES *pGainValues);

static A_INT32
ar5513AdjustGain(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues);

void
ar5513GetGainFCorrection(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues);

static void
ar5513SetRateDurationTable(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_STATUS
ar5513SetXrMode(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType);

static A_BOOL
ar5513IsEarEngaged(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static A_BOOL
ar5513EarModify(WLAN_DEV_INFO *pDev, EAR_LOC_CHECK loc, CHAN_VALUES *pChval, A_UINT32 *modifier);

static A_UINT16
ar5513EarDoRH(WLAN_DEV_INFO *pDev, REGISTER_HEADER *pRH);

static A_UINT32 *
ar5513GetRfBank(WLAN_DEV_INFO *pDev, A_UINT16 bank);

static A_UINT16
ar5513CFlagsToEarMode(WLAN_CFLAGS cflags);

static A_BOOL
ar5513IsChannelInEarRegHead(A_UINT16 earChannel, A_UINT16 channelMhz);

static A_UINT32
ar5513AdjustPhaseRamp(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

static void
ar5513InitSynth2_4War(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

/* const globals for export to the attach */

typedef struct ar5513RfBanks5112 {
    A_UINT32 Bank1Data[sizeof(ar5212Bank1) / sizeof(*(ar5212Bank1))];
    A_UINT32 Bank2Data[sizeof(ar5212Bank2) / sizeof(*(ar5212Bank2))];
    A_UINT32 Bank3Data[sizeof(ar5212Bank3) / sizeof(*(ar5212Bank3))];
    A_UINT32 Bank6Data[sizeof(ar5212Bank6) / sizeof(*(ar5212Bank6))];
    A_UINT32 Bank7Data[sizeof(ar5212Bank7) / sizeof(*(ar5212Bank7))];
} AR5513_RF_BANKS_5112;

const struct RfHalFuncs ar5112Funcs = {
    ar5513SetChannel5112,
    ar5513SetRfRegs5112,
    ar5513SetPowerTable5112
};

/* Additional Time delay to wait after activiting the Base band */
#define BASE_ACTIVATE_DELAY    100 //100 usec
#define PLL_SETTLE_DELAY       300 //300 usec

/* FALCON ONLY DEFINITION */
#ifdef BUILD_AP
/* Assumes AV10 */
#define FALCON_BAND_EIRP_HEADROOM_IN_DB 6
#else
/* Assumes CB63 */
#define FALCON_BAND_EIRP_HEADROOM_IN_DB 0
#endif
#define FALCON_TWICE_SINGLE_CHAIN_GAIN     5
#define FALCON_TWICE_POWER_MAX_OFFSET      6 
#define FALCON_TWICE_COHERENT_COMB_GAIN    12 

/*
 * Bug 10685, need to provide a registry based control for
 * the maximum number of registers that can be written to by
 * REG_WRITE_ARRAY and REG_WRITE_RF_ARRAY, without a 1 us delay
 * This is to provide some level of flexibility for only the ndis
 * drivers to better interoperate with out drivers, namely the
 * sound drivers.
 * Provided a new Registry entry writeBlockSize, defaults to 64
 * note, for the BUILD_AP this will always be 64.
 * Also this was only implemented for the AR5513 hal and not the
 * AR5211 hal.
 */
#define REG_WRITE_ARRAY(regArray, column, regWar, writeBlockSize)           \
{   int entries, r;                                                         \
    udelay(1);                                                              \
    entries = sizeof(regArray) / sizeof(*(regArray));                       \
    for (r = 0; r < entries; r++) {                                         \
        A_REG_WR(pDev, (regArray)[r][0], (regArray)[r][(column)]);          \
        if ((++(regWar) % writeBlockSize) == 0) {                           \
            udelay(1);                                                      \
        }                                                                   \
    }                                                                       \
}

#define REG_WRITE_RF_ARRAY(regAddress, regData, regWar, writeBlockSize)     \
{   int entries, r;                                                         \
    udelay(1);                                                              \
    entries = sizeof(regAddress) / sizeof(*(regAddress));                   \
    for (r = 0; r < entries; r++) {                                         \
        A_REG_WR(pDev, (regAddress)[r][0], (regData)[r]);                   \
        if ((++(regWar) % writeBlockSize) == 0) {                           \
            udelay(1);                                                      \
        }                                                                   \
    }                                                                       \
}

/*
 * WAR for bug 6773.  udelay() does a PIO READ on the PCI bus which allows
 * other cards' DMA reads to complete in the middle of our reset.
 */
#define WAR_6773(x)          \
    if ((++(x) % 64) == 0) { \
        udelay(1);           \
    }

#if defined(WMAC_WRITE_REG)
int dump5513Regs = 1;

#ifdef BUILD_AP
void writePlatformReg(WLAN_DEV_INFO *pdevInfo, ULONG reg, ULONG val)
{
    (*((volatile ULONG *)((pdevInfo)->baseAddress + (reg))) = (val));
    if (dump5513Regs) {
	uiPrintf("0x%04lX: 0x%08lX\n", reg, val);
    }
}
#endif /* BUILD_AP */
#endif /* WMAC_WRITE_REG */

#ifdef DEBUG
int printPhaseCal = 0;
#endif

/**************************************************************
 * ar5513Reset
 *
 * Places the device in and out of reset and then places sane
 * values in the registers based on EEPROM config, initialization
 * vectors (as determined by the mode), and station configuration
 *
 * bChannelChange is used to preserve DMA/PCU registers across
 * a HW Reset during channel change.
 */
A_STATUS
ar5513Reset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange)
{
    A_UINT32           ulAddressLow, ulAddressHigh;
    A_UINT32           macStaId1;
    A_UINT32           i;
    A_UINT32           reg, data, synthDelay;
    A_UINT16           rfXpdGain[2];
    A_STATUS           status;
    A_UINT16           arrayEntries;
    A_INT16            powerLimit = MAX_RATE_POWER;
    A_INT16            cckOfdmPwrDelta = 0;
    A_UINT16           modesIndex = 0, freqIndex = 0;
    A_UINT32           saveDefAntenna, saveFrameSeqCount = 0;
    A_UINT32           modifier = 0;
    A_BOOL             earHere;
    struct eepMap      *pEepData = pDev->pHalInfo->pEepData;
    WLAN_STA_CONFIG    *pCfg = &pDev->staConfig;
    int                regWrites = 0;
    const RF_HAL_FUNCS *pRfHal = pDev->pHalInfo->pRfHal;
    A_UINT32           useMic;
    VPORT_BSS          *pVportBaseBss = GET_BASE_BSS(pDev);
    A_UINT32           testReg;
    A_STATUS           ret;
    A_UINT32           writeBlockSize;

    ASSERT(((pChval->channelFlags & CHANNEL_2GHZ) || 0) ^
           ((pChval->channelFlags & CHANNEL_5GHZ) || 0));
    ASSERT(pEepData->version >= EEPROM_VER3);

    if (halGetCapability(pDev, HAL_GET_MIC_SUPPORT, PRIV_KEY_TYPE_TKIP)) {
        useMic = MAC_STA_ID1_CRPT_MIC_ENABLE;
    } else {
        useMic = 0;
    }

    earHere = ar5513IsEarEngaged(pDev, pChval);

    /* Preserve certain DMA hardware registers on a channel change */
    if (bChannelChange) {
        /*
         * AR5513 WAR
         *
         * On Venice, the TSF is almost preserved across a reset;
         * it requires the WAR of doubling writes to the RESET_TSF
         * bit in the MAC_BEACON register; it also has the quirk
         * of the TSF going back in time on the station (station
         * latches onto the last beacon's tsf during a reset 50%
         * of the times); the latter is not a problem for adhoc
         * stations since as long as the TSF is behind, it will
         * get resynchronized on receiving the next beacon; the
         * TSF going backwards in time could be a problem for the
         * sleep operation (supported on infrastructure stations
         * only) - the best and most general fix for this situation
         * is to resynchronize the various sleep/beacon timers on
         * the receipt of the next beacon i.e. when the TSF itself
         * gets resynchronized to the AP's TSF - power save is
         * needed to be temporarily disabled until that time
         *
         * Need to save the sequence number to restore it after
         * the reset!
         *
         */
        saveFrameSeqCount = readPlatformReg(pDev, MAC_D0_SEQNUM);

        /* Save off the latest NF calibration */
        ar5513GetNf(pDev, pDev->staConfig.phwChannel);
    }

    /*
     * Preserve the antenna on a channel change
     */
    saveDefAntenna = readPlatformReg(pDev, MAC_DEF_ANTENNA);

    /* Save hardware flag before chip reset clears the register */
    macStaId1 = readPlatformReg(pDev, MAC_STA_ID1) & MAC_STA_ID1_BASE_RATE_11B;

    /* Move to refClk operation */
    ar5513RestoreSleepRegisters(pDev);

    /* Adjust gain parameters before reset if there's an outstanding gain update */
    ar5513GetRfgain(pDev);

    if (pDev->powerMgmt.powerState == D3_STATE) {
#ifdef PCI_INTERFACE
        /*
         * We just wake up from D3 mode, need repogram DSL register
         */
        uiPrintf ("wake up from D3 and reporgram DSL register\n");
        writePlatformReg(pDev, RST_AHB_ARB_CTL, PCI_AHB_MASTER | WMAC_AHB_MASTER);
        A_REG_CLR_BIT(pDev, RST_BYTESWAP_CTL, WMAC);
        A_REG_CLR_BIT(pDev, RST_BYTESWAP_CTL, PCI);
        writePlatformReg(pDev, RST_PLLV_CTL,POWER_DOWN_ENABLE);
        writePlatformReg(pDev, RST_MIMR, 0x00);
        /*
        * Set the AMBA Clk to the proper CLk source, 
        * 0xac5f to xx64 register for 92Mhz operation 
        * 0xac7b to xx64 register for 120Mhz  operation 
        */ 

        writePlatformReg(pDev, RST_PLLC_CTL, 0xac7b);
        udelay(10000);
        writePlatformReg(pDev, RST_AMBACLK_CTL, AMBACLK_PLLC_DIV3_CLK);

#ifdef DEBUG
        A_REG_SET_BIT(pDev, RST_OBS_CTL, RX_CLEAR_EN);
#endif /* DEBUG */

        /* Set Observation control register for LED operation */
        A_REG_SET_BIT2(pDev, RST_OBS_CTL, LED_0, LED_1);

#ifndef FALCON_E1_SILICON
        writePlatformReg(pDev, RST_DSL_SLEEP_CTL, CLIENT_SLEEP_CTL);
#endif /* !FALCON_E1_SILICON */
#endif /* PCI_INTERFACE */
    }
    pDev->pHalInfo->halInit = FALSE;

    status = ar5513ChipReset(pDev, pChval);
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
    case CHANNEL_XR_A:
        modesIndex = 1;
        freqIndex  = 1;
        break;
    case CHANNEL_T:
    case CHANNEL_XR_T:
        modesIndex = 2;
        freqIndex  = 1;
        break;
    case CHANNEL_B:
        modesIndex = 3;
        freqIndex  = 2;
        break;
    case CHANNEL_G:
    case CHANNEL_XR_G:
        modesIndex = 4;
        freqIndex  = 2;
        break;
    case CHANNEL_108G:
        modesIndex = 5;
        freqIndex  = 2;
        break;
    default:
        /* Ah, a new wireless mode */
        ASSERT(0);
        break;
    }

    /* Set correct Baseband to analog shift setting to access analog chips. */
    writePlatformReg(pDev, PHY_BASE, 0x00000007);

#ifdef BUILD_AP
    writeBlockSize = MAX_WRITEBLOCKSIZE;
#else
    writeBlockSize = pDev->staConfig.writeBlockSize;
#endif

    REG_WRITE_ARRAY(ar5212Modes, modesIndex, regWrites, writeBlockSize);
    /* Write Common Array Parameters */
    arrayEntries = sizeof(ar5212Common) / sizeof(*ar5212Common);
    for (i = 0; i < arrayEntries; i++) {
        reg = ar5212Common[i][0];

        /* On channel change, don't reset the PCU registers */
        if ( !(bChannelChange && (reg >= 0x8000 && reg < 0x9000)) ) {
            /* TODO: Is this still needed here for 5212? */
            A_REG_WR(pDev, reg, ar5212Common[i][1]);
            WAR_6773(regWrites);
        }
    }
    if (IS_5112(pDev)) {
        REG_WRITE_ARRAY(ar5212Modes, modesIndex, regWrites, writeBlockSize);
        REG_WRITE_ARRAY(ar5212Common, 1, regWrites, writeBlockSize);
        REG_WRITE_ARRAY(ar5212BB_RfGain, freqIndex, regWrites, writeBlockSize);
    }

    /*
    ** Set Antenna Fast Diversity Bias to 10 dB in order
    ** to allow software to make majority of Antenna
    ** selections. 1 Bias unit is 0.5 dB.
    */
#ifdef AR5513_ENABLE_FAST_DIVERSITY   /* TODO: AR5513 currently does not support fast diversity */
    reg = A_REG_RD(pDev,PHY_MULTICHN_GAIN_CTRL);
    reg = (reg & ~PHY_ANT_FAST_DIV_BIAS_MASK) | 
	(PHY_10_DB_BIAS << PHY_ANT_FAST_DIV_BIAS_SHFT);
    A_REG_WR(pDev,PHY_MULTICHN_GAIN_CTRL,reg);
#endif

    A_REG_WR(pDev,PHY_MULTICHN_ENB, 0);

    switch (pDev->staConfig.txChainCtrl) {
    case CHAIN_FIXED_A:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_0_ENB << PHY_TX_CHN_SHIFT);
        A_REG_WR(pDev, PHY_MULTICHN_ENB, reg);

        reg = A_REG_RD(pDev,PHY_TXBF_CTRL);
        reg |= PHY_CHN_0_FORCE;        /* Force Chain 0 as static chain sel */
        A_REG_WR(pDev,PHY_TXBF_CTRL,reg);
        break;

    case CHAIN_FIXED_B:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_1_ENB << PHY_TX_CHN_SHIFT);
        A_REG_WR(pDev,PHY_MULTICHN_ENB,reg);

        reg = A_REG_RD(pDev,PHY_TXBF_CTRL);
        reg |= PHY_CHN_1_FORCE;       /* Force Chain 1 as static chain sel */
        A_REG_WR(pDev,PHY_TXBF_CTRL,reg);
        break;

    case DUAL_CHAIN:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_ALL_ENB << PHY_TX_CHN_SHIFT);
        A_REG_WR(pDev,PHY_MULTICHN_ENB,reg);

        /*
         * Compute Phase Ramp between Chain 0 & Chain 1.
         */

        /*
         * Note that AR5513 in 2.4 GHz modes will overwrite this later
         * as a result of the workaround for E2.0 chips.
         */
        if (!(IS_CHAN_2GHZ(pChval->channelFlags))) {
            ar5513AdjustPhaseRamp(pDev, pChval);
#ifndef NDIS_HW
        } else {
            if (!(apCfgWarOptionGet() & WAR_SYNTHCALNORST)) {
                /* if configured to re-do synth cal for each wlan reset */
                pDev->pHalInfo->done_synth_state_check_2_4 = FALSE;
                pDev->pHalInfo->synth_state_2_4            = 0;
                pDev->pHalInfo->synth_state_flag           = 0;
            }
#endif
        }

        /*
         * Initialize 2.4 GHz synth workaround logic.
         * 
         * Note that antenna is overwritten later by ar5513SetAntennaSwitch(),
         * phaseRampInit0 and frame type capability table will be overwritten
         * by ar5513UpdateSynth2_4War().
         */
        ar5513InitSynth2_4War(pDev, pChval);
        break;
    }

    /*
    ** Initialize the Frame Type Capabilities Table.
    ** Enable beam forming on all 802.11 Packet types
    ** with the exception of BFCOEF_TX_ENABLE_MCAST.
    */

    ar5513InitFrmTypeCapTable(pDev);


    /* Workaround for 11b mode performance problem (bug 14611)
     *
     * STA, 11b channel => disable receive combining
     *
     */
    if (serviceType == WLAN_STA_SERVICE) {
#ifdef NDIS_HW
        /* Not for STA mode on AV10 */
        pDev->staConfig.rxChainCtrl = pDev->defaultStaConfig.rxChainCtrl;
#endif
        if (IS_CHAN_B(pChval->channelFlags)) {
            pDev->staConfig.rxChainCtrl = CHAIN_FIXED_A;
        }
    }

    switch (pDev->staConfig.rxChainCtrl) {
    case CHAIN_FIXED_A:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_0_ENB << PHY_RX_CHN_SHIFT);
        A_REG_WR(pDev,PHY_MULTICHN_ENB,reg);
        break;

    case CHAIN_FIXED_B:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_1_ENB << PHY_RX_CHN_SHIFT);
        A_REG_WR(pDev,PHY_MULTICHN_ENB,reg);
        break;

    case DUAL_CHAIN:
        reg = A_REG_RD(pDev, PHY_MULTICHN_ENB);
        reg |= (PHY_CHN_ALL_ENB << PHY_RX_CHN_SHIFT);
        A_REG_WR(pDev,PHY_MULTICHN_ENB,reg);
        break;
    }

    ar5513SetRateDurationTable(pDev, pChval);

    /* Overwrite INI values for revised chipsets */
    if (pDev->phyRev >= PHY_CHIP_ID_REV_2) {
        /* ADC_CTL */
        A_REG_WR(pDev, PHY_ADC_CTL, A_FIELD_VALUE(PHY_ADC_CTL, OFF_INBUFGAIN, 2) |
            A_FIELD_VALUE(PHY_ADC_CTL, ON_INBUFGAIN, 2) |
            PHY_ADC_CTL_OFF_PWDDAC | PHY_ADC_CTL_OFF_PWDADC);

        /* TX_PWR_ADJ */
        if (pChval->channel == 2484) {
            cckOfdmPwrDelta = SCALE_OC_DELTA(pEepData->pEepHeader->cckOfdmPwrDelta - pEepData->pEepHeader->scaledCh14FilterCckDelta);
        } else {
            cckOfdmPwrDelta = SCALE_OC_DELTA(pEepData->pEepHeader->cckOfdmPwrDelta);
        }
        A_REG_WR(pDev, PHY_TXPWRADJ, A_FIELD_VALUE(PHY_TXPWRADJ, CCK_GAIN_DELTA, pEepData->pEepHeader->cckOfdmGainDelta * -1) |
            A_FIELD_VALUE(PHY_TXPWRADJ, CCK_PCDAC_INDEX, cckOfdmPwrDelta));

        /* Add barker RSSI thresh enable */
        A_REG_SET_BIT(pDev, PHY_DAG_CTRLCCK, EN_RSSI_THR);
        A_REG_RMW_FIELD(pDev, PHY_DAG_CTRLCCK, RSSI_THR, 2);

        /* Set the mute mask to the correct default */
        A_REG_WR(pDev, MAC_SEQ_MASK, 0);
    }
    if (pDev->phyRev >= PHY_CHIP_ID_REV_3) {
        /* Clear reg to allow RX_CLEAR line debug */
        A_REG_WR(pDev, PHY_BLUETOOTH, 0);
    }

    if (IS_5312_2_X(pDev)) {
        /* ADC_CTRL and max input changes */
        A_REG_WR(pDev, PHY_SIGMA_DELTA, A_FIELD_VALUE(PHY_SIGMA_DELTA, ADC_SEL, 2) |
            A_FIELD_VALUE(PHY_SIGMA_DELTA, FILT2, 4) | A_FIELD_VALUE(PHY_SIGMA_DELTA, FILT1, 0x16) |
            A_FIELD_VALUE(PHY_SIGMA_DELTA, ADC_CLIP, 0));

        if (IS_CHAN_2GHZ(pChval->channelFlags)) {
            A_REG_RMW_FIELD(pDev, PHY_RXGAIN, TXRX_RF_MAX, 0xF);
        }
    }

    /* Setup the transmit power values. */
    if (!pDev->invalidTxChannel && pDev->bssDescr && IS_ELEMENT_USED(&pDev->bssDescr->tpcIe)) {
        powerLimit = A_MIN(pDev->bssDescr->tpcIe.pwrLimit, powerLimit);
    }

    if (ar5513SetTransmitPower(pDev, pChval, rfXpdGain,
                               powerLimit, TRUE, CHAIN_0) != A_OK)
    {
        return A_ERROR;
    }

    if (ar5513SetTransmitPower(pDev, pChval, rfXpdGain,
                               powerLimit, TRUE, CHAIN_1) != A_OK)
    {
        return A_ERROR;
    }

    /* Write the analog registers */
    pRfHal->ar5513SetRfRegs(pDev, pEepData->pEepHeader, pChval, modesIndex, rfXpdGain);

    /* Write delta slope for OFDM enabled modes (A, G, Turbo) */
    if (IS_CHAN_OFDM(pChval->channelFlags)) {
        ar5513SetDeltaSlope(pDev, pChval);
    }

    /* Setup board specific options for EEPROM version 3 */
    /* this fcn calls ar5513SetAntennaSwitch() */
    ar5513SetBoardValues(pDev, pEepData->pEepHeader, pChval);

    /* Restore certain DMA hardware registers on a channel change */
    if (bChannelChange) {
        writePlatformReg(pDev, MAC_D0_SEQNUM, saveFrameSeqCount);
    }

    ulAddressLow  = cpu2le32(pCfg->macAddr.st.word),
    ulAddressHigh = cpu2le16(pCfg->macAddr.st.half);
    writePlatformReg(pDev, MAC_STA_ID0, ulAddressLow);

    pDev->pHalInfo->halInit = TRUE;

    if (serviceType == WLAN_AP_SERVICE) {
        writePlatformReg(pDev, MAC_STA_ID1, macStaId1 | ulAddressHigh |
                         MAC_STA_ID1_STA_AP | MAC_STA_ID1_KSRCH_MODE |
                         MAC_STA_ID1_RTS_USE_DEF | useMic);
        writePlatformReg(pDev, MAC_BSS_ID0, ulAddressLow);
        writePlatformReg(pDev, MAC_BSS_ID1, ulAddressHigh);
    } else {
        /*
         * Set the IBSS mode only if we were configured to be so. If bssType is
         * ANY_BSS then the scan will determine what mode we finally end up in.
         */
        ASSERT((pCfg->bssType == INDEPENDENT_BSS) || (pCfg->bssType == INFRASTRUCTURE_BSS));
        writePlatformReg(pDev, MAC_STA_ID1, macStaId1 | ulAddressHigh |
                         ((pCfg->bssType == INDEPENDENT_BSS) ?
                          MAC_STA_ID1_AD_HOC : 0) | MAC_STA_ID1_KSRCH_MODE |
                         MAC_STA_ID1_RTS_USE_DEF | useMic);
        /* then our BSSID */
        ulAddressLow  = cpu2le32(pVportBaseBss->bss.bssId.st.word);
        ulAddressHigh = cpu2le16(pVportBaseBss->bss.bssId.st.half);
        writePlatformReg(pDev, MAC_BSS_ID0, ulAddressLow);
        writePlatformReg(pDev, MAC_BSS_ID1, ulAddressHigh);

        /* It looks like prefetch causes TX_HANG on Venice card, so I will temporarily
         * comment out this code. Since we're adding this for performance, I won't
         * delete it since we decide performance is more important than TX_HANG
         */

        /*
         * enable prefetch on DCU 0 when only single queue is operational
         */
        if (pDev->txPrefetchStats.hungCount == 0)
        {
            A_REG_WR(pDev, MAC_D_FPCTL, MAC_D_FPCTL_PREFETCH_EN |
                     A_FIELD_VALUE(MAC_D_FPCTL, DCU, 0) |
                     A_FIELD_VALUE(MAC_D_FPCTL, BURST_PREFETCH, 1));
        }
    }

    /* Restore previous antenna */
    ar5513SetDefAntenna(pDev, saveDefAntenna);

    /* Restore slot time */
    if (IS_CHAN_G(pChval->channelFlags) || IS_CHAN_XR_G(pChval->channelFlags)) {
        /* force chip back to sw state */
        ar5513UseShortSlotTime(pDev, pDev->useShortSlotTime, !pDev->useShortSlotTime);
    }

    writePlatformReg(pDev, MAC_ISR, 0xFFFFFFFF); // cleared on write

    /* Set CFG, little endian for register access. */
    pDev->pHalInfo->swSwapDesc = FALSE;
#ifdef BIG_ENDIAN
    /* need swap tx desc by SW if compression is enabled due to bug 7300 */
    pDev->pHalInfo->swSwapDesc = pDev->pHalInfo->halCapabilities.halCompressSupport &&
        ((pDev->staConfig.abolt & ABOLT_COMPRESSION) == ABOLT_COMPRESSION);

    writePlatformReg(pDev, MAC_CFG, pDev->pHalInfo->swSwapDesc ?
                     MAC_CFG_SWRD : MAC_CFG_SWTD | MAC_CFG_SWRD);

#else
    /*
     * WAR for Bug 7987
     * Some versions of the ENE PCI chipset can not handle
     * back to back pci operation on the same PCI GRANT.
     * Disabled  multiple PCI operation on a single grant.
     * This may degrade performance but the impact should
     * be insignificant.
     * Tested with turbo operation against Chariot without
     * any performance change with and without this register
     * setting.
     */
#ifndef NDIS_HW
    writePlatformReg(pDev, MAC_CFG, INIT_CONFIG_STATUS |
                     (1 << MAC_CFG_PCI_MASTER_REQ_Q_THRESH_S));
#endif
#endif

    writePlatformReg(pDev, MAC_RSSI_THR, INIT_RSSI_THR);

    status = pRfHal->ar5513SetChannel(pDev, pChval);
    if (status != A_OK) {
        return status;
    }

    /* Write additional initialization registers for overwrite debug functionality */
    if (pDev->pInitRegs) {
        for (i = 0; i < MAX_REG_ADD_COUNT; i++) {
            if (pDev->pInitRegs[i].Offset == 0) {
                break;
            }
            writePlatformReg(pDev, pDev->pInitRegs[i].Offset, pDev->pInitRegs[i].Value);
        }
    }

    /*
     * Setup fast diversity.
     * Fast diversity can be enabled or disabled via regadd.txt.
     * Default is enabled.
     * For reference,
     *    Disable: reg        val
     *             0x00009860 0x00009d18 (if 11a / 11g, else no change)
     *             0x00009970 0x192bb514
     *             0x0000a208 0xd03e4648
     *
     *    Enable:  0x00009860 0x00009d10 (if 11a / 11g, else no change)
     *             0x00009970 0x192fb514
     *             0x0000a208 0xd03e6788
     */
    /* TODO: THIS IS LEGACY, NEEDS UPDATE FOR AR5513 */
    pDev->cachedDefAnt = (A_UINT8)readPlatformReg(pDev, MAC_DEF_ANTENNA);
    pDev->countOtherRxAnt = 0;
    if (A_REG_IS_BIT_SET(pDev, PHY_CCK_DETECT, BB_ENABLE_ANT_FAST_DIV)) {
        pDev->useFastDiversity = TRUE;
    } else {
        pDev->useFastDiversity = FALSE;
    }

    /* Setup pre PHY ENABLE EAR additions */
    if (earHere) {
        ar5513EarModify(pDev, EAR_LC_PHY_ENABLE, pChval, &modifier);
    }

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

    /* Activate the PHY (includes baseband activate and synthesizer on) */
    writePlatformReg(pDev, PHY_ACTIVE, PHY_ACTIVE_EN);

    /*
     * There is an issue if the AP starts the calibration before the base
     * band timeout completes, could result in the rx_clear false triggering.
     * Added an extra delay, BASE_ACTIVATE_DELAY, to ensure that this
     * condition will not happen.
     */
    udelay(synthDelay + BASE_ACTIVATE_DELAY);

    /*
     * WAR for bug 9031
     * The udelay method is not reliable with notebooks.
     * Need to check to see if the baseband is ready
     */
    testReg = A_REG_RD(pDev, PHY_TESTCTRL);

    A_REG_WR(pDev, PHY_TESTCTRL, 0x3800);   /* Selects the Tx hold */

    i = 0;

    while ((i++ < 20) &&
           (A_REG_RD(pDev, 0x9c24) & 0x10)) /* test if baseband not ready */
    {
        udelay(200);                        /* delay 200 us */
    }

    ASSERT(i < 20);                         /* ASSERT i limit is reached */

    A_REG_WR(pDev, PHY_TESTCTRL, testReg);

    /* Should the Ear disable offset cal? */
    if (!(earHere && ar5513EarModify(pDev, EAR_LC_RESET_OFFSET, pChval, &modifier))) {
        A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, CAL);
    }

    /* Should the Ear disable noise floor cal? */
    if (!(earHere && ar5513EarModify(pDev, EAR_LC_RESET_NF, pChval, &modifier))) {
        A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, NF);
    }

    /*
     * Start IQ calibration w/ 2^(INIT_IQCAL_LOG_COUNT_MAX+1) samples
     * -Cal is not necessary for CCK-only operation or scan-only channels
     * -Don't run cal if previous results exist (and no current request)
     */
    if ((pCfg->calibrationTime != 0) && !IS_CHAN_B(pChval->channelFlags) &&
        !(pDev->pHalInfo->iqCalState == IQ_CAL_DONE) &&
        !(earHere && ar5513EarModify(pDev, EAR_LC_RESET_IQ, pChval, &modifier)))
    {
        /* Start IQ calibration w/ 2^(INIT_IQCAL_LOG_COUNT_MAX+1) samples */
        A_REG_RMW_FIELD(pDev, PHY_TIMING_CTRL4, IQCAL_LOG_COUNT_MAX, INIT_IQCAL_LOG_COUNT_MAX);
        A_REG_SET_BIT(pDev, PHY_TIMING_CTRL4, DO_IQCAL);
        pDev->pHalInfo->iqCalState       = IQ_CAL_RUNNING;
    } else {
        pDev->pHalInfo->iqCalState       = IQ_CAL_INACTIVE;
    }

    /* Set 1:1 QCU to DCU mapping for all queues */
    for (i = 0; i < MAC_NUM_DCU; i++) {
        writePlatformReg(pDev, MAC_D0_QCUMASK + (i * sizeof(A_UINT32)), 1 << i);
    }

    /* Now set up the Interrupt Mask Register and save it for future use */
    pDev->MaskReg = INIT_INTERRUPT_MASK;

    /* TURBO_PRIME */
#ifdef BUILD_AP
    pDev->MaskReg |= MAC_IMR_TXDESC;
#endif

    writePlatformReg(pDev, MAC_IMR, pDev->MaskReg);

    /* Enable bus error interrupts */
    A_REG_SET_BIT3(pDev, MAC_IMR_S2, MCABT, SSERR, DPERR);

    /* Enable interrupts specific to AP */
    if (serviceType == WLAN_AP_SERVICE) {
        pDev->MaskReg |= MAC_IMR_MIB;
        A_REG_SET_BIT(pDev, MAC_IMR, MIB);
        /* undef the following code segment to enable receiving chirps */
#if 0
        pDev->MaskReg |= (MAC_IMR_MIB | MAC_IMR_RXCHIRP);
        A_REG_SET_BIT(pDev, MAC_IMR, MIB);
        A_REG_SET_BIT(pDev, MAC_IMR, RXCHIRP);
#endif
    } else {
        /* Enable DTIM interrupt in STA in primary and secondary masks */
        pDev->MaskReg |= MAC_IMR_BCNMISC;
        A_REG_SET_BIT(pDev, MAC_IMR, BCNMISC);
        A_REG_SET_BIT(pDev, MAC_IMR_S2, DTIM);
#ifdef AR5513_CCC
        /* Enable BCNTO (Beacon timeout) interrupt */
        A_REG_SET_BIT(pDev, MAC_IMR_S2, BCNTO);
#endif

    }

    if (ar5513GetRfKill(pDev)) {
        ar5513EnableRfKill(pDev);
    }

    /* Should the Ear disable offset cal? */
    if (!(earHere && ar5513EarModify(pDev, EAR_LC_RESET_OFFSET, pChval, &modifier))) {
        /* At the end of reset - poll for a completed offset calibration */
        for (i = 0; i < 100; i++) {
            if ( (A_REG_RD(pDev, PHY_AGC_CONTROL) & PHY_AGC_CONTROL_CAL) == 0 ) {
                break;
            }
            udelay(10);
        }
        if (i == 100) {
            uiPrintf("WARNING: ar5513Reset: offset calibration failed to complete in 1 ms - noisy environment?\n");
        }
    }

    /*
     * Set sleep clocking back to slow clocks if sleep is used and
     * a slow clock exists
     */
    ar5513SetupSleepRegisters(pDev);

    /*
     * Writing to MAC_BEACON will start timers. Hence it should be the last
     * register to be written. Do not reset tsf, do not enable beacons at this point,
     * but preserve other values like beaconInterval.
     */
    writePlatformReg(pDev, MAC_BEACON, (readPlatformReg(pDev, MAC_BEACON) &
                                       ~(MAC_BEACON_EN | MAC_BEACON_RESET_TSF)));

    /* Setup post reset EAR additions */
    if (earHere) {
        ar5513EarModify(pDev, EAR_LC_POST_RESET, pChval, &modifier);
    }

    /*
     * Enable XR in the chip if the hardware supports it and
     * not disabled by the user.
     */
    ret = A_OK;
    if ((pDev->staConfig.abolt & ABOLT_XR) != 0) {
        ret = ar5513SetXrMode(pDev, serviceType);
    }
    /*
     * If the station is associated with the AP in XR mode, we
     * infer this through the CHANNEL_XR flag, we put the chip
     * in polling mode.
     */
    if ((ret == A_OK) && (IS_CHAN_XR(pChval->channelFlags)) &&
        (serviceType == WLAN_STA_SERVICE)) {
        A_REG_SET_BIT(pDev, MAC_XRMODE, XR_WAIT_FOR_POLL);
    }

    /* Initialize all the TXOPs to max values */
    A_REG_WR(pDev, MAC_TXOPX,       0x000000FF);
    A_REG_WR(pDev, MAC_TXOP_0_3,    0xFFFFFFFF);
    A_REG_WR(pDev, MAC_TXOP_4_7,    0xFFFFFFFF);
    A_REG_WR(pDev, MAC_TXOP_8_11,   0xFFFFFFFF);
    A_REG_WR(pDev, MAC_TXOP_12_15,  0xFFFFFFFF);

    return A_OK;
}

/**************************************************************
 * ar5513SetXrMode
 *
 * Configuration needed to put the station in XR mode
 */
A_STATUS
ar5513SetXrMode(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType)
{
    WLAN_MACADDR bssIdMask;

#define MAC_CLKS(_usec) ((_usec) * 40)

    A_REG_RMW_FIELD(pDev, MAC_XRMODE, XR_POLL_TYPE, FRAME_DATA);
    A_REG_RMW_FIELD(pDev, MAC_XRMODE, XR_POLL_SUBTYPE, SUBT_NODATA_CFPOLL);

    /* Values used in simulation */
    writePlatformReg(pDev, PHY_SCRM_SEQ_XR, 0x13c889af);
    writePlatformReg(pDev, PHY_HEADER_DETECT_XR, 0x38490a20);
    writePlatformReg(pDev, PHY_CHIRP_DETECTED_XR, 0x0007bb6);

    /*
     * Acks to XR frames require longer timeouts - we end
     * up increasing timeout corresponding to all types of
     * frames on the AP; the sta will anyway do XR frames
     * only; given by HW guys to be 100usec
     */
    A_REG_RMW_FIELD(pDev, MAC_TIME_OUT, ACK, MAC_CLKS(100));

    if (serviceType == WLAN_AP_SERVICE) {
        /*
         * timeout values on the AP side corresponding to
         * ctsChirp->xrData and grpPoll->rtsChirp; the former is
         * from the detection of the rtsChirp to the detection
         * of XR data - so a rtsChirp slot, a ctsChirp slot,
         * the XR data detection time and a fudge; the latter
         * i.e. the group poll timeout value depends on the
         * XR mode aifs and cwMin/Max in slot time units plus
         * time for a chirp and some fudge; the AP also needs
         * the slot delay to figure out the timing between
         * rtsChirp->ctsChirp
         */
        A_REG_RMW_FIELD(pDev, MAC_XRTO, CHIRP_TO,
            MAC_CLKS(2 * XR_SLOT_DELAY + XR_DATA_DETECT_DELAY + 80));
        A_REG_RMW_FIELD(pDev, MAC_XRTO, POLL_TO,
            MAC_CLKS((XR_AIFS + LOG_TO_CW(XR_LOG_CWMIN)) * XR_SLOT_DELAY + XR_CHIRP_DUR + 16));
        A_REG_RMW_FIELD(pDev, MAC_XRDEL, SLOT_DELAY,
            MAC_CLKS(XR_SLOT_DELAY));

        /* modify the bssId mask to accept frames for the XR BSS */
        A_MACADDR_COPY(&broadcastMacAddr, &bssIdMask);
        setXrBssIdMask(&bssIdMask);
        A_REG_WR(pDev, MAC_BSS_ID_MASK0, cpu2le32(bssIdMask.st.word));
        A_REG_WR(pDev, MAC_BSS_ID_MASK1, cpu2le16(bssIdMask.st.half));

    } else {
        /*
         * used by the station to figure out timing between
         * end of rtsChirp->xrData; there is one ctsChirp
         * slot to be left in between - so two slots minus
         * the chirp duration and a fudge
         */
        A_REG_RMW_FIELD(pDev, MAC_XRDEL, CHIRP_DATA_DELAY,
            MAC_CLKS(2 * XR_SLOT_DELAY - (XR_CHIRP_DUR + 1)));

        A_REG_WR(pDev, MAC_D_GBL_IFS_SLOT, MAC_CLKS(XR_SLOT_DELAY));
    }

    return A_OK;
}

/**************************************************************
 * ar5513PhyDisable
 *
 * Places the PHY and Radio chips into reset.  A full reset
 * must be called to leave this state.  The PCI/MAC/PCU are
 * not placed into reset as we must receive interrupt to
 * re-enable the hardware.
 */
A_STATUS
ar5513PhyDisable(WLAN_DEV_INFO *pDev)
{
    pDev->pHalInfo->halInit = FALSE;
    pDev->pHalInfo->macReset = TRUE;
    return ar5513SetResetReg(pDev, MAC_RC_BB);
}

/**************************************************************
 * ar5513Disable
 *
 * Places all of hardware into reset
 */
A_STATUS
ar5513Disable(WLAN_DEV_INFO *pDev)
{
    A_STATUS status;

    status = ar5513SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    ar5513RestoreSleepRegisters(pDev);

    pDev->pHalInfo->halInit = FALSE;
    pDev->pHalInfo->macReset = TRUE;
    /* Reset the HW - PCI must be reset after the rest of the device has been reset. */
    status = ar5513SetResetReg(pDev, MAC_RC_MAC | MAC_RC_BB | MAC_RC_PCI);
    if (status != A_OK) {
        return status;
    }

    return status;
}

/**************************************************************
 * ar5513ChipReset
 *
 * Places the hardware into reset and then pulls it out of reset
 *
 * TODO: Only write the PLL if we're changing to or from CCK/dynamic mode
 *
 * Attach calls with pChval == NULL, as the coldreset should have
 * us in the correct mode and we cannot check the hwchannel flags.
 *
 * WARNING: The order of the PLL and mode registers must be correct.
 */
A_STATUS
ar5513ChipReset(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_BOOL   resetOnly    = TRUE;
    A_UINT32 rfMode       = 0;
    A_UINT32 turbo        = 0;
    A_UINT32 phyPLLCtlVal = 0, newPll = 0;
    A_UINT32 currentPllVal = 0;
    A_STATUS status;

    if (pChval) {
        WLAN_CFLAGS cflags = pChval->channelFlags;

        /* xxx: 'G' should set CCK Flag! */
        if (IS_CHAN_G(cflags)) {
            cflags |= CHANNEL_CCK;
        }

        if (IS_5112(pDev)) {
            rfMode       = PHY_MODE_AR5112;
            phyPLLCtlVal = IS_CHAN_CCK(cflags) ? PHY_PLL_CTL_44_5112 : PHY_PLL_CTL_40_5112;
        } else {
            rfMode       = PHY_MODE_AR5111;
            phyPLLCtlVal = IS_CHAN_CCK(cflags) ? PHY_PLL_CTL_44 : PHY_PLL_CTL_40;
        }

        if (IS_CHAN_OFDM(cflags) && IS_CHAN_CCK(cflags)) {
            rfMode |= PHY_MODE_DYNAMIC;
        } else if (IS_CHAN_OFDM(cflags)) {
            rfMode |= PHY_MODE_OFDM;
        } else {
            ASSERT(IS_CHAN_CCK(cflags));
            rfMode |= PHY_MODE_CCK;
        }

        if (IS_CHAN_5GHZ(cflags)) {
            rfMode |= PHY_MODE_RF5GHZ;
        } else {
            ASSERT(IS_CHAN_2GHZ(cflags));
            rfMode |= PHY_MODE_RF2GHZ;
        }

        /*
         * Enable XR in the chip if the hardware supports it and
         * not disabled by the user.
         */
        if ((pDev->staConfig.abolt & ABOLT_XR) != 0) {
            rfMode |= PHY_MODE_XR;
        }

        turbo = IS_CHAN_TURBO(cflags) ? (PHY_FC_TURBO_MODE | PHY_FC_TURBO_SHORT) : 0;

        resetOnly = FALSE;

        if (ar5513IsEarEngaged(pDev, pChval) && ar5513EarModify(pDev, EAR_LC_PLL, pChval, &newPll)) {
            phyPLLCtlVal = newPll;
        }
    } else {
        /* Cold starts need to initialize E2.0 chip 2.4 synth workaround logic */
        pDev->pHalInfo->done_synth_state_check_2_4 = FALSE;
        pDev->pHalInfo->synth_state_2_4            = 0;
        pDev->pHalInfo->synth_state_flag           = 0;
    }


#if defined(PCI_INTERFACE)
    pDev->pHalInfo->macReset = TRUE;

    /* Reset the HW - PCI must be reset after the rest of the device has been reset */
    status = ar5513SetResetReg(pDev, MAC_RC_MAC | MAC_RC_BB | MAC_RC_PCI);
    if (status != A_OK) {
        return status;
    }

    /* Bring out of sleep mode */
    status = ar5513SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    /* Clear warm reset register */
    status = ar5513SetResetReg(pDev, 0);

    pDev->pHalInfo->macReset = FALSE;
    pDev->pHalInfo->pciCfg = readPlatformReg(pDev, MAC_PCICFG);
    /*
     * Perform warm reset before the mode/PLL/turbo registers are changed
     * in order to deactivate the radio.  Mode changes with an active radio can
     * result in corrupted shifts to the radio device.
     */

    /*
     * PLL, Mode, and Turbo values must be written in the correct order to ensure:
     * -The PLL cannot be set to 44 unless the CCK or DYNAMIC mode bit is set
     * -Turbo cannot be set at the same time as CCK or DYNAMIC
     */
    if (!resetOnly) {
        #ifndef FALCON_EMUL
        currentPllVal = readPlatformReg(pDev, PHY_PLL_CTL);
        #else /* FALCON_EMUL */
        currentPllVal = phyPLLCtlVal;
        #endif /* FALCON_EMUL */
        /* xxx: 'G' should set the CCK Flag! */
        if (IS_CHAN_CCK(pChval->channelFlags) || IS_CHAN_G(pChval->channelFlags)) {
            writePlatformReg(pDev, PHY_TURBO, turbo);
            writePlatformReg(pDev, PHY_MODE, rfMode);

            if (currentPllVal != phyPLLCtlVal) {
                #ifndef FALCON_EMUL
                writePlatformReg(pDev, PHY_PLL_CTL, phyPLLCtlVal);
                #endif /* !FALCON_EMUL */
                /* Wait for the PLL to settle */
                udelay(PLL_SETTLE_DELAY);
            }
        } else {
            if (currentPllVal != phyPLLCtlVal) {
                #ifndef FALCON_EMUL
                writePlatformReg(pDev, PHY_PLL_CTL, phyPLLCtlVal);
                #endif /* !FALCON_EMUL */
                /* Wait for the PLL to settle */
                udelay(PLL_SETTLE_DELAY);
            }
            writePlatformReg(pDev, PHY_MODE, rfMode);
            writePlatformReg(pDev, PHY_TURBO, turbo);
        }
    }

#elif defined(AR5312) || defined(AR5513)
    /*
     * The 5312 Reset performs a cold reset and hence follows a different ordering than the
     * warm reset procedure above
     */
    /*
     * At this point, if resetOnly == FALSE then
     * phyTurboVal, phyPLLCtlVal, and phyModeVal all
     * contain the new values to write.
     *
     * We have to first write the new PLL value, then
     * RESET the hardware, then write the remaining
     * values.
     */

    status = ar5513SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    if (!resetOnly) {
        #ifndef FALCON_EMUL
        currentPllVal = readPlatformReg(pDev, PHY_PLL_CTL);
        #else /* FALCON_EMUL */
        currentPllVal = phyPLLCtlVal;
        #endif /* FALCON_EMUL */

        if (currentPllVal != phyPLLCtlVal) {
            #ifndef FALCON_EMUL
            writePlatformReg(pDev, PHY_PLL_CTL, phyPLLCtlVal);
            #endif /* !FALCON_EMUL */
            /* Wait for the PLL to settle */
            udelay(PLL_SETTLE_DELAY);
        }
    }

    /* Reset the HW - PCI must be reset after the rest of the device has been reset */
    status = ar5513SetResetReg(pDev, MAC_RC_MAC | MAC_RC_BB | MAC_RC_PCI);
    if (status != A_OK) {
        return status;
    }

    /* Bring out of sleep mode (AGAIN) */
    status = ar5513SetPowerMode(pDev, AWAKE, TRUE);
    if (status != A_OK) {
        return status;
    }

    /* Clear reset register */
    status = ar5513SetResetReg(pDev, 0);

    if (!resetOnly) {
        writePlatformReg(pDev, PHY_MODE, rfMode);
        writePlatformReg(pDev, PHY_TURBO, turbo);
    }
#endif /* defined(AR5312) || defined(AR5513) */

    return status;
}

#define IQ_CAL_TRIES 10

/**************************************************************************
 * ar5513IQCalibrationChain - Periodic calibration of PHY
 *
 * Recalibrate the lower PHY chips to account for temperature/environment
 * changes.
 */
LOCAL A_STATUS
ar5513IQCalibrationChain(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval, int chainIndex)
{
    A_STATUS status = A_OK;
    A_INT32  iqCorrMeas  = 0;
    A_UINT32 powerMeasQ  = 0;
    A_UINT32 powerMeasI  = 0;
    A_INT32  qCoff;
    A_INT32  qCoffDenom;
    A_INT32  iCoff;
    A_INT32  iCoffDenom;
    A_UINT32 i;
    A_UINT32 reg;
    A_UINT32 chainOffset = chainIndex ? (CHN_1_BASE - CHN_0_BASE) : 0;

    /* Workaround for misgated IQ Cal results */
    for (i = 0; i < IQ_CAL_TRIES; i++) {
        /* Read calibration results. */
        powerMeasI = A_REG_RD(pDev, PHY_IQCAL_RES_PWR_MEAS_I + chainOffset);
        powerMeasQ = A_REG_RD(pDev, PHY_IQCAL_RES_PWR_MEAS_Q + chainOffset);
        iqCorrMeas = A_REG_RD(pDev, PHY_IQCAL_RES_IQ_CORR_MEAS + chainOffset);

        if (powerMeasI && powerMeasQ) {
            break;
        }

#ifdef CALIBRATION_DEBUG
        uiPrintf("****************** MISGATED IQ CAL! *******************\n");
        uiPrintf("time       = %d, i = %d, \n", A_MS_TICKGET(), i);
        uiPrintf("powerMeasI = 0x%08x\n", powerMeasI);
        uiPrintf("powerMeasQ = 0x%08x\n", powerMeasQ);
#endif

        /* Signal caller to restart IQ calibration */
        status = A_HARDWARE;
    }

    /*
     * Prescale these values to remove 64-bit operation requirement at the loss
     * of a little precision.
     */
    iCoffDenom = (powerMeasI / 2 + powerMeasQ / 2) / 128;
    qCoffDenom = powerMeasQ / 128;

    /* Protect against divide-by-0 and loss of sign bits */
    if ((iCoffDenom != 0) && (qCoffDenom >= 2)) {
        /* IQCORR_Q_I_COFF is a signed 6 bit number */
        iCoff = (A_INT8)((-iqCorrMeas) / iCoffDenom);
        if (iCoff < -32) {
            iCoff = -32;
        } else if (iCoff > 31) {
            iCoff = 31;
        }

        /* IQCORR_Q_Q_COFF is a signed 5 bit number */
        qCoff = (powerMeasI / qCoffDenom) - 128;
        if (qCoff < -16) {
            qCoff = -16;
        } else if (qCoff > 15) {
            qCoff = 15;
        }

#ifdef CALIBRATION_DEBUG
        uiPrintf("****************** PERIODIC CALIBRATION *******************\n");
        uiPrintf("time       = %d\n", A_MS_TICKGET());
        uiPrintf("powerMeasI = 0x%08x\n", powerMeasI);
        uiPrintf("powerMeasQ = 0x%08x\n", powerMeasQ);
        uiPrintf("iqCorrMeas = 0x%08x\n", iqCorrMeas);
        uiPrintf("iCoff      = %d\n", iCoff);
        uiPrintf("qCoff      = %d\n", qCoff);
#endif

        /* Write values and enable correction */
        reg = PHY_TIMING_CTRL4 + chainOffset;
        A_REG_WR(pDev, reg, 
	             readPlatformReg(pDev, reg) |
                 PHY_TIMING_CTRL4_IQCORR_ENABLE |
                 A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_I_COFF, iCoff) |
                 A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_Q_COFF, qCoff));

        /* Save IQ values for any resets back to this channel */
        pChval->iqCalValid = TRUE;
        pChval->iCoff = (A_INT8)iCoff;
        pChval->qCoff = (A_INT8)qCoff;
    }

    return status;
}
/**************************************************************************
 * ar5513IQCalibration - Periodic calibration of PHY
 *
 * Recalibrate the lower PHY chips to account for temperature/environment
 * changes.
 */
LOCAL void
ar5513IQCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_UINT32 modifier;
    A_BOOL   earHere;
    WLAN_STA_CONFIG *pConfig = &pDev->staConfig;
    A_STATUS statusChain0 = A_OK, statusChain1 = A_OK;

    earHere = ar5513IsEarEngaged(pDev, pChval);

    /* IQ calibration in progress. Check to see if it has finished. */
    if ((pDev->pHalInfo->iqCalState == IQ_CAL_RUNNING) &&
        !(A_REG_RD(pDev, PHY_TIMING_CTRL4) & PHY_TIMING_CTRL4_DO_IQCAL) &&
        !pConfig->iqOverride)
    {
        /* IQ Calibration has finished.  Set to INACTIVE unless cal is valid */
        pDev->pHalInfo->iqCalState = IQ_CAL_INACTIVE;

        /* Process IQ calibration per chain */
        if (pDev->staConfig.rxChainCtrl == DUAL_CHAIN ||
            pDev->staConfig.rxChainCtrl == CHAIN_FIXED_A) 
        {
            statusChain0 = ar5513IQCalibrationChain(pDev, pChval, CHAIN_0);
        }

        if (pDev->staConfig.rxChainCtrl == DUAL_CHAIN ||
            pDev->staConfig.rxChainCtrl == CHAIN_FIXED_B) 
        {
            statusChain1 = ar5513IQCalibrationChain(pDev, pChval, CHAIN_1);
        }

        if ((statusChain0 == A_OK) && (statusChain1 == A_OK)) {
            pDev->pHalInfo->iqCalState = IQ_CAL_DONE;
         
        } else {
            /* Misgated IQ Cal => restart IQ Cal */
            A_REG_SET_BIT(pDev, PHY_TIMING_CTRL4, DO_IQCAL);
            ASSERT(!A_REG_IS_BIT_SET(pDev, PHY_TIMING_CTRL4, DO_IQCAL));

            pDev->pHalInfo->iqCalState = IQ_CAL_RUNNING;
        }
    } else if (!IS_CHAN_B(pChval->channelFlags) &&
               (pDev->pHalInfo->iqCalState == IQ_CAL_DONE) && (pChval->iqCalValid == FALSE) &&
               !(earHere && ar5513EarModify(pDev, EAR_LC_RESET_IQ, pChval, &modifier)) &&
               !pConfig->iqOverride)
    {
        /* Start IQ calibration if configured channel has changed */
        A_REG_RMW_FIELD(pDev, PHY_TIMING_CTRL4, IQCAL_LOG_COUNT_MAX,
                        pConfig->iqLogCountMax);
        A_REG_SET_BIT(pDev, PHY_TIMING_CTRL4, DO_IQCAL);
        pDev->pHalInfo->iqCalState = IQ_CAL_RUNNING;
    }
}

/**************************************************************************
 * ar5513PerCalibration - Periodic calibration of PHY
 *
 * Recalibrate the lower PHY chips to account for temperature/environment
 * changes.
 */
A_STATUS
ar5513PerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_STATUS status      = A_OK;
    A_UINT32 modifier;
    A_BOOL   earHere;

    earHere = ar5513IsEarEngaged(pDev, pChval);

    /* IQ Calibration (chain 0, chain 1) */
    ar5513IQCalibration(pDev, pChval);

    /* Check noise floor results */
    ar5513GetNf(pDev, pChval);
    if (pChval->channelFlags & CHANNEL_CW_INT) {
        return A_EBADCHANNEL;
    }

    if (!(earHere && ar5513EarModify(pDev, EAR_LC_PER_NF, pChval, &modifier))) {
        /* Run noise floor calibration */
        if (pDev->staConfig.nfCalHistEnable) {
            A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, ENABLE_NF);
            A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, NO_UPDATE_NF);
        }
        A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, NF);
    }

    /* Perform calibration for 5GHz channels and any OFDM on 5112 */
    if ( (IS_CHAN_5GHZ(pChval->channelFlags) ||
         (IS_5112(pDev) && IS_CHAN_OFDM(pChval->channelFlags))))
    {
        if (!(earHere && ar5513EarModify(pDev, EAR_LC_PER_FIXED_GAIN, pChval, &modifier))) {
            ar5513RequestRfgain(pDev);
        }
    }

    /* Setup post periodic calibration EAR additions */
    if (earHere) {
        ar5513EarModify(pDev, EAR_LC_POST_PER_CAL, pChval, &modifier);
    }

    return status;
}

#ifdef BUILD_AP
#define MAX_RESET_WAIT                         2000
#else
#define MAX_RESET_WAIT                         10
#endif

#define TX_QUEUEPEND_CHECK                     1
#define TX_ENABLE_CHECK                        2
#define RX_ENABLE_CHECK                        4
int dbgMacStop = 0;
/**************************************************************
 * ar5513MacStop
 *
 * Disables all active QCUs and ensure that the mac is in a
 * quiessence state.
 */
#ifndef BUILD_AP
ENTRY_FN(A_STATUS, ar5513MacStop,
         (WLAN_DEV_INFO *pDev), (pDev),
         ICEPT_MACSTOP)
#else
A_STATUS
ar5513MacStop(WLAN_DEV_INFO *pDev)
#endif
{
#if 0
    A_STATUS Status;
    A_UINT32 ulCount;
    A_UINT32 ultemp;
    A_UINT32 ulState;
    A_UINT32 ulQueue;
    /* Logic **************************************************
     1. Disable TX operation.
        a) set Q_TXD (0x880) clearing all 1's
        b) poll Q_TXE (0x840) till TXE is all cleared
        c) Poll Q_STS till pending frame count is zero
           - 0x0A00 + (Q No. << 2)
        d) clear Q_TXD (0x880) bits
        - Need to do a reg dump
          0x0800 + Q << 2
          0x840
          0x880
          0x8c0 + Q << 2
          0x900 + Q << 2
          0x940
     2. Disable RX operation.
        a) disable Rx in CR (0x0008)
        b) set dummy value in RxDP and see when it reads back
     3. issue reset to bb & mac
     **********************************************************/

    Status = A_ERROR;
#ifdef DEBUG
    if (dbgMacStop & 0x01) {
        ar5513RxRegDump(pDev);
        ar5513TxRegDump(pDev);
        ar5513DmaRegDump(pDev);
    }
#endif

    writePlatformReg(pDev, MAC_SREV, 0x0000DEAD);

    /* Disable Rx Operation ***********************************/
    A_REG_SET_BIT(pDev, MAC_CR, RXD);

    /* Disable TX Operation ***********************************/
    writePlatformReg(pDev, MAC_Q_TXD, 0x3ff); /* Disable all QCUs */

    /* Polling operation for completion of disable ************/
    ulState = 6;
    for (ulCount = 0; ulCount < 100; ulCount++) {
        if (ulState & 0x4) {
            if (!A_REG_IS_BIT_SET(pDev, MAC_CR, RXE)) {
                A_REG_WR(pDev, MAC_SREV, 0x0004DEAD);
                ulState &= ~0x4;
            }
        }

        if (ulState & 0x2) {
            if ((A_REG_RD(pDev, MAC_Q_TXE) & MAC_Q_TXE_M) == 0) {
                ulState &= ~ 0x2;
                ulState |= 0x1;
                A_REG_WR(pDev, MAC_SREV, 0x0002DEAD);
            }
        }

        if (ulState & 0x1) {
            ultemp = 0;
            for (ulQueue = 0; ulQueue < 10; ulQueue++) {
                ultemp += A_REG_RD(pDev, MAC_Q0_STS + (ulQueue << 2));
            }
            if (ultemp == 0) {
                ulState &= ~0x1;
                A_REG_WR(pDev, MAC_SREV, 0x0001DEAD);
            }
        }

        if (ulState == 0) {
            Status = A_OK;
            uiPrintf("MAC Stop Okay ulCount 0x%x **************\n",  ulCount);
#ifdef DEBUG
            if (dbgMacStop & 0x02) {
                ar5513RxRegDump(pDev);
                ar5513TxRegDump(pDev);
                ar5513DmaRegDump(pDev);
            }
#endif
            A_REG_WR(pDev, MAC_SREV, 0x0000BEEF);
                /* Disable TX Operation ***********************************/
            writePlatformReg(pDev, MAC_Q_TXD, 0); /* Clear TXD */

            break;
        }
        udelay(50);
    }

    if (Status != A_OK) {
        uiPrintf("***************************************************\n");
        uiPrintf("ar5513MacStop FAILED ******************************\n");
        uiPrintf("***************************************************\n");
    }

#else

    A_STATUS Status;
    A_UINT32 count;
    A_UINT32 pendFrameCount;
    A_UINT32 macStateFlag;
    A_UINT32 queue;

    Status = A_ERROR;

    /* Disable Rx Operation ***********************************/
    A_REG_SET_BIT(pDev, MAC_CR, RXD);

    /* Disable TX Operation ***********************************/
    A_REG_SET_BIT(pDev, MAC_Q_TXD, M);

    /* Polling operation for completion of disable ************/
    macStateFlag = TX_ENABLE_CHECK | RX_ENABLE_CHECK;

    for (count = 0; count < MAX_RESET_WAIT; count++) {
        if (macStateFlag & RX_ENABLE_CHECK) {
            if (!A_REG_IS_BIT_SET(pDev, MAC_CR, RXE)) {
                macStateFlag &= ~RX_ENABLE_CHECK;
            }
        }

        if (macStateFlag & TX_ENABLE_CHECK) {
            if ((A_REG_RD(pDev, MAC_Q_TXE) & MAC_Q_TXE_M) == 0) {
                macStateFlag &= ~TX_ENABLE_CHECK;
                macStateFlag |= TX_QUEUEPEND_CHECK;
            }
        }

        if (macStateFlag & TX_QUEUEPEND_CHECK) {
            pendFrameCount = 0;
            for (queue = 0; queue < MAC_NUM_DCU; queue++) {
                pendFrameCount += A_REG_RD(pDev, MAC_Q0_STS + (queue * 4));
            }
            if (pendFrameCount == 0) {
                macStateFlag &= ~TX_QUEUEPEND_CHECK;
            }
        }

        if (macStateFlag == 0) {
            Status = A_OK;
            break;
        }
        udelay(50);
    }

    if (Status != A_OK) {
        uiPrintf("ar5513MacStop:Failed to stop the MAC state 0x%x\n", macStateFlag);
    }

#endif

    return Status;
}

/**************************************************************
 * ar5513SetResetReg
 *
 * Writes the given reset bit mask into the reset register
 */
A_STATUS
ar5513SetResetReg(WLAN_DEV_INFO *pDev, A_UINT32 resetMask)
{
#if defined(PCI_INTERFACE)
    A_UINT32    mask = resetMask ? resetMask : ~0;
    A_UINT32    i;
    A_STATUS    status;
#ifndef BUILD_AP
    A_UINT32    reg32;
    A_UINT32    slot= pDev->pOSHandle->PciSlotNumber;
    NDIS_HANDLE handle = pDev->pOSHandle->NicAdapterHandle;
#endif

    if (resetMask & (MAC_RC_MAC | MAC_RC_PCI)) {
        /*
         * To ensure that the driver can reset the
         * MAC, wake up the chip
         */
        status = ar5513SetPowerMode(pDev, AWAKE, TRUE);

        if (status != A_OK) {
            return status;
        }

        /*
         * Disable interrupts
         */
        A_REG_WR(pDev, MAC_IER, MAC_IER_DISABLE);
#ifndef NDIS_HW  
        A_REG_RD(pDev, MAC_IER);
#endif

        if (ar5513MacStop(pDev) != A_OK) {
            /* need some delay before flush any pending MMR writes */
            udelay(15);
            A_REG_RD(pDev, MAC_RXDP);

            resetMask |= MAC_RC_PCI | MAC_RC_MAC | MAC_RC_BB;

#ifndef BUILD_AP
            /*
             * Flush the park address of the PCI controller
             * WAR 7428
             */
            for (i = 0; i < 32; i++) {
                NdisReadPciSlotInformation(handle, slot, PCI_STATUS_REGISTER, &reg32,  4);
            }
#endif
        } else {
            resetMask &= ~MAC_RC_PCI;

            /* need some delay before flush any pending MMR writes */
            udelay(15);
            A_REG_RD(pDev, MAC_RXDP);
        }
    }

    /* need some delay before flush any pending MMR writes */
    udelay(15);
    A_REG_RD(pDev, MAC_RXDP);

    A_REG_WR(pDev, MAC_RC, resetMask);

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
            if (ar5513SetPowerMode(pDev, AWAKE, TRUE) == A_OK) {
                A_REG_RD(pDev, MAC_ISR_RAC);
            }
            return A_OK;
        }
        udelay(50);
    }

    uiPrintf("ar5513SetResetReg: resetHW failed set %x != %lx\n", resetMask,
             readPlatformReg(pDev, MAC_RC));

    return A_HARDWARE;

#elif defined(AR5513)

    A_UINT32 resetBB, resetBits, regMask;
    A_UINT32 reg;

    resetBB   = RESET_WARM_WLAN0_BB;
    resetBits = RESET_WARM_WLAN0_MAC;
    regMask   = ~(RESET_WARM_WLAN0_MAC | RESET_WARM_WLAN0_BB);

    /*
    **  !!!!  HARDWARE WORK-AROUND FIX for Falcon Emulation and 1.0 !!!!
    **  Added by Gerald Stanton 01/16/2004
    **
    **  IMPORTANT:  The WLAN0_MAC Sleep Control registers must
    **  be cleared prior to writing the RESET bit for the WLAN MAC.
    **  The delays are important as well!!
    */

/*
**  FALCON 1.0 Work-Around for reset clock sync bug (#12019)
**  Switch AMBA Clock to REF clk when warm resetting then
**  switch back to normal divider thereafter.
*/
#if defined(FALCON1_0_WAR)
    sysRegWrite(AR5513_AMBACLK, 0x3);
#endif /* FALCON1_0_WAR */

#ifndef MAC_SCR
#define  MAC_SCR 0x4004
#endif

    /* Clear WMAC Sleep Control register in the PCI address space */
    sysRegWrite(AR5513_PCI + MAC_SCR, 0);

#ifndef MAC_PCI_CFG
#define MAC_PCI_CFG 0x4010
#endif 

    /* Wait for WMAC to AWAKEN - VERY IMPORTANT!!! */
    while (sysRegRead(AR5513_PCI + MAC_PCI_CFG) & 0x10000);

    /*
    **  END OF HARDWARE WORK-AROUND FIX for Falcon Emulation and 1.0
    */

    reg = sysRegRead(AR5513_RESET);             /* read before */

    if (resetMask) {
        /*
         * Reset the MAC and baseband.  This is a bit different than
         * the PCI version, but holding in reset causes problems.
         */
        reg &= regMask;

        if (resetMask & MAC_RC_BB)
            reg |= resetBB;

        if (resetMask & MAC_RC_MAC)
            reg |= resetBits;

        sysRegWrite(AR5513_RESET, reg);

#if defined(FALCON1_0_WAR)
        sysRegWrite(AR5513_AMBACLK, 0x1);
#endif /* FALCON1_0_WAR */

        udelay(100);
    }

#if defined(FALCON1_0_WAR)
    sysRegWrite(AR5513_AMBACLK, 0x3);
#endif /* FALCON1_0_WAR */

    /* Bring MAC and baseband out of reset */
    reg &= regMask;
    sysRegWrite(AR5513_RESET, reg);

#if defined(FALCON1_0_WAR)
    sysRegWrite(AR5513_AMBACLK, 0x1);
#endif /* FALCON1_0_WAR */

    udelay(100);

    return A_OK;
#endif
}

/**************************************************************
 * ar5513SetChannel5112
 *
 * Takes the MHz channel value and sets the Channel value
 *
 * ASSUMES: Writes enabled to analog bus
 */
/* TODO: cleanup constants */
A_STATUS
ar5513SetChannel5112(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_UINT32 channelSel  = 0;
    A_UINT32 lowChannel, highChannel;
    A_UINT32 bModeSynth  = 0;
    A_UINT32 aModeRefSel = 0;
    A_UINT32 reg32       = 0;
    A_UINT16 freq        = pChval->channel;

    lowChannel = halGetCapability(pDev, HAL_GET_LOW_CHAN_EDGE, pChval->channelFlags);
    highChannel = halGetCapability(pDev, HAL_GET_HIGH_CHAN_EDGE, pChval->channelFlags);
    if ((pChval->channel < lowChannel) || (pChval->channel > highChannel)) {
        uiPrintf("ar5513SetChannel: Invalid Channel Number %4d MHz\n", pChval->channel);
        return A_EINVAL;
    }

    if (freq < 4800) {
        if (((freq - 2192) % 5) == 0) {
            channelSel = ((freq - 672) * 2 - 3040)/10;
            bModeSynth = 0;
        } else if (((freq - 2224) % 5) == 0) {
            channelSel = ((freq - 704) * 2 - 3040) / 10;
            bModeSynth = 1;
        } else {
            uiPrintf("ar5513SetChannel5112: %d is an illegal 5112 channel\n", freq);
            return A_EINVAL;
        }

        channelSel = (channelSel << 2) & 0xff;
        channelSel = reverseBits(channelSel, 8);

        if (freq == 2484) {
            /* Enable channel spreading for channel 14 */
            A_REG_SET_BIT(pDev, PHY_CCK_TX_CTRL, JAPAN);
        } else {
            A_REG_CLR_BIT(pDev, PHY_CCK_TX_CTRL, JAPAN);
        }
    } else {
        if (((freq % 20) == 0) && (freq >= 5120)){
            channelSel = reverseBits(((freq - 4800) / 20 << 2), 8);
            aModeRefSel = reverseBits(3, 2);
        } else if ((freq % 10) == 0) {
            channelSel = reverseBits(((freq - 4800) / 10 << 1), 8);
            aModeRefSel = reverseBits(2, 2);
        } else if ((freq % 5) == 0) {
            channelSel = reverseBits((freq - 4800) / 5, 8);
            aModeRefSel = reverseBits(1, 2);
        } else {
            uiPrintf("ar5513SetChannel5112: %d is an illegal 5112 channel\n", freq);
            return A_EINVAL;
        }
    }

    reg32 = (channelSel << 4) | (aModeRefSel << 2) | (bModeSynth << 1)| (1 << 12) | 0x1;
    writePlatformReg(pDev, CHN_ALL_BASE + (39 << 2), (reg32 & 0xff));

    reg32 = (reg32 >> 8) & 0x7f;
    writePlatformReg(pDev, CHN_ALL_BASE + (54 << 2), reg32);

    pDev->staConfig.phwChannel = pChval;
    return A_OK;
}

/* TODO: temp debug */
void logNfCal_Add(WLAN_DEV_INFO *pDev, int flag);

/**************************************************************************
 * Read the NF and check it against the noise floor threshhold
 *
 * Returns: The signed NF value.
 */
void
ar5513GetNf(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_INT16       nf = 0, nfThresh, temp, *curr, tmpNfBuffer[NF_CAL_HIST_MAX];
    HEADER_WMODE  eepMode = 0;
    NFCAL_HIST    *h;
    A_INT32        val, base[2] = {(PHY_BASE+(25<<2)), (CHN_1_BASE+(25<<2))};
    int            i, nchains, j, startIndex;

    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
    case CHANNEL_T:
    case CHANNEL_XR_A:
    case CHANNEL_XR_T:
        eepMode = headerInfo11A;
        break;
    case CHANNEL_B:
        eepMode = headerInfo11B;
        break;
    case CHANNEL_G:
    case CHANNEL_108G:
    case CHANNEL_XR_G:
        eepMode = headerInfo11G;
        break;
    default:
        ASSERT(0);
        break;
    }

    if (readPlatformReg(pDev, PHY_AGC_CONTROL) & PHY_AGC_CONTROL_NF) {
        /* TODO: temp debug */
        logNfCal_Add(pDev, 1);
#ifdef DEBUG
        uiPrintf("NF failed to complete in calibration window\n");
#endif
    } else {
        /* TODO: temp debug */
        logNfCal_Add(pDev, 0);
        /* Finished NF cal, check against threshold */
        nfThresh = pDev->pHalInfo->pEepData->pEepHeader->noiseFloorThresh[eepMode];
        if (pDev->staConfig.rxChainCtrl == DUAL_CHAIN) {
            startIndex = 0;
            nchains    = 2;
        } else if (pDev->staConfig.rxChainCtrl == CHAIN_FIXED_A) {
            startIndex = 0;
            nchains    = 1;
        } else {
            startIndex = 1;
            nchains    = 1;
        }

        /*
         * Run NF cal algo according to chain config selected
         */

        for (j = startIndex; ;) {
            nf = (A_INT16)(readPlatformReg(pDev, base[j]) >> 19) & 0x1FF;

            if (nf & 0x100) {
                nf = 0 - ((nf ^ 0x1ff) + 1);
            }
            if (nf > nfThresh) {
                uiPrintf("NF failed detected %d higher than thresh %d\n", 
                          nf, nfThresh);
                pChval->channelFlags |= CHANNEL_CW_INT;
            }

            if (!pDev->staConfig.nfCalHistEnable) {
                goto done;
            }
            
            h = &pChval->nfCalHist[j];
            h->nfCalBuffer[h->currIndex] = nf;
            h->currIndex ++;

            if(!h->isBufferFull) {
                if (h->currIndex == pDev->staConfig.nfCalHistSize) {
                    h->isBufferFull = 1;
                }
            }
            uiPrintf("nf value measured chain %d: %d\n", j, nf);

            if (h->isBufferFull) {

                A_BCOPY(h->nfCalBuffer, tmpNfBuffer, sizeof(tmpNfBuffer));

                for(i = 1; i < pDev->staConfig.nfCalHistSize; i++) {
                    for (curr = &tmpNfBuffer[i]; (curr > tmpNfBuffer) && 
                         (*(curr - 1) > *curr); curr --)
                    {
                        temp = *curr;
                        *curr = *(curr - 1);
                        *(curr - 1) = temp;
                    }
                }
                switch(pDev->staConfig.nfCalHistSize) {
                    case 3:
                        nf = tmpNfBuffer[1];
                        break;
                    case 5:
                        nf = tmpNfBuffer[2];
                        break;
                }
                if (h->currIndex == pDev->staConfig.nfCalHistSize) {
                    h->currIndex = 0;
                }
            }

            val = readPlatformReg(pDev, base[j]);
            val &= 0xFFFFFE00;
            val |= (((A_INT32)nf & 0x1FF) << 1);
            writePlatformReg(pDev, base[j], val);
        }

        A_REG_CLR_BIT(pDev, PHY_AGC_CONTROL, ENABLE_NF);
        A_REG_CLR_BIT(pDev, PHY_AGC_CONTROL, NO_UPDATE_NF);
        A_REG_SET_BIT(pDev, PHY_AGC_CONTROL, NF);
        while (readPlatformReg(pDev, PHY_AGC_CONTROL) & PHY_AGC_CONTROL_NF) {
            udelay(10);
        }
        /*
         * Now load a high maxCCAPower value again so that we're not capped
         * by the median we just loaded
         */
        for (j = startIndex; ; ){
            uiPrintf("CCA1 chn %d: 0x%lx\n", j, readPlatformReg(pDev, base[j]));
            val &= 0xFFFFFE00;
            val |= (((A_INT32)(-50) & 0x1FF) << 1);
            writePlatformReg(pDev, base[j], val);
            uiPrintf("CCA2 chn %d: 0x%lx\n", j, readPlatformReg(pDev, base[j]));

            if (++j >= nchains) {
                break;
            }
        }
    }
done:
    pChval->rawNoiseFloor = nf;
}

/**************************************************************
 * ar5513SetRfRegs5112
 *
 * Reads EEPROM header info from device structure and programs
 * all rf registers
 *
 * REQUIRES: Access to the analog rf device
 */
static void
ar5513SetRfRegs5112(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo,
                    CHAN_VALUES *pChval, A_UINT16 modesIndex, A_UINT16 *rfXpdGain)
{
    int                   arrayEntries, i, regWrites = 0;
    A_UINT16              rfXpdSel, gainI;
    A_UINT16              ob5GHz = 0, db5GHz = 0;
    A_UINT16              ob2GHz = 0, db2GHz = 0;
    A_BOOL                arrayMode = 0;
    A_UINT32              modifier;
    AR5513_RF_BANKS_5112  *pRfBanks = pDev->pHalInfo->pAnalogBanks;
    GAIN_VALUES           *pGainValues = pDev->pHalInfo->pGainValues;
    A_UINT32              writeBlockSize;

    ASSERT(pRfBanks);

    /* Setup rf parameters */
    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
    case CHANNEL_T:
    case CHANNEL_XR_A:
    case CHANNEL_XR_T:
        if ((pChval->channel > 4000) && (pChval->channel < 5260)) {
            ob5GHz = pHeaderInfo->ob1;
            db5GHz = pHeaderInfo->db1;
        } else if ((pChval->channel >= 5260) && (pChval->channel < 5500)) {
            ob5GHz = pHeaderInfo->ob2;
            db5GHz = pHeaderInfo->db2;
        } else if ((pChval->channel >= 5500) && (pChval->channel < 5725)) {
            ob5GHz = pHeaderInfo->ob3;
            db5GHz = pHeaderInfo->db3;
        } else if (pChval->channel >= 5725) {
            ob5GHz = pHeaderInfo->ob4;
            db5GHz = pHeaderInfo->db4;
        }
        arrayMode = headerInfo11A;
        break;

    case CHANNEL_B:
        ob2GHz = pHeaderInfo->ob2GHz[0];
        db2GHz = pHeaderInfo->db2GHz[0];
        arrayMode = headerInfo11B;
        break;

    case CHANNEL_G:
    case CHANNEL_108G:
    case CHANNEL_XR_G:
        ob2GHz = pHeaderInfo->ob2GHz[1];
        db2GHz = pHeaderInfo->db2GHz[1];
        arrayMode = headerInfo11G;
        break;

    default:
        ASSERT(0);
        break;
    }

    rfXpdSel  = pHeaderInfo->xpd[arrayMode];
    gainI     = pHeaderInfo->gainI[arrayMode];

    /* Setup Bank 1 Write */
    arrayEntries = sizeof(ar5212Bank1) / sizeof(*ar5212Bank1);
    for (i = 0; i < arrayEntries; i++) {
        pRfBanks->Bank1Data[i] = ar5212Bank1[i][1];
    }

    /* Setup Bank 2 Write */
    arrayEntries = sizeof(ar5212Bank2) / sizeof(*ar5212Bank2);
    for (i = 0; i < arrayEntries; i++) {
        pRfBanks->Bank2Data[i] = ar5212Bank2[i][modesIndex];
    }

    /* Setup Bank 3 Write */
    arrayEntries = sizeof(ar5212Bank3) / sizeof(*ar5212Bank3);
    for (i = 0; i < arrayEntries; i++) {
        pRfBanks->Bank3Data[i] = ar5212Bank3[i][modesIndex];
    }

    /* Setup Bank 6 Write */
    arrayEntries = sizeof(ar5212Bank6) / sizeof(*ar5212Bank6);
    for (i = 0; i < arrayEntries; i++) {
        pRfBanks->Bank6Data[i] = ar5212Bank6[i][modesIndex];
    }
    ar5513ModifyRfBuffer(pRfBanks->Bank6Data, rfXpdSel,     1, 302, 0);

    ar5513ModifyRfBuffer(pRfBanks->Bank6Data, rfXpdGain[0], 2, 270, 0);
    ar5513ModifyRfBuffer(pRfBanks->Bank6Data, rfXpdGain[1], 2, 257, 0);

    if (IS_CHAN_OFDM(pChval->channelFlags)) {
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_138], 1, 168, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_137], 1, 169, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_136], 1, 170, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_132], 1, 174, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_131], 1, 175, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, pGainValues->currStep->paramVal[GP_PWD_130], 1, 176, 3);
    }

    /* Only the 5 or 2 GHz OB/DB need to be set for a mode */
    if (IS_CHAN_2GHZ(pChval->channelFlags)) {
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, ob2GHz, 3, 287, 0);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, db2GHz, 3, 290, 0);
    } else {
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, ob5GHz, 3, 279, 0);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, db5GHz, 3, 282, 0);
    }

    /* Decrease Power Consumption for 5312/5213 and up */
    if (pDev->phyRev >= PHY_CHIP_ID_REV_2) {
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, 1, 1, 281, 1);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, 1, 2, 1, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, 1, 2, 3, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, 1, 1, 139, 3);
        ar5513ModifyRfBuffer(pRfBanks->Bank6Data, 1, 1, 140, 3);
    }

    /* Setup Bank 7 Setup */
    arrayEntries = sizeof(ar5212Bank7) / sizeof(*ar5212Bank7);
    for (i = 0; i < arrayEntries; i++) {
        pRfBanks->Bank7Data[i] = ar5212Bank7[i][modesIndex];
    }
    if (IS_CHAN_OFDM(pChval->channelFlags)) {
        ar5513ModifyRfBuffer(pRfBanks->Bank7Data, pGainValues->currStep->paramVal[GP_MIXGAIN_OVR], 2, 37, 0);
    }

    ar5513ModifyRfBuffer(pRfBanks->Bank7Data, gainI, 6, 14, 0);

    /* Analog registers are setup - EAR can modify */
    if (ar5513IsEarEngaged(pDev, pChval)) {
        ar5513EarModify(pDev, EAR_LC_RF_WRITE, pChval, &modifier);
    }

#ifdef BUILD_AP
    writeBlockSize = MAX_WRITEBLOCKSIZE;
#else
    writeBlockSize = pDev->staConfig.writeBlockSize;
#endif

    /* Write Analog registers */
    REG_WRITE_RF_ARRAY(ar5212Bank1, pRfBanks->Bank1Data, regWrites, writeBlockSize);
    REG_WRITE_RF_ARRAY(ar5212Bank2, pRfBanks->Bank2Data, regWrites, writeBlockSize);
    REG_WRITE_RF_ARRAY(ar5212Bank3, pRfBanks->Bank3Data, regWrites, writeBlockSize);
    REG_WRITE_RF_ARRAY(ar5212Bank6, pRfBanks->Bank6Data, regWrites, writeBlockSize);
    REG_WRITE_RF_ARRAY(ar5212Bank7, pRfBanks->Bank7Data, regWrites, writeBlockSize);

    /* Now that we have reprogrammed rfgain value, clear the flag. */
    pDev->pHalInfo->rfgainState = RFGAIN_INACTIVE;
}

/**************************************************************
 * ar5513ModifyRfBuffer
 *
 * Perform analog "swizzling" of parameters into their location
 */
static void
ar5513ModifyRfBuffer(A_UINT32 *rfBuf, A_UINT32 reg32, A_UINT32 numBits,
                     A_UINT32 firstBit, A_UINT32 column)
{
    A_UINT32 tmp32, mask, arrayEntry, lastBit;
    A_INT32  bitPosition, bitsLeft;

    ASSERT(column <= 3);
    ASSERT(numBits <= 32);
    ASSERT(firstBit + numBits <= MAX_ANALOG_START);

    tmp32 = reverseBits(reg32, numBits);
    arrayEntry = (firstBit - 1) / 8;
    bitPosition = (firstBit - 1) % 8;
    bitsLeft = numBits;
    while (bitsLeft > 0) {
        lastBit = (bitPosition + bitsLeft > 8) ? (8) : (bitPosition + bitsLeft);
        mask = (((1 << lastBit) - 1) ^ ((1 << bitPosition) - 1)) << (column * 8);
        rfBuf[arrayEntry] &= ~mask;
        rfBuf[arrayEntry] |= ((tmp32 << bitPosition) << (column * 8)) & mask;
        bitsLeft -= (8 - bitPosition);
        tmp32 = tmp32 >> (8 - bitPosition);
        bitPosition = 0;
        arrayEntry++;
    }
}

/**************************************************************
 * ar5513GetRfField
 *
 * Find analog bits of given parameter data and return a reversed value
 */
A_UINT32
ar5513GetRfField(A_UINT32 *rfBuf, A_UINT32 numBits, A_UINT32 firstBit, A_UINT32 column)
{
    A_UINT32 reg32 = 0, mask, arrayEntry, lastBit;
    A_INT32  bitPosition, bitsLeft, bitsShifted;

    ASSERT(column <= 3);
    ASSERT(numBits <= 32);
    ASSERT(firstBit + numBits <= MAX_ANALOG_START);

    arrayEntry = (firstBit - 1) / 8;
    bitPosition = (firstBit - 1) % 8;
    bitsLeft = numBits;
    bitsShifted = 0;
    while (bitsLeft > 0) {
        lastBit = (bitPosition + bitsLeft > 8) ? (8) : (bitPosition + bitsLeft);
        mask = (((1 << lastBit) - 1) ^ ((1 << bitPosition) - 1)) << (column * 8);
        reg32 |= (((rfBuf[arrayEntry] & mask) >> (column * 8)) >> bitPosition) << bitsShifted;
        bitsShifted += lastBit - bitPosition;
        bitsLeft -= (8 - bitPosition);
        bitPosition = 0;
        arrayEntry++;
    }
    reg32 = reverseBits(reg32, numBits);
    return reg32;
}

/**************************************************************
 * ar5513AllocateRfBanks
 *
 * Allocate memory for analog bank scratch buffers
 * Scratch Buffer will be reinitialized every reset so no need to zero now
 */
A_BOOL
ar5513AllocateRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo)
{
    ASSERT(pHalInfo->pAnalogBanks == NULL);
    pHalInfo->pAnalogBanks = A_DRIVER_MALLOC(sizeof(AR5513_RF_BANKS_5112));
    if (pHalInfo->pAnalogBanks == NULL) {
        return FALSE;
    }
    return TRUE;
}

/**************************************************************
 * ar5513FreeRfBanks
 *
 * Free memory for analog bank scratch buffers
 */
void
ar5513FreeRfBanks(WLAN_DEV_INFO *pDev, HAL_INFO *pHalInfo)
{
    if (pHalInfo->pAnalogBanks) {
        A_DRIVER_FREE(pHalInfo->pAnalogBanks, sizeof(AR5513_RF_BANKS_5112));
        pHalInfo->pAnalogBanks = NULL;
    }
}

#define NO_FALSE_DETECT_BACKOFF   2
#define CB22_FALSE_DETECT_BACKOFF 6
/**************************************************************
 * ar5513SetBoardValues
 *
 * Reads EEPROM header info and programs the device for correct operation
 * given the channel value
 */
void
ar5513SetBoardValues(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, CHAN_VALUES *pChval)
{
    HEADER_WMODE eepMode = 0;
    GAIN_VALUES  *pGainValues = pDev->pHalInfo->pGainValues;
    int          falseDectectBackoff;
    int          eepVersion = pDev->pHalInfo->pEepData->version;
    int          is2GHz = IS_CHAN_2GHZ(pChval->channelFlags);
    int          iCoff, qCoff;
    A_UINT32     regAddr;
    WLAN_STA_CONFIG *pConfig = &pDev->staConfig;

    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
    case CHANNEL_T:
    case CHANNEL_XR_A:
    case CHANNEL_XR_T:
        eepMode = headerInfo11A;
        if (!IS_5112(pDev)) {
            /* TX Clip should not be modified for 5112 */
            A_REG_RMW_FIELD(pDev, PHY_FRAME_CTL, TX_CLIP, pGainValues->currStep->paramVal[GP_TXCLIP]);
        }
        break;
    case CHANNEL_B:
        eepMode = headerInfo11B;
        break;
    case CHANNEL_G:
    case CHANNEL_108G:
    case CHANNEL_XR_G:
        eepMode = headerInfo11G;
        break;
    default:
        ASSERT(0);
        break;
    }

    /* Set the antenna register */
    writePlatformReg(pDev, CHN_0_BASE + (68 << 2),
        (readPlatformReg(pDev, CHN_0_BASE + (68 << 2)) & 0xFFFFFC06) |
        (pHeaderInfo->antennaControl[0][eepMode] << 4) | 0x1);

    writePlatformReg(pDev, CHN_1_BASE + (68 << 2),
        (readPlatformReg(pDev, CHN_1_BASE + (68 << 2)) & 0xFFFFFC06) |
        (pHeaderInfo->antennaControl[0][eepMode] << 4) | 0x1);

    ar5513SetAntennaSwitch(pDev, pDev->staConfig.diversityControl, pChval);

    /* Set the Noise Floor Thresh on ar5211 devices */
    writePlatformReg(pDev, PHY_BASE + (90 << 2), (pHeaderInfo->noiseFloorThresh[eepMode] & 0x1FF) |
                     (1 << 9));

    writePlatformReg(pDev, PHY_BASE + (17 << 2),
        (readPlatformReg(pDev, PHY_BASE + (17 << 2)) & 0xFFFFC07F) |
        ((pHeaderInfo->switchSettling[eepMode] << 7) & 0x3F80));
    writePlatformReg(pDev, PHY_BASE + (18 << 2),
        (readPlatformReg(pDev, PHY_BASE + (18 << 2)) & 0xFFFC0FFF) |
        ((pHeaderInfo->txrxAtten[eepMode] << 12) & 0x3F000));
    writePlatformReg(pDev, PHY_BASE + (20 << 2),
        (readPlatformReg(pDev, PHY_BASE + (20 << 2)) & 0xFFFF0000) |
        ((pHeaderInfo->pgaDesiredSize[eepMode] << 8) & 0xFF00) |
        (pHeaderInfo->adcDesiredSize[eepMode] & 0x00FF));
    writePlatformReg(pDev, PHY_BASE + (13 << 2),
        (pHeaderInfo->txEndToXPAOff[eepMode] << 24) |
        (pHeaderInfo->txEndToXPAOff[eepMode] << 16) |
        (pHeaderInfo->txFrameToXPAOn[eepMode] << 8) |
        pHeaderInfo->txFrameToXPAOn[eepMode]);
    writePlatformReg(pDev, PHY_BASE + (10 << 2),
        (readPlatformReg(pDev, PHY_BASE + (10 << 2)) & 0xFFFF00FF) |
        (pHeaderInfo->txEndToXLNAOn[eepMode] << 8));
    writePlatformReg(pDev, PHY_BASE + (25 << 2),
        (readPlatformReg(pDev, PHY_BASE + (25 << 2)) & 0xFFF80FFF) |
        ((pHeaderInfo->thresh62[eepMode] << 12) & 0x7F000));

    /*
     * False detect backoff - suspected 32 MHz spur causes false detects in OFDM,
     * causing Tx Hangs.  Decrease weak signal sensitivity for this card.
     */
    falseDectectBackoff = NO_FALSE_DETECT_BACKOFF;
    if (eepVersion < EEPROM_VER3_3) {
        if (pDev->pciInfo.SubVendorDeviceID == 0x1022 &&
            IS_CHAN_OFDM(pChval->channelFlags))
        {
            falseDectectBackoff += CB22_FALSE_DETECT_BACKOFF;
        }
    } else {
        if (IS_SPUR_CHAN(pChval->channel)) {
            falseDectectBackoff += pHeaderInfo->falseDetectBackoff[eepMode];
        }
    }
    writePlatformReg(pDev, 0x9924, (readPlatformReg(pDev, 0x9924) & 0xFFFFFF01) |
                     ((falseDectectBackoff << 1) & 0xF7));

    /* Write previous IQ results or use initial values */
    if (pConfig->iqOverride) {
        iCoff = pConfig->iCoff;
        qCoff = pConfig->qCoff;
    } else if (pChval->iqCalValid) {
        iCoff = pChval->iCoff;
        qCoff = pChval->qCoff;
    } else {
        iCoff = pHeaderInfo->iqCalI[is2GHz];
        qCoff = pHeaderInfo->iqCalQ[is2GHz];
    }

#ifdef CALIBRATION_DEBUG
    uiPrintf("************************* INITIAL CALIBRATION **********************\n");
    uiPrintf("iCoff      = %d\n", iCoff);
    uiPrintf("qCoff      = %d\n", qCoff);
#endif

    

#ifdef CALIBRATION_DEBUG
    uiPrintf("****************** RESET CALIBRATION *******************\n");
    uiPrintf("time       = %d\n", A_MS_TICKGET());
    uiPrintf("iCoff      = %d\n", iCoff);
    uiPrintf("qCoff      = %d\n", qCoff);
#endif

    /* Write values and enable correction */

    /* Chain 0 */
    regAddr = CHN_0_BASE + (PHY_TIMING_CTRL4 - PHY_BASE);
    A_REG_WR(pDev, regAddr, 
	         readPlatformReg(pDev, regAddr) |
             PHY_TIMING_CTRL4_IQCORR_ENABLE |
             A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_I_COFF, iCoff) |
             A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_Q_COFF, qCoff));

    /* Chain 1 */
    regAddr = CHN_1_BASE + (PHY_TIMING_CTRL4 - PHY_BASE);
    A_REG_WR(pDev, regAddr, 
	         readPlatformReg(pDev, regAddr) |
             PHY_TIMING_CTRL4_IQCORR_ENABLE |
             A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_I_COFF, iCoff) |
             A_FIELD_VALUE(PHY_TIMING_CTRL4, IQCORR_Q_Q_COFF, qCoff));

    if ((eepVersion >= EEPROM_VER4_1) && !IS_CHAN_108G(pChval->channelFlags)) {
        A_REG_RMW_FIELD(pDev, PHY_GAIN_2GHZ, RXTX_MARGIN, pHeaderInfo->rxtxMargin[eepMode]);
    }
}

#define COEF_SCALE_S 24
/**************************************************************
 * ar5513SetDeltaSlope
 *
 * Delta slope coefficient computation.
 * Required for OFDM operation.
 */
void
ar5513SetDeltaSlope(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    unsigned long coef_scaled, coef_exp, coef_man, ds_coef_exp, ds_coef_man;
    unsigned long clockMhz;

    if (pChval->channelFlags & CHANNEL_TURBO) {
        clockMhz = 80;
    } else {
        clockMhz = 40;
    }

    /*
     * ALGO -> coef = 1e8/fcarrier*fclock/40;
     * scaled coef to provide precision for this floating calculation
     */
    coef_scaled = ((5 * (clockMhz << COEF_SCALE_S)) / 2) / pChval->channel;

    /*
     * ALGO -> coef_exp = 14-floor(log2(coef));
     * floor(log2(x)) is the highest set bit position
     */
    for (coef_exp = 31; coef_exp > 0; coef_exp--) {
        if ((coef_scaled >> coef_exp) & 0x1) {
            break;
        }
    }
    /* A coef_exp of 0 is a legal bit position but an unexpected coef_exp */
    ASSERT(coef_exp);
    coef_exp = 14 - (coef_exp - COEF_SCALE_S);

    /*
     * ALGO -> coef_man = floor(coef* 2^coef_exp+0.5);
     * The coefficient is already shifted up for scaling
     */
    coef_man = coef_scaled + (1 << (COEF_SCALE_S - coef_exp - 1));
    ds_coef_man = coef_man >> (COEF_SCALE_S - coef_exp);
    ds_coef_exp = coef_exp - 16;

    A_REG_RMW_FIELD(pDev, PHY_TIMING3, DSC_MAN, ds_coef_man);
    A_REG_RMW_FIELD(pDev, PHY_TIMING3, DSC_EXP, ds_coef_exp);

    //uiPrintf("Delta slope coefficient for %4d MHz is m0x%lx, e%ld - RegVal is now 0x%08X\n", pChval->channel,
    //    ds_coef_man, ds_coef_exp, A_REG_RD(pDev, PHY_TIMING3));
}

int TxPowerDebugLevel = TXPOWER_DEBUG_OFF; 

void
ar5513PrintPowerPerRate(A_UINT16 *pRatesPower)
{
    const A_UCHAR *rateString[] = {" 6mb OFDM", " 9mb OFDM", "12mb OFDM", "18mb OFDM",
                                   "24mb OFDM", "36mb OFDM", "48mb OFDM", "54mb OFDM",
                                   "1L   CCK ", "2L   CCK ", "2S   CCK ", "5.5L CCK ",
                                   "5.5S CCK ", "11L  CCK ", "11S  CCK ", "XR       "
    };
    int i, halfRates = NUM_RATES / 2;

    for (i = 0; i < halfRates; i++) {
        uiPrintf(" %s %3d.%1d dBm | %s %3d.%1d dBm\n",
            rateString[i], pRatesPower[i] / 2, (pRatesPower[i] % 2) * 5,
            rateString[i + halfRates], pRatesPower[i + halfRates] / 2, (pRatesPower[i + halfRates] % 2) * 5);
    }
}

/**************************************************************
 * ar5513SetTransmitPower
 *
 * Sets the transmit power in the baseband for the given
 * operating channel and mode.
 */
A_STATUS
ar5513SetTransmitPower(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval,
                       A_UINT16 *rfXpdGain, A_INT16 powerLimit, 
                       A_BOOL calledDuringReset, A_UINT8 chnIdx)
{
    static const A_UINT16 tpcScaleReductionTable[5] = {0, 3, 6, 9, MAX_RATE_POWER};

    A_UINT16        pcdacTable[PWR_TABLE_SIZE];
    A_UINT16        ratesArray[NUM_RATES];
    A_UINT16        regOffset;
    A_UINT32        reg32;
    A_STATUS        status = A_OK;
    struct eepMap   *pData;
    int             i;
    const A_BOOL    paPreDEnable = 0;
    const A_UINT16  mask = 0x3f;
    A_INT16         minPower, maxPower;
    A_UINT16        powerIndexOffset;
    A_INT16         cckOfdmPwrDelta = 0;
    A_INT16         tpcInDb = 0;
    A_BOOL          pcdacTableModified = FALSE;
    A_UINT16        chain_base = chnIdx ? CHN_1_BASE : CHN_0_BASE;
    A_INT16         twiceBandEdgePower;
    A_INT16         twiceBandEirp;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    A_MEM_ZERO(pcdacTable, sizeof(A_UINT16) * PWR_TABLE_SIZE);
    A_MEM_ZERO(ratesArray, sizeof(A_UINT16) * NUM_RATES);

    pData = pDev->pHalInfo->pEepData;

    /* WAR BUG #XXXXX
     * Return if requested chain index not enabled/configured
     */
    if (pDev->staConfig.txChainCtrl == CHAIN_FIXED_A && chnIdx != CHAIN_0) {
        return status;
    }
    /* end WAR */

    /* Calculate transmit power control reductions or limits */
    if (IS_CHAN_5GHZ(pChval->channelFlags)) {
        powerLimit = A_MIN(pDev->staConfig.tpcHalfDbm5, powerLimit);
    } else {
        powerLimit = A_MIN(pDev->staConfig.tpcHalfDbm2, powerLimit);
    }
    powerLimit = A_MIN(MAX_RATE_POWER, powerLimit);
    if ((powerLimit >= MAX_RATE_POWER) || (powerLimit == 0)) {
        tpcInDb = tpcScaleReductionTable[pDev->staConfig.tpScale];
    }

    ar5513SetRateTable(pDev, ratesArray, pData->chain[chnIdx].pTrgtPowerInfo, pChval,
                       tpcInDb, powerLimit, pData, &minPower, &maxPower, &twiceBandEdgePower, chnIdx);

    if (TxPowerDebugLevel >= TXPOWER_DEBUG_BASIC) {
        uiPrintf("*** Rates per Power (chain_base = 0x%04x) - exact dBm (pre power table adjustment) ***\n",chain_base);
        ar5513PrintPowerPerRate(ratesArray);
    }

    pDev->pHalInfo->pRfHal->ar5513SetPowerTable(pDev, pcdacTable, &minPower, &maxPower, pChval, rfXpdGain, chnIdx);

    /* Adjust XR power/rate up by 2 dB to account for greater peak to avg ratio */
    ratesArray[15] += 4;

    if (IS_5112(pDev)) {
        /* Move 5112 rates to match power tables where the max power table entry corresponds with maxPower */
        ASSERT(maxPower <= PCDAC_STOP);
        powerIndexOffset = PCDAC_STOP - maxPower;
        for (i = 0; i < NUM_RATES; i++) {
            ratesArray[i] += powerIndexOffset;
            if (ratesArray[i] > 63) {
                ratesArray[i] = 63;
            }
        }
    } else {
        powerIndexOffset = 0;
    }
    pDev->pHalInfo->txPowerIndexOffset = powerIndexOffset;

    if (TxPowerDebugLevel >= TXPOWER_DEBUG_EXTREME) {
        uiPrintf("\nChain_base = 0x%04x\n",chain_base);
        uiPrintf("\nTX Power settings: curve(s) returned minPower %d (2x dBm), maxPower %d (2x dBm)\n\tPower Offset %d (2x dBm)\n",
            minPower, maxPower, powerIndexOffset);
        uiPrintf("XpdGain's [0] 0x%x, [1] 0x%x\n", rfXpdGain[0], rfXpdGain[1]);
    }


    /* WAR to correct gain deltas for 5212 G operation  - Removed with revised chipset */
    if ((pDev->phyRev < PHY_CHIP_ID_REV_2) && IS_CHAN_G(pChval->channelFlags)) {
        if (pChval->channel == 2484) {
            cckOfdmPwrDelta = SCALE_OC_DELTA(pData->pEepHeader->cckOfdmPwrDelta - pData->pEepHeader->scaledCh14FilterCckDelta);
        } else {
            cckOfdmPwrDelta = SCALE_OC_DELTA(pData->pEepHeader->cckOfdmPwrDelta);
        }
        ar5513CorrectGainDelta(pDev, ratesArray, pcdacTable, cckOfdmPwrDelta);
        /* Requires a rewrite of the pcdac table in all cases */
        pcdacTableModified = TRUE;
    }

    if (calledDuringReset || pcdacTableModified) {
        /* Finally, write the power values into the baseband power table */
        regOffset = chain_base + (608 << 2);
        for (i = 0; i < 32; i++) {
            if (TxPowerDebugLevel >= TXPOWER_DEBUG_VERBOSE) {
                uiPrintf("dBm %2d -> pcdac %2d | dBm %2d.5 -> pcdac %2d\n",
                    i, pcdacTable[2 * i], i, pcdacTable[2 * i + 1]);
            }

            /*
            **  New AR5513 PCDAC Table entry format
            */
            reg32 = 0xffff & (pcdacTable[2 * i + 1]);
            reg32 = (reg32 << 16) | (0xffff & (pcdacTable[2 * i]));

            writePlatformReg(pDev, regOffset, reg32);
            regOffset += 4;
        }
    }

    if (TxPowerDebugLevel >= TXPOWER_DEBUG_VERBOSE) {
        uiPrintf("\n*** Actual power per rate table as programmed ***\n");
        ar5513PrintPowerPerRate(ratesArray);
    }

    /* Write the OFDM power per rate set */
    reg32 = (((paPreDEnable & 1)<< 30) | ((ratesArray[3] & mask) << 24) |
             ((paPreDEnable & 1)<< 22) | ((ratesArray[2] & mask) << 16) |
             ((paPreDEnable & 1)<< 14) | ((ratesArray[1] & mask) <<  8) |
             ((paPreDEnable & 1)<< 6 ) |  (ratesArray[0] & mask));
    writePlatformReg(pDev, chain_base + (PHY_POWER_TX_RATE1 - CHN_0_BASE), reg32);

    reg32 = (((paPreDEnable & 1)<< 30) | ((ratesArray[7] & mask) << 24) |
             ((paPreDEnable & 1)<< 22) | ((ratesArray[6] & mask) << 16) |
             ((paPreDEnable & 1)<< 14) | ((ratesArray[5] & mask) <<  8) |
             ((paPreDEnable & 1)<< 6 ) |  (ratesArray[4] & mask));
    writePlatformReg(pDev, chain_base + (PHY_POWER_TX_RATE2 - CHN_0_BASE), reg32);

    /* Write the CCK power per rate set */
    reg32 = (((ratesArray[10] & mask) << 24) |
             ((ratesArray[9]  & mask) << 16) |
             ((ratesArray[15] & mask) <<  8) |   // XR target power
              (ratesArray[8]  & mask));
    writePlatformReg(pDev, chain_base + (PHY_POWER_TX_RATE3 - CHN_0_BASE), reg32);

    reg32 = (((ratesArray[14] & mask) << 24) |
             ((ratesArray[13] & mask) << 16) |
             ((ratesArray[12] & mask) <<  8) |
              (ratesArray[11] & mask));
    writePlatformReg(pDev, chain_base + (PHY_POWER_TX_RATE4 - CHN_0_BASE), reg32);

    if (IS_CHAN_OFDM(pChval->channelFlags) && pDev->staConfig.eirpLtdMode) {
    
        twiceBandEirp = twiceBandEdgePower + FALCON_TWICE_COHERENT_COMB_GAIN;
        twiceBandEirp = A_MIN((wlanGetChannelPower(pDev, pChval) * 2), twiceBandEirp);
        /* Set the band eirp for eirp limted mode */
        A_REG_RMW_FIELD (pDev, PHY_TXBF_CTRL, BAND_EIRP, twiceBandEirp);
        if (TxPowerDebugLevel >= TXPOWER_DEBUG_VERBOSE) {
            uiPrintf("Band EIRP %3d.%1d dBm\n", twiceBandEirp / 2, (twiceBandEirp % 2) * 5);
        }

        /* Set the EIRP limited mode on Channel G */
        A_REG_RMW_FIELD (pDev, PHY_TXBF_CTRL, EIRP_MODE, 1);

    } 
    A_REG_WR(pDev, PHY_POWER_TX_RATE_MAX, MAX_RATE_POWER);
    return status;
}

/**************************************************************
 * ar5513SetTxPowerLimit
 *
 * Sets a limit on the overall output power.  Used for dynamic
 * transmit power control and the like.
 *
 * NOTE: The power passed in is in units of 0.5 dBm.
 */
void
ar5513SetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit)
{
    /* These are filled in by the function but unused here */
    A_UINT16 dummyXpdGains[2];

    /* Make sure we don't exceed the maximum. */
    limit = A_MIN(limit, MAX_RATE_POWER);

    ar5513SetTransmitPower(pDev, pDev->staConfig.pChannel, dummyXpdGains, (A_INT16) limit, FALSE, CHAIN_0);

    ar5513SetTransmitPower(pDev, pDev->staConfig.pChannel, dummyXpdGains, (A_INT16) limit, FALSE, CHAIN_1);

}

/*
 * LARGE TODO: update power calibration with upcoming changes and cleanup the
 * direct from DK code below to better merge with current similar functions
 * and stylize the Atheros SW way
 */
A_INT16
GetInterpolatedValue_Signed16(A_UINT16 target, A_UINT16 srcLeft, A_UINT16 srcRight,
                             A_INT16 targetLeft, A_INT16 targetRight)
{
  A_INT16 returnValue;

  if (srcRight != srcLeft) {
        returnValue = (A_INT16)( ( (target - srcLeft)*targetRight + (srcRight - target)*targetLeft)/(srcRight - srcLeft));
  }
  else {
        returnValue = targetLeft;
  }
  return (returnValue);
}


// returns indices surrounding the value in sorted integer lists. used for channel and pcdac lists
void
GetLowerUpperIndex (A_UINT16 value, A_UINT16 *pList, A_UINT16 listSize, A_UINT32 *pLowerValue,
                    A_UINT32 *pUpperValue)
{
    A_UINT16    i;
    A_UINT16    listEndValue = *(pList + listSize - 1);
    A_UINT16    target = value ;

    //see if value is lower than the first value in the list
    //if so return first value
    if (target <= (*pList)) {
        *pLowerValue = 0;
        *pUpperValue = 0;
        return;
    }

    //see if value is greater than last value in list
    //if so return last value
    if (target >= listEndValue) {
        *pLowerValue = listSize - 1;
        *pUpperValue = listSize - 1;
        return;
    }

    //look for value being near or between 2 values in list
    for(i = 0; i < listSize; i++) {
        //if value is close to the current value of the list
        //then target is not between values, it is one of the values
        if (pList[i] == target) {
            *pLowerValue = i;
            *pUpperValue = i;
            return;
        }

        //look for value being between current value and next value
        //if so return these 2 values
        if (target < pList[i + 1]) {
            *pLowerValue = i;
            *pUpperValue = i + 1;
            return;
        }
    }
}

A_BOOL
getFullPwrTable(A_UINT16 numPcdacs, A_UINT16 *pcdacs, A_INT16 *power, A_INT16 maxPower, A_INT16 *retVals)
{
    A_UINT16    ii;
    A_UINT16    idxL = 0;
    A_UINT16    idxR = 1;

    if (numPcdacs < 2) {
        uiPrintf("getFullPwrTable: at least 2 pcdac values needed in getFullPwrTable - [%d]\n", numPcdacs);
        return(FALSE);
    }

    for (ii=0; ii<64; ii++) {
        if ((ii>pcdacs[idxR]) && (idxR < (numPcdacs-1))) {
            idxL++;
            idxR++;
        }
        retVals[ii] = GetInterpolatedValue_Signed16(ii, pcdacs[idxL], pcdacs[idxR], power[idxL], power[idxR]);
        if (retVals[ii] >= maxPower) {
            while (ii<64) {
                retVals[ii++] = maxPower;
            }
        }
    }
    return(TRUE);
}

/**************************************************************
 * getPminAndPcdacTableFromPowerTable
 *
 * Takes a single calibration curve and creates a power table.
 * Adjusts the new power table so the max power is relative
 * to the maximum index in the power table.
 *
 * WARNING: rates must be adjusted for this relative power table
 */
A_INT16
getPminAndPcdacTableFromPowerTable(A_INT16 *pwrTableT4, A_UINT16 retVals[])
{
    A_INT16 ii, jj, jjMax;
    A_INT16 pMin, currPower, pMax;

    /* If the spread is > 31.5dB, keep the upper 31.5dB range */
    if ((pwrTableT4[63] - pwrTableT4[0]) > 126) {
        pMin = pwrTableT4[63] - 126;
    } else {
        pMin = pwrTableT4[0];
    }

    pMax = pwrTableT4[63];
    jjMax = 63;

    /* Search for highest pcdac 0.25dB below maxPower */
    while ((pwrTableT4[jjMax] > (pMax - 1) ) && (jjMax >= 0)) {
        jjMax--;
    }

    jj = jjMax;
    currPower = pMax;
    /*
    currPower += 4 * FALCON_BAND_EIRP_HEADROOM_IN_DB;
    pMin      += 4 * FALCON_BAND_EIRP_HEADROOM_IN_DB;
    */
    currPower = 126;
    pMin      = 0;

    for (ii = 63; ii >= 0; ii--) {
        while ((jj < 64) && (jj > 0) && (pwrTableT4[jj] >= currPower)) {
            jj--;
        }
        if (jj == 0) {
            while (ii >= 0) {
                retVals[ii] = retVals[ii + 1];
                ii--;
            }
            break;
        }
        retVals[ii] = jj;
        currPower -= 2;  // corresponds to a 0.5dB step
    }
    return(pMin);
}

/**************************************************************
 * getPminAndPcdacTableFromTwoPowerTables
 *
 * Combines the XPD curves from two calibration sets into a single
 * power table and adjusts the power table so the max power is relative
 * to the maximum index in the power table
 *
 * WARNING: rates must be adjusted for this relative power table
 */
A_INT16
getPminAndPcdacTableFromTwoPowerTables(A_INT16 *pwrTableLXpdT4, A_INT16 *pwrTableHXpdT4,
                                       A_UINT16 retVals[], A_INT16 *pMid)
{
    A_INT16     ii, jj, jjMax;
    A_INT16     pMin, pMax, currPower;
    A_INT16     *pwrTableT4;
    A_UINT16    msbFlag = 0x40;  // turns on the 7th bit of the pcdac

    /* If the spread is > 31.5dB, keep the upper 31.5dB range */
    if ((pwrTableLXpdT4[63] - pwrTableHXpdT4[0]) > 126) {
        pMin = pwrTableLXpdT4[63] - 126;
    } else {
        pMin = pwrTableHXpdT4[0];
    }

    pMax = pwrTableLXpdT4[63];
    jjMax = 63;
    /* Search for highest pcdac 0.25dB below maxPower */
    while ((pwrTableLXpdT4[jjMax] > (pMax - 1) ) && (jjMax >= 0)){
        jjMax--;
    }

    *pMid = pwrTableHXpdT4[63];
    jj = jjMax;
    ii = 63;
    currPower = pMax;
    pwrTableT4 = &(pwrTableLXpdT4[0]);
    /*
    currPower += 4 * FALCON_BAND_EIRP_HEADROOM_IN_DB;
    pMin      += 4 * FALCON_BAND_EIRP_HEADROOM_IN_DB;
    */
    currPower = 126;
    pMin      = 0;

    while (ii >= 0) {
        if ((currPower <= *pMid) || ( (jj == 0) && (msbFlag == 0x40))){
            msbFlag = 0x00;
            pwrTableT4 = &(pwrTableHXpdT4[0]);
            jj = 63;
        }
        while ((jj > 0) && (pwrTableT4[jj] >= currPower)) {
            jj--;
        }
        if ((jj == 0) && (msbFlag == 0x00)) {
            while (ii >= 0) {
                retVals[ii] = retVals[ii+1];
                ii--;
            }
            break;
        }
        retVals[ii] = jj | msbFlag;
        currPower -= 2;  // corresponds to a 0.5dB step
        ii--;
    }
    return(pMin);
}

/**************************************************************
 * ar5513SetPowerTable5112
 *
 * Read the transmit power levels from the structures taken from EEPROM
 * Interpolate read transmit power values for this channel
 * Organize the transmit power values into a table for writing into the hardware
 */
void
ar5513SetPowerTable5112(WLAN_DEV_INFO *pDev, A_UINT16 *pPcdacTable, A_INT16 *pPowerMin, A_INT16 *pPowerMax, CHAN_VALUES *pChval, A_UINT16 *rfXpdGain, A_UINT8 chnIdx)
{
    A_UINT32    numXpdGain = 2;
    A_UINT32    xpdGainMask = 0;
    A_INT16     powerMid, *pPowerMid = &powerMid; // Out     (2 x power)

    EEP_MAP                    *pMap = pDev->pHalInfo->pEepData;
    EXPN_DATA_PER_CHANNEL_5112 *pRawCh;
    EEPROM_POWER_EXPN_5112     *pPowerExpn = NULL;

    A_UINT32    ii, jj, kk;
    A_INT16     minPwr_t4, maxPwr_t4, Pmin, Pmid;

    A_UINT32    chan_idx_L, chan_idx_R;
    A_UINT16    chan_L, chan_R;

    A_INT16     pwr_table0[64];
    A_INT16     pwr_table1[64];
    A_UINT16    pcdacs[10];
    A_INT16     powers[10];
    A_UINT16    numPcd;
    A_INT16     powTableLXPD[2][64];
    A_INT16     powTableHXPD[2][64];
    A_INT16     tmpPowerTable[64];
    A_UINT16    xgainList[2];
    A_UINT16    xpdMask;

    ASSERT(pMap);

    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
    case CHANNEL_T:
    case CHANNEL_XR_A:
    case CHANNEL_XR_T:
        pPowerExpn = &pMap->chain[chnIdx].modePowerArray5112[headerInfo11A];
        xpdGainMask = pMap->pEepHeader->xgain[headerInfo11A];
        break;

    case CHANNEL_B:
        pPowerExpn = &pMap->chain[chnIdx].modePowerArray5112[headerInfo11B];
        xpdGainMask = pMap->pEepHeader->xgain[headerInfo11B];
        break;
    case CHANNEL_G:
    case CHANNEL_108G:
    case CHANNEL_XR_G:
        pPowerExpn = &pMap->chain[chnIdx].modePowerArray5112[headerInfo11G];
        xpdGainMask = pMap->pEepHeader->xgain[headerInfo11G];
        break;
    }

    if ((xpdGainMask & pPowerExpn->xpdMask) < 1) {
        uiPrintf("ar5513SetPowerTable5112: desired xpdGainMask not supported by calibrated xpdMask\n");
        ASSERT(0);
    }

    maxPwr_t4 = (A_INT16)(2*(*pPowerMax));   // pwr_t2 -> pwr_t4
    minPwr_t4 = (A_INT16)(2*(*pPowerMin));    // pwr_t2 -> pwr_t4

    xgainList[0] = 0xDEAD;
    xgainList[1] = 0xDEAD;

    kk = 0;
    xpdMask = pPowerExpn->xpdMask;
    for (jj = 0; jj < NUM_XPD_PER_CHANNEL; jj++) {
        if (((xpdMask >> jj) & 1) > 0) {
            if (kk > 1) {
                uiPrintf("A maximum of 2 xpdGains supported in pExpnPower data\n");
                return;
            }
            xgainList[kk++] = (A_UINT16)jj;
        }
    }

    GetLowerUpperIndex(pChval->channel, &(pPowerExpn->pChannels[0]), pPowerExpn->numChannels, &(chan_idx_L), &(chan_idx_R));

    kk = 0;
    for (ii=chan_idx_L; ii<=chan_idx_R; ii++) {
        pRawCh = &(pPowerExpn->pDataPerChannel[ii]);
        if (xgainList[1] == 0xDEAD) {
            jj = xgainList[0];
            numPcd = pRawCh->pDataPerXPD[jj].numPcdacs;
            memcpy(&(pcdacs[0]), &(pRawCh->pDataPerXPD[jj].pcdac[0]), numPcd * sizeof(A_UINT16));
            memcpy(&(powers[0]), &(pRawCh->pDataPerXPD[jj].pwr_t4[0]), numPcd * sizeof(A_INT16));
            if (!getFullPwrTable(numPcd, &(pcdacs[0]), &(powers[0]), pRawCh->maxPower_t4, &(tmpPowerTable[0]))) {
                ASSERT(0);
                return;
            } else {
                A_DRIVER_BCOPY(&(tmpPowerTable[0]), &(powTableLXPD[kk][0]), 64*sizeof(A_INT16));
            }
        } else {
            jj = xgainList[0];
            numPcd = pRawCh->pDataPerXPD[jj].numPcdacs;
            A_DRIVER_BCOPY(&(pRawCh->pDataPerXPD[jj].pcdac[0]), &(pcdacs[0]), numPcd*sizeof(A_UINT16));
            A_DRIVER_BCOPY(&(pRawCh->pDataPerXPD[jj].pwr_t4[0]), &(powers[0]), numPcd*sizeof(A_INT16));
            if (!getFullPwrTable(numPcd, &(pcdacs[0]), &(powers[0]), pRawCh->maxPower_t4, &(tmpPowerTable[0]))) {
                ASSERT(0);
                return;
            } else {
                A_DRIVER_BCOPY(&(tmpPowerTable[0]), &(powTableLXPD[kk][0]), 64 * sizeof(A_INT16));
            }

            jj = xgainList[1];
            numPcd = pRawCh->pDataPerXPD[jj].numPcdacs;
            A_DRIVER_BCOPY(&(pRawCh->pDataPerXPD[jj].pcdac[0]), &(pcdacs[0]), numPcd * sizeof(A_UINT16));
            A_DRIVER_BCOPY(&(pRawCh->pDataPerXPD[jj].pwr_t4[0]), &(powers[0]), numPcd * sizeof(A_INT16));
            if (!getFullPwrTable(numPcd, &(pcdacs[0]), &(powers[0]), pRawCh->maxPower_t4, &(tmpPowerTable[0]))) {
                ASSERT(0);
                return;
            } else {
                A_DRIVER_BCOPY(&(tmpPowerTable[0]), &(powTableHXPD[kk][0]), 64 * sizeof(A_INT16));
            }
        }

        kk++;
    }

    chan_L = pPowerExpn->pChannels[chan_idx_L];
    chan_R = pPowerExpn->pChannels[chan_idx_R];
    kk = chan_idx_R - chan_idx_L;

    if (xgainList[1] == 0xDEAD) {
        for (jj = 0; jj < 64; jj++) {
            pwr_table0[jj] = GetInterpolatedValue_Signed16(pChval->channel, chan_L, chan_R, powTableLXPD[0][jj], powTableLXPD[kk][jj]);
        }
        Pmin = getPminAndPcdacTableFromPowerTable(&(pwr_table0[0]), pPcdacTable);
        *pPowerMin = (A_INT16) (Pmin / 2);
        *pPowerMid = (A_INT16) (pwr_table0[63] / 2);
        *pPowerMax = (A_INT16) (pwr_table0[63] / 2);
        rfXpdGain[0] = xgainList[0];
        rfXpdGain[1] = rfXpdGain[0];
    } else {
        for (jj = 0; jj < 64; jj++) {
            pwr_table0[jj] = GetInterpolatedValue_Signed16(pChval->channel, chan_L, chan_R, powTableLXPD[0][jj], powTableLXPD[kk][jj]);
            pwr_table1[jj] = GetInterpolatedValue_Signed16(pChval->channel, chan_L, chan_R, powTableHXPD[0][jj], powTableHXPD[kk][jj]);
        }
        if (numXpdGain == 2) {
            Pmin = getPminAndPcdacTableFromTwoPowerTables(&(pwr_table0[0]), &(pwr_table1[0]), pPcdacTable, &Pmid);
            *pPowerMin = (A_INT16) (Pmin / 2);
            *pPowerMid = (A_INT16) (Pmid / 2);
            *pPowerMax = (A_INT16) (pwr_table0[63] / 2);
            rfXpdGain[0] = xgainList[0];
            rfXpdGain[1] = xgainList[1];
        } else {
            if ( (minPwr_t4  <= pwr_table1[63]) && (maxPwr_t4  <= pwr_table1[63])) {
                Pmin = getPminAndPcdacTableFromPowerTable(&(pwr_table1[0]), pPcdacTable);
                rfXpdGain[0] = xgainList[1];
                rfXpdGain[1] = rfXpdGain[0];
                *pPowerMin = (A_INT16) (Pmin / 2);
                *pPowerMid = (A_INT16) (pwr_table1[63] / 2);
                *pPowerMax = (A_INT16) (pwr_table1[63] / 2);
            } else {
                Pmin = getPminAndPcdacTableFromPowerTable(&(pwr_table0[0]), pPcdacTable);
                rfXpdGain[0] = xgainList[0];
                rfXpdGain[1] = rfXpdGain[0];
                *pPowerMin = (A_INT16) (Pmin/2);
                *pPowerMid = (A_INT16) (pwr_table0[63] / 2);
                *pPowerMax = (A_INT16) (pwr_table0[63] / 2);
            }
        }
    }

    /*
    ** NOTE: FALCON ONLY limitation
    */
    *pPowerMax = 63;
}

/**************************************************************
 * ar5513SetRateTable
 *
 * Sets the transmit power in the baseband for the given
 * operating channel and mode.
 */
void
ar5513SetRateTable(WLAN_DEV_INFO *pDev, A_UINT16 *pRatesPower,
                   TRGT_POWER_ALL_MODES *pTargetPowers, CHAN_VALUES *pChval,
                   A_INT16 tpcScaleReduction, A_INT16 powerLimit, struct eepMap *pData,
                   A_INT16 *pMinPower, A_INT16 *pMaxPower, A_INT16 *pBandEdge, A_UINT8 chnIdx)
{
    A_UINT16        twiceMaxEdgePower = MAX_RATE_POWER;
    A_UINT16        twiceMaxEdgePowerCck = MAX_RATE_POWER;
    A_UINT16        twiceMaxRDPower = MAX_RATE_POWER;
    A_UINT16        i;
    A_UINT8         cfgCtl;
    A_INT8          twiceAntennaGain, twiceAntennaReduction = 0;
    A_UINT16        chanMode;
    RD_EDGES_POWER  *pRdEdgesPower = NULL, *pRdEdgesPowerCck = NULL;
    TRGT_POWER_INFO targetPowerOfdm, targetPowerCck;
    A_INT16         scaledPower, maxAvailPower = 0;

    chanMode = pChval->channelFlags & CHANNEL_ALL;
    twiceMaxRDPower = wlanGetChannelPower(pDev, pChval) * 2;
    *pMaxPower = -MAX_RATE_POWER;
    *pMinPower = MAX_RATE_POWER;

    /* Get conformance test limit maximum for this channel */
    cfgCtl = wlanGetCtl(pDev, pChval);

    for (i = 0; i < pData->pEepHeader->numCtls; i++) {
        A_UINT16 twiceMinEdgePower;

        if (pData->pEepHeader->ctl[i] == 0) {
            continue;
        } else if ((pData->pEepHeader->ctl[i] == cfgCtl) ||
            (cfgCtl == ((pData->pEepHeader->ctl[i] & CTL_MODE_M) | SD_NO_CTL)))
        {
            pRdEdgesPower = &(pData->chain[chnIdx].pRdEdgesPower[i * NUM_EDGES]);
            twiceMinEdgePower = ar5513GetMaxEdgePower(pChval->channel, pRdEdgesPower);

            if ((cfgCtl & ~CTL_MODE_M) == SD_NO_CTL) {
                /* Find the minimum of all CTL edge powers that apply to this channel */
                twiceMaxEdgePower = A_MIN(twiceMaxEdgePower, twiceMinEdgePower);
            } else {
                twiceMaxEdgePower = twiceMinEdgePower;
                break;
            }
        }
    }
    if (IS_CHAN_G(chanMode)) {
        /* Check for a CCK CTL for 11G CCK powers */
        cfgCtl = (cfgCtl & ~CTL_MODE_M) | CTL_11B;
        for (i = 0; i < pData->pEepHeader->numCtls; i++) {
            A_UINT16 twiceMinEdgePowerCck;

            if (pData->pEepHeader->ctl[i] == 0) {
                continue;
            } else if ((pData->pEepHeader->ctl[i] == cfgCtl) ||
                (cfgCtl == ((pData->pEepHeader->ctl[i] & CTL_MODE_M) | SD_NO_CTL)))
            {
                pRdEdgesPowerCck = &(pData->chain[chnIdx].pRdEdgesPower[i * NUM_EDGES]);
                twiceMinEdgePowerCck = ar5513GetMaxEdgePower(pChval->channel, pRdEdgesPowerCck);
                if ((cfgCtl & ~CTL_MODE_M) == SD_NO_CTL) {
                    /* Find the minimum of all CTL edge powers that apply to this channel */
                    twiceMaxEdgePowerCck = A_MIN(twiceMaxEdgePowerCck, twiceMinEdgePowerCck);
                } else {
                    twiceMaxEdgePowerCck = twiceMinEdgePowerCck;
                    break;
                }
            }
        }
    } else {
        /* Set the 11B cck edge power to the one found before */
        twiceMaxEdgePowerCck = twiceMaxEdgePower;
    }

    /* Get Antenna Gain reduction */
    if (IS_CHAN_5GHZ(pChval->channelFlags)) {
        twiceAntennaGain = pData->pEepHeader->antennaGainMax[0];
    } else {
        twiceAntennaGain = pData->pEepHeader->antennaGainMax[1];
    }
    twiceAntennaReduction = wlanGetAntennaReduction(pDev, pChval, twiceAntennaGain);

    if (IS_CHAN_OFDM(chanMode)) {
        /* Get final OFDM target powers */
        if (IS_CHAN_2GHZ(chanMode)) {
            ar5513GetTargetPowers(pDev, pChval, pTargetPowers->trgtPwr_11g,
                pTargetPowers->numTargetPwr_11g, &targetPowerOfdm);
        } else {
            ar5513GetTargetPowers(pDev, pChval, pTargetPowers->trgtPwr_11a,
                pTargetPowers->numTargetPwr_11a, &targetPowerOfdm);
        }

        /* Get Maximum OFDM power */
        /* Minimum of antenna reduced and edge powers */
        scaledPower = A_MIN(twiceMaxEdgePower, twiceMaxRDPower - twiceAntennaReduction);

        /* Don't limit AP power levels for turbo */
#ifndef BUILD_AP
        if (IS_CHAN_TURBO(pChval->channelFlags)) {
            /* If turbo is set, reduce power to keep power consumption under 2 Watts */
            if (pData->version >= EEPROM_VER3_1) {
                scaledPower = A_MIN(scaledPower, pData->pEepHeader->turbo2WMaxPower5);
                /* EEPROM Version 4_0 adds a separate turbo 2GHz power constraint */
                if ((pData->version >= EEPROM_VER4_0) &&  IS_CHAN_2GHZ(chanMode)) {
                    scaledPower = A_MIN(scaledPower, pData->pEepHeader->turbo2WMaxPower2);
                }
            }

        }
#endif
        *pBandEdge = scaledPower;
        if (pDev->staConfig.eirpLtdMode) {
            /* Add 3 db for EIRP Mode limited */
            scaledPower = scaledPower + FALCON_TWICE_SINGLE_CHAIN_GAIN;
        }

        maxAvailPower = A_MIN(scaledPower, targetPowerOfdm.twicePwr6_24);

        /* Reduce Power by user selection */
        scaledPower -= (tpcScaleReduction * 2);
        scaledPower = (scaledPower < 0)? 0: scaledPower;
        scaledPower = A_MIN(scaledPower, powerLimit);
        
        scaledPower = A_MIN(scaledPower, targetPowerOfdm.twicePwr6_24);

        /* Override XR, 6-24 Tx Power to any level - 36-54 still cannot exceed target power levels */
        if (pDev->staConfig.overRideTxPower) {
            scaledPower = pDev->staConfig.overRideTxPower * 2;
        }

        /* Set OFDM rates XR, 6, 9, 12, 18, 24 */
        pRatesPower[0] = pRatesPower[1] = pRatesPower[2] = pRatesPower[3] = pRatesPower[4] = scaledPower;

        /* Set OFDM rates 36, 48, 54, XR */
        pRatesPower[5] = A_MIN(scaledPower, targetPowerOfdm.twicePwr36);
        pRatesPower[6] = A_MIN(scaledPower, targetPowerOfdm.twicePwr48);
        pRatesPower[7] = A_MIN(scaledPower, targetPowerOfdm.twicePwr54);

        if (pData->version >= EEPROM_VER4_0) {
            /* Setup XR target power from EEPROM */
            pRatesPower[15] = A_MIN(scaledPower, (IS_CHAN_2GHZ(chanMode) ?
                pData->pEepHeader->xrTargetPower2 : pData->pEepHeader->xrTargetPower5));
        } else {
            /* XR uses 6mb power */
            pRatesPower[15] = pRatesPower[0];
        }

        *pMinPower = pRatesPower[7];
        *pMaxPower = pRatesPower[0];

        pDev->tx6PowerInHalfDbm = *pMaxPower;
        pDev->pHalInfo->ofdmTxPower = *pMaxPower;

        if (TxPowerDebugLevel >= TXPOWER_DEBUG_VERBOSE) {
            uiPrintf("OFDM 2X Maxes: MaxRD: %d TurboMax: %d MaxCTL: %d TPC_Reduction %d Final Scaled Power %d\n",
                twiceMaxRDPower, pData->pEepHeader->turbo2WMaxPower5, twiceMaxEdgePower,
                (tpcScaleReduction * 2), scaledPower);
        }

    }

    /* xxx: 'G' should set the CCK Flag! */
    if (IS_CHAN_CCK(chanMode) || IS_CHAN_G(chanMode)) {
        /* Get final CCK target powers */
        ar5513GetTargetPowers(pDev, pChval, pTargetPowers->trgtPwr_11b,
            pTargetPowers->numTargetPwr_11b, &targetPowerCck);

        /* Reduce power by max regulatory domain allowed restrictions */
        scaledPower = A_MIN(twiceMaxEdgePowerCck, twiceMaxRDPower - twiceAntennaReduction);

        if (maxAvailPower < A_MIN(scaledPower, targetPowerCck.twicePwr6_24)) {
            maxAvailPower = A_MIN(scaledPower, targetPowerCck.twicePwr6_24);
        }

        /* Reduce Power by user selection */
        scaledPower -= (tpcScaleReduction * 2);
        scaledPower = (scaledPower < 0)? 0: scaledPower;
        scaledPower = A_MIN(scaledPower, powerLimit);

        /* Set CCK rates 2L, 2S, 5.5L, 5.5S, 11L, 11S */
        pRatesPower[8]  = A_MIN(scaledPower, targetPowerCck.twicePwr6_24);
        pRatesPower[9]  = A_MIN(scaledPower, targetPowerCck.twicePwr36);
        pRatesPower[10] = pRatesPower[9];
        pRatesPower[11] = A_MIN(scaledPower, targetPowerCck.twicePwr48);
        pRatesPower[12] = pRatesPower[11];
        pRatesPower[13] = A_MIN(scaledPower, targetPowerCck.twicePwr54);
        pRatesPower[14] = pRatesPower[13];

        if (pDev->staConfig.overRideTxPower) {
            pRatesPower[8] = pRatesPower[9] = pRatesPower[10] = pRatesPower[11] = pDev->staConfig.overRideTxPower * 2;
            pRatesPower[12] = pRatesPower[13] = pRatesPower[14] = pDev->staConfig.overRideTxPower * 2;
        }

        /* Set min/max power based off OFDM values or initialization */
        if (pRatesPower[13] < *pMinPower) {
            *pMinPower = pRatesPower[13];
        }
        if (pRatesPower[8] > *pMaxPower) {
            *pMaxPower = pRatesPower[8];
        }
    }

    /* Report the current power as the max set power */
    pDev->tx6PowerInHalfDbm = *pMaxPower;

    pDev->maxTxPowerAvail = maxAvailPower;
}

/***************************************************************************
 * ar5513CorrectGainDelta
 *
 * Corrects for the gain-delta between ofdm and cck
 * mode target powers. writes to the rate table and
 * the power table.
 *
 *   Conventions :
 *   1. pRatesPower[ii] is the integer value of 2*(desired power
 *    for the rate ii in dBm) to provide 0.5dB resolution. rate
 *    mapping is as following :
 *     [0..7]  --> ofdm 6, 9, .. 48, 54
 *     [8..14] --> cck 1L, 2L, 2S, .. 11L, 11S
 *     [15]    --> XR (all rates get the same power)
 *   2. pPowerValues[ii]  is the pcdac corresponding to ii/2 dBm.
 */
void
ar5513CorrectGainDelta(WLAN_DEV_INFO *pDev, A_UINT16 *pRatesPower,
                       A_UINT16 *pPowerValues, int twiceOfdmCckDelta)
{
    A_INT16  ratesIndex[16];
    A_UINT16 ii, jj, iter;
    A_INT32  cckIndex;
    A_INT16  gainDeltaAdjust = pDev->pHalInfo->pEepData->pEepHeader->cckOfdmGainDelta;

    /* make a local copy of desired powers as initial indices */
    A_DRIVER_BCOPY(pRatesPower, ratesIndex, NUM_RATES * sizeof(A_UINT16));

    /* fix the CCK indices */
    for (ii = 8; ii < 15; ii++) {
        /* apply a gain_delta correction of -15 for CCK */
        ratesIndex[ii] -= gainDeltaAdjust;

        /* Now check for contention with all ofdm target powers */
        jj = 0;
        iter = 0;
        /* indicates not all ofdm rates checked forcontention yet */
        while (jj < 16) {
            if (ratesIndex[ii] < 0) {
                ratesIndex[ii] = 0;
            }
            if (jj == 8) {
                 /* skip CCK rates */
                jj = 15;
                continue;
            }
            if (ratesIndex[ii] == pRatesPower[jj]) {
                if (pRatesPower[jj] == 0) {
                    ratesIndex[ii]++;
                } else {
                    if (iter > 50) {
                        /* to avoid pathological case of ofdm target powers 0 and 0.5dBm */
                        ratesIndex[ii]++;
                    } else {
                        ratesIndex[ii]--;
                    }
                }
                /* check with all rates again */
                jj = 0;
                iter++;
            } else {
                jj++;
            }
        }
        if (ratesIndex[ii] >= PWR_TABLE_SIZE) {
            ratesIndex[ii] = PWR_TABLE_SIZE -1;
        }
        cckIndex = pRatesPower[ii] - twiceOfdmCckDelta;
        if (cckIndex < 0) {
            cckIndex = 0;
        }

        /*
         * Validate that the indexes for the pPowerValues are not
         * out of bounds. BUG 6694
         */
        ASSERT((cckIndex < PWR_TABLE_SIZE) && (ratesIndex[ii] < PWR_TABLE_SIZE));
        pPowerValues[ratesIndex[ii]] = pPowerValues[cckIndex];
    }

    /* Override rate per power table with new values */
    for (ii = 8; ii < 15; ii++) {
        pRatesPower[ii] = ratesIndex[ii];
    }
}

/***************************************************************************
 * ar5513GetMaxEdgePower
 *
 * Find the maximum conformance test limit for the given channel and CTL info
 */
A_UINT16
ar5513GetMaxEdgePower(A_UINT16 channel, RD_EDGES_POWER *pRdEdgesPower)
{
    A_UINT16        i, numEdges;
    A_UINT16        tempChannelList[NUM_EDGES]; /* temp array for holding edge channels */
    A_UINT16        lowerChannel, upperChannel;
    A_UINT16        twiceMaxEdgePower = MAX_RATE_POWER;

    /* Get the edge power */
    for (i = 0; i < NUM_EDGES; i++) {
        if (pRdEdgesPower[i].rdEdge == 0) {
            break;
        }
        tempChannelList[i] = pRdEdgesPower[i].rdEdge;
    }
    numEdges = i;

    ar5513GetLowerUpperValues(channel, tempChannelList, numEdges, &lowerChannel, &upperChannel);
    /* Get the index for the lower channel */
    for (i = 0; i < numEdges; i++) {
        if (lowerChannel == tempChannelList[i]) {
            break;
        }
    }
    /* Is lower channel ever outside the rdEdge? */
    ASSERT(i != numEdges);

    if (((lowerChannel == upperChannel) && (lowerChannel == channel)) ||
        (pRdEdgesPower[i].flag))
    {
        /*
         * If there's an exact channel match or an inband flag set
         * on the lower channel use the given rdEdgePower
         */
        twiceMaxEdgePower = pRdEdgesPower[i].twice_rdEdgePower;
        ASSERT(twiceMaxEdgePower > 0);
    }
    return twiceMaxEdgePower;
}

/***************************************************************************
 * ar5513GetTargetPowers
 *
 * Return the four rates of target power for the given target power table
 * channel, and number of channels
 */
void
ar5513GetTargetPowers(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval, TRGT_POWER_INFO *pPowerInfo,
                      A_UINT16 numChannels, TRGT_POWER_INFO *pNewPower)
{
    A_UINT16   tempChannelList[NUM_TEST_FREQUENCIES]; /* temp array for holding target power channels */
    A_UINT16   i;
    A_UINT16   lowerChannel, lowerIndex=0, lowerPower=0;
    A_UINT16   upperChannel, upperIndex=0, upperPower=0;

    /* Copy the target powers into the temp channel list */
    for (i = 0; i < numChannels; i++) {
        tempChannelList[i] = pPowerInfo[i].testChannel;
    }

    ar5513GetLowerUpperValues(pChval->channel, tempChannelList, numChannels, &lowerChannel, &upperChannel);

    /* Get the index for the channel */
    for (i = 0; i < numChannels; i++) {
        if (lowerChannel == tempChannelList[i]) {
            lowerIndex = i;
        }
        if (upperChannel == tempChannelList[i]) {
            upperIndex = i;
            break;
        }
    }

    /* Get the lower and upper channels target powers and interpolate between them */
    lowerPower = pPowerInfo[lowerIndex].twicePwr6_24;
    upperPower = pPowerInfo[upperIndex].twicePwr6_24;
    pNewPower->twicePwr6_24 = ar5513GetInterpolatedValue(pChval->channel, lowerChannel,
        upperChannel, lowerPower, upperPower, 0);
    lowerPower = pPowerInfo[lowerIndex].twicePwr36;
    upperPower = pPowerInfo[upperIndex].twicePwr36;
    pNewPower->twicePwr36 = ar5513GetInterpolatedValue(pChval->channel, lowerChannel,
        upperChannel, lowerPower, upperPower, 0);
    lowerPower = pPowerInfo[lowerIndex].twicePwr48;
    upperPower = pPowerInfo[upperIndex].twicePwr48;
    pNewPower->twicePwr48 = ar5513GetInterpolatedValue(pChval->channel, lowerChannel,
        upperChannel, lowerPower, upperPower, 0);
    lowerPower = pPowerInfo[lowerIndex].twicePwr54;
    upperPower = pPowerInfo[upperIndex].twicePwr54;
    pNewPower->twicePwr54 = ar5513GetInterpolatedValue(pChval->channel, lowerChannel,
        upperChannel, lowerPower, upperPower, 0);
}

/**************************************************************
 * ar5513GetInterpolatedValue
 *
 * Returns interpolated or the scaled up interpolated value
 */
A_UINT16
ar5513GetInterpolatedValue(A_UINT16 target, A_UINT16 srcLeft, A_UINT16 srcRight,
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
 * ar5513GetLowerUpperValues
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
ar5513GetLowerUpperValues(A_UINT16 value, A_UINT16 *pList, A_UINT16 listSize,
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

/************** Transmit Gain Optimization Routines ******************/
const static GAIN_OPTIMIZATION_LADDER GainLadder = {
    9, //numStepsInLadder
    4, //defaultStepNum
    { { {4, 1, 1, 1},  6, "FG8"}, // most fixed gain
      { {4, 0, 1, 1},  4, "FG7"},
      { {3, 1, 1, 1},  3, "FG6"},
      { {4, 0, 0, 1},  1, "FG5"},
      { {4, 1, 1, 0},  0, "FG4"},
      { {4, 0, 1, 0}, -2, "FG3"},
      { {3, 1, 1, 0}, -3, "FG2"},
      { {4, 0, 0, 0}, -4, "FG1"},
      { {2, 1, 1, 0}, -6, "FG0"}  // least fixed gain
    }
};

const static GAIN_OPTIMIZATION_LADDER GainLadder5112 = {
    8, //numStepsInLadder
    1, //defaultStepNum
    { { {3, 0,0,0, 0,0,0},   6, "FG7"}, // most fixed gain
      { {2, 0,0,0, 0,0,0},   0, "FG6"},
      { {1, 0,0,0, 0,0,0},  -3, "FG5"},
      { {0, 0,0,0, 0,0,0},  -6, "FG4"},
      { {0, 1,1,0, 0,0,0},  -8, "FG3"},
      { {0, 1,1,0, 1,1,0}, -10, "FG2"},
      { {0, 1,0,1, 1,1,0}, -13, "FG1"},
      { {0, 1,0,1, 1,0,1}, -16, "FG0"}, // least fixed gain
    }
};

#ifdef DEBUG
static void
ar5513dumpGainValues(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues)
{
    uiPrintf("currStepNum %d\n", pGainValues->currStepNum);
    uiPrintf("currGain %d\n", pGainValues->currGain);
    uiPrintf("targetGain %d\n", pGainValues->targetGain);
    uiPrintf("gainFCorrection %d\n", pGainValues->gainFCorrection);
    uiPrintf("loTrig %d\n", pGainValues->loTrig);
    uiPrintf("hiTrig %d\n", pGainValues->hiTrig);
    if (IS_5112(pDev)) {
        uiPrintf("[Cur] StepName: %s StepGain: %2d rf_mixgain_ovr %d rf_pwd138 %d "
                 "rf_pwd137 %d rf_pwd136 %d\n\t rf_pwd132 %d rf_pwd131 %d rf_pwd130 %d\n ",
                 pGainValues->currStep->stepName, pGainValues->currStep->stepGain,
                 pGainValues->currStep->paramVal[0], pGainValues->currStep->paramVal[1],
                 pGainValues->currStep->paramVal[2], pGainValues->currStep->paramVal[3],
                 pGainValues->currStep->paramVal[4], pGainValues->currStep->paramVal[5],
                 pGainValues->currStep->paramVal[6]);
    } else {
        uiPrintf("[Cur] StepName: %s StepGain: %2d bb_tx_clip %d rf_pwd_90 %d "
                 "rf_pwd_84 %d rf_rfgainsel %d\n",
                 pGainValues->currStep->stepName, pGainValues->currStep->stepGain,
                 pGainValues->currStep->paramVal[0], pGainValues->currStep->paramVal[1],
                 pGainValues->currStep->paramVal[2], pGainValues->currStep->paramVal[3]);
    }
}
#endif

/**************************************************************
 * ar5513InitializeGainValues
 *
 * Initialize the gain structure to good values
 */
void
ar5513InitializeGainValues(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues)
{
    /* initialize gain optimization values */
    if (IS_5112(pDev)) {
        pGainValues->currStepNum = GainLadder5112.defaultStepNum;
        pGainValues->currStep    = &(GainLadder5112.optStep[GainLadder5112.defaultStepNum]);
        pGainValues->active      = TRUE;
        pGainValues->loTrig      = 20;
        pGainValues->hiTrig      = 85;
    } else {
        pGainValues->currStepNum = GainLadder.defaultStepNum;
        pGainValues->currStep    = &(GainLadder.optStep[GainLadder.defaultStepNum]);
        pGainValues->active      = TRUE;
        pGainValues->loTrig      = 20;
        pGainValues->hiTrig      = 35;
    }
}

/**************************************************************
 * ar5513InvalidGainReadback
 */
A_BOOL
ar5513InvalidGainReadback(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues, A_UINT16 channelFlags)
{
    A_UINT32    gStep, g, mixOvr;
    A_UINT32    L1, L2, L3, L4;

    if (IS_5112(pDev)) {
        mixOvr = ar5513GetRfField(ar5513GetRfBank(pDev, 7), 1, 36, 0);
        L1 = 0;
        L2 = 107 ;
        L3 = 0;
        L4 = 107 ;
        if (mixOvr == 1) {
            L2 = 83;
            L4 = 83;
            pGainValues->hiTrig = 55;
        }
    } else {
        gStep = ar5513GetRfField(ar5513GetRfBank(pDev, 7), 6, 37, 0);

        L1 = 0;
        L2 = (gStep == 0x3f) ? 50 : gStep + 4;
        L3 = (gStep != 0x3f) ? 0x40 : L1;
        L4 = L3 + 50;

        pGainValues->loTrig = L1 + ((gStep == 0x3f) ? DYN_ADJ_LO_MARGIN : 0);
        pGainValues->hiTrig = L4 - ((gStep == 0x3f) ? DYN_ADJ_UP_MARGIN : -5); // never adjust if != 0x3f
    }

    g = pGainValues->currGain;

    return ( ! ( ( (g>=L1) && (g<=L2) ) || ( (g>=L3) && (g<=L4) ) ) );
}

/**************************************************************
 * ar5513RequestRfgain
 *
 * Enable the probe gain check on the next packet
 */
void
ar5513RequestRfgain(WLAN_DEV_INFO *pDev)
{
    A_UINT32 reg32, probePowerIndex;

    /* Enable the gain readback probe */
    probePowerIndex = pDev->pHalInfo->ofdmTxPower + pDev->pHalInfo->txPowerIndexOffset;
    reg32 = ((probePowerIndex << PHY_PAPD_PROBE_POWERTX_S)  & PHY_PAPD_PROBE_POWERTX_M) |
        PHY_PAPD_PROBE_NEXT_TX;
    A_REG_WR(pDev, PHY_PAPD_PROBE, reg32);

    pDev->pHalInfo->rfgainState = RFGAIN_READ_REQUESTED;
}

/**************************************************************
 * ar5513GetRfgain
 *
 * Exported call to check for a recent gain reading and return
 * the current state of the thermal calibration gain engine.
 */
RFGAIN_STATES
ar5513GetRfgain(WLAN_DEV_INFO *pDev)
{
    GAIN_VALUES *pGainValues = pDev->pHalInfo->pGainValues;
    A_UINT32    reg32, probeType;

    if (!pGainValues->active) {
        return -1;
    }

    if (pDev->pHalInfo->rfgainState == RFGAIN_READ_REQUESTED) {
        /* Caller had asked to setup a new reading. Check it. */
        reg32 = A_REG_RD(pDev, PHY_PAPD_PROBE);

        if (!(reg32 & PHY_PAPD_PROBE_NEXT_TX)) {
            /* bit got cleared, we have a new reading. */
            pGainValues->currGain = (reg32 >> PHY_PAPD_PROBE_GAINF_S);
            probeType = (reg32 & PHY_PAPD_PROBE_TYPE_M) >> PHY_PAPD_PROBE_TYPE_S;
            /* Normalize CCK frames to OFDM power levels for consistent compare */
            if (probeType == PROBE_TYPE_CCK) {
                ASSERT(IS_5112(pDev));
                if (pDev->phyRev >= PHY_CHIP_ID_REV_2) {
                    pGainValues->currGain += pDev->pHalInfo->pEepData->pEepHeader->cckOfdmGainDelta;
                } else {
                    pGainValues->currGain += PHY_PROBE_CCK_CORRECTION;
                }
            }
            if (IS_5112(pDev)) {
                ar5513GetGainFCorrection(pDev, pGainValues);
                if (pGainValues->currGain >= pGainValues->gainFCorrection) {
                    pGainValues->currGain -= pGainValues->gainFCorrection;
                } else {
                    pGainValues->currGain = 0;
                }
            }
#ifdef DEBUG
            if (GainDebug) {
                uiPrintf("New Gain Value: %2d packet type: %d\n",
                    pGainValues->currGain, probeType);
            }
#endif
            pDev->pHalInfo->rfgainState = RFGAIN_INACTIVE; // inactive by default

            if (!ar5513InvalidGainReadback(pDev, pGainValues, pDev->staConfig.pChannel->channelFlags) &&
                ar5513IsGainAdjustNeeded(pGainValues) &&
                ar5513AdjustGain(pDev, pGainValues) > 0)
            {
                /* Change needed. Copy ladder info into eeprom info. */
#ifdef DEBUG
                if (GainDebug) {
                    ar5513dumpGainValues(pDev, pGainValues);
                }
#endif
                pDev->pHalInfo->rfgainState = RFGAIN_NEED_CHANGE;
                /* Request IQ recalibration for temperature change */
                pDev->pHalInfo->iqCalState = IQ_CAL_INACTIVE;
            }
        }
    }

    return pDev->pHalInfo->rfgainState;
}

/**************************************************************
 * ar5513IsGainAdjustNeeded
 *
 * Check to see if our readback gain level sits within the linear
 * region of our current variable attenuation window
 */
A_BOOL
ar5513IsGainAdjustNeeded(GAIN_VALUES *pGainValues)
{
    return ((pGainValues->currGain <= pGainValues->loTrig) ||
            (pGainValues->currGain >= pGainValues->hiTrig));
}

/**************************************************************
 * ar5513AdjustGain
 *
 * Move the rabbit ears in the correct direction.
 */
A_INT32
ar5513AdjustGain(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues)
{
    const static GAIN_OPTIMIZATION_LADDER *pGainLadder;
    if (IS_5112(pDev)) {
        pGainLadder = &GainLadder5112;
    } else {
        pGainLadder = &GainLadder;
    }

    pGainValues->currStep = &(pGainLadder->optStep[pGainValues->currStepNum]);

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
        while ((pGainValues->targetGain >= pGainValues->hiTrig) &&
               (pGainValues->currStepNum > 0))
        {
            pGainValues->targetGain -= 2 * (pGainLadder->optStep[--(pGainValues->currStepNum)].stepGain -
                                       pGainValues->currStep->stepGain);
            pGainValues->currStep    = &(pGainLadder->optStep[pGainValues->currStepNum]);
        }

#ifdef DEBUG
        uiPrintf("targG=%d [%s]\n", pGainValues->targetGain, pGainValues->currStep->stepName);
#endif
        return 1;
    }

    if (pGainValues->currGain <= pGainValues->loTrig) {
        if (pGainValues->currStepNum == (pGainLadder->numStepsInLadder - 1)) {
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
               (pGainValues->currStepNum < (pGainLadder->numStepsInLadder - 1)))
        {
            pGainValues->targetGain -= 2 * (pGainLadder->optStep[++(pGainValues->currStepNum)].stepGain -
                                       pGainValues->currStep->stepGain);
            pGainValues->currStep    = &(pGainLadder->optStep[pGainValues->currStepNum]);
        }

#ifdef DEBUG
        uiPrintf("targG=%d [%s]\n", pGainValues->targetGain, pGainValues->currStep->stepName);
#endif

        return 2;
    }

    return 0;
}

/**************************************************************
 * ar5513GetGainFCorrection
 *
 * Read rf register to determine if gainF needs correction
 */
void
ar5513GetGainFCorrection(WLAN_DEV_INFO *pDev, GAIN_VALUES *pGainValues)
{
    A_UINT32    mixOvr, gainStep, mixGain;

    pGainValues->gainFCorrection = 0;
    mixOvr = ar5513GetRfField(ar5513GetRfBank(pDev, 7), 1, 36, 0);

    if (mixOvr == 1) {
        mixGain = pGainValues->currStep->paramVal[0];
        gainStep = ar5513GetRfField(ar5513GetRfBank(pDev, 7), 4, 32, 0);
        switch (mixGain) {
        case 0 :
            pGainValues->gainFCorrection = 0;
            break;
        case 1 :
            pGainValues->gainFCorrection = gainStep;
            break;
        case 2 :
            pGainValues->gainFCorrection = 2 * gainStep - 5;
            break;
        case 3 :
            pGainValues->gainFCorrection = 2 * gainStep;
            break;
        }
    }
}

/**************************************************************
 * ar5513SetRateDurationTable
 *
 * Sets the rate to duration values in MAC - used for multi-
 * rate retry.
 * The rate duration table needs to cover all valid rate codes;
 * the XR table covers all ofdm and xr rates, while the 11b table
 * covers all cck rates => all valid rates get covered between
 * these two mode's ratetables!
 * But if we're turbo, the ofdm phy is replaced by the turbo phy
 * and xr or cck is not valid with turbo => all rates get covered
 * by the turbo ratetable only
 */
void
ar5513SetRateDurationTable(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    const RATE_TABLE *pRateTable;
    A_UINT32         reg;
    A_UINT16         duration;
    int              i;

    pRateTable = IS_CHAN_TURBO(pChval->channelFlags)
                 ? pDev->hwRateTable[WIRELESS_MODE_TURBO]
                 : pDev->hwRateTable[WIRELESS_MODE_XR];
    for (i = 0; i < pRateTable->rateCount; ++i) {
        reg = MAC_RATE_DURATION_0 + (pRateTable->info[i].rateCode * sizeof(A_UINT32));
        duration = PHY_COMPUTE_TX_TIME(pRateTable, WLAN_CTRL_FRAME_SIZE,
                                pRateTable->info[i].controlRate, FALSE);
        writePlatformReg(pDev, reg, duration);
    }

    if (IS_CHAN_TURBO(pChval->channelFlags)) {
        return;
    }

    pRateTable = pDev->hwRateTable[WIRELESS_MODE_11b];
    for (i = 0; i < pRateTable->rateCount; ++i) {
        reg = MAC_RATE_DURATION_0 + (pRateTable->info[i].rateCode * sizeof(A_UINT32));
        duration = PHY_COMPUTE_TX_TIME(pRateTable, WLAN_CTRL_FRAME_SIZE,
                                pRateTable->info[i].controlRate, FALSE);
        writePlatformReg(pDev, reg, duration);

        /* cck rates have short preamble option also */
        if (pRateTable->info[i].shortPreamble) {
            reg += (pRateTable->info[i].shortPreamble * sizeof(A_UINT32));
            duration = PHY_COMPUTE_TX_TIME(pRateTable, WLAN_CTRL_FRAME_SIZE,
                                        pRateTable->info[i].controlRate, TRUE);
            writePlatformReg(pDev, reg, duration);
        }
    }
}

/******************** EAR Entry Routines ********************/
/**************************************************************
 * ar5513IsEarEngaged
 *
 * Given the reset mode and channel - does the EAR exist and
 * does it have any changes for this mode and channel
 */
static A_BOOL
ar5513IsEarEngaged(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    EAR_HEADER   *pEarHead;
    int          i;

    pEarHead = pDev->pHalInfo->pEarHead;

    /* Check for EAR */
    if (!pEarHead) {
        return FALSE;
    }

    /* Check for EAR found some reg headers */
    if (pEarHead->numRHs == 0) {
        return FALSE;
    }

    /* Check register headers for match on channel and mode */
    for (i = 0; i < pEarHead->numRHs; i++) {
        if (pEarHead->pRH[i].modes & ar5513CFlagsToEarMode(pChval->channelFlags) &&
            ar5513IsChannelInEarRegHead(pEarHead->pRH[i].channel, pChval->channel))
        {
            return TRUE;
        }
    }

    return FALSE;
}

/**************************************************************
 * ar5513EarModify
 *
 * The location sensitive call the can enable/disable calibrations
 * or check for additive register changes
 */
static A_BOOL
ar5513EarModify(WLAN_DEV_INFO *pDev, EAR_LOC_CHECK loc, CHAN_VALUES *pChval, A_UINT32 *modifier)
{
    EAR_HEADER   *pEarHead;
    int          i;

    pEarHead = pDev->pHalInfo->pEarHead;
    ASSERT(pEarHead);
    /* Should only be called if EAR is engaged */
    ASSERT(ar5513IsEarEngaged(pDev, pChval));

    /* Check to see if there are any EAR modifications for the given code location */
    for (i = 0; i < pEarHead->numRHs; i++) {
        /* Check register headers for match on channel and mode */
        if (pEarHead->pRH[i].modes & ar5513CFlagsToEarMode(pChval->channelFlags) &&
            ar5513IsChannelInEarRegHead(pEarHead->pRH[i].channel, pChval->channel))
        {
            switch (loc) {
                /* Stage: 0 */
            case EAR_LC_RF_WRITE:
                if (pEarHead->pRH[i].stage == EAR_STAGE_ANALOG_WRITE) {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Performing stage 0 ops on RH %d\n", i);
                    }
                    ar5513EarDoRH(pDev, &(pEarHead->pRH[i]));
                }
                break;
                /* Stage: 1 */
            case EAR_LC_PHY_ENABLE:
                if (pEarHead->pRH[i].stage == EAR_STAGE_PRE_PHY_ENABLE) {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Performing stage 1 ops on RH %d\n", i);
                    }
                    ar5513EarDoRH(pDev, &(pEarHead->pRH[i]));
                }
                break;
                /* Stage: 2 */
            case EAR_LC_POST_RESET:
                if (pEarHead->pRH[i].stage == EAR_STAGE_POST_RESET) {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Performing stage 2 ops on RH %d\n", i);
                    }
                    ar5513EarDoRH(pDev, &(pEarHead->pRH[i]));
                }
                break;
                /* Stage: 3 */
            case EAR_LC_POST_PER_CAL:
                if (pEarHead->pRH[i].stage == EAR_STAGE_POST_PER_CAL) {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Performing stage 3 ops on RH %d\n", i);
                    }
                    ar5513EarDoRH(pDev, &(pEarHead->pRH[i]));
                }
                break;
                /* Stage: Any */
            case EAR_LC_PLL:
                if (IS_PLL_SET(pEarHead->pRH[i].disabler.disableField)) {
                    *modifier = pEarHead->pRH[i].disabler.pllValue;
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Modifying PLL value!!! to %d\n", *modifier);
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_RESET_OFFSET:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 8) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling reset code Offset cal\n");
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_RESET_NF:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 9) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling reset code Noise Floor cal\n");
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_RESET_IQ:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 10) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling reset code IQ cal\n");
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_PER_FIXED_GAIN:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 11) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling periodic fixed gain calibration\n");
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_PER_NF:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 12) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling periodic noise floor cal\n");
                    }
                    return TRUE;
                }
                break;
                /* Stage: Any */
            case EAR_LC_PER_GAIN_CIRC:
                if ((pEarHead->pRH[i].disabler.valid) &&
                    !((pEarHead->pRH[i].disabler.disableField >> 13) & 0x1))
                {
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarModify: Disabling gain circulation\n");
                    }
                    return TRUE;
                }
                break;
            }
        }
    }
    /* TODO: Track modified analog registers outside of stage 0 and write them with protection */

    return FALSE;
}


/******************** EAR Support Routines ********************/

/**************************************************************
 * ar5513EarDoRH
 *
 * The given register header needs to be executed by performing
 * a register write, modify, or analog bank modify
 *
 * If an ananlog bank is modified - return the bank number as a
 * bit mask.  (bit 0 is bank 0)
 */
static A_UINT16
ar5513EarDoRH(WLAN_DEV_INFO *pDev, REGISTER_HEADER *pRH)
{
    A_UINT16 tag, regLoc = 0, num, opCode;
    A_UINT16 bitsThisWrite, numBits, startBit, bank, column, last, i;
    A_UINT32 address, reg32, mask, regInit;
    A_UINT32 *rfBuf;
    A_UINT16 rfBankMask = 0;

    switch (pRH->type) {
    case EAR_TYPE0:
        do {
            address = pRH->regs[regLoc] & ~T0_TAG_M;
            tag = pRH->regs[regLoc] & T0_TAG_M;
            regLoc++;
            if ((tag == T0_TAG_32BIT) || (tag == T0_TAG_32BIT_LAST)) {
                /* Full word writes */
                reg32 = pRH->regs[regLoc++] << 16;
                reg32 |= pRH->regs[regLoc++];
                if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                    uiPrintf("ar5513EarDoRH: Type 0 EAR 32-bit write to 0x%04X : 0x%08x\n", address, reg32);
                }
                A_REG_WR(pDev, address, reg32);
            } else if (tag == T0_TAG_16BIT_LOW) {
                /* Half word read/modify/write low data */
                if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                    uiPrintf("ar5513EarDoRH: Type 0 EAR 16-bit low write to 0x%04X : 0x%04x\n",
                             address, pRH->regs[regLoc]);
                }
                A_REG_RMW(pDev, address, pRH->regs[regLoc++], 0xFFFF);
            } else {
                /* Half word read/modify/write high data */
                if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                    uiPrintf("ar5513EarDoRH: Type 0 EAR 16-bit high write to 0x%04X : 0x%04x\n",
                             address, pRH->regs[regLoc]);
                }
                A_REG_RMW(pDev, address, pRH->regs[regLoc++] << 16, 0xFFFF0000);
            }
        } while (tag != T0_TAG_32BIT_LAST);
        break;
    case EAR_TYPE1:
        address = pRH->regs[regLoc] & ~T1_NUM_M;
        num = pRH->regs[regLoc] & T1_NUM_M;
        regLoc++;
        for (i = 0; i < num + 1; i++) {
            /* Sequential Full word writes */
            reg32 = pRH->regs[regLoc++] << 16;
            reg32 |= pRH->regs[regLoc++];
            if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                uiPrintf("ar5513EarDoRH: Type 1 EAR 32-bit write to 0x%04X : 0x%08x\n", address, reg32);
            }
            A_REG_WR(pDev, address, reg32);
            address += sizeof(A_UINT32);
        }
        break;
    case EAR_TYPE2:
        do {
            last = IS_TYPE2_LAST(pRH->regs[regLoc]);
            bank = (pRH->regs[regLoc] & T2_BANK_M) >> T2_BANK_S;
            /* Record which bank is written */
            rfBankMask |= 1 << bank;
            column = (pRH->regs[regLoc] & T2_COL_M) >> T2_COL_S;
            startBit = pRH->regs[regLoc] & T2_START_M;
            rfBuf = ar5513GetRfBank(pDev, bank);
            if (IS_TYPE2_EXTENDED(pRH->regs[regLoc++])) {
                num = A_DIV_UP(pRH->regs[regLoc], 16);
                numBits = pRH->regs[regLoc];
                regLoc++;
                for (i = 0; i < num; i++) {
                    /* Modify the bank 16 bits at a time */
                    bitsThisWrite = (numBits > 16) ? 16 : numBits;
                    numBits -= bitsThisWrite;
                    reg32 = pRH->regs[regLoc++];
                    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                        uiPrintf("ar5513EarDoRH: Type 2 EAR E analog modify bank %d, startBit %d, column %d numBits %d data %d\n",
                                 bank, startBit, column, bitsThisWrite, reg32);
                    }
                    ar5513ModifyRfBuffer(rfBuf, reg32, bitsThisWrite, startBit, column);
                    startBit += bitsThisWrite;
                }
            } else {
                numBits = (pRH->regs[regLoc] & T2_NUMB_M) >> T2_NUMB_S;
                reg32   = pRH->regs[regLoc++] & T2_DATA_M;
                if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                    uiPrintf("ar5513EarDoRH: Type 2 EAR analog modify bank %d, startBit %d, column %d numBits %d data %d\n",
                             bank, startBit, column, numBits, reg32);
                }
                ar5513ModifyRfBuffer(rfBuf, reg32, numBits, startBit, column);
            }
        } while (!last);
        break;
    case EAR_TYPE3:
        do {
            opCode = (pRH->regs[regLoc] & T3_OPCODE_M) >> T3_OPCODE_S;
            last = IS_TYPE3_LAST(pRH->regs[regLoc]);
            startBit = (pRH->regs[regLoc] & T3_START_M) >> T3_START_S;
            numBits = pRH->regs[regLoc] & T3_NUMB_M;
            regLoc++;
            /* Grab Address */
            address = pRH->regs[regLoc++];
            if (numBits > 16) {
                reg32 = pRH->regs[regLoc++] << 16;
                reg32 |= pRH->regs[regLoc++];
            } else {
                reg32 = pRH->regs[regLoc++];
            }
            mask = ((1 << numBits) - 1) << startBit;
            regInit = A_REG_RD(pDev, address);
            switch (opCode) {
            case T3_OC_REPLACE:
                reg32 = (regInit & ~mask) | ((reg32 << startBit) & mask);
                break;
            case T3_OC_ADD:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) + (reg32 << startBit)) & mask);
                break;
            case T3_OC_SUB:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) - (reg32 << startBit)) & mask);
                break;
            case T3_OC_MUL:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) * (reg32 << startBit)) & mask);
                break;
            case T3_OC_XOR:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) ^ (reg32 << startBit)) & mask);
                break;
            case T3_OC_OR:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) | (reg32 << startBit)) & mask);
                break;
            case T3_OC_AND:
                reg32 = (regInit & ~mask) |
                    (((regInit & mask) & (reg32 << startBit)) & mask);
                break;
            }
            A_REG_WR(pDev, address, reg32);
            if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
                uiPrintf("ar5513EarDoRH: Type 3 EAR 32-bit rmw to 0x%04X : 0x%08x\n", address, reg32);
            }
        } while (!last);
        break;
    }
    return rfBankMask;
}

/**************************************************************
 * ar5513GetRfBank
 *
 * Get the UINT16 pointer to the start of the requested RF Bank
 */
static A_UINT32 *
ar5513GetRfBank(WLAN_DEV_INFO *pDev, A_UINT16 bank)
{
    AR5513_RF_BANKS_5112 *pRfBank5112;
    A_UINT32             *pRf;

    if (IS_5112(pDev)) {
        pRfBank5112 = pDev->pHalInfo->pAnalogBanks;
        ASSERT(pRfBank5112);
        switch (bank) {
        case 1:
            pRf = pRfBank5112->Bank1Data;
            break;
        case 2:
            pRf = pRfBank5112->Bank2Data;
            break;
        case 3:
            pRf = pRfBank5112->Bank3Data;
            break;
        case 6:
            pRf = pRfBank5112->Bank6Data;
            break;
        case 7:
            pRf = pRfBank5112->Bank7Data;
            break;
        default:
            pRf = NULL;
            uiPrintf("ar5513GetRfBank: Unknown RF Bank %d requested\n", bank);
            break;
        }
    } 
    else {
	pRf = NULL;
    }

    return pRf;
}

/**************************************************************
 * ar5513CFlagsToEarMode
 *
 * Convert from WLAN_CFLAGS to the EAR bitmap format
 */
static A_UINT16
ar5513CFlagsToEarMode(WLAN_CFLAGS cflags)
{
    if (IS_CHAN_B(cflags)) {
        return EAR_11B;
    }
    if (IS_CHAN_G(cflags)) {
        return EAR_11G;
    }
    if (IS_CHAN_108G(cflags)) {
        return EAR_2T;
    }
    if (IS_CHAN_T(cflags)) {
        return EAR_5T;
    }

    if (IS_CHAN_XR_A(cflags)) {
        return EAR_11A | EAR_XR;
    }
    if (IS_CHAN_XR_G(cflags)) {
        return EAR_11G | EAR_XR;
    }
    if (IS_CHAN_XR_T(cflags)) {
        return EAR_5T | EAR_XR;
    }

    return EAR_11A;
}

/**************************************************************
 * ar5513IsChannelInEarRegHead
 *
 * Convert from WLAN_CFLAGS to the EAR bitmap format
 */
static A_BOOL
ar5513IsChannelInEarRegHead(A_UINT16 earChannel, A_UINT16 channelMhz)
{
    int i;
    const A_UINT16 earFlaggedChannels[14][2] = {
        {2412, 2412}, {2417, 2437}, {2442, 2457}, {2462, 2467}, {2472, 2472}, {2484, 2484}, {2300, 2407},
        {4900, 5160}, {5170, 5180}, {5190, 5250}, {5260, 5300}, {5310, 5320}, {5500, 5700}, {5725, 5825}
    };

    /* No channel specified - applies to all channels */
    if (earChannel == 0) {
        return TRUE;
    }

    /* Do we match the single channel check if its set? */
    if (IS_CM_SINGLE(earChannel)) {
        return ((earChannel & ~0x8000) == channelMhz) ? TRUE : FALSE;
    }

    /* Rest of the channel modifier bits can be set as one or more ranges - check them all */
    if ((IS_CM_SPUR(earChannel)) && (IS_SPUR_CHAN(channelMhz))) {
        return TRUE;
    }
    for (i = 0; i < 14; i++) {
        if ((earChannel >> i) & 1) {
            if ((earFlaggedChannels[0][i] <= channelMhz) && (earFlaggedChannels[1][i] >= channelMhz)) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

/**************************************************************
 * ar5513AdjustPhaseRamp
 *
 * Compute Phase Ramp between Radio chains 0 & 1. The EEPROM
 * supplies two points of phase ramp for 11g; one at 2412 and the other 
 * at 2472 MHz. 11a is supplied with five phase ramp values at 
 * 5000, 5200, 5400, 5600, and 5800 MHz. If the desired channels falls
 * in between any of these freqencies then the phase ramp is interpolated 
 * between the two closest phase cal points supplied.
 */

static A_UINT32
ar5513AdjustPhaseRamp(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    A_INT32 freq;
    A_INT32 phaseCal0, phaseCal1, freqDiff0, freqDiff1;
    A_UINT32 phaseRampInit0;
    A_INT32 phase = 0;
    EEP_HEADER_INFO *pEepHdr = pDev->pHalInfo->pEepData->pEepHeader;
    A_UINT32 reg;

#ifndef PHASE_CAL_UNITS
#define PHASE_CAL_UNITS 10
#endif /* PHASE_CAL_UNITS */

    phaseCal0 = phaseCal1 = freqDiff0 = freqDiff1 = 0;

    /*
     * TODO: Currently did not change eeprom version for the
     * AR5513 E2.0 2.4 GHz synth workaround, if version changes
     * in future, will need to add a check for eeprom version.
     */
    if (pDev->pHalInfo->pEepData->version != EEPROM_VER4_9) {
        return 0;
    }

    /*
    ** Determine correct phase ramp angle based on calibration data and
    ** desired channel
    */

    if (IS_CHAN_2GHZ(pChval->channelFlags)) {
        A_INT32 offsetCal0, offsetCal1, offset;

        freq = pChval->channel;

        freqDiff0 = freq - 2412;
        freqDiff1 = 2472 - freq;

        /*
         * Workaround for AR5513 E2.0 2.4 GHz synth problem.
         * Cal values in eeprom are actually offsets not phases.
         * phase is calculated as such:
         *    phase_ramp = -2 * rx_synth + offsetCal
         *        rx_synth is filled in by ar5513UpdateSynth2_4War()
         *        after a valid BF coef entry is found.
         */

        offsetCal0 = pEepHdr->phaseCal11g[0] * PHASE_CAL_UNITS;
        offsetCal1 = pEepHdr->phaseCal11g[1] * PHASE_CAL_UNITS;

#ifndef NDIS_HW
        if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
            /*
             * Indicate synth cal war trigger.
             * Only print if WAR_SHOWPHASECAL is set.
             * Over time, as more history is gathered with this
             * workaround, these prints can become DEBUG build only.
             */
            uiPrintf("+ar5513AdjustPhaseRamp @2.4: E %d, %d; ",
                        offsetCal0, offsetCal1);
        }
#endif
        /*
         * Interpolate offset value for this frequency.
         * Negative slope across freq is NOT assumed.
         * It is assumed that offset values are less than 180 deg
         * apart and are saved in eeprom as positive values.
         * Compensate for 360 deg offset wrap between interpolation
         * points first.
         */
        if (offsetCal1 > offsetCal0 + 180) {
            offsetCal1 += 360;
        } else if (offsetCal0 > offsetCal1 + 180) {
            offsetCal0 += 360;
        }
        offset = (((freqDiff1 * offsetCal0) + (freqDiff0 * offsetCal1)) / 
                  (freqDiff0 + freqDiff1)) % 360;

        /* calculate phase cal from offset, normalize to positive value */
        phase = (offset - 2 * pDev->pHalInfo->synth_state_2_4) % 360;
        if (phase < 0) {
            phase += 360;
        }

#ifndef NDIS_HW
        if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
            /*
             * Indicate synth cal war trigger.
             * Only print if WAR_SHOWPHASECAL is set.
             */
            uiPrintf("C %d, %d; P %d, %d; p = %d\n",
                            offsetCal0,
                            offsetCal1,
                            (720 + offsetCal0 - 2 * pDev->pHalInfo->synth_state_2_4) % 360,
                            (720 + offsetCal1 - 2 * pDev->pHalInfo->synth_state_2_4) % 360,
                            phase);
        }
#endif
    }

    if (IS_CHAN_5GHZ(pChval->channelFlags)) {
        freq = pChval->channel;
    
        if (freq <= 5000) {
            phase = pEepHdr->phaseCal11a[0] * PHASE_CAL_UNITS;
        } else if (freq >= 5800) {
            phase = pEepHdr->phaseCal11a[4] * PHASE_CAL_UNITS;
        } else {
            if (freq > 5400) {
                if (freq > 5600) {
                    freqDiff0 = freq - 5600;
                    phaseCal0 = pEepHdr->phaseCal11a[3] * PHASE_CAL_UNITS;
                    freqDiff1 = 5800 - freq;
                    phaseCal1 = pEepHdr->phaseCal11a[4] * PHASE_CAL_UNITS;
                } else {
                    freqDiff0 = freq - 5400;
                    phaseCal0 = pEepHdr->phaseCal11a[2] * PHASE_CAL_UNITS;
                    freqDiff1 = 5600 - freq;
                    phaseCal1 = pEepHdr->phaseCal11a[3] * PHASE_CAL_UNITS;
                }
            } else if (freq > 5200) {
                freqDiff0 = freq - 5200;
                phaseCal0 = pEepHdr->phaseCal11a[1] * PHASE_CAL_UNITS;
                freqDiff1 = 5400 - freq;
                phaseCal1 = pEepHdr->phaseCal11a[2] * PHASE_CAL_UNITS;
            } else {
                freqDiff0 = freq - 5000;
                phaseCal0 = pEepHdr->phaseCal11a[0] * PHASE_CAL_UNITS;
                freqDiff1 = 5200 - freq;
                phaseCal1 = pEepHdr->phaseCal11a[1] * PHASE_CAL_UNITS;
            }
            /* Check for phase wrap between interpolation points first,
             * then interpolate.  Assumes negative slope of phase across freq.
             */
            if (phaseCal1 > phaseCal0) {
                phaseCal0 += 360;  
            } 
            phase = ((freqDiff1 * phaseCal0) + (freqDiff0 * phaseCal1)) / 
                    (freqDiff0 + freqDiff1);
        }
    }

    /* Bounds check on phase result, wrap to between 0 and 360 degrees */
    if (phase > 360) {
        phase -= 360;
    } else if (phase < 0) {
        phase += 360;
    }

    /*
    ** Convert to phaseRampInit0 register units
    */
    phaseRampInit0 = (phase * 511) / 720;

#ifdef DEBUG
    if (printPhaseCal) {
        uiPrintf("Phase cal 11g: %d, %d\n", pEepHdr->phaseCal11g[0], 
                pEepHdr->phaseCal11g[1]);
        uiPrintf("Phase cal 11a: %d  %d  %d  %d  %d\n", pEepHdr->phaseCal11a[0],
                pEepHdr->phaseCal11a[1], pEepHdr->phaseCal11a[2], 
                pEepHdr->phaseCal11a[3], pEepHdr->phaseCal11a[4]);
        uiPrintf("Freq diff 0/1: %d / %d, phaseCal 0/1: %d / %d \n",
                freqDiff0, freqDiff1, phaseCal0, phaseCal1);
        uiPrintf("Final phase cal result: %d \n", phase);

        uiPrintf("phaseRampInit0 (phase*511/720): %d, reg value: 0x%x\n",
                phaseRampInit0, (phaseRampInit0 << PHY_PHASE_RAMP_INIT0_S) );
    }
#endif

#ifndef NDIS_HW
    if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
        /*
         * Indicate synth cal war trigger.
         * Only print if WAR_SHOWPHASECAL is set.
         */
        uiPrintf("+ar5513AdjustPhaseRamp phaseRampInit0 %d\n", phaseRampInit0);
    }
#endif

    reg = (phaseRampInit0 << PHY_PHASE_RAMP_INIT0_S) | PHY_PHASE_RAMP_ENB_M;
    A_REG_WR(pDev,PHY_CHN0_PHASE_ANT0,reg);
    A_REG_WR(pDev,PHY_CHN0_PHASE_ANT1,reg);

    A_REG_WR(pDev,PHY_CHN1_PHASE_ANT0,PHY_PHASE_RAMP_ENB_M);
    A_REG_WR(pDev,PHY_CHN1_PHASE_ANT1,PHY_PHASE_RAMP_ENB_M);

    return (phaseRampInit0 << PHY_PHASE_RAMP_INIT0_S);
}

/******************************************************/
#ifdef DEBUG
/* TURBO_PRIME */
void dumpPrime(WLAN_DEV_INFO *pDev)
{
    uiPrintf("on thresh     = %x\n", pDev->turboPrimeInfo.upThresh);
    uiPrintf("off thresh    = %x\n", pDev->turboPrimeInfo.dnThresh);
    uiPrintf("In time       = %x\n", pDev->turboPrimeInfo.periodIn);
    uiPrintf("Hold time     = %x\n", pDev->turboPrimeInfo.periodHold);
    uiPrintf("Out time      = %x\n", pDev->turboPrimeInfo.periodOut);
    uiPrintf("Allowed       = %x\n", pDev->turboPrimeInfo.turboPrimeAllowed);
    uiPrintf("Legacy        = %x\n", pDev->turboPrimeInfo.legacyPresent);
    uiPrintf("Rate Recn     = %x\n", pDev->turboPrimeInfo.rateRecn);
    uiPrintf("channel flags = %x\n", pDev->staConfig.pChannel->channelFlags);
    uiPrintf("channel       = %d\n", pDev->staConfig.pChannel->channel);
    uiPrintf("boost         = %d\n", pDev->bssDescr->athAdvCapElement.info.boost);
    uiPrintf("MAC_IMR_S0    = %x\n", (unsigned int)readPlatformReg(pDev,MAC_IMR_S0));
    uiPrintf("txDescIntMask = %x\n", pDev->pHalInfo->txDescIntMask);
    uiPrintf("PHY_TURBO     = %x\n", (unsigned int)readPlatformReg(pDev, PHY_TURBO));
    uiPrintf("PHY_MODE      = %x\n", (unsigned int)readPlatformReg(pDev, PHY_MODE));
    uiPrintf("PHY_PLL_CTL   = %x\n", (unsigned int)readPlatformReg(pDev, PHY_PLL_CTL));
}

void dumpXrReg(WLAN_DEV_INFO *pDev)
{
    uiPrintf("STA_ID0   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_STA_ID0));
    uiPrintf("STA_ID1   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_STA_ID1));
    uiPrintf("BSS_ID0   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_BSS_ID0));
    uiPrintf("BSS_ID1   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_BSS_ID1));
    uiPrintf("BSS_ID_MASK0   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_BSS_ID_MASK0));
    uiPrintf("BSS_ID_MASK1   = %x\n", (unsigned int)readPlatformReg(pDev, MAC_BSS_ID_MASK1));
    uiPrintf("pDev Mask value = %x\n", pDev->MaskReg);
    uiPrintf("Mask Register Value = %x\n", (unsigned int)readPlatformReg(pDev, MAC_IMR));
    uiPrintf("Extended Range Chirp Value = %x\n", (unsigned int)readPlatformReg(pDev, MAC_XRCRP));
    uiPrintf("ISR Value = %x\n", (unsigned int)readPlatformReg(pDev, MAC_ISR));
    uiPrintf("Phy error = %x\n", (unsigned int) readPlatformReg(pDev, MAC_PHY_ERR));
    uiPrintf("RXCFG value = %x\n", (unsigned int) readPlatformReg(pDev, MAC_RXCFG));
}

#endif

/******************************************************/
void ar5513RxRegDump(WLAN_DEV_INFO *pDev)
{
    uiPrintf("RXDP 0x%08x \t Command Reg 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_RXDP),
             (unsigned int)readPlatformReg(pDev, MAC_CR));
} /* ar5513RxRegDump */

/******************************************************/
void
ar5513TxRegDumpHelper(WLAN_DEV_INFO *pDev)
{
    uiPrintf("/** Tx Reg Dump ********************************** \n");
    uiPrintf(
        "Q_TXD 0x%08x \t\t Q_TXE 0x%08x\n",
        (unsigned int)readPlatformReg(pDev, MAC_Q_TXD),
        (unsigned int)readPlatformReg(pDev, MAC_Q_TXE));

    uiPrintf(
        "Q_ONESHOTARM_SC 0x%08x \t\t Q)ONESHOTARM_CC 0x%08x\n",
        (unsigned int)readPlatformReg(pDev, MAC_Q_ONESHOTARM_SC),
        (unsigned int)readPlatformReg(pDev, MAC_Q_ONESHOTARM_CC));

    uiPrintf(
        "Q_RDYTIMESHDN 0x%08x\n",
        (unsigned int)readPlatformReg(pDev, MAC_Q_RDYTIMESHDN));

    uiPrintf("Queue \t Q_TXDP \t Q_STS\t \t Q_CBRCFG \t Q_MISC \t Q_RDYTIMECFG\t\n");
    uiPrintf("0 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q0_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q0_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q0_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q0_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q0_RDYTIMECFG));

    uiPrintf("1 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q1_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q1_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q1_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q1_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q1_RDYTIMECFG));

    uiPrintf("2 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q2_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q2_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q2_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q2_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q2_RDYTIMECFG));

    uiPrintf("3 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q3_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q3_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q3_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q3_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q3_RDYTIMECFG));

    uiPrintf("4 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q4_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q4_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q4_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q4_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q4_RDYTIMECFG));

    uiPrintf("5 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q5_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q5_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q5_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q5_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q5_RDYTIMECFG));

    uiPrintf("6 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q6_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q6_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q6_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q6_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q6_RDYTIMECFG));

    uiPrintf("7 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q7_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q7_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q7_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q7_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q7_RDYTIMECFG));

    uiPrintf("8 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q8_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q8_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q8_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q8_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q8_RDYTIMECFG));

    uiPrintf("9 \t 0x%08x \t 0x%08x\t 0x%08x \t 0x%08x \t 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, MAC_Q9_TXDP),
             (unsigned int)readPlatformReg(pDev, MAC_Q9_STS),
             (unsigned int)readPlatformReg(pDev, MAC_Q9_CBRCFG),
             (unsigned int)readPlatformReg(pDev, MAC_Q9_MISC),
             (unsigned int)readPlatformReg(pDev, MAC_Q9_RDYTIMECFG));
}

DEFINE_DEBUG_DUMP_FN(ar5513TxRegDump, "TX Register");

/******************************************************/
void
ar5513DmaRegDumpHelper(WLAN_DEV_INFO *pDev)
{
    uiPrintf("DMA Debug Reg 0 0x%08x \t Reg 1 0x%08x \t Reg 2 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, 0x00e0),
             (unsigned int)readPlatformReg(pDev, 0x00e4),
             (unsigned int)readPlatformReg(pDev, 0x00e8));

    uiPrintf("Reg 3 0x%08x \t Reg 4 0x%08x \t Reg 5 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, 0x00ec),
             (unsigned int)readPlatformReg(pDev, 0x00f0),
             (unsigned int)readPlatformReg(pDev, 0x00f4));

    uiPrintf("Reg 6 0x%08x \t Reg 7 0x%08x\n",
             (unsigned int)readPlatformReg(pDev, 0x00f8),
             (unsigned int)readPlatformReg(pDev, 0x00fc));
}

DEFINE_DEBUG_DUMP_FN(ar5513DmaRegDump, "DMA Register");


/*
 *  NF calibration log dump
 */
#define LOG_NF_CAL_SIZE 1000

#if LOG_NF_CAL_SIZE > 0 && defined (BUILD_AP)

int
nfRegToInt(A_UINT32 reg)
{
    int n;

    n = (int) (reg >> 19) & 0x1ff;
    if (n & 0x100) {
        n = 0 - ((n ^ 0x1ff) + 1);
    }
    return n;
}
typedef struct log_nfcal_entry {
    A_UINT32    var0;
    A_UINT32    var1;
    A_UINT32    var2;
    A_UINT32    var3;
    A_UINT32    var4;
} LOG_NFCAL_ENTRY;

LOG_NFCAL_ENTRY logNfCal_Buf[LOG_NF_CAL_SIZE];

int logNfCal_Idx = 0;
int logNfCal_Cnt = 0;
int logNfCal_Enable = 1;

void
logNfCal_Add(WLAN_DEV_INFO *pDev, int flag)
{
    int lock;

    if (logNfCal_Enable) {
        lock = intLock();
        if (logNfCal_Idx < LOG_NF_CAL_SIZE) {

            logNfCal_Buf[logNfCal_Idx].var0 = A_MS_TICKGET();
            logNfCal_Buf[logNfCal_Idx].var1 = nfRegToInt(readPlatformReg(pDev, 0x9864));
            logNfCal_Buf[logNfCal_Idx].var2 = nfRegToInt(readPlatformReg(pDev, 0xa864));
            logNfCal_Buf[logNfCal_Idx].var3 = readPlatformReg(pDev, 0x9930);
            logNfCal_Buf[logNfCal_Idx].var4 = (flag << 28)  |
                ((pDev->pHalInfo->rfgainState & 0x0f) << 8) |
                (pDev->hwResetCount > 0xff ? 0xff : pDev->hwResetCount);

            logNfCal_Cnt++;
            logNfCal_Idx++;
            if (logNfCal_Idx >= LOG_NF_CAL_SIZE) {
                logNfCal_Idx = 0;
            }
        }
        intUnlock(lock);
    }
}

void
logNfCal_Dump(int d)
{
    int i, j, k;
    int count, current;
    int lock;
    LOG_NFCAL_ENTRY *scratch;

    scratch = (LOG_NFCAL_ENTRY *) calloc(LOG_NF_CAL_SIZE, sizeof(LOG_NFCAL_ENTRY));
    if (scratch == NULL) {
        uiPrintf("Could allocate scratch buffer for Dump\n");
        return;
    }

    lock = intLock();
    if (d == -1) {
        /* clear log */
        logNfCal_Idx = 0;
        logNfCal_Cnt = 0;
    }

    current = logNfCal_Idx;
    count   = logNfCal_Cnt;
    bcopy((char *)logNfCal_Buf, (char *)scratch, sizeof(logNfCal_Buf));
    intUnlock(lock);

    uiPrintf("AR5513 NF CAL LOG: %u events\n", count);
    if (count <= LOG_NF_CAL_SIZE) {
        i = 0;
        k = count;
    } else {
        i = current;
        k = LOG_NF_CAL_SIZE;
    }
    for (j = 0; j < k; j++) {
        uiPrintf("%5d: %10u %4d %4d %8.8X %8.8X\n",
                    i,
                    (scratch + i)->var0,
                    (scratch + i)->var1,
                    (scratch + i)->var2,
                    (scratch + i)->var3,
                    (scratch + i)->var4);
        i++;
        if (i >= LOG_NF_CAL_SIZE) {
            i = 0;
        }
    }

    free(scratch);
}
#else
void
logNfCal_Add(WLAN_DEV_INFO *pDev, int flag)
{
}

void
logNfCal_Dump(int d)
{
    uiPrintf("\nNF Cal Log not present\n");
}
#endif


#ifndef NDIS_HW /* TODO: change to runtime check for tx dual chain */
/*
 * Workaround for AR5513 2.4 GHZ synth state problem for Beamforming.
 *
 * The BF aging table is checked for a valid entry after each Tx completion.
 * When a valid entry is found, rx_synth phase is calculated by a weighted,
 * average over 46 of the entries, 3 entries on either side of DC are skipped.
 * The 11g phasecal values in eeprom are actually offset values and the
 * phasecal is determined as 
 *         phasecal = (rx_synth * -2) + offfset
 */
//#define TEMP_USE_FLOAT 1    /* Define for debug, algorithm validation, and logging */

#define SYNTH_PHASE_THRESH 20

#define GPIO_LNA_SHUNT      GPIO_SYNTHCAL
#define LNA_SHUNT_ON(_r)    (_r |  (GPIO_CR_M(GPIO_LNA_SHUNT)))
#define LNA_SHUNT_OFF(_r)   (_r & ~(GPIO_CR_M(GPIO_LNA_SHUNT)))

#ifdef TEMP_USE_FLOAT
#include "math.h"
#endif

extern int sysCountGet(void);
void dumpsynthwar(int f);
void logsynthwar(WLAN_DEV_INFO *pDev, int s);

/*
 * 0 is chain 0 diagnostic log
 * 1 is chain 0 capture
 * 2 is chain 1 diagnostic log
 * 3 is chain 1 capture
 *
 * not optimized in that it grabs the entire cache for diagnostic purposes.
 */
LOCAL A_UINT32 swCopyOfBfCache[4][216];

/*
 * 0 is save last entry
 * 1 is check current entry
 * 2 is for diagnostic logging
 */
LOCAL struct log_debugBfSynth {
    A_UINT32 tab[8];
} logBfSynth[3];

void logsynthbc(WLAN_DEV_INFO *pDev, A_UINT32 *, int l);
void logsynthbcB(WLAN_DEV_INFO *pDev, A_UINT32 *, int l);

LOCAL A_UINT32 txTriggerCount    = 0;
LOCAL A_UINT32 txTriggerValCount = 0;
LOCAL A_UINT32 txTriggerBadCount = 0;
LOCAL A_UINT32 txTriggerThrCount = 0;
LOCAL A_UINT32 txNumTriggers     = 0;
LOCAL int      saveValidIdx      = -1;

int
sgn(unsigned int num)
{
    int ret;
    ret = (num & 0x80) ? (num & 0x7f) - 0x80 : (num & 0x7f);
    return ret;
}

#ifdef TEMP_USE_FLOAT
float
getMag(int i, int q)
{
    return ((float) sqrt((double)i * (double)i + (double)q * (double)q));
}

float
getPhase(int i, int q)
{
    double p;
    p = 180.0 * atan2((double)q, (double)i) / 3.14159;
    if (p < 0.0) {
        p += 360.0;
    }
    return ((float)p);
}

float
getPhaseDiff(float p0, float p1)
{
    float diff;

    diff = p0 - p1;
    if (diff < 0.0) {
        diff += 360.0;
    }
    return (diff);
}
#endif /* TEMP_USE_FLOAT */

/*
 * Determine phase angle from I,Q pair without floating point.
 * Returns positive integer value:  0 <= value < 360.
 */
static int atanlut[] = {
 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7,
 7, 8, 8, 8, 9, 9,10,10,11,11,12,12,12,13,13,14,
14,15,15,15,16,16,17,17,17,18,18,19,19,19,20,20,
21,21,21,22,22,23,23,23,24,24,24,25,25,26,26,26,
27,27,27,28,28,28,29,29,29,30,30,30,31,31,31,32,
32,32,33,33,33,34,34,34,35,35,35,35,36,36,36,37,
37,37,37,38,38,38,39,39,39,39,40,40,40,40,41,41,
41,41,42,42,42,42,43,43,43,43,44,44,44,44,45,45,
45};

int
getPhaseLookup(int i, int q)
{
    int mask = 0;
    int retval = -1;
    int ratio = 0;

    if (i == 0 && q == 0) {
        return -1;
    }

    /* bit mask: 2,1,0 == |Q| > |I|, Q < 0, I < 0 */
    if (i < 0) {
        i = -i;
        mask |= 1;
    }

    if (q < 0) {
        q = -q;
        mask |= 2;
    }

    if (q > i) {
        mask |= 4;
        i <<= 7;
    } else {
        q <<= 7;
    }

    switch (mask) {
    case 0:
        /* first octant */
        ratio = q / i;
        retval = atanlut[ratio];
        break;
    case 4:
        /* second octant */
        ratio = i / q;
        retval = 90 - atanlut[ratio];
        break;
    case 5:
        /* third octant */
        ratio = i / q;
        retval = 90 + atanlut[ratio];
        break;
    case 1:
        /* fourth octant */
        ratio = q / i;
        retval = 180 - atanlut[ratio];
        break;
    case 3:
        /* fifth octant */
        ratio = q / i;
        retval = 180 + atanlut[ratio];
        break;
    case 7:
        /* sixth octant */
        ratio = i / q;
        retval = 270 - atanlut[ratio];
        break;
    case 6:
        /* seventh octant */
        ratio = i / q;
        retval = 270 + atanlut[ratio];
        break;
    case 2:
        /* eighth octant */
        ratio = q / i;
        retval = 360 - atanlut[ratio];
        if (retval == 360) {
            retval = 0;
        }
        break;
    default:
        ASSERT(0);
        break;
    }

    return retval;
}

/*
 * Process coef cache to calculate and return rx_synth phase diff.
 * Also returns via reference upper and lower values.
 * Uses fixed point lookup routines.
 */
int
getWeightedAveragePhase(A_UINT32 *pCoefChain0, A_UINT32 *pCoefChain1,
                        int *pLower, int *pUpper)
{
    int ii, jj;
    int x00, y00, x01, y01, x10, y10, x11, y11;
    int xl_rem, yl_rem, xh_rem, yh_rem;
    int x_sum, y_sum;
    int x_sum_raw, y_sum_raw;   /* for algorithm validation */
    int x_sumL, y_sumL, x_sumU, y_sumU;
    int phase, phaseU, phaseL;

    x_sum = y_sum = 0;
    x_sum_raw = y_sum_raw = 0;
    x_sumL = y_sumL = x_sumU = y_sumU = 0;

    /* Start weighted Average loop */
    jj = 0;
    for (ii = 0; ii < 27; ii++) {
        x00 = sgn((pCoefChain0[ii] >>  8) & 0xff);
        y00 = sgn((pCoefChain0[ii])       & 0xff);
        x01 = sgn((pCoefChain0[ii] >> 24) & 0xff);
        y01 = sgn((pCoefChain0[ii] >> 16) & 0xff);

        x10 = sgn((pCoefChain1[ii] >>  8) & 0xff);
        y10 = sgn((pCoefChain1[ii])       & 0xff);
        x11 = sgn((pCoefChain1[ii] >> 24) & 0xff);
        y11 = sgn((pCoefChain1[ii] >> 16) & 0xff);

        xl_rem = x00 * x10 + y00 * y10;
        yl_rem = y00 * x10 - x00 * y10;

        xh_rem = x01 * x11 + y01 * y11;
        yh_rem = y01 * x11 - x01 * y11;

        /*
         * Process even entries.
         * Skip entry 26 as it is a scale word.
         * Also skip 3 entries either side of DC.
         */
        if (jj != 26) {
            if (jj != 24 && jj != 28) {
                x_sum += xl_rem;
                y_sum += yl_rem;
                if(jj < 26) {
                    x_sumL += xl_rem;
                    y_sumL += yl_rem;
                } else if (jj > 26) {
                    x_sumU += xl_rem;
                    y_sumU += yl_rem;
                }
            }
            x_sum_raw += xl_rem;
            y_sum_raw += yl_rem;
        }

        jj++;

        /*
         * Process odd entries.
         * Skip entry 53 as it is a scale word
         * Also skip 3 entries either side of DC.
         */
        if (jj != 53) {
            if (jj != 23 && jj != 25 && jj != 27 && jj != 29) {
                x_sum += xh_rem;
                y_sum += yh_rem;
                if(jj < 26) {
                    x_sumL += xh_rem;
                    y_sumL += yh_rem;
                } else if (jj > 26) {
                    x_sumU += xh_rem;
                    y_sumU += yh_rem;
                }
            }
            x_sum_raw += xh_rem;
            y_sum_raw += yh_rem;
        }

        jj++;
    }

    phase  = getPhaseLookup(x_sum, y_sum);
    phaseL = getPhaseLookup(x_sumL, y_sumL);
    phaseU = getPhaseLookup(x_sumU, y_sumU);

    if (pLower) {
        *pLower = phaseL;
    }
    if (pUpper) {
        *pUpper = phaseU;
    }

    if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
        /*
         * Indicate synth cal war trigger.
         * Only print if WAR_SHOWPHASECAL is set.
         */
        uiPrintf("\n");
#ifdef TEMP_USE_FLOAT
        uiPrintf("+Raw    Avg:      %6.1f\n",
                                    getPhase(x_sum_raw, y_sum_raw));
        uiPrintf("+Weight Avg: %4d %6.1f\n",
                                    phase,
                                    getPhase(x_sum, y_sum));
        uiPrintf("+Lower  Avg: %4d %6.1f\n",
                                    phaseL,
                                    getPhase(x_sumL, y_sumL));
        uiPrintf("+Upper  Avg: %4d %6.1f\n",
                                    phaseU,
                                    getPhase(x_sumU, y_sumU));
#else
        uiPrintf("+Raw    Avg: %4d\n", getPhaseLookup(x_sum_raw, y_sum_raw));
        uiPrintf("+Weight Avg: %4d\n", phase);
        uiPrintf("+Lower  Avg: %4d\n", phaseL);
        uiPrintf("+Upper  Avg: %4d\n", phaseU);
#endif /* TEMP_USE_FLOAT */
    }

    return phase;
}

/*
 * Lock BF cache
 * Disables h/w update of BF coef cache.
 */
LOCAL void
ar5513LockBfUpdate(WLAN_DEV_INFO *pDev)
{
    A_UINT32 reg;

    //PHY_MULTICHN_TXBF_CTRL, 108
    reg = readPlatformReg(pDev, 0x99b0);
    reg &= ~0x00001000;
    writePlatformReg(pDev, 0x99b0, reg);
}

/*
 * Unlock BF cache
 * Re-Enable h/w update of BF coef cache
 */
LOCAL void
ar5513UnlockBfUpdate(WLAN_DEV_INFO *pDev)
{
    A_UINT32 reg;

    //PHY_MULTICHN_TXBF_CTRL, 108
    reg = readPlatformReg(pDev, 0x99b0);
    reg |= 0x00001000;
    writePlatformReg(pDev, 0x99b0, reg);
}

#endif /* !NDIS_HW */

/*
 *  Initialize WAR logic
 */
static void
ar5513InitSynth2_4War(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
#ifndef NDIS_HW /* TODO: change to runtime check for tx dual chain */

    A_UINT32 reg;
    int      ii;
    int      lock;

#ifdef TEMP_USE_FLOAT
    floatInit();
#endif

    /* Note: pDev->pHalInfo is initially zero'd during halAttach() */

    /* If done_synth_state_check_2_4, return */
    if (pDev->pHalInfo->done_synth_state_check_2_4) {
        return;
    }

    /* If not 2.4 GHz, set done_synth_state_check_2_4 and return */
    if (!(IS_CHAN_2GHZ(pChval->channelFlags))) {
        pDev->pHalInfo->done_synth_state_check_2_4 = TRUE;
        return;
    }

    txTriggerCount    = 0;
    txTriggerValCount = 0;
    txTriggerBadCount = 0;
    txTriggerThrCount = 0;
    saveValidIdx      = -1;

    /* Turn off 2.4 GHz LNA for chain 1 */
    /*    uses the antenna switch table */
    /*    set flag so ar5513SetAntennaSwitch() knows about LNA Chain1 disable */
    pDev->pHalInfo->synth_state_flag |= 1;

    /* Turn on the post-LNA feed-through circuitry with GPIO */
    /*     Configure GPIO for LNA shunt control */
    lock = intLock();
    reg = sysRegRead(AR5513_GPIO_CR0);
    reg |= GPIO_CR_M(GPIO_LNA_SHUNT);
    sysRegWrite(AR5513_GPIO_CR0, reg);

    /*     Set LNA shunt on */
    reg = sysRegRead(AR5513_GPIO_DO0);
    sysRegWrite(AR5513_GPIO_DO0, LNA_SHUNT_ON(reg));
    intUnlock(lock);

    /* Clear BF Coef Cache */
    for (ii = 0; ii < 216; ii++) {
        writePlatformReg(pDev, 0xa400 + ii * 4, 0);
        writePlatformReg(pDev, 0xb400 + ii * 4, 0);
    }

    /* Clear out the BF coef cache valid bits */
    for (ii = 0; ii < 8; ii++) {
        writePlatformReg(pDev, 0x8180 + ii * 4, 0);
        logBfSynth[0].tab[ii] = 0;
    }

    if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
        /*
         * Indicate synth cal war trigger.
         * Only print if WAR_SHOWPHASECAL is set.
         */
        uiPrintf("+ar5513InitSynth2_4War %d\n", txNumTriggers);
    }
#endif /* !NDIS_HW */
}


/*
 *  Update phase cal info for WAR logic
 */
void
ar5513UpdateSynth2_4War(WLAN_DEV_INFO *pDev)
{
#ifndef NDIS_HW /* TODO: change to runtime check for tx dual chain when supported by hw */

    int jj;
    A_UINT32 reg;
    int rxDeltaPhase, rxDeltaPhaseL, rxDeltaPhaseU;
    int lock;

    int debugValidFlag = 0;

    txTriggerCount++;

    /* Copy BF Aging Table */
    logsynthwar(pDev, 1);

    /* Disable h/w update of BF coef cache */
    ar5513LockBfUpdate(pDev);

    /*
     * Check for trigger condition:
     *    aging entry valid &&
     *    aging entry timestamp progress &&
     *    |UpperPhase - LowerPhase| < 20 &&
     *    Chain0_CoefScale[13] != Chain0_CoefScale[26] &&
     *    Chain1_CoefScale[13] != Chain1_CoefScale[26] &&
     *    Chain0_CoefCopy1[ 0] == Chain0_CoefCopy2[ 0] &&
     *    Chain0_CoefCopy1[13] == Chain0_CoefCopy2[13] &&
     *    Chain0_CoefCopy1[26] == Chain0_CoefCopy2[26] &&
     *    Chain1_CoefCopy1[ 0] == Chain1_CoefCopy2[ 0] &&
     *    Chain1_CoefCopy1[13] == Chain1_CoefCopy2[13] &&
     *    Chain1_CoefCopy1[26] == Chain1_CoefCopy2[26]
     */

    /* keep trying TX frames until valid coef aging changes */
    for (jj = 0; jj < 8; jj++) {
        /* look for valid aging entry and updated timestamp */
        if ((logBfSynth[1].tab[jj] & 0x80000000) &&
                ((logBfSynth[1].tab[jj] & 0x00ffffff) !=
                 (logBfSynth[0].tab[jj] & 0x00ffffff)))
        {
            if (logBfSynth[1].tab[jj] & 0x80000000) {
                debugValidFlag = 1;
            }
            break;
        }
    }

    if (jj >= 8) {
        if (debugValidFlag) {
            txTriggerValCount++;
            if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
                /*
                 * Indicate synth cal war trigger.
                 * Only print if WAR_SHOWPHASECAL is set.
                 */
                uiPrintf("+updateSynth2_4War ERR VAL \n");
                if (apCfgWarOptionGet() & WAR_DUMPPHASECAL) {
                    dumpsynthwar(8);
                }
            }
        }

        /* no valid entry, re-arm and return */
        /* re-arm TSF compare */
        A_BCOPY(&logBfSynth[1], &logBfSynth[0], 8 * sizeof(A_UINT32));

        /* Re-Enable h/w update of BF coef cache */
        ar5513UnlockBfUpdate(pDev);
        return;
    }

    /* Found valid aging entry, grab index */
    saveValidIdx = jj;
    jj = saveValidIdx * 27;
    if (jj >= 216 || jj < 0) {
        jj = 0;
        ASSERT(0);
    }

    /*
     * copy incoming BF coef cache entry into scratch buffer
     */
    logsynthbc(pDev, &swCopyOfBfCache[1][0], 216);
    logsynthbcB(pDev, &swCopyOfBfCache[3][0], 216);

    /* TODO: Temporary re-check of BF coef cache */
    logsynthbc(pDev, &swCopyOfBfCache[0][0], 216);
    logsynthbcB(pDev, &swCopyOfBfCache[2][0], 216);

    /* TODO: Temporary re-check of BF ageing table */
    logsynthwar(pDev, 2);

    /* Further qualification of trigger */
    if (swCopyOfBfCache[1][jj + 13] == swCopyOfBfCache[1][jj + 26] ||
        swCopyOfBfCache[3][jj + 13] == swCopyOfBfCache[3][jj + 26] ||
        swCopyOfBfCache[1][jj     ] != swCopyOfBfCache[0][jj     ] ||
        swCopyOfBfCache[1][jj + 13] != swCopyOfBfCache[0][jj + 13] ||
        swCopyOfBfCache[1][jj + 26] != swCopyOfBfCache[0][jj + 26] ||
        swCopyOfBfCache[3][jj     ] != swCopyOfBfCache[2][jj     ] ||
        swCopyOfBfCache[3][jj + 13] != swCopyOfBfCache[2][jj + 13] ||
        swCopyOfBfCache[3][jj + 26] != swCopyOfBfCache[2][jj + 26])
    {
        txTriggerBadCount++;
        if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
            /*
             * Indicate synth cal war trigger.
             * Only print if WAR_SHOWPHASECAL is set.
             */
            uiPrintf("+updateSynth2_4War ERR BAD\n");
            if (apCfgWarOptionGet() & WAR_DUMPPHASECAL) {
                dumpsynthwar(8);
                dumpsynthwar(300);
            }
        }

        /* re-arm TSF compare */
        A_BCOPY(&logBfSynth[1], &logBfSynth[0], 8 * sizeof(A_UINT32));

        /* Re-Enable h/w update of BF coef cache */
        ar5513UnlockBfUpdate(pDev);
        return;
    }

    /* compute mean phase difference from scratch value */
    rxDeltaPhase = getWeightedAveragePhase(&swCopyOfBfCache[1][jj],
                                           &swCopyOfBfCache[3][jj],
                                           &rxDeltaPhaseL,
                                           &rxDeltaPhaseU);

    /* Further qualification of trigger */
    if (rxDeltaPhaseL - rxDeltaPhaseU >  SYNTH_PHASE_THRESH ||
        rxDeltaPhaseL - rxDeltaPhaseU < -SYNTH_PHASE_THRESH)
    {
        txTriggerThrCount++;
        if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
            /*
             * Indicate synth cal war trigger.
             * Only print if WAR_SHOWPHASECAL is set.
             */
            uiPrintf("+updateSynth2_4War ERR THR %d %d %d\n",
                            rxDeltaPhase,
                            rxDeltaPhaseL,
                            rxDeltaPhaseU);
            if (apCfgWarOptionGet() & WAR_DUMPPHASECAL) {
                dumpsynthwar(8);
                dumpsynthwar(300);
            }
        }

        /* re-arm TSF compare */
        A_BCOPY(&logBfSynth[1], &logBfSynth[0], 8 * sizeof(A_UINT32));

        /* Re-Enable h/w update of BF coef cache */
        ar5513UnlockBfUpdate(pDev);
        return;
    }

    /*
     * Trigger is succesful
     */
    if (apCfgWarOptionGet() & WAR_SHOWPHASECAL) {
        /*
         * Indicate synth cal war trigger.
         * Only print if WAR_SHOWPHASECAL is set.
         */
        uiPrintf("+updateSynth2_4War %d tx phase = %d\n",
                        txTriggerCount, rxDeltaPhase);
    }

    /* update synth_state_2_4 */
    pDev->pHalInfo->synth_state_2_4 = rxDeltaPhase;

    /* update phase cal value */
    ar5513AdjustPhaseRamp(pDev, pDev->staConfig.pChannel);

    /* turn off the post-LNA feed-through circuitry with GPIO */
    /*     Set LNA shunt off */
    lock = intLock();
    reg = sysRegRead(AR5513_GPIO_DO0);
    sysRegWrite(AR5513_GPIO_DO0, LNA_SHUNT_OFF(reg));
    intUnlock(lock);

    /* Turn on 2.4 GHz LNA for chain 1 */
    pDev->pHalInfo->synth_state_flag &= ~1;
    halSetAntennaSwitch(pDev, ANTENNA_CONTROLLABLE, pDev->staConfig.pChannel);

    /* restore frame type capabilities for Tx BF */
    ar5513InitFrmTypeCapTable(pDev);

    /* Enable h/w update of BF coef cache */
    ar5513UnlockBfUpdate(pDev);

    /* update done_synth_state_check_2_4 */
    pDev->pHalInfo->done_synth_state_check_2_4 = TRUE;
    txNumTriggers++;

#endif /* !NDIS_HW */
}


#ifndef NDIS_HW /* TODO: change to runtime check for tx dual chain */

/* Save the contents of the Ch0 BF coef cache registers to a buffer */
void
logsynthbc(WLAN_DEV_INFO *pDev, A_UINT32 *buf, int lw)
{
    int jj;

    for (jj = 0; jj < lw; jj++) {
        *buf = (A_UINT32) readPlatformReg(pDev, 0xa400 + jj * 4);
        buf++;
    }
}

/* Save the contents of the Ch1 BF coef cache registers to a buffer */
void
logsynthbcB(WLAN_DEV_INFO *pDev, A_UINT32 *buf, int lw)
{
    int jj;

    for (jj = 0; jj < lw; jj++) {
        *buf = (A_UINT32) readPlatformReg(pDev, 0xb400 + jj * 4);
        buf++;
    }
}

/* Save the contents of the BF coef aging table registers */
void
logsynthwar(WLAN_DEV_INFO *pDev, int s)
{
    int ii;

    for (ii = 0; ii < 8; ii++) {
        logBfSynth[s].tab[ii] = (A_UINT32) readPlatformReg(pDev, 0x8180 + ii * 4);
    }
}


void
dumpsynthwar(int f)
{
    int ii, jj;
    WLAN_DEV_INFO *pDev;
    int newBase;
    int x00, y00, x01, y01, x10, y10, x11, y11;
    int xl_rem, yl_rem, xh_rem, yh_rem;
    int x_sum, y_sum;
#ifdef TEMP_USE_FLOAT
    float phaseCh0L, phaseCh0H, phaseCh1L, phaseCh1H, phaseDeltaL, phaseDeltaH;
    float mag, phase;
#endif

    pDev = gDrvInfo.pDev[0];

    uiPrintf("+Dump Debug synth %d\n", f);


    /*
     * Dump synth state workaround state
     */
    if (f == 6) {
        uiPrintf("State Triggered  = %s\n",
                pDev->pHalInfo->done_synth_state_check_2_4 ? "Yes" : "No");
        uiPrintf("State Flags      = %4.4X\n",
                pDev->pHalInfo->synth_state_flag);
        uiPrintf("Delta Phase      = %4d\n",
                pDev->pHalInfo->synth_state_2_4);
        uiPrintf("tx trigger count = %10d\n", txTriggerCount);
        uiPrintf("not valid errors = %10d\n", txTriggerValCount);
        uiPrintf("bad value errors = %10d\n", txTriggerBadCount);
        uiPrintf("threshold errors = %10d\n", txTriggerThrCount);
        uiPrintf("total triggers   = %10d\n", txNumTriggers);
    }

    /*
     * Dump Bf coef aging table log
     */
    uiPrintf("\n");
    if (f == 8) {
        for (jj = 0; jj < 3; jj++) {
            for (ii = 0; ii < 8; ii++) {
                uiPrintf("%8.8x ", logBfSynth[jj].tab[ii]);
            }
            uiPrintf("\n");
        }
    }

    if (pDev == (WLAN_DEV_INFO *)0) {
        return;
    }

    /*
     * Clear synth war trigger, 0xa
     */
    if (f == 10) {
        void clearsynthflag(void);
        clearsynthflag();
    }

    /*
     * Dump current state of Bf coef aging table registers, 0x64
     */
    if (f == 100) {
        A_UINT32 idxArray[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
        A_UINT32 tempVal, tempVal2;
        int kcIdx;
        for (ii = 0; ii < 8; ii++) {
            tempVal = (A_UINT32) readPlatformReg(pDev, 0x8180 + ii * 4);
            uiPrintf("%8.8x ", tempVal);
            if (tempVal & 0xff000000) {
                idxArray[ii] = tempVal;
            }
        }
        uiPrintf("\n");
        uiPrintf("<%8.8x> \n", (A_UINT32) readPlatformReg(pDev, 0x81c0));

        /* look up macaddr for entries */
        for (ii = 0; ii < 8; ii++) {
            if (idxArray[ii] != -1 ) {
                kcIdx = (idxArray[ii] & 0x7f000000) >> 24;
                uiPrintf("KC %3d - ", kcIdx);
                tempVal = (A_UINT32) readPlatformReg(pDev, 0x8800 + 0x20 * kcIdx + 0x18);
                uiPrintf("%2.2X:%2.2X:%2.2X:%2.2X:",
                    (tempVal <<  1) & 0xff,
                    (tempVal >>  7) & 0xff,
                    (tempVal >> 15) & 0xff,
                    (tempVal >> 23) & 0xff);
                tempVal2 = (A_UINT32) readPlatformReg(pDev, 0x8800 + 0x20 * kcIdx + 0x1c);
                uiPrintf("%2.2X:%2.2X\n",
                    ((tempVal2 <<  1) | (tempVal&0x80000000?1:0)) & 0xff,
                    (tempVal2 >>  7) & 0xff);
            }
        }
    }

    /*
     * Dump current state of BfCache registers, 0xc8
     */
    if (f == 200) {
        for (ii = 0; ii < 216; ii++) {
            if ((ii & 7) == 0) {
                uiPrintf("\n");
            }
            uiPrintf("%8.8x ", (A_UINT32) readPlatformReg(pDev, 0xa400 + ii * 4));
        }
        uiPrintf("\n");
    }

    /*
     * Dump all logged BfCache copies, 0x12c
     */
    if (f == 300) {
        /*  */
        uiPrintf("\nswCopyOfBfCache2: chain 0, first sample");
        for (ii = 0; ii < 216; ii++) {
            if ((ii & 7) == 0) {
                uiPrintf("\n");
            }
            uiPrintf("%8.8x ", swCopyOfBfCache[1][ii]);
        }
        uiPrintf("\n");

        /*  */
        uiPrintf("\nswCopyOfBfCache4: chain 1, first sample");
        uiPrintf("\n");        for (ii = 0; ii < 216; ii++) {
            if ((ii & 7) == 0) {
                uiPrintf("\n");
            }
            uiPrintf("%8.8x ", swCopyOfBfCache[3][ii]);
        }
        uiPrintf("\n");

        /*  */
        uiPrintf("\nswCopyOfBfCache1: chain 0, second sample");
        for (ii = 0; ii < 216; ii++) {
            if ((ii & 7) == 0) {
                uiPrintf("\n");
            }
            uiPrintf("%8.8x ", swCopyOfBfCache[0][ii]);
        }
        uiPrintf("\n");

        /*  */
        uiPrintf("\nswCopyOfBfCache3: chain 1, second sample");
        uiPrintf("\n");        for (ii = 0; ii < 216; ii++) {
            if ((ii & 7) == 0) {
                uiPrintf("\n");
            }
            uiPrintf("%8.8x ", swCopyOfBfCache[2][ii]);
        }
        uiPrintf("\n");
    }

    /*
     * Dump log BfCache 2, 4 in mag, phase format, 0x190
     */
    if (f == 400) {
        int i1, q1, i2, q2;
        int jj;
        int x_sumL, y_sumL, x_sumU, y_sumU;
        int x_sum_raw, y_sum_raw;   /* for algorithm validation */

        if (saveValidIdx == -1) {
            saveValidIdx = 0;
        }


        /*
         * Start Chain 0 Loop
         */
        uiPrintf("\nChain 0: M, P; idx = %d\n", saveValidIdx);
        newBase = saveValidIdx * 27;

        jj = 0;
        for (ii = newBase; ii < newBase + 27; ii++) {
            i1 = sgn((swCopyOfBfCache[1][ii] >>  8) & 0xff);
            q1 = sgn((swCopyOfBfCache[1][ii])       & 0xff);
            i2 = sgn((swCopyOfBfCache[1][ii] >> 24) & 0xff);
            q2 = sgn((swCopyOfBfCache[1][ii] >> 16) & 0xff);


#ifdef TEMP_USE_FLOAT
            if (jj == 26) {
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d -- %8.8x  SCALE 26\n",
                            jj, (float)i1, (float)q1, i1, q1, swCopyOfBfCache[1][ii]);
            } else {
                mag   = getMag(i1, q1);
                phase = getPhase(i1, q1);
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d -- %8.8x\n",
                            jj, mag, phase, i1, q1, swCopyOfBfCache[1][ii]);
            }
#else
            if (jj == 26) {
                uiPrintf("%4d:                -- %6d %6d -- %8.8x  SCALE 26\n",
                            jj, i1, q1, swCopyOfBfCache[1][ii]);
            } else {
                uiPrintf("%4d:         %6d -- %6d %6d -- %8.8x\n",
                            jj, getPhaseLookup(i1, q1), i1, q1, swCopyOfBfCache[1][ii]);
            }
#endif

            jj++;
#ifdef TEMP_USE_FLOAT
            if (jj == 53) {
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d --           SCALE 53\n",
                            jj, (float)i2, (float)q2, i2, q2);
            } else {
                mag   = getMag(i2, q2);
                phase = getPhase(i2, q2);
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d\n",
                            jj, mag, phase, i2, q2);
            }
#else
            if (jj == 53) {
                uiPrintf("%4d:                -- %6d %6d --           SCALE 53\n",
                            jj, i2, q2);
            } else {
                uiPrintf("%4d:         %6d -- %6d %6d\n",
                            jj, getPhaseLookup(i2, q2), i2, q2);
            }
#endif
            jj++;
        } /* end chain 0 */


        /*
         * Start Chain 1 Loop
         */
        uiPrintf("\nChain 1: M, P\n");

        jj = 0;
        for (ii = newBase; ii < newBase+27; ii++) {
            i1 = sgn((swCopyOfBfCache[3][ii] >>  8) & 0xff);
            q1 = sgn((swCopyOfBfCache[3][ii])       & 0xff);
            i2 = sgn((swCopyOfBfCache[3][ii] >> 24) & 0xff);
            q2 = sgn((swCopyOfBfCache[3][ii] >> 16) & 0xff);

#ifdef TEMP_USE_FLOAT
            if (jj == 26) {
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d -- %8.8x  SCALE 26\n",
                            jj, (float)i1, (float)q1, i1, q1, swCopyOfBfCache[3][ii]);
            } else {
                mag   = getMag(i1, q1);
                phase = getPhase(i1, q1);
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d -- %8.8x\n",
                            jj, mag, phase, i1, q1, swCopyOfBfCache[3][ii]);
            }
#else
            if (jj == 26) {
                uiPrintf("%4d:                -- %6d %6d -- %8.8x  SCALE 26\n",
                            jj, i1, q1, swCopyOfBfCache[3][ii]);
            } else {
                uiPrintf("%4d:         %6d -- %6d %6d -- %8.8x\n",
                            jj, getPhaseLookup(i1, q1), i1, q1, swCopyOfBfCache[3][ii]);
            }
#endif

            jj++;
#ifdef TEMP_USE_FLOAT
            if (jj == 53) {
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d --           SCALE 53\n",
                            jj, (float)i2, (float)q2, i2, q2);
            } else {
                mag   = getMag(i2, q2);
                phase = getPhase(i2, q2);
                uiPrintf("%4d: %6.1f, %6.1f -- %6d %6d\n",
                            jj, mag, phase, i2, q2);
            }
#else
            if (jj == 53) {
                uiPrintf("%4d:                -- %6d %6d --           SCALE 53\n",
                            jj, i2, q2);
            } else {
                uiPrintf("%4d:         %6d -- %6d %6d\n",
                            jj, getPhaseLookup(i2, q2), i2, q2);
            }
#endif
            jj++;
        } /* end chain 1 */


        /*
         * Start weighted Average Loop
         */
        uiPrintf("\nWeighted Average Loop\n");

        x_sum = y_sum = 0;
#ifdef TEMP_USE_FLOAT
        phaseDeltaL = phaseDeltaH = 0;
#endif
        x_sumL = y_sumL = x_sumU = y_sumU = 0;
        x_sum_raw = y_sum_raw = 0;

        jj = 0;
        for (ii = newBase; ii < newBase+27; ii++) {
            x00 = sgn((swCopyOfBfCache[1][ii] >>  8) & 0xff);
            y00 = sgn((swCopyOfBfCache[1][ii])       & 0xff);
            x01 = sgn((swCopyOfBfCache[1][ii] >> 24) & 0xff);
            y01 = sgn((swCopyOfBfCache[1][ii] >> 16) & 0xff);

            x10 = sgn((swCopyOfBfCache[3][ii] >>  8) & 0xff);
            y10 = sgn((swCopyOfBfCache[3][ii])       & 0xff);
            x11 = sgn((swCopyOfBfCache[3][ii] >> 24) & 0xff);
            y11 = sgn((swCopyOfBfCache[3][ii] >> 16) & 0xff);

            xl_rem = x00 * x10 + y00 * y10;
            yl_rem = y00 * x10 - x00 * y10;

            xh_rem = x01 * x11 + y01 * y11;
            yh_rem = y01 * x11 - x01 * y11;

            if (jj == 26) {
                uiPrintf("%4d: skip\n", jj);
            } else {
                if (jj != 24 && jj != 28) {
                    x_sum += xl_rem;
                    y_sum += yl_rem;
                    if(jj < 26) {
                        x_sumL += xl_rem;
                        y_sumL += yl_rem;
                    } else if (jj > 26) {
                        x_sumU += xl_rem;
                        y_sumU += yl_rem;
                    }
                }
                x_sum_raw += xl_rem;
                y_sum_raw += yl_rem;
#ifdef TEMP_USE_FLOAT
                phaseCh0L = getPhase(x00, y00);
                phaseCh1L = getPhase(x10, y10);
                phaseDeltaL += getPhaseDiff(phaseCh0L, phaseCh1L);

                uiPrintf("%4d: %6.1f\n", jj, getPhaseDiff(phaseCh0L, phaseCh1L));
#else
                uiPrintf("%4d: %4d\n",
                    jj,
                    (getPhaseLookup(x00, y00) - getPhaseLookup(x10, y10))%360);
#endif
            }

            jj++;

            if (jj == 53) {
                uiPrintf("%4d: skip\n", jj);
            } else {
                if (jj != 23 && jj != 25 && jj != 27 && jj != 29) {
                    x_sum += xh_rem;
                    y_sum += yh_rem;
                    if(jj < 26) {
                        x_sumL += xh_rem;
                        y_sumL += yh_rem;
                    } else if (jj > 26) {
                        x_sumU += xh_rem;
                        y_sumU += yh_rem;
                    }
                }
                x_sum_raw += xh_rem;
                y_sum_raw += yh_rem;
#ifdef TEMP_USE_FLOAT
                phaseCh0H = getPhase(x01, y01);
                phaseCh1H = getPhase(x11, y11);
                phaseDeltaH += getPhaseDiff(phaseCh0H, phaseCh1H);

                uiPrintf("%4d: %6.1f\n", jj, getPhaseDiff(phaseCh0H, phaseCh1H));
#else
                uiPrintf("%4d: %4d\n",
                    jj,
                    (getPhaseLookup(x01, y01) - getPhaseLookup(x11, y11))%360);
#endif
            }

            jj++;
        }
        uiPrintf("\n");

#ifdef TEMP_USE_FLOAT
        uiPrintf("Raw Weighted Avg: %7.0f, %6.1f\n",
                                    getMag(x_sum_raw, y_sum_raw),
                                    getPhase(x_sum_raw, y_sum_raw));
        uiPrintf("Weighted     Avg: %7.0f, %6.1f\n",
                                    getMag(x_sum, y_sum),
                                    getPhase(x_sum, y_sum));
        uiPrintf("Linear       Avg:          %6.1f\n",
                                    (phaseDeltaL+phaseDeltaH)/52.0);
        uiPrintf("Lower Weight Avg: %7.0f, %6.1f\n",
                                    getMag(x_sumL, y_sumL),
                                    getPhase(x_sumL, y_sumL));
        uiPrintf("Upper Weight Avg: %7.0f, %6.1f\n",
                                    getMag(x_sumU, y_sumU),
                                    getPhase(x_sumU, y_sumU));
#else
        uiPrintf("Raw Weighted Avg:          %4d\n",
                                    getPhaseLookup(x_sum_raw, y_sum_raw));
        uiPrintf("Weighted     Avg:          %4d\n",
                                    getPhaseLookup(x_sum, y_sum));
        uiPrintf("Lower Weight Avg:          %4d\n",
                                    getPhaseLookup(x_sumL, y_sumL));
        uiPrintf("Upper Weight Avg:          %4d\n",
                                    getPhaseLookup(x_sumU, y_sumU));
#endif
    }

}

/*
 * For diagnostic testing only.
 * Currently can race with the h/w as the chip is not stopped first.
 */
void
clearsynthflag(void)
{
    WLAN_DEV_INFO *pDev;
    pDev = gDrvInfo.pDev[0];

    pDev->pHalInfo->done_synth_state_check_2_4 = FALSE;
    pDev->pHalInfo->synth_state_2_4            = 0;
    pDev->pHalInfo->synth_state_flag           = 0;

    ar5513InitSynth2_4War(pDev, pDev->staConfig.phwChannel);
    /* pDev->pHalInfo->synth_state_flag is made TRUE above */

    /* turn off LNA and disconnect Rx Ant for Ch1 */
    halSetAntennaSwitch(pDev, ANTENNA_CONTROLLABLE, pDev->staConfig.pChannel);
    /* disable Tx BF in frame capabilities */
    ar5513InitFrmTypeCapTable(pDev);

    uiPrintf("Chain settings: tx %d, rx %d\n",
                pDev->staConfig.txChainCtrl, pDev->staConfig.rxChainCtrl);

}

#ifdef DEBUG
void
validateLookup()
{
    int ii;
    int testDataI[] = {0,  10, 8, 5, 2,    0, -2, -5, -8,   -10, -8, -5, -2,   0,   2,  5,  8};
    int testDataQ[] = {0,   0, 2, 5, 8,   10,  8,  5,  2,     0, -2, -5, -8,  -10, -8, -5, -2};

    for (ii = 0; ii < sizeof(testDataI)/sizeof(int); ii++) {
        uiPrintf("LUT %4d, %4d = %4d\n",
                        testDataI[ii],
                        testDataQ[ii],
                        getPhaseLookup(testDataI[ii], testDataQ[ii]));
    }
}
#endif /* DEBUG */

#endif /* !NDIS_HW */

#endif /* #ifdef BUILD_AR5513 */
