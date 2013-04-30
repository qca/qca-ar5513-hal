/*
 *  Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips specific device attachment and device info collection
 *  Connects Init Reg Vectors, EEPROM Data, and device Functions to pDev
 */

#ifdef BUILD_AR5513

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Attach.c#13 $"

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
#include "ar5513/ar5513MacReg.h"
#include "ar5513/ar5513Attach.h"
#include "ar5513/ar5513Phy.h"
#include "ar5513/ar5513Mac.h"

/* Header files exporting vector mapped functions */
#include "ar5513/ar5513Reset.h"
#include "ar5513/ar5513Misc.h"
#include "ar5513/ar5513KeyCache.h"
#include "ar5513/ar5513Power.h"
#include "ar5513/ar5513Beacon.h"
#include "ar5513/ar5513Transmit.h"
#include "ar5513/ar5513Receive.h"
#include "ar5513/ar5513Interrupts.h"

#if defined(AR5513)
#include "ar5513reg.h"
#endif

/* Forward Declarations */
static A_STATUS
ar5513Detach(WLAN_DEV_INFO *pDev);

static A_STATUS
ar5513FillCapabilityInfo(WLAN_DEV_INFO *pDev);

static A_STATUS
ar5513RecordSerialNumber(WLAN_DEV_INFO *pDev, A_CHAR *pSerNum);

static A_STATUS
ar5513VerifyEepromChecksum(A_UINT16 *pRawEeprom, A_UINT32 eepEndLoc);

static A_STATUS
ar5513AllocateProm(EEP_MAP *pEepMap, A_UINT8 chainCnt);

static A_STATUS
ar5513ReadEepromIntoDataset(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT8 chnIdx);

static void
ar5513ReadHeaderInfo(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, A_UINT16 *pRawEeprom);

static void
getPcdacInterceptsFromPcdacMinMax(A_UINT16 version, A_UINT16 pcdacMin,
    A_UINT16 pcdacMax, A_UINT16 *pPcdacValues);

static A_UINT16
fbin2freq(A_UINT16 version, A_UINT16 fbin, A_BOOL is2GHz);

static int
ar5513EarPreParse(A_UINT16 *in, int numLocs, EAR_ALLOC *earAlloc);

static A_BOOL
ar5513EarAllocate(EAR_ALLOC *earAlloc, EAR_HEADER **ppEarHead);

static A_BOOL
ar5513EarCheckAndFill(EAR_HEADER *earHead, A_UINT16 *in, int totalLocs, EAR_ALLOC *pEarAlloc);

static A_UINT16
ar5513ReadRadioChipRev(WLAN_DEV_INFO *pDev, A_UINT16 chain_base);

#ifdef DEBUG
static void
printEar(EAR_HEADER *earHead, EAR_ALLOC *earAlloc);

static void
printModeString(A_UINT16 mode);

static void
showEarAlloc(EAR_ALLOC *earAlloc);
#endif

/* Constants */
static const HW_FUNCS ar5513Funcs = {
    ar5513Detach,
    ar5513FillCapabilityInfo,

    /* Reset Functions */
    ar5513Reset,
    ar5513PhyDisable,
    ar5513Disable,
    ar5513PerCalibration,
    ar5513GetRfgain,
    ar5513SetTxPowerLimit,

    /* TX Functions */
    ar5513UpdateTxTrigLevel,
    ar5513SetupTxQueue,
    ar5513ReleaseTxQueue,
    ar5513GetTxDP,
    ar5513SetTxDP,
    ar5513StartTxDma,
    ar5513NumTxPending,
    ar5513StopTxDma,
    ar5513GetTxFilter,
    ar5513SetTxFilter,
    ar5513GetTxFilterArraySize,
    ar5513GetTxFilterArray,
    ar5513SetTxFilterArray,
    ar5513PauseTx,

    ar5513SetupTxDesc,
    ar5513ProcessTxDesc,
    ar5513GetTxDescDone,
#if defined(DEBUG) || defined(_DEBUG)
    ar5513DebugPrintTxDesc,
#endif

    /* RX Functions */
    ar5513GetRxDP,
    ar5513SetRxDP,
    ar5513EnableReceive,
    ar5513StopDmaReceive,
    ar5513StartPcuReceive,
    ar5513StopPcuReceive,
    ar5513SetMulticastFilter,
    ar5513MulticastFilterIndex,
    ar5513RxFilter,
    ar5513ProcessRxDesc,
#ifdef UPSD
    NULL,
#endif
    ar5513SetupRxDesc,

    /* Misc Functions */
    ar5513SetRegulatoryDomain,
    ar5513SetLedState,
    ar5513WriteAssocid,
    ar5513GpioCfgInput,
    ar5513GpioCfgOutput,
    ar5513GpioGet,
    ar5513GpioSet,
    ar5513GpioSetIntr,
    ar5513SetStaBeaconTimers,
    ar5513GetTsf,
    ar5513ResetTsf,
    ar5513SetAdhocMode,
    ar5513SetBasicRate,
    ar5513GetRandomSeed,
    ar5513DetectCardPresent,
    ar5513MibControl,
    ar5513GetChannelData,
    ar5513ProcessNoiseFloor,
    ar5513EnableRadarDetection,
    ar5513EnableFriendlyDetection,
    ar5513DisableFriendlyDetection,
    ar5513AniControl,
    ar5513AniGetListenTime,
    ar5513GetCurRssi,
    ar5513GetDefAntenna,
    ar5513SetDefAntenna,
    ar5513SetAntennaSwitch,
    ar5513UpdateAntenna,
    ar5513UseShortSlotTime,
    ar5513DmaDebugDump,
    ar5513GetMacTimer3,
    ar5513SetDecompMask,
    ar5513SwapHwDesc,
    ar5513GetHwDescWord,
    ar5513GetApSwitchHelper,
    ar5513SetApSwitchHelper,
    ar5513SendXrChirp,
#if defined(DEBUG) || defined(_DEBUG)
    ar5513MonitorRxTxActivity,
#endif

    /* Key Cache Functions */
    ar5513GetKeyCacheSize,
    ar5513ReserveHalKeyCacheEntries,
    ar5513ResetKeyCacheEntry,
    ar5513SetKeyCacheEntry,
    ar5513KeyCacheAlloc,
    ar5513KeyCacheFree,

    /* Power Management Functions */
    ar5513SetPowerMode,
    ar5513GetPowerMode,
    ar5513GetPowerStatus,
    ar5513SetupPsPollDesc,

    /* Beacon Functions */
    ar5513SetupBeaconDesc,
    ar5513BeaconInit,
    ar5513SetupGrpPollChain,

    /* Interrupt Functions */
    ar5513IsInterruptPending,
    ar5513GetInterrupts,
    ar5513EnableInterrupts,
    ar5513DisableInterrupts,
#if (defined(UPSD) && defined(BUILD_AP))
    ar5513UpsdResponse,
#endif
};

static const A_UINT16 channels11b[] = {2412, 2447, 2484};
static const A_UINT16 channels11g[] = {2312, 2412, 2484};


#if defined (LOG_ANT_DIVERSITY)
/* TODO: For AR5513 BRINGUP and Diversity  */
AR5513_DIVLOG ar5513DivLog;
A_UINT32 log_defant_writes;
A_UINT32 log_war_count;

/* DEBUG bb_minCCApwr in CCA issues */
#define AR5513_CCALOG_DEPTH  20

#define CCA_VAL(_x)  (((_x)>>19) & 0x1ff)

typedef struct ar5513_ccalog_entry {
    A_UINT32 p0;
    A_UINT32 p1;
    A_UINT32 p2;
    A_UINT32 p3;
} AR5513_CCALOG_ENTRY;

typedef struct ar5513_ccalog {
    int         count;
    int         cur;
    int         stop;
    int         trig_field;
    int         triggered;
    int         trig_after;
    AR5513_CCALOG_ENTRY log[AR5513_CCALOG_DEPTH];
    AR5513_CCALOG_ENTRY lastLog;
} AR5513_CCALOG;

AR5513_CCALOG ar5513CCALog;

void
ar5513CCALogInit()
{
    int lock = intLock();

    ar5513CCALog.count      = 0;
    ar5513CCALog.cur        = 0;
    ar5513CCALog.triggered  = 0;
    ar5513CCALog.trig_after = 0;
    ar5513CCALog.stop  = 0;

    intUnlock(lock);
}

void
ar5513CCALogAdd(A_UINT32 p0, A_UINT32 p1, A_UINT32 p2, A_UINT32 p3)
{
    int lock = intLock();

    if (!ar5513CCALog.stop) {
        ar5513CCALog.log[ar5513CCALog.cur].p0 = p0;
        ar5513CCALog.log[ar5513CCALog.cur].p1 = p1;
        ar5513CCALog.log[ar5513CCALog.cur].p2 = p2;
        ar5513CCALog.log[ar5513CCALog.cur].p3 = p3;
    }

    ar5513CCALog.count++;
    if (++ar5513CCALog.cur >= AR5513_CCALOG_DEPTH) {
        ar5513CCALog.cur = 0;
    }

    intUnlock(lock);
}

void ar5513CCALogIsrPrint()
{
    int i, j, k;

    logMsg("CCA\n", 0,0,0,0,0,0);

    /* display last 10 CCA */
    if (ar5513CCALog.count <= AR5513_CCALOG_DEPTH) {
        i = 0;
        k = ar5513CCALog.count;
    } else {
        i = ar5513CCALog.cur;
        k = AR5513_CCALOG_DEPTH;
    }
    for (j = 0; j < k; j++) {
        logMsg("%05d %08d %08d %08d %08d\n",
                    i,
                    CCA_VAL(ar5513CCALog.log[i].p0),
                    CCA_VAL(ar5513CCALog.log[i].p1),
                    CCA_VAL(ar5513CCALog.log[i].p2),
                    CCA_VAL(ar5513CCALog.log[i].p3),
                    0);
        i++;
        if (i >= AR5513_CCALOG_DEPTH) {
            i = 0;
        }
    }
}

/* end CCA */

void
ar5513DivLogInit(int cmd)
{
    int lock = intLock();

    ar5513DivLog.count      = 0;
    ar5513DivLog.cur        = 0;
    ar5513DivLog.triggered  = 0;
    ar5513DivLog.trig_after = 0;

    log_defant_writes   = 0;
    log_war_count       = 0;


    if (cmd < 0) {
        ar5513DivLog.trig_field = 0;
    } else if (cmd > 0 && cmd <= 8) {
        ar5513DivLog.trig_field = cmd;
    }

    ar5513DivLog.stop  = 0;

    intUnlock(lock);
}

void
ar5513DivLogAdd(A_UINT32 p0, A_UINT32 p1, A_UINT32 p2, A_UINT32 p3,
             A_UINT32 p4, A_UINT32 p5, A_UINT32 p6, A_UINT32 p7)
{
    int         lock = intLock();

    if (!ar5513DivLog.stop) {
        ar5513DivLog.log[ar5513DivLog.cur].p0 = p0;
        ar5513DivLog.log[ar5513DivLog.cur].p1 = p1;
        ar5513DivLog.log[ar5513DivLog.cur].p2 = p2;
        ar5513DivLog.log[ar5513DivLog.cur].p3 = p3;
        ar5513DivLog.log[ar5513DivLog.cur].p4 = p4;
        ar5513DivLog.log[ar5513DivLog.cur].p5 = p5;
        ar5513DivLog.log[ar5513DivLog.cur].p6 = p6;
        ar5513DivLog.log[ar5513DivLog.cur].p7 = p7;

        if (ar5513DivLog.triggered == 0 && ar5513DivLog.count) {

            /*
             * cmd = 0x0000: normal dump
             *       0x0100: set a trigger on war event
             *       0x0001: set a trigger on antenna change event, chain 0, 0 -> 1
             *       0x0002: set a trigger on antenna change event, chain 0, 1 -> 0
             *       0x0004: set a trigger on antenna change event, chain 1, 0 -> 1
             *       0x0008: set a trigger on antenna change event, chain 1, 1 -> 0
             */

            if (ar5513DivLog.trig_field & 0x0100) {
                if ((0xffff0000 & ar5513DivLog.log[ar5513DivLog.cur].p0) != 
                    (0xffff0000 & ar5513DivLog.lastLog.p0))
                {
                    ar5513DivLog.triggered = 1;
                }
            }

            if (ar5513DivLog.triggered) {
                uiPrintf("Log triggered! %5d %2d\n",
                            ar5513DivLog.cur,
                            ar5513DivLog.trig_field);
            }
        }

        ar5513DivLog.count++;
        if (++ar5513DivLog.cur >= AR5513_LOG_DEPTH) {
            ar5513DivLog.cur = 0;
        }

        ar5513DivLog.lastLog.p0 = ar5513DivLog.log[ar5513DivLog.cur].p0;
    }
    intUnlock(lock);
}

/*
 * Overloaded CLI diagnostic dump routine.
 * 
 * cmd =  0: normal diversity log dump
 *        2: bb_minCCApwr CCA log dump
 *        3: AOS log dump
 *        4: Sync Incr log dump
 *        5: NF calibration log dump
 *        6: 2.4 Synth War log dump
 * 
 */
void
ar5513DivLogDump(int cnt, int *args)
{
    int i, j, k;
    int cmd  = 0;
    int disp = 0;
    int filterFlag = 0;

    ar5513DivLog.stop = 1;

    uiPrintf("AR5513 LOG: count %5d, cur %5d, at 0x%8.8x  %4d (%d:",
        (A_UINT32) ar5513DivLog.count,
        (A_UINT32) ar5513DivLog.cur,
        (A_UINT32) &ar5513DivLog.log[0],
        (A_UINT32) sizeof(AR5513_DIVLOG_ENTRY),
        cnt);
    for (i = 0; i < cnt; i++) {
        uiPrintf(" %d", *(args + i));
    }
    uiPrintf(")\n");

    if (cnt >= 1) {
        cmd = *args;
    }
    if (cnt >= 2) {
        disp = *(args+1);
    }

    /* DEBUG bb_minCCApwr in CCA issues */
    if (cnt >= 1 && cmd == 2) {
        uiPrintf("AR5513 CCA LOG: \n");

        /* display last CCA */
        if (ar5513CCALog.count <= AR5513_CCALOG_DEPTH) {
            i = 0;
            k = ar5513CCALog.count;
        } else {
            i = ar5513CCALog.cur;
            k = AR5513_CCALOG_DEPTH;
        }
        for (j = 0; j < k; j++) {
            uiPrintf("%05d %08d %08d %08d %08d\n",
                        i,
                        CCA_VAL(ar5513CCALog.log[i].p0),
                        CCA_VAL(ar5513CCALog.log[i].p1),
                        CCA_VAL(ar5513CCALog.log[i].p2),
                        CCA_VAL(ar5513CCALog.log[i].p3));
            i++;
            if (i >= AR5513_CCALOG_DEPTH) {
                i = 0;
            }
        }
        /* clear log for next time */
        ar5513CCALogInit();
        return;
    }

    /* Dump Log DEBUG AOS */
#if 0
    if (cnt >= 1 && cmd == 3) {
        void logAosTx_Dump(void);
        void logAosTx_Start(void);

        logAosTx_Dump();
        logAosTx_Start();
        return;
    }
#endif

    /* Dump Log Test Sync Incr */
    if (cnt >= 1 && cmd == 4) {
        void logSyncIncr_Dump(void);

        logSyncIncr_Dump();
        return;
    }

    /* NF calibration log dump */
    if (cnt >= 1 && cmd == 5) {
        void logNfCal_Dump(int d);

        logNfCal_Dump(disp);
        return;
    }

    /* 2.4 Synth log dump */
    if (cnt >= 1 && cmd == 6) {
        void dumpsynthwar(int f);
        dumpsynthwar(disp);
        return;
    }

    /* Init diversity test log */
    if (cnt >= 1 && cmd > 10) {
        ar5513DivLogInit(cmd);
        return;
    }

    /* Diversity test log, but only dump '0' & '1' entries */
    if (cnt >= 1 && cmd == 9) {
        filterFlag = 1;
    }

    if (ar5513DivLog.count <= AR5513_LOG_DEPTH) {
        i = 0;
        k = ar5513DivLog.count;
    } else {
        i = ar5513DivLog.cur;
        k = AR5513_LOG_DEPTH;
    }
    for (j = 0; j < k; j++) {
        if (filterFlag & 1 && 
            (((ar5513DivLog.log[i].p0 & 7) == 0) ||
            ((ar5513DivLog.log[i].p0 & 7) == 1)))
        {
            uiPrintf("%05d %08x %08x %08x %08x %08x %08x %08x %08x\n",
                        i,
                        ar5513DivLog.log[i].p0,
                        ar5513DivLog.log[i].p1,
                        ar5513DivLog.log[i].p2,
                        ar5513DivLog.log[i].p3,
                        ar5513DivLog.log[i].p4,
                        ar5513DivLog.log[i].p5,
                        ar5513DivLog.log[i].p6,
                        ar5513DivLog.log[i].p7);
        } else if (!filterFlag) {
            uiPrintf("%05d %08x %08x %08x %08x %08x %08x %08x %08x\n",
                        i,
                        ar5513DivLog.log[i].p0,
                        ar5513DivLog.log[i].p1,
                        ar5513DivLog.log[i].p2,
                        ar5513DivLog.log[i].p3,
                        ar5513DivLog.log[i].p4,
                        ar5513DivLog.log[i].p5,
                        ar5513DivLog.log[i].p6,
                        ar5513DivLog.log[i].p7);
        }

        i++;
        if (i >= AR5513_LOG_DEPTH) {
            i = 0;
        }
    }

    ar5513DivLogInit(0);
    return;
}
#endif /* LOG_ANT_DIVERSITY */

#if defined(AR5513_TRACE)
AR5513_TRACE_LOG ar5513TrcLog;

inline void
ar5513TrcInit(void) {
    ar5513TrcLog.base = ar5513TrcLog.cur = &ar5513TrcLog.log[0]; 
    ar5513TrcLog.wrap = &ar5513TrcLog.log[TRC_DEPTH-1] + 1;
}

inline void
ar5513Trc(A_UINT32 func, A_UINT32 p1, A_UINT32 p2, A_UINT32 p3) {

    int log = 0;

    if (func == 0x00037ffe)
	log = 1;

    if (p2 == 0x00037ffe)
	log = 1;

    if (p3 == 0x00037ffe)
	log = 1;

    if (p1 == 0x00037ffe)
	log = 1;

    if (log == 0) {
	ar5513TrcLog.cur->func = func;
	ar5513TrcLog.cur->p1 = p1;
	ar5513TrcLog.cur->p2 = p2;
	ar5513TrcLog.cur->p3 = p3;
	if (++ar5513TrcLog.cur >= ar5513TrcLog.wrap)
	    ar5513TrcLog.cur = ar5513TrcLog.base;
    }
    else {
	log = 2;
    }
}

inline void
ar5513TrcDump(void) {
    int i;

    uiPrintf("0x%08x  0x%08x  0x%08x\n",(A_UINT32) ar5513TrcLog.base,
	     (A_UINT32) ar5513TrcLog.cur, (A_UINT32) ar5513TrcLog.wrap);
    for (i=0; i < TRC_DEPTH; i++) {
	uiPrintf("0x%08x  0x%08x  0x%08x  0x%08x  0x%08x\n",
		 (A_UINT32) &ar5513TrcLog.log[i],ar5513TrcLog.log[i].func, 
		 ar5513TrcLog.log[i].p1, ar5513TrcLog.log[i].p2, 
		 ar5513TrcLog.log[i].p3);
    }
}
#endif /* AR5513_TRACE */

/**************************************************************
 * ar5513Attach
 *
 * ar5513Attach performs the device specific functions for halAttach()
 */
A_STATUS
ar5513Attach(WLAN_DEV_INFO *pDev, A_UINT16 deviceID)
{
    struct gainValues *pGainValues;
    A_UINT16  i;
    EEP_MAP   *pEMap;
    A_UINT16  revId0, revId1;
    A_UINT32  promSize;
    A_STATUS  status;
    A_UINT16  eepVersion;
    A_UINT16  artBuildNo = 0;
    A_UINT16  *pRawEeprom = NULL, *pData16, data;
    A_UINT32  macRev;
    A_UINT32  addr, loop, wrData, rdData, pattern, eepEndLoc;
    A_UINT32  regAddr[2] = {MAC_STA_ID0, PHY_BASE+(8 << 2)};
    A_UINT32  regHold[2];
    A_UINT32  patternData[4] = {0x55555555, 0xaaaaaaaa, 0x66666666, 0x99999999};
    EAR_ALLOC earAlloc;
    int       earLocs = 0, parsedLocs;
    A_UINT16  *earBuffer = NULL;
    A_UINT8   chainCnt = 0;
    A_UINT8   chain;
#if defined(PCI_INTERFACE)
    A_UINT32  chipRev;
#endif

    ASSERT(pDev);

#ifdef DEBUG
    uiPrintf("Attach AR5513 0x%x 0x%x\n", deviceID, (unsigned int)pDev);
#endif

#if defined(AR5513_TRACE)
    uiPrintf("AR5513 Trace Log located at addr: 0x%08x\n",(A_UINT32) &ar5513TrcLog);
    ar5513TrcInit();
#endif /* AR5513_TRACE */
#if defined (LOG_ANT_DIVERSITY)
    /* TODO: For AR5513 BRINGUP */
    ar5513DivLogInit(-1);
#endif

#if defined(PCI_INTERFACE)
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
#endif

    /* Set Observation control register for LED operation */
    A_REG_SET_BIT2(pDev, RST_OBS_CTL, LED_0, LED_1);
    
#ifndef FALCON_E1_SILICON
    writePlatformReg(pDev, RST_DSL_SLEEP_CTL, CLIENT_SLEEP_CTL);
#endif /* FALCON_E1_SILICON */

#endif /* PCI_INTERFACE */

    pDev->pHalInfo->halInit = FALSE;
    pDev->pHalInfo->macReset = FALSE;

    /* Pull Chip out of Reset */
    status = ar5513ChipReset(pDev, NULL);
    if (status != A_OK) {
        return status;
    }
#if defined(FALCON_EMUL) && defined(DEBUG)
    {
	int qCnt = 0;

	pattern = (A_UINT32) 0x42345678;
	for (i=0; i < 5; i++) {
	    addr = (A_UINT32) (0x800 + (i * 4));
	    writePlatformReg(pDev,addr,pattern);
	    rdData = readPlatformReg(pDev, addr);
	    if (rdData == pattern) {
		++qCnt;
	    }
	}
	uiPrintf("ar5513Attach: Falcon MAC Queue Count ==> %d <==\n",qCnt);
    }
#endif  /* FALCON_EMUL */

    /* Read Revisions from Chips */
#if defined(PCI_INTERFACE)
    macRev           = readPlatformReg(pDev, MAC_SREV) & MAC_SREV_ID_M;
    pDev->macVersion = (A_UINT16) (macRev >> MAC_SREV_ID_S);
    pDev->macRev     = (A_UINT16) (macRev & MAC_SREV_REVISION_M);
    
    chipRev          = readPlatformReg(pDev, RST_SREV) & RST_SREV_M;
    pDev->pHalInfo->chipVersion = (A_UINT16) (chipRev >> RST_SREV_VERSION_S);
    pDev->pHalInfo->chipRev     = (A_UINT16) (chipRev & RST_SREV_REVISION_M);
#elif defined(AR5513)
    macRev           = sysRegRead(AR5513_SREV);
    pDev->macVersion = (macRev & REV_MAJ) >> REV_MAJ_S;
    pDev->macRev     = (macRev & REV_MIN) >> REV_MIN_S;
#endif

    pDev->phyRev = (A_UINT16)readPlatformReg(pDev, PHY_CHIP_ID);

    /* Self test PHY & MAC registers */
    for (i = 0; i < 2; i++) {
        addr       = regAddr[i];
        regHold[i] = readPlatformReg(pDev, addr);

        for (loop = 0; loop < 0x100; loop++) {

            wrData = (loop << 16) | loop;
            writePlatformReg(pDev, addr, wrData);
            rdData = readPlatformReg(pDev, addr);
            if (rdData != wrData) {
                uiPrintf("ar5513Attach: address test failed addr: 0x%08x - wr:0x%08x != rd:0x%08x\n",
                         addr, wrData, rdData);
                return A_HARDWARE;
            }
        }
        for (pattern = 0; pattern < 4; pattern++) {
            wrData = patternData[pattern];
            writePlatformReg(pDev, addr, wrData);
            rdData = readPlatformReg(pDev, addr);
            if (wrData != rdData) {
                uiPrintf("ar5513Attach: address test failed addr: 0x%08x - wr:0x%08x != rd:0x%08x\n",
                         addr, wrData, rdData);
                return A_HARDWARE;
            }
        }
        writePlatformReg(pDev, regAddr[i], regHold[i]);
    }
    udelay(100);

    /* Set correct Baseband to analog shift setting to access analog chips. */
    writePlatformReg(pDev, PHY_BASE, 0x00000007);

    /* Read Radio Chip Rev Extract for Chain 0 */
    revId0 = ar5513ReadRadioChipRev(pDev,CHN_0_BASE);

    if ((pDev->staConfig.txChainCtrl == DUAL_CHAIN) || (pDev->staConfig.rxChainCtrl == DUAL_CHAIN)) {
        /* Read Radio Chip Rev Extract for Chain 1 */
        revId1 = ar5513ReadRadioChipRev(pDev,CHN_1_BASE);

        /* Verify Radio Chain 0 and 1 are the SAME */
        if (revId0 != revId1) {
            uiPrintf("ar5513Attach: Radio Chip Rev MISMATCH between 0 and 1\n");
            ASSERT(0);
        }
    }

    pDev->analog5GhzRev = revId0;

    /* Complain when unrecognized Analog REV's are attached - but don't fail attach in production */

    if (((pDev->analog5GhzRev & 0xF0) != RAD5112_SREV_MAJOR) && ((pDev->analog5GhzRev & 0xF0) != RAD2112_SREV_MAJOR))
    {
        uiPrintf("ar5513Attach: Radio Chip Rev 0x%02X is unknown to this driver\n",pDev->analog5GhzRev);
        ASSERT(0);

        /** WAR for Bug 10062
         * When HW RF_Silent is used, the analog chip is 
         * had in reset.  So when the system is booted up
         * with the radio switch off, the ar5513Attach can 
         * not determine the RF chip rev.  So it would 
         * default to use the 5111 rf chip, for Hainan
         * this would result in a BSOD and improperly 
         * programmed RF so the scan list will be empty.
         * For 2.4.2.x, hotfix this by detecting that the 
         * MAC is a hainan Mac then set the rf rev to 
         * Derby.
         */
#ifdef PCI_INTERFACE
/* 
** XXXX pDev->phyRev = 0x43 in the if looks like a bug - GDS 04/14/2004 
*/
        if ((pDev->analog5GhzRev == 0) &&
            (pDev->macVersion == MAC_SREV_VERSION_VENICE) &&
            (pDev->macRev == 9) &&
            (pDev->phyRev = 0x43))
        {
            /* if this is a Hainan board then must be using Derby */
            pDev->analog5GhzRev = 0x46;
        }
#endif /* PCI_INTERFACE */
        /*
         ** WAR for Bug 10062 */
    }

    if (IS_RAD5112_REV1(pDev)) {
        uiPrintf("ar5513Attach: 5112 Rev 1 is no longer supported - failing attach\n");
        return A_HARDWARE;
    }

    /* EEPROM version and size checking */
    status = ar5513EepromRead(pDev, ATHEROS_EEPROM_OFFSET + 1, &eepVersion);
    if (status != A_OK) {
        uiPrintf("ar5513Attach: Could not access EEPROM\n");
        return A_HARDWARE;
    }

#if defined(PCI_INTERFACE)
#if defined(BUILD_AR5513)
    promSize = MAC_PCICFG_EEPROM_SIZE_16K;
#else
    promSize = (readPlatformReg(pDev, MAC_PCICFG) & MAC_PCICFG_EEPROM_SIZE_M) >>
               MAC_PCICFG_EEPROM_SIZE_S;
#endif
#else /* !PCI_INTERFACE */
    promSize = MAC_PCICFG_EEPROM_SIZE_16K;
#endif /* !PCI_INTERFACE */

    if ( (eepVersion < EEPROM_VER3_2) || (promSize != MAC_PCICFG_EEPROM_SIZE_16K) ) {
        uiPrintf("ar5513Attach: A Rev 3.2 or greater 16k calibrated EEPROM is required for correct operation\n");
        return A_HARDWARE;
    }

    if ( (eepVersion != EEPROM_VER4_9) ) {
        uiPrintf("ar5513Attach: !!! WARNING !!! A Rev 4.9 calibrated EEPROM is required for correct AR5513 operation\n");
    }

    ar5513EepromRead(pDev, ATHEROS_EEPROM_OFFSET + 7, &artBuildNo);
    artBuildNo = (artBuildNo >> 10) & 0x3f;
    uiPrintf("ar5513Attach: ART build number is %d\n", artBuildNo);
    if (artBuildNo < 38) {
        uiPrintf("ar5513Attach: !!! WARNING !!! A Build 38 or greater calibrated EEPROM is required for correct AR5513 operation\n");
    }

    /* Read sizing information */
    status = ar5513EepromRead(pDev, EEPROM_SIZE_UPPER, &data);
    if (status != A_OK) {
        uiPrintf("ar5513Attach: Could not access EEPROM\n");
        return A_HARDWARE;
    }
    
    if (data == 0) {
        eepEndLoc = ATHEROS_EEPROM_END_DEFAULT;
    } else {
        eepEndLoc = (data & EEPROM_SIZE_UPPER_M) << 12;
        status = ar5513EepromRead(pDev, EEPROM_SIZE_LOWER, &data);
        if (status != A_OK) {
            uiPrintf("ar5513Attach: Could not access EEPROM\n");
            return A_HARDWARE;
        }
        eepEndLoc |= data;
    }
    ASSERT(eepEndLoc > ATHEROS_EEPROM_OFFSET);

    /* Cache EEPROM for checksum verification and multiple parse passes */
    pRawEeprom = (A_UINT16 *)A_DRIVER_MALLOC(sizeof(A_UINT16) * (eepEndLoc - ATHEROS_EEPROM_OFFSET));
    if (!pRawEeprom) {
        uiPrintf("ar5513Attach: Could not allocate space to cache the EEPROM\n");
        return A_NO_MEMORY;
    }

    /* Allocate EEPROM Map */
    pEMap = (struct eepMap *)A_DRIVER_MALLOC(sizeof(struct eepMap));
    if (!pEMap) {
        uiPrintf("ar5513Attach: Could not allocate memory for version 3 EEPROM\n");
        goto attachError;
    }
    A_MEM_ZERO(pEMap, sizeof(struct eepMap));
    pDev->pHalInfo->pEepData = pEMap;

    /* TODO AR5513: look at 1 vs 2 chain initialization w.r.t. A, B, D */
    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN || 
        pDev->staConfig.rxChainCtrl == DUAL_CHAIN)
    {
        chainCnt = 2;
    } else {
        chainCnt = 1;
    }

    /*
     * Initialize defAnt and data structures for s/w diversity,
     * read quadAnt from staConfig, fill in pDev version of quadAnt,
     * set defant accordingly
     */
    rdData = ~7 & readPlatformReg(pDev, MAC_DEF_ANTENNA);
#if 0
    /* TODO: Someday AR5513 will init from pDev->staConfig */
    pDev->quadAnt.chainStrong    = pDev->staConfig.quadAnt.chainStrong;
    pDev->quadAnt.chain0Ant      = pDev->staConfig.quadAnt.chain0Ant;
    pDev->quadAnt.chain1Ant      = pDev->staConfig.quadAnt.chain1Ant;
    pDev->quadAnt.divSxChnStrong = pDev->staConfig.quadAnt.divSxChnStrong;
    pDev->quadAnt.divSxChn0      = pDev->staConfig.quadAnt.divSxChn0;
    pDev->quadAnt.divSxChn1      = pDev->staConfig.quadAnt.divSxChn1;
#else
    pDev->quadAnt.chainStrong    = 0;
    pDev->quadAnt.chain0Ant      = 0;
    pDev->quadAnt.chain1Ant      = 0;
    pDev->quadAnt.divSxChnStrong = 0;
    pDev->quadAnt.divSxChn0      = 0;
    pDev->quadAnt.divSxChn1      = 0;
#endif

    rdData |= (pDev->quadAnt.chainStrong ? MAC_DEF_ANT_CHN_SEL  : 0) |
              (pDev->quadAnt.chain1Ant   ? MAC_DEF_ANT_CHN1_ANT : 0) |
              (pDev->quadAnt.chain0Ant   ? MAC_DEF_ANT_CHN0_ANT : 0);

    ar5513SetDefAntenna(pDev, rdData);

    status = ar5513AllocateProm(pEMap, chainCnt);
    if (status != A_OK) {
        goto attachError;
    }

    /* Set Version so EEPROM Header can be correctly interpreted */
    pEMap->version = eepVersion;

    /*
    **  When AR5513 is configured with Dual Radio chains,
    **  2 16Kbit EEPROM images must be processed in order
    **  to load the calibration data for each Radio chain.
    */
/* TODO: this needs fixing to properly initialize for single chain B */
    for (chain = 0; chain < chainCnt; chain++) {
	
        A_MEM_ZERO(pRawEeprom, sizeof(A_UINT16) * (eepEndLoc - ATHEROS_EEPROM_OFFSET));
        pData16 = pRawEeprom;
	
        for (addr = ((chain * eepEndLoc) + ATHEROS_EEPROM_OFFSET); addr < ((chain * eepEndLoc) + eepEndLoc); addr++) {
            status = ar5513EepromRead(pDev, addr, &data);
            if (status != A_OK) {
                goto attachError;
            }
            *(pData16++) = data;
        }

    	/* Checksum validation */
    	status = ar5513VerifyEepromChecksum(pRawEeprom, eepEndLoc);
    	if (status != A_OK) {
    	    goto attachError;
    	}
    
    
    	/* Parse the calibration portion of EEPROM into driver structures */
    	status = ar5513ReadEepromIntoDataset(pDev, pRawEeprom, chain);
    	if (status != A_OK) {
    	    goto attachError;
    	}
    }

    status = ar5513EepromRead(pDev, EEPROM_PROTECT_OFFSET, &pEMap->protect);
    if (status != A_OK) {
        goto attachError;
    }

    status = ar5513RecordSerialNumber(pDev, pDev->pHalInfo->serialNumber);
    if (status != A_OK) {
        goto attachError;
    }

    status = ar5513FillCapabilityInfo(pDev);
    if (status != A_OK) {
        goto attachError;
    }

    /* Allocate thermal gain adjustment structure */
    pGainValues = (struct gainValues *)A_DRIVER_MALLOC(sizeof(struct gainValues));
    if (!pGainValues) {
        uiPrintf("ar5513Attach: Could not allocate memory for gain optimization values\n");
        goto attachError;
    }
    A_MEM_ZERO(pGainValues, sizeof(struct gainValues));
    pDev->pHalInfo->pGainValues = pGainValues;
    /* Initialize gain ladder thermal calibration structure */
    ar5513InitializeGainValues(pDev, pGainValues);

    /* Allocate analog bank scratch buffer */
    if (ar5513AllocateRfBanks(pDev, pDev->pHalInfo) == FALSE) {
        uiPrintf("ar5513Attach: Could not allocate memory for RF Banks\n");
        goto attachError;
    }

    /*
     * It's okay for the address to be assigned by software, so we ignore the
     * following error value if returned.
     */
    (void) ar5513GetMacAddr(pDev, &pDev->staConfig.macPermAddr);

#ifdef DEBUG
    uiPrintf("wlan%d revisions: mac %d.%d phy %d.%d analog %d.%d eeprom %d.%d\n",
             pDev->devno, pDev->macVersion, pDev->macRev,
             pDev->phyRev >> 4, pDev->phyRev & 0xF,
             pDev->analog5GhzRev >> 4, pDev->analog5GhzRev & 0xF,
             eepVersion >> 12, eepVersion & 0xFFF);
#endif

    /* Attach rate table for various wireless modes to the hal */
    ar5513AttachRateTables(pDev);

    /* Attach device specific functions to the hal */
    pDev->pHwFunc = &ar5513Funcs;

    /* Attach the RF Hal */
    if (IS_5112(pDev)) {
        pDev->pHalInfo->pRfHal = &ar5112Funcs;
    } 

    /*
    **  NOTE - EAR SUPPORT in Dual Chain Configuration
    **  The following EAR code uses pRawEeprom buffer,
    **  which contains the 2nd EEPROM image.
    */

    if (pEMap->pEepHeader->earStart) {
        ASSERT(pEMap->pEepHeader->earStart > ATHEROS_EEPROM_OFFSET);
        ASSERT(pEMap->pEepHeader->earStart < eepEndLoc);
        earBuffer = &pRawEeprom[pEMap->pEepHeader->earStart - ATHEROS_EEPROM_OFFSET];
        earLocs = eepEndLoc - pEMap->pEepHeader->earStart;
        ASSERT(earBuffer);
        ASSERT(earLocs);
    }
#ifdef DEBUG
    /* Debug EAR overrides normal EAR */
    if (pDev->earDebugLength > 0) {
        earBuffer = pDev->pEarDebug;
        earLocs = pDev->earDebugLength / sizeof(A_UINT16);
    }
#endif

    if (earBuffer) {
        parsedLocs = ar5513EarPreParse(earBuffer, earLocs, &earAlloc);

#ifdef DEBUG
        if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
            showEarAlloc(&earAlloc);
        }
#endif

        /* Do not allocate EAR if no registers exist */
        if (earAlloc.numRHs) {
            if (ar5513EarAllocate(&earAlloc, &(pDev->pHalInfo->pEarHead)) == FALSE) {
                uiPrintf("ar5513Attach: Could not allocate memory for EAR structures\n");
                goto attachError;
            }

            if(!ar5513EarCheckAndFill(pDev->pHalInfo->pEarHead, earBuffer, parsedLocs, &earAlloc)) {
                /* Ear is bad */
                uiPrintf("ar5513Attach: An unrecoverable EAR error was detected\n");
                goto attachError;
            }
#ifdef DEBUG
            if (EarDebugLevel >= EAR_DEBUG_BASIC) {
                printEar(pDev->pHalInfo->pEarHead, &earAlloc);
            }
#endif
        }
    }

    /* Cleanup cached EEPROM */
    A_DRIVER_FREE(pRawEeprom, sizeof(A_UINT16) * (eepEndLoc - ATHEROS_EEPROM_OFFSET));

#if defined(PCI_INTERFACE)
    writePlatformReg(pDev, RST_CIMR, WMAC_INTERRUPT_MASK);
    writePlatformReg(pDev, RST_IF_CTL, ENABLE_PCI_INTERFACE | PCI_CLIENT_INT_ENABLE);
    rdData = readPlatformReg(pDev, PCI_MCFG);
    rdData = rdData | ADDRESS_SHIFT;
    writePlatformReg(pDev, PCI_MCFG, rdData);
#endif

    return A_OK;

attachError:
    uiPrintf("ar5513Attach: EEPROM-related failure\n");
    if (pRawEeprom != NULL) {
        A_DRIVER_FREE(pRawEeprom, sizeof(A_UINT16) * (eepEndLoc - ATHEROS_EEPROM_OFFSET));
    }
    ar5513Detach(pDev);
    return A_HARDWARE;
}

/**************************************************************
 * ar5513Detach
 *
 * Detach removes all hwLayer allocated data structures and places the
 * device into reset.
 */
A_STATUS
ar5513Detach(WLAN_DEV_INFO *pDev)
{
    HAL_INFO       *pInfo;
    struct eepMap  *pData;
    HEADER_WMODE   headerMode;
    int            numChannels;
    int i;
    A_UINT8 chainCnt = 0;

    ASSERT(pDev && pDev->pHalInfo);

#if defined(PCI_INTERFACE)
    writePlatformReg(pDev, 0x14018, 0x01);
#endif

    pInfo = pDev->pHalInfo;
    pData = pInfo->pEepData;

    if (pData) {
        ASSERT(pData->version >= EEPROM_VER3);

        if (pData->pEepHeader) {
            A_DRIVER_FREE(pData->pEepHeader, sizeof(EEP_HEADER_INFO));
        }

        if (pDev->staConfig.txChainCtrl == DUAL_CHAIN ||
            pDev->staConfig.rxChainCtrl == DUAL_CHAIN)
        {
            chainCnt = 2;
        } else {
            chainCnt = 1;
        }

	for (i = 0; i < chainCnt; i++) {
	    if (pData->chain[i].pPcdacInfo) {
		A_DRIVER_FREE(pData->chain[i].pPcdacInfo, sizeof(PCDACS_ALL_MODES));
	    }

	    if (pData->chain[i].pTrgtPowerInfo) {
		A_DRIVER_FREE(pData->chain[i].pTrgtPowerInfo, sizeof(TRGT_POWER_ALL_MODES));
	    }

	    if (pData->chain[i].pRdEdgesPower) {
		A_DRIVER_FREE(pData->chain[i].pRdEdgesPower,
			      sizeof(RD_EDGES_POWER) * NUM_EDGES * NUM_CTLS_3_3);
	    }

	    for (headerMode = headerInfo11A; headerMode <= headerInfo11G; headerMode++) {
		numChannels = pData->chain[i].modePowerArray5112[headerMode].numChannels;
		if (pData->chain[i].modePowerArray5112[headerMode].pChannels) {
		    A_DRIVER_FREE(pData->chain[i].modePowerArray5112[headerMode].pChannels,
				  sizeof(A_UINT16) * numChannels);
		}
		if (pData->chain[i].modePowerArray5112[headerMode].pDataPerChannel) {
		    A_DRIVER_FREE(pData->chain[i].modePowerArray5112[headerMode].pDataPerChannel,
				  sizeof(EXPN_DATA_PER_CHANNEL_5112) * numChannels);
		}
	    }
	}

        A_DRIVER_FREE(pData, sizeof(struct eepMap));
        pInfo->pEepData = NULL;
    }

    if (pInfo->pGainValues) {
        A_DRIVER_FREE(pInfo->pGainValues, sizeof(struct gainValues));
        pInfo->pGainValues = NULL;
    }

    ar5513FreeRfBanks(pDev, pInfo);

    if (pInfo->pEarHead) {
        if (pInfo->pEarHead->numRHs) {
            A_DRIVER_FREE(pInfo->pEarHead->pRH, pInfo->pEarHead->earSize);
            pInfo->pEarHead->pRH = NULL;
        }

        A_DRIVER_FREE(pInfo->pEarHead, sizeof(EAR_HEADER));
        pInfo->pEarHead = NULL;
    }

    return A_OK;
}

/**************************************************************
 * ar5513FillCapabilityInfo
 *
 * Fill all software cached or static hardware state information
 *
 * Returns failure for various EEPROM reading failure cases
 */
static A_STATUS
ar5513FillCapabilityInfo(WLAN_DEV_INFO *pDev)
{
    A_STATUS         status = A_OK;
    HAL_CAPABILITIES *pCap = &(pDev->pHalInfo->halCapabilities);
    EEP_HEADER_INFO  *pEepHeader = pDev->pHalInfo->pEepData->pEepHeader;
    A_UINT16         regDmn;
    A_UINT32         wMode = 0;
    
    /* Read and save EEPROM regDomain */
    status = ar5513EepromRead(pDev, REGULATORY_DOMAIN_OFFSET, &regDmn);
    if (status != A_OK) {
        return status;
    }

    /* Modify reg domain on newer cards that need to work with older sw */
#ifndef BUILD_AP
    if (pDev->pciInfo.SubVendorDeviceID == SUBSYSTEM_ID_NEW_A) {
        if ((regDmn == 0x64) || (regDmn == 0x65)) {
            regDmn += 5;
        } else if (regDmn == 0x41) {
            regDmn = 0x43;
        }
    }
#endif

    pCap->halRegDmn = regDmn;

    /* Get wireless mode */
    if (pEepHeader->Amode) {
        wMode |= MODE_SELECT_11A;
        if (!pEepHeader->turbo5Disable) {
            wMode |= MODE_SELECT_TURBO;
        }
    }
    if (pEepHeader->Bmode) {
        wMode |= MODE_SELECT_11B;
    }
    if (pEepHeader->Gmode) {
        if ((pDev->pciInfo.SubVendorID != SUBVENDOR_ID_NO_G) ||
            (pDev->staConfig.removeNoGSubId))
        {
            wMode |= MODE_SELECT_11G;
            if (!pEepHeader->turbo2Disable) {
                if (pDev->staConfig.abolt & ABOLT_TURBO_G) {
                    wMode |= MODE_SELECT_108G;
                }
            }
        }
    }
    pCap->halWirelessModes = wMode;

    pCap->halLow2GhzChan = 2312;
    if (IS_5112(pDev)) {
        pCap->halHigh2GhzChan = 2500;
    } else {
        pCap->halHigh2GhzChan = 2732;
    }
    pCap->halLow5GhzChan = 4920;
    pCap->halHigh5GhzChan = 6100;
   
    pCap->halCipherCkipSupport = FALSE;
    pCap->halCipherTkipSupport = TRUE;
    pCap->halCipherAesCcmSupport = TRUE;

    pCap->halMicCkipSupport    = FALSE;
    pCap->halMicTkipSupport    = TRUE;
    pCap->halMicAesCcmSupport  = TRUE;

    pCap->halChanSpreadSupport = TRUE;
    pCap->halSleepAfterBeaconBroken = TRUE;
    pCap->halCompressSupport   = FALSE;
    pCap->halBurstSupport      = TRUE;
    pCap->halFastFramesSupport = TRUE;
    pCap->halChapTuningSupport = TRUE;
    pCap->halTurboGSupport     = TRUE;
    pCap->halTurboPrimeSupport = FALSE; // TODO: test TRUE;
    pCap->halDeviceType        = pEepHeader->deviceType;
    pCap->halXrSupport         = FALSE;

    pCap->halTotalQueues = HAL_NUM_TX_QUEUES;
    pCap->halKeyCacheSize = MAC_KEY_CACHE_SIZE;
    
    pCap->halBeamFormSupport = TRUE;
    pCap->halRxCombSupport = TRUE;

    return status;
}

/**************************************************************
 * ar5513RecordSerialNumber
 *
 * Copy EEPROM locations with serial number into provided string
 *
 * Returns failure for various EEPROM reading failure cases
 */
static A_STATUS
ar5513RecordSerialNumber(WLAN_DEV_INFO *pDev, A_CHAR *pSerNum)
{
    A_STATUS status = A_OK;
    A_UINT16 eepVal;
    int      i;

    for (i = 0; i < (EEPROM_SERIAL_NUM_SIZE / 2); i++) {
        status = ar5513EepromRead(pDev, EEPROM_SERIAL_NUM_OFFSET + i, &eepVal);
        if (status != A_OK) {
            break;
        }
        pSerNum[2 * i]     = (A_CHAR)(eepVal & 0xFF);
        pSerNum[2 * i + 1] = (A_CHAR)((eepVal >> 8) & 0xFF);
    }

    /* 11012 misprogram - string reverse all but the last char */
    if ((A_UCHAR)pSerNum[EEPROM_SERIAL_NUM_SIZE - 1] == 0xFF) {
        A_CHAR tempChar;
        for (i = 0; i < (EEPROM_SERIAL_NUM_SIZE - 1) / 2; i ++) {
            tempChar = pSerNum[i];
            pSerNum[i] = pSerNum[EEPROM_SERIAL_NUM_SIZE - 2 - i];
            pSerNum[EEPROM_SERIAL_NUM_SIZE - 2 - i] = tempChar;
        }
        pSerNum[EEPROM_SERIAL_NUM_SIZE - 1] = '\0';
    }

    pSerNum[EEPROM_SERIAL_NUM_SIZE] = '\0';

    return status;
}

/************** EEPROM REV 3 ATTACH FUNCTIONS *****************/

/**************************************************************
 * ar5513VerifyEepromChecksum
 *
 * Verifies the EEPROM calibration data XOR checksum
 */
A_STATUS
ar5513VerifyEepromChecksum(A_UINT16 *pRawEeprom, A_UINT32 eepEndLoc)
{
    A_STATUS status = A_OK;
    A_UINT16 *pData16 = pRawEeprom;
    A_UINT16 chkSum = 0;
    A_UINT32 addr;

    /* EEPROM checksum validation */
    for (addr = ATHEROS_EEPROM_OFFSET; addr < eepEndLoc; addr++) {
        chkSum ^= *(pData16++);
    }

    if (chkSum != 0xffff) {
        uiPrintf("ar5513VerifyEepromChecksum: Checksum %04x != 0xffff\n", chkSum);
        status = A_HARDWARE;
    }

    return status;
}

/**************************************************************
 * ar5513AllocateProm
 *
 * Allocates memory for the EEPROM structures
 */
A_STATUS
ar5513AllocateProm(EEP_MAP *pEepMap, A_UINT8 chainCnt)
{
    /* TODO: only allocate 2.4 info if configured */
    int i;
    int j;

    /* allocate the struct to hold the header info */
    pEepMap->pEepHeader = (EEP_HEADER_INFO *)A_DRIVER_MALLOC(sizeof(EEP_HEADER_INFO));
    if (!pEepMap->pEepHeader) {
        uiPrintf("Unable to allocate eeprom structure for header info\n");
        return A_NO_MEMORY;
    }
    A_MEM_ZERO(pEepMap->pEepHeader, sizeof(EEP_HEADER_INFO));

    for (j = 0; j < chainCnt; j++) {
	/* allocate the struct to hold the pcdac/power info */
	pEepMap->chain[j].pPcdacInfo = (PCDACS_ALL_MODES *)A_DRIVER_MALLOC(sizeof(PCDACS_ALL_MODES));
	if (!pEepMap->chain[j].pPcdacInfo) {
	    uiPrintf("Unable to allocate eeprom structure for pcdac/power info\n");
	    return A_NO_MEMORY;
	}
	A_MEM_ZERO(pEepMap->chain[j].pPcdacInfo, sizeof(PCDACS_ALL_MODES));

	pEepMap->chain[j].pPcdacInfo->numChannels11a = NUM_11A_EEPROM_CHANNELS;
	pEepMap->chain[j].pPcdacInfo->numChannels2_4 = NUM_2_4_EEPROM_CHANNELS;

	for (i = 0; i < NUM_11A_EEPROM_CHANNELS; i ++) {
	    pEepMap->chain[j].pPcdacInfo->DataPerChannel11a[i].numPcdacValues = NUM_PCDAC_VALUES;
	}

	/* the channel list for 2.4 is fixed, fill this in here */
	for (i = 0; i < NUM_2_4_EEPROM_CHANNELS; i++) {
	    pEepMap->chain[j].pPcdacInfo->Channels11b[i] = channels11b[i];
	    pEepMap->chain[j].pPcdacInfo->Channels11g[i] = channels11g[i];
	    pEepMap->chain[j].pPcdacInfo->DataPerChannel11b[i].numPcdacValues = NUM_PCDAC_VALUES;
	    pEepMap->chain[j].pPcdacInfo->DataPerChannel11g[i].numPcdacValues = NUM_PCDAC_VALUES;
	}

	/* allocate the structure to hold target power info */
	pEepMap->chain[j].pTrgtPowerInfo = (TRGT_POWER_ALL_MODES *)A_DRIVER_MALLOC(sizeof(TRGT_POWER_ALL_MODES));
	if (!pEepMap->chain[j].pTrgtPowerInfo) {
	    uiPrintf("Unable to allocate eeprom structure for target power info\n");
	    return A_NO_MEMORY;
	}
	A_MEM_ZERO(pEepMap->chain[j].pTrgtPowerInfo, sizeof(TRGT_POWER_ALL_MODES));

	/* allocate structure for RD edges */
	pEepMap->chain[j].pRdEdgesPower = (RD_EDGES_POWER *)A_DRIVER_MALLOC(sizeof(RD_EDGES_POWER) *
								   NUM_EDGES * NUM_CTLS_3_3);
	if (!pEepMap->chain[j].pRdEdgesPower) {
	    uiPrintf("Unable to allocate eeprom structure for RD edges info\n");
	    return A_NO_MEMORY;
	}
	A_MEM_ZERO(pEepMap->chain[j].pRdEdgesPower, sizeof(RD_EDGES_POWER) * NUM_EDGES * NUM_CTLS_3_3);
    }

    return A_OK;
}

/**************************************************************
 * ar5513ReadEepromFreqPierInfo
 *
 * Now copy EEPROM frequency pier contents into the allocated space
 */
INLINE void
ar5513ReadEepromFreqPierInfo(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT16 offsetIn, A_UINT8 chnIdx)
{
    A_UINT16             tempValue;
    A_UINT32             i;
    A_UINT16             offset = offsetIn;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap           = pDev->pHalInfo->pEepData;
    pEepromData    = pMap->chain[chnIdx].pPcdacInfo;

    if (pMap->version >= EEPROM_VER3_3) {
        for (i = 0; i < pEepromData->numChannels11a; i += 2) {
            tempValue = pRawEeprom[offset++];
            pEepromData->Channels11a[i] = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK_3_3);
            pEepromData->Channels11a[i + 1] = (A_UINT16)( tempValue & FREQ_MASK_3_3);
        }
    } else {
        tempValue = pRawEeprom[offset++];
        pEepromData->Channels11a[0] = (A_UINT16)(( tempValue >> 9 ) & FREQ_MASK);
        pEepromData->Channels11a[1] = (A_UINT16)(( tempValue >> 2 ) & FREQ_MASK);
        pEepromData->Channels11a[2] = (A_UINT16)(( tempValue << 5 ) & FREQ_MASK);

        tempValue = pRawEeprom[offset++];
        pEepromData->Channels11a[2] = (A_UINT16)((( tempValue >> 11 ) & 0x1f) | pEepromData->Channels11a[2]);
        pEepromData->Channels11a[3] = (A_UINT16)(( tempValue >> 4 ) & FREQ_MASK);
        pEepromData->Channels11a[4] = (A_UINT16)(( tempValue << 3 ) & FREQ_MASK);

        tempValue = pRawEeprom[offset++];
        pEepromData->Channels11a[4] = (A_UINT16)((( tempValue >> 13 ) & 0x7) | pEepromData->Channels11a[4]);
        pEepromData->Channels11a[5] = (A_UINT16)(( tempValue >> 6 ) & FREQ_MASK);
        pEepromData->Channels11a[6] =  (A_UINT16)(( tempValue << 1 ) & FREQ_MASK);

        tempValue = pRawEeprom[offset++];
        pEepromData->Channels11a[6] = (A_UINT16)((( tempValue >> 15 ) & 0x1) | pEepromData->Channels11a[6]);
        pEepromData->Channels11a[7] = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK);
        pEepromData->Channels11a[8] =  (A_UINT16)(( tempValue >> 1 ) & FREQ_MASK);
        pEepromData->Channels11a[9] =  (A_UINT16)(( tempValue << 6 ) & FREQ_MASK);

        tempValue = pRawEeprom[offset++];
        pEepromData->Channels11a[9] = (A_UINT16)((( tempValue >> 10 ) & 0x3f) | pEepromData->Channels11a[9]);
    }

    for (i = 0; i < pEepromData->numChannels11a; i++ ) {
        pEepromData->Channels11a[i] = fbin2freq(pMap->version, pEepromData->Channels11a[i], FALSE);
    }
}

/**************************************************************
 * ar5513ReadEepromRawPowerCalInfo
 *
 * Now copy EEPROM Raw Power Calibration per frequency contents
 * into the allocated space
 */
INLINE void
ar5513ReadEepromRawPowerCalInfo(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT16 offSetIn, A_UINT8 chnIdx)
{
    A_UINT16             tempValue;
    A_UINT32             i, j;
    A_UINT32             offset = 0;
    HEADER_WMODE         mode;
    A_UINT16             enable24;
    A_UINT16             numChannels = 0;
    DATA_PER_CHANNEL     *pChannelData = NULL;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap            = pDev->pHalInfo->pEepData;
    pEepromData     = pMap->chain[chnIdx].pPcdacInfo;

    enable24        = pMap->pEepHeader->Bmode;
    /*
     * Group 2:  read raw power data for all frequency piers
     *
     * NOTE: Group 2 contains the raw power calibration information
     * for each of the channels that we recorded in ar5513ReadEepromFreqPiersInfo()
     */
    for (mode = headerInfo11A; mode <= headerInfo11G; mode++) {
        A_UINT16 *pChannels = NULL;
        offset = offSetIn;

        switch (mode) {
        case headerInfo11A:
            offset      += GROUP2_OFFSET;
            numChannels  = pEepromData->numChannels11a;
            pChannelData = pEepromData->DataPerChannel11a;
            pChannels    = pEepromData->Channels11a;
            break;

        case headerInfo11B:
            if (!enable24) {
                continue;
            }
            offset      += GROUP3_OFFSET;
            numChannels  = pEepromData->numChannels2_4;
            pChannelData = pEepromData->DataPerChannel11b;
            pChannels    = pEepromData->Channels11b;

            break;

        case headerInfo11G:
            if (!enable24) {
                continue;
            }
            offset      += GROUP4_OFFSET;
            numChannels  = pEepromData->numChannels2_4;
            pChannelData = pEepromData->DataPerChannel11g;
            pChannels    = pEepromData->Channels11g;
            break;

        default:
            ASSERT(0);
        }

        for (i = 0; i < numChannels; i++) {
            pChannelData->channelValue = pChannels[i];

            tempValue = pRawEeprom[offset++];
            pChannelData->pcdacMax     = (A_UINT16)(( tempValue >> 10 ) & PCDAC_MASK);
            pChannelData->pcdacMin     = (A_UINT16)(( tempValue >> 4  ) & PCDAC_MASK);
            pChannelData->PwrValues[0] = (A_UINT16)(( tempValue << 2  ) & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pChannelData->PwrValues[0] = (A_UINT16)(((tempValue >> 14 ) & 0x3 ) | pChannelData->PwrValues[0]);
            pChannelData->PwrValues[1] = (A_UINT16)((tempValue >> 8 ) & POWER_MASK);
            pChannelData->PwrValues[2] = (A_UINT16)((tempValue >> 2 ) & POWER_MASK);
            pChannelData->PwrValues[3] = (A_UINT16)((tempValue << 4 ) & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pChannelData->PwrValues[3] = (A_UINT16)(( ( tempValue >> 12 ) & 0xf ) | pChannelData->PwrValues[3]);
            pChannelData->PwrValues[4] = (A_UINT16)(( tempValue >> 6 ) & POWER_MASK);
            pChannelData->PwrValues[5] = (A_UINT16)(tempValue  & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pChannelData->PwrValues[6] = (A_UINT16)(( tempValue >> 10 ) & POWER_MASK);
            pChannelData->PwrValues[7] = (A_UINT16)(( tempValue >> 4  ) & POWER_MASK);
            pChannelData->PwrValues[8] = (A_UINT16)(( tempValue << 2  ) & POWER_MASK);

            tempValue = pRawEeprom[offset++];
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
}

/************** EEPROM REV 4 5112 Power Extract FUNCTIONS *****************/
/**************************************************************
 * ar5513readPowerDataFromEeprom5112
 *
 * Pulls the 5112 power data from EEPROM and stores it in a temporary structure
 * that records only the EEPROM calibraiton
 */
static void
ar5513readPowerDataFromEeprom5112(WLAN_DEV_INFO *pDev, EEPROM_POWER_5112 *pPowerSet,
                                  A_UINT16 startOffset, A_UINT16 maxPiers, A_UINT16 *pRawEeprom,
                                  HEADER_WMODE headerMode)
{
    A_UINT16    i;
    A_UINT16    dbmmask             = 0xff;
    A_UINT16    pcdac_delta_mask    = 0x1f;
    A_UINT16    pcdac_mask          = 0x3f;
    A_UINT16    freqmask            = 0xff;
    A_UINT16    offset, numPiers = 0;
    A_UINT16    freq[NUM_11A_EEPROM_CHANNELS];
    A_UINT16    *pCalPiers;
    A_UINT16    version = pDev->pHalInfo->pEepData->version;
    EEP_HEADER_INFO *pHeader = pDev->pHalInfo->pEepData->pEepHeader;

    offset = startOffset;
    if (headerMode == headerInfo11A) {
        while (numPiers < maxPiers) {
            if ((pRawEeprom[offset] & freqmask) == 0) {
                offset++;
                break;
            }
            freq[numPiers] = fbin2freq(version, (pRawEeprom[offset] & freqmask), FALSE);
            numPiers++;

            if (((pRawEeprom[offset] >> 8) & freqmask) == 0) {
                offset++;
                break;
            }
            freq[numPiers] = fbin2freq(version, ((pRawEeprom[offset] >> 8) & freqmask), FALSE);
            numPiers++;
            offset++;
        }
        offset = startOffset + (maxPiers / 2);
    } else {
        pCalPiers = (headerMode == headerInfo11B) ? (pHeader->calPier11b) : (pHeader->calPier11g);
        for (i = 0; i < NUM_2_4_EEPROM_CHANNELS; i++) {
            if (pCalPiers[i] != CHANNEL_UNUSED) {
                freq[numPiers++] = pCalPiers[i];
            }
        }
    }

    pPowerSet->numChannels = numPiers;

    for (i = 0; i < numPiers; i++) {
        pPowerSet->pChannels[i] = freq[i];
        pPowerSet->pDataPerChannel[i].channelValue = freq[i];

        pPowerSet->pDataPerChannel[i].pwr1_xg0 = (A_INT16)((pRawEeprom[offset] & dbmmask) - ((pRawEeprom[offset] >> 7) & 0x1)*256);
        pPowerSet->pDataPerChannel[i].pwr2_xg0 = (A_INT16)(((pRawEeprom[offset] >> 8) & dbmmask) - ((pRawEeprom[offset] >> 15) & 0x1)*256);

        offset++;
        pPowerSet->pDataPerChannel[i].pwr3_xg0 = (A_INT16)((pRawEeprom[offset] & dbmmask) - ((pRawEeprom[offset] >> 7) & 0x1)*256);
        pPowerSet->pDataPerChannel[i].pwr4_xg0 = (A_INT16)(((pRawEeprom[offset] >> 8) & dbmmask) - ((pRawEeprom[offset] >> 15) & 0x1)*256);

        offset++;
        pPowerSet->pDataPerChannel[i].pcd2_delta_xg0 = (A_UINT16)(pRawEeprom[offset] & pcdac_delta_mask);
        pPowerSet->pDataPerChannel[i].pcd3_delta_xg0 = (A_UINT16)((pRawEeprom[offset] >> 5) & pcdac_delta_mask);
        pPowerSet->pDataPerChannel[i].pcd4_delta_xg0 = (A_UINT16)((pRawEeprom[offset] >> 10) & pcdac_delta_mask);

        offset++;
        pPowerSet->pDataPerChannel[i].pwr1_xg3 = (A_INT16)((pRawEeprom[offset] & dbmmask) - ((pRawEeprom[offset] >> 7) & 0x1)*256);
        pPowerSet->pDataPerChannel[i].pwr2_xg3 = (A_INT16)(((pRawEeprom[offset] >> 8) & dbmmask) - ((pRawEeprom[offset] >> 15) & 0x1)*256);

        offset++;
        pPowerSet->pDataPerChannel[i].pwr3_xg3 = (A_INT16)((pRawEeprom[offset] & dbmmask) - ((pRawEeprom[offset] >> 7) & 0x1)*256);
        if (version >= EEPROM_VER4_3) {
            pPowerSet->pDataPerChannel[i].maxPower_t4 = pPowerSet->pDataPerChannel[i].pwr4_xg0;     
            pPowerSet->pDataPerChannel[i].pcd1_xg0 = (A_UINT16)((pRawEeprom[offset] >> 8) & pcdac_mask);
        } else {
            pPowerSet->pDataPerChannel[i].maxPower_t4 = (A_INT16)(((pRawEeprom[offset] >> 8) & dbmmask) - ((pRawEeprom[offset] >> 15) & 0x1)*256);
            pPowerSet->pDataPerChannel[i].pcd1_xg0 = 1;
        }

        offset++;
    }
    pPowerSet->xpdMask = pHeader->xgain[headerMode];
}

/**************************************************************
 * ar5513AllocExpnPower5112
 *
 * Allocate the power information based on the number of channels
 * recorded by the calibration.  These values are then initialized.
 */
static A_STATUS
ar5513AllocExpnPower5112(EEPROM_POWER_EXPN_5112 *pPowerExpn, A_UINT16 numChannels,
                               A_UINT16 *pChanList)
{
    A_UINT16    i, j, channelValue;

    /* Allocate the channel array */
    pPowerExpn->pChannels = (A_UINT16 *)A_DRIVER_MALLOC(sizeof(A_UINT16) * numChannels);
    if (NULL == pPowerExpn->pChannels) {
        uiPrintf("unable to allocate raw data struct (gen3)\n");
        return(A_NO_MEMORY);
    }

    /* Allocate the Power Data for each channel */
    pPowerExpn->pDataPerChannel = (EXPN_DATA_PER_CHANNEL_5112 *)A_DRIVER_MALLOC(sizeof(EXPN_DATA_PER_CHANNEL_5112) * numChannels);
    if (NULL == pPowerExpn->pDataPerChannel) {
        uiPrintf("unable to allocate raw data struct data per channel(gen3)\n");
        A_DRIVER_FREE(pPowerExpn->pChannels, sizeof(A_UINT16) * numChannels);
        return(A_NO_MEMORY);
    }

    pPowerExpn->numChannels = numChannels;

    for (i = 0; i < numChannels; i++) {
        channelValue = pChanList[i];

        pPowerExpn->pChannels[i] = channelValue;
        pPowerExpn->pDataPerChannel[i].channelValue = channelValue;

        for (j = 0; j < NUM_XPD_PER_CHANNEL; j++) {
            pPowerExpn->pDataPerChannel[i].pDataPerXPD[j].xpd_gain = j;
            pPowerExpn->pDataPerChannel[i].pDataPerXPD[j].numPcdacs = 0;
        }
        pPowerExpn->pDataPerChannel[i].pDataPerXPD[0].numPcdacs = 4;
        pPowerExpn->pDataPerChannel[i].pDataPerXPD[3].numPcdacs = 3;
    }

    return(A_OK);
}

/**************************************************************
 * ar5513ExpandPower5112
 *
 * Expand the dataSet from the calibration information into the
 * final power structure for 5112
 */
static A_BOOL
ar5513ExpandPower5112(EEPROM_POWER_5112 *pCalDataset, EEPROM_POWER_EXPN_5112 *pPowerExpn)
{
    A_UINT16                     ii, jj, kk;
    A_INT16                      maxPower_t4;
    EXPN_DATA_PER_XPD_5112       *pExpnXPD;
    EEPROM_DATA_PER_CHANNEL_5112 *pCalCh;   /* ptr to array of info held per channel */
    A_UINT16                     xgainList[2];
    A_UINT16                     xpdMask;

    pPowerExpn->xpdMask = pCalDataset->xpdMask;

    xgainList[0] = 0xDEAD;
    xgainList[1] = 0xDEAD;

    kk = 0;
    xpdMask = pPowerExpn->xpdMask;

    for (jj = 0; jj < NUM_XPD_PER_CHANNEL; jj++) {
        if (((xpdMask >> jj) & 1) > 0) {
            if (kk > 1) {
                uiPrintf("ar5513ExpandPower5112: Error - A maximum of 2 xpdGains supported in dataset\n");
                return FALSE;
            }
            xgainList[kk++] = (A_UINT16) jj;
        }
    }

    pPowerExpn->numChannels = pCalDataset->numChannels;
    for (ii = 0; ii < pPowerExpn->numChannels; ii++) {
        pCalCh = &(pCalDataset->pDataPerChannel[ii]);
        pPowerExpn->pDataPerChannel[ii].channelValue = pCalCh->channelValue;
        pPowerExpn->pDataPerChannel[ii].maxPower_t4  = pCalCh->maxPower_t4;
        maxPower_t4 = pPowerExpn->pDataPerChannel[ii].maxPower_t4;

        if (xgainList[1] == 0xDEAD) {
            for (jj=0; jj < NUM_XPD_PER_CHANNEL; jj++) {
                pPowerExpn->pDataPerChannel[ii].pDataPerXPD[jj].numPcdacs = 0;
            }

            jj = xgainList[0];
            pExpnXPD = &(pPowerExpn->pDataPerChannel[ii].pDataPerXPD[jj]);
            pExpnXPD->numPcdacs = 4;
            pExpnXPD->pcdac[0] = pCalCh->pcd1_xg0;
            pExpnXPD->pcdac[1] = (A_UINT16)(pExpnXPD->pcdac[0] + pCalCh->pcd2_delta_xg0);
            pExpnXPD->pcdac[2] = (A_UINT16)(pExpnXPD->pcdac[1] + pCalCh->pcd3_delta_xg0);
            pExpnXPD->pcdac[3] = (A_UINT16)(pExpnXPD->pcdac[2] + pCalCh->pcd4_delta_xg0);

            pExpnXPD->pwr_t4[0] = pCalCh->pwr1_xg0;
            pExpnXPD->pwr_t4[1] = pCalCh->pwr2_xg0;
            pExpnXPD->pwr_t4[2] = pCalCh->pwr3_xg0;
            pExpnXPD->pwr_t4[3] = pCalCh->pwr4_xg0;

        } else {
            for (jj=0; jj < NUM_XPD_PER_CHANNEL; jj++) {
                pPowerExpn->pDataPerChannel[ii].pDataPerXPD[jj].numPcdacs = 0;
            }

            pPowerExpn->pDataPerChannel[ii].pDataPerXPD[xgainList[0]].pcdac[0] = pCalCh->pcd1_xg0;
            pPowerExpn->pDataPerChannel[ii].pDataPerXPD[xgainList[1]].pcdac[0] = 20;
            pPowerExpn->pDataPerChannel[ii].pDataPerXPD[xgainList[1]].pcdac[1] = 35;
            pPowerExpn->pDataPerChannel[ii].pDataPerXPD[xgainList[1]].pcdac[2] = 63;

            jj = xgainList[0];
            pExpnXPD = &(pPowerExpn->pDataPerChannel[ii].pDataPerXPD[jj]);
            pExpnXPD->numPcdacs = 4;
            pExpnXPD->pcdac[1] = (A_UINT16)(pExpnXPD->pcdac[0] + pCalCh->pcd2_delta_xg0);
            pExpnXPD->pcdac[2] = (A_UINT16)(pExpnXPD->pcdac[1] + pCalCh->pcd3_delta_xg0);
            pExpnXPD->pcdac[3] = (A_UINT16)(pExpnXPD->pcdac[2] + pCalCh->pcd4_delta_xg0);
            pExpnXPD->pwr_t4[0] = pCalCh->pwr1_xg0;
            pExpnXPD->pwr_t4[1] = pCalCh->pwr2_xg0;
            pExpnXPD->pwr_t4[2] = pCalCh->pwr3_xg0;
            pExpnXPD->pwr_t4[3] = pCalCh->pwr4_xg0;

            jj = xgainList[1];
            pExpnXPD = &(pPowerExpn->pDataPerChannel[ii].pDataPerXPD[jj]);
            pExpnXPD->numPcdacs = 3;

            pExpnXPD->pwr_t4[0] = pCalCh->pwr1_xg3;
            pExpnXPD->pwr_t4[1] = pCalCh->pwr2_xg3;
            pExpnXPD->pwr_t4[2] = pCalCh->pwr3_xg3;
        }
    }
    return TRUE;
}

/**************************************************************
 * ar5513ReadEepromPowerCal5112
 *
 * Allocate, expand and fill expanded power structure for 5112
 * cal'ed EEPROM across all modes.
 */
static A_STATUS
ar5513ReadEepromPowerCal5112(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom,
                             A_UINT16 offSetIn, EEPROM_POWER_EXPN_5112 *pPowerExpn)
{
    A_STATUS          status = A_OK;
    A_UINT16          offset= offSetIn;
    A_UINT16          maxPiers = 0;
    EEPROM_POWER_5112 eepPower;
    HEADER_WMODE      headerMode;
    EEP_HEADER_INFO   *pHeaderInfo;

    pHeaderInfo = pDev->pHalInfo->pEepData->pEepHeader;

    for (headerMode = headerInfo11A; headerMode <= headerInfo11G; headerMode++) {
        switch (headerMode) {
        case headerInfo11A:
            if (!pHeaderInfo->Amode) {
                continue;
            }
            maxPiers = NUM_11A_EEPROM_CHANNELS;
            break;
        case headerInfo11B:
            if (!pHeaderInfo->Bmode) {
                continue;
            }
            maxPiers = NUM_2_4_EEPROM_CHANNELS;
            break;
        case headerInfo11G:
            if (!pHeaderInfo->Gmode) {
                continue;
            }
            maxPiers = NUM_2_4_EEPROM_CHANNELS;;
            break;
        }

        /* Zero the reused basic power structure */
        A_MEM_ZERO(&eepPower, sizeof(EEPROM_POWER_5112));

        /* Copy from the EEPROM locations to the basic power structure */
        ar5513readPowerDataFromEeprom5112(pDev, &eepPower, offset, maxPiers, pRawEeprom, headerMode);

        /* Allocate the expanded Power structure for easier use by reset */
        status = ar5513AllocExpnPower5112(&pPowerExpn[headerMode], eepPower.numChannels, eepPower.pChannels);
        if (status != A_OK) {
            uiPrintf("ar5513ReadEepromPowerCal5112: Failed to allocate power structs\n");
            return status;
        }

        /* Copy and expand the basic power structure into the expanded structure */
        if(ar5513ExpandPower5112(&eepPower, &pPowerExpn[headerMode]) == FALSE) {
            return A_ERROR;
        }
        offset += (eepPower.numChannels * 5);
        if (headerMode == headerInfo11A) {
            /* Add 5 locations for the 11A frequency piers */
            offset += 5;
        }
    }

    return(status);
}
/************** END EEPROM REV 4 5112 Power Extract *****************/

/**************************************************************
 * ar5513ReadEepromTargetPowerCalInfo
 *
 * Copy EEPROM Target Power Calbration per rate contents
 * into the allocated space
 */
INLINE void
ar5513ReadEepromTargetPowerCalInfo(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT16 offsetIn, A_UINT16 *endOffset, A_UINT8 chnIdx)
{
    A_UINT16             tempValue;
    A_UINT32             i;
    A_UINT16             offset = 0;
    A_UINT16             enable24;
    HEADER_WMODE         mode;
    A_UINT16             numChannels = 0;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap            = pDev->pHalInfo->pEepData;
    enable24        = pMap->pEepHeader->Bmode;

    for (mode = headerInfo11A; mode <= headerInfo11G; mode++) {
        TRGT_POWER_INFO *pPowerInfo = NULL;
        A_UINT16        *pNumTrgtChannels = NULL;

        offset = offsetIn;

        switch (mode) {
        case headerInfo11A:
            offset          += GROUP5_OFFSET;
            numChannels      = NUM_TEST_FREQUENCIES;
            pPowerInfo       = pMap->chain[chnIdx].pTrgtPowerInfo->trgtPwr_11a;
            pNumTrgtChannels = &pMap->chain[chnIdx].pTrgtPowerInfo->numTargetPwr_11a;
            break;

        case headerInfo11B:
            offset          += GROUP6_OFFSET;
            if (!enable24) {
                /* Keep the offset consistent */
                offset += NUM_TARGET_POWER_LOCATIONS_11B;
                continue;
            }
            numChannels      = 2;
            pPowerInfo       = pMap->chain[chnIdx].pTrgtPowerInfo->trgtPwr_11b;
            pNumTrgtChannels = &pMap->chain[chnIdx].pTrgtPowerInfo->numTargetPwr_11b;
            break;

        case headerInfo11G:
            offset          += GROUP7_OFFSET;
            if (!enable24) {
                /* Keep the offset consistent */
                offset += NUM_TARGET_POWER_LOCATIONS_11G;
                continue;
            }
            numChannels      = 3;
            pPowerInfo       = pMap->chain[chnIdx].pTrgtPowerInfo->trgtPwr_11g;
            pNumTrgtChannels = &pMap->chain[chnIdx].pTrgtPowerInfo->numTargetPwr_11g;
            break;

        default:
            ASSERT(0);
        } //end mode switch

        *pNumTrgtChannels = 0;
        for (i = 0; i < numChannels; i++) {
            tempValue = pRawEeprom[offset++];
            if (pMap->version >= EEPROM_VER3_3) {
                pPowerInfo->testChannel = (A_UINT16)(( tempValue >> 8 ) & 0xff);
            } else {
                pPowerInfo->testChannel = (A_UINT16)(( tempValue >> 9 ) & 0x7f);
            }

            if (pPowerInfo->testChannel != 0) {
                /* get the channel value and read rest of info */
                if (mode == headerInfo11A) {
                    pPowerInfo->testChannel = fbin2freq(pMap->version, pPowerInfo->testChannel, FALSE);
                } else {
                    pPowerInfo->testChannel = fbin2freq(pMap->version, pPowerInfo->testChannel, TRUE);
                }

                if (pMap->version >= EEPROM_VER3_3) {
                    pPowerInfo->twicePwr6_24 = (A_UINT16)(( tempValue >> 2 ) & POWER_MASK);
                    pPowerInfo->twicePwr36   = (A_UINT16)(( tempValue << 4 ) & POWER_MASK);
                } else {
                    pPowerInfo->twicePwr6_24 = (A_UINT16)(( tempValue >> 3 ) & POWER_MASK);
                    pPowerInfo->twicePwr36   = (A_UINT16)(( tempValue << 3 ) & POWER_MASK);
                }

                tempValue = pRawEeprom[offset++];
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
            } else {
                /* Keep the offset consistent */
                offset++;
            }
            pPowerInfo++;
        }
    }
    *endOffset = offset;
}

/**************************************************************
 * ar5513ReadEepromCTLInfo
 *
 * Now copy EEPROM Coformance Testing Limits contents
 * into the allocated space
 */
INLINE void
ar5513ReadEepromCTLInfo(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT16 offsetIn, A_UINT8 chnIdx)
{
    A_UINT16             tempValue;
    A_UINT32             i, j;
    A_UINT32             offset = offsetIn;
    RD_EDGES_POWER       *pRdEdgePwrInfo;
    PCDACS_ALL_MODES     *pEepromData;
    struct eepMap        *pMap;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pMap           = pDev->pHalInfo->pEepData;
    pEepromData    = pMap->chain[chnIdx].pPcdacInfo;
    pRdEdgePwrInfo = pMap->chain[chnIdx].pRdEdgesPower;

    for (i = 0; i < pMap->pEepHeader->numCtls; i++) {
        if (pMap->pEepHeader->ctl[i] == 0) {
            /* Move offset and edges */
            offset += ((pMap->version >= EEPROM_VER3_3) ? 8 : 7);
            pRdEdgePwrInfo += NUM_EDGES;
            continue;
        }

        if (pMap->version >= EEPROM_VER3_3) {
            for (j = 0; j < NUM_EDGES; j += 2) {
                tempValue = pRawEeprom[offset++];
                pRdEdgePwrInfo[j].rdEdge = (A_UINT16)((tempValue >> 8) & FREQ_MASK_3_3);
                pRdEdgePwrInfo[j + 1].rdEdge = (A_UINT16)(tempValue & FREQ_MASK_3_3);
            }
            for (j = 0; j < NUM_EDGES; j += 2) {
                tempValue = pRawEeprom[offset++];
                pRdEdgePwrInfo[j].twice_rdEdgePower =
                                (A_UINT16)((tempValue >> 8) & POWER_MASK);
                pRdEdgePwrInfo[j].flag = (A_BOOL)((tempValue >> 14) & 1);
                pRdEdgePwrInfo[j + 1].twice_rdEdgePower =
                                (A_UINT16)(tempValue & POWER_MASK);
                pRdEdgePwrInfo[j + 1].flag = (A_BOOL)((tempValue >> 6) & 1);
            }
        } else {
            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[0].rdEdge = (A_UINT16)(( tempValue >> 9 ) & FREQ_MASK);
            pRdEdgePwrInfo[1].rdEdge = (A_UINT16)(( tempValue >> 2 ) & FREQ_MASK);
            pRdEdgePwrInfo[2].rdEdge = (A_UINT16)(( tempValue << 5 ) & FREQ_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[2].rdEdge = (A_UINT16)((( tempValue >> 11 ) & 0x1f) | pRdEdgePwrInfo[2].rdEdge);
            pRdEdgePwrInfo[3].rdEdge = (A_UINT16)(( tempValue >> 4 ) & FREQ_MASK);
            pRdEdgePwrInfo[4].rdEdge = (A_UINT16)(( tempValue << 3 ) & FREQ_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[4].rdEdge = (A_UINT16)((( tempValue >> 13 ) & 0x7) | pRdEdgePwrInfo[4].rdEdge);
            pRdEdgePwrInfo[5].rdEdge = (A_UINT16)(( tempValue >> 6 ) & FREQ_MASK);
            pRdEdgePwrInfo[6].rdEdge = (A_UINT16)(( tempValue << 1 ) & FREQ_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[6].rdEdge = (A_UINT16)((( tempValue >> 15 ) & 0x1) | pRdEdgePwrInfo[6].rdEdge);
            pRdEdgePwrInfo[7].rdEdge = (A_UINT16)(( tempValue >> 8 ) & FREQ_MASK);

            pRdEdgePwrInfo[0].twice_rdEdgePower = (A_UINT16)(( tempValue >> 2 ) & POWER_MASK);
            pRdEdgePwrInfo[1].twice_rdEdgePower = (A_UINT16)(( tempValue << 4 ) & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[1].twice_rdEdgePower = (A_UINT16)((( tempValue >> 12 ) & 0xf) | pRdEdgePwrInfo[1].twice_rdEdgePower);
            pRdEdgePwrInfo[2].twice_rdEdgePower = (A_UINT16)(( tempValue >> 6 ) & POWER_MASK);
            pRdEdgePwrInfo[3].twice_rdEdgePower = (A_UINT16)(tempValue & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[4].twice_rdEdgePower = (A_UINT16)(( tempValue >> 10 ) & POWER_MASK);
            pRdEdgePwrInfo[5].twice_rdEdgePower = (A_UINT16)(( tempValue >> 4 ) & POWER_MASK);
            pRdEdgePwrInfo[6].twice_rdEdgePower = (A_UINT16)(( tempValue << 2 ) & POWER_MASK);

            tempValue = pRawEeprom[offset++];
            pRdEdgePwrInfo[6].twice_rdEdgePower = (A_UINT16)((( tempValue >> 14 ) & 0x3) | pRdEdgePwrInfo[6].twice_rdEdgePower);
            pRdEdgePwrInfo[7].twice_rdEdgePower = (A_UINT16)(( tempValue >> 8 ) & POWER_MASK);
        }

        for (j = 0; j < NUM_EDGES; j++ ) {
            if (pRdEdgePwrInfo[j].rdEdge != 0 || pRdEdgePwrInfo[j].twice_rdEdgePower != 0) {
                if (((pMap->pEepHeader->ctl[i] & CTL_MODE_M) == CTL_11A) ||
                    ((pMap->pEepHeader->ctl[i] & CTL_MODE_M) == CTL_TURBO))
                {
                    pRdEdgePwrInfo[j].rdEdge = fbin2freq(pMap->version, pRdEdgePwrInfo[j].rdEdge, FALSE);
                } else {
                    pRdEdgePwrInfo[j].rdEdge = fbin2freq(pMap->version, pRdEdgePwrInfo[j].rdEdge, TRUE);
                }
            }
        }
        pRdEdgePwrInfo += NUM_EDGES;
    }
}

/**************************************************************
 * ar5513ReadEepromIntoDataset
 *
 * Now verify and copy EEPROM contents into the allocated space
 */
static A_STATUS
ar5513ReadEepromIntoDataset(WLAN_DEV_INFO *pDev, A_UINT16 *pRawEeprom, A_UINT8 chnIdx)
{
    A_STATUS      status = A_OK;
    struct eepMap *pMap;
    A_UINT16      offset, endOffset;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    pMap = pDev->pHalInfo->pEepData;

    /* Read the header information here */
    ar5513ReadHeaderInfo(pDev, pMap->pEepHeader, pRawEeprom);

    /* Require 5112 devices to have EEPROM 4.0 EEP_MAP set */
    if (IS_5112(pDev) && !pMap->pEepHeader->eepMap) {
        uiPrintf("ar5513ReadEepromIntoDataset: ERROR - 5112 devices must have an EEPROM 4.0 with the EEP_MAP set\n");
        status = A_HARDWARE;
        return status;
    }

    /*
     * Group 1: frequency pier locations readback
     * check that the structure has been populated with enough space to hold the channels
     * Frequency piers are selected calibrated frequencies... all other frequencies
     * interpolate their values between the nearest piers.
     *
     * NOTE: Group 1 contains the 5 GHz channel numbers that have dBm->pcdac calibrated information
     */
    offset = (A_UINT16)(((pMap->version >= EEPROM_VER3_3) ? EEPROM_GROUPS_OFFSET3_3 : EEPROM_GROUPS_OFFSET3_2) +
        GROUP1_OFFSET - ATHEROS_EEPROM_OFFSET);
    ar5513ReadEepromFreqPierInfo(pDev, pRawEeprom, offset, chnIdx);

    /*
     * Group 2:  readback data for all frequency piers
     *
     * NOTE: Group 2 contains the raw power calibration information for each of the channels
     * that we recorded above
     */
    offset = (A_UINT16)(((pMap->version >= EEPROM_VER3_3) ?
             EEPROM_GROUPS_OFFSET3_3 : EEPROM_GROUPS_OFFSET3_2) - ATHEROS_EEPROM_OFFSET);
    if ((pMap->version >= EEPROM_VER4_0) && pMap->pEepHeader->eepMap) {
        status = ar5513ReadEepromPowerCal5112(pDev, pRawEeprom, offset, pMap->chain[chnIdx].modePowerArray5112);
        if (status != A_OK) {
            return status;
        }
    } else {
        ar5513ReadEepromRawPowerCalInfo(pDev, pRawEeprom, offset, chnIdx);
    }

    /*
     * Group 5: target power values per rate
     *
     * NOTE: Group 5 contains the recorded maximum power in dB that can be attained
     * for the given rate.
     */
    /* Read the power per rate info for test channels */
    if (pMap->version >= EEPROM_VER4_0) {
        offset = (A_UINT16)(pMap->pEepHeader->targetPowersStart - ATHEROS_EEPROM_OFFSET - GROUP5_OFFSET);
    } else if (pMap->version >= EEPROM_VER3_3) {
        offset = EEPROM_GROUPS_OFFSET3_3 - ATHEROS_EEPROM_OFFSET;
    } else {
        offset = EEPROM_GROUPS_OFFSET3_2 - ATHEROS_EEPROM_OFFSET;
    }
    ar5513ReadEepromTargetPowerCalInfo(pDev, pRawEeprom, offset, &endOffset, chnIdx);

    /*
     * Group 8: Conformance Test Limits information
     *
     * NOTE: Group 8 contains the values to limit the maximum transmit power
     * value based on any band edge violations.
     */
    /* Read the RD edge power limits */
    ar5513ReadEepromCTLInfo(pDev, pRawEeprom, endOffset, chnIdx);

    return status;
}

/**************************************************************
 * ar5513ReadHeaderInfo
 *
 * Read the individual header fields for a Rev 3 EEPROM
 */
static void
ar5513ReadHeaderInfo(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo, A_UINT16* pRawEeprom)
{
    A_UINT16 tempValue;
    A_UINT32 offset;
    A_UINT16 i;
    A_UINT32 *pHeadOff;
    A_UINT16 version = pDev->pHalInfo->pEepData->version;

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

    /* Initialize cckOfdmGainDelta for < 4.2 EEPROM's */
    pHeaderInfo->cckOfdmGainDelta = CCK_OFDM_GAIN_DELTA;
    pHeaderInfo->scaledCh14FilterCckDelta = TENX_CH14_FILTER_CCK_DELTA_INIT;

    if (version >= EEPROM_VER3_3) {
        pHeadOff = headerOffset3_3;
        pHeaderInfo->numCtls = NUM_CTLS_3_3;
    } else {
        pHeadOff = headerOffset3_0;
        pHeaderInfo->numCtls = NUM_CTLS;
    }

    offset = pHeadOff[0] - ATHEROS_EEPROM_OFFSET;
    tempValue = pRawEeprom[offset++];
    pHeaderInfo->rfKill           = (tempValue >> 14) & 0x01;
    pHeaderInfo->deviceType       = (tempValue >> 11) & 0x07;
    pHeaderInfo->turbo2WMaxPower5 = (tempValue >> 4) & 0x7F;
    pHeaderInfo->turbo5Disable    = (tempValue >> 15) & 0x01;

    if (version >= EEPROM_VER4_0) {
        pHeaderInfo->turbo2Disable = (tempValue >> 3) & 0x01;
    } else {
        pHeaderInfo->turbo2Disable = (pDev->staConfig.disableTurboG == USE_ABOLT) ? 0 : 1;
    }
    pHeaderInfo->Gmode            = (tempValue >> 2) & 0x01;
    pHeaderInfo->Bmode            = (tempValue >> 1) & 0x01;
    pHeaderInfo->Amode            = tempValue & 0x01;

    offset = pHeadOff[1] - ATHEROS_EEPROM_OFFSET;
    tempValue = pRawEeprom[offset++];
    pHeaderInfo->antennaGainMax[0] = (A_INT8)((tempValue >> 8) & 0xFF);
    pHeaderInfo->antennaGainMax[1] = (A_INT8)(tempValue & 0xFF);

    if (version >= EEPROM_VER4_0) {
        tempValue = pRawEeprom[offset++];
        pHeaderInfo->eepMap   = (tempValue >> 14) & 0x03;
        pHeaderInfo->earStart = tempValue & 0x0FFF;
        tempValue = pRawEeprom[offset++];
        pHeaderInfo->targetPowersStart = tempValue & 0x0FFF;
        pHeaderInfo->exist32kHzCrystal = (A_BOOL)((tempValue >> 14) & 0x01);
    }

    /* Read the moded sections of the EEPROM header in the order A, B, G */
    for (i = headerInfo11A; i <= headerInfo11G; i++) {
        /* Set the offset via the index */
        offset = pHeadOff[2 + i] - ATHEROS_EEPROM_OFFSET;

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->switchSettling[i] = (A_UINT16)((tempValue >> 8) & 0x7f);
        pHeaderInfo->txrxAtten[i] = (A_UINT16)((tempValue >> 2) & 0x3f);
        pHeaderInfo->antennaControl[0][i] = (A_UINT16)((tempValue << 4) & 0x3f);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->antennaControl[0][i] = (A_UINT16)(((tempValue >> 12) & 0x0f) | pHeaderInfo->antennaControl[0][i]);
        pHeaderInfo->antennaControl[1][i] = (A_UINT16)((tempValue >> 6) & 0x3f);
        pHeaderInfo->antennaControl[2][i] = (A_UINT16)(tempValue  & 0x3f);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->antennaControl[3][i] = (A_UINT16)((tempValue >> 10)  & 0x3f);
        pHeaderInfo->antennaControl[4][i] = (A_UINT16)((tempValue >> 4)  & 0x3f);
        pHeaderInfo->antennaControl[5][i] = (A_UINT16)((tempValue << 2)  & 0x3f);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->antennaControl[5][i] = (A_UINT16)(((tempValue >> 14)  & 0x03) | pHeaderInfo->antennaControl[5][i]);
        pHeaderInfo->antennaControl[6][i] = (A_UINT16)((tempValue >> 8)  & 0x3f);
        pHeaderInfo->antennaControl[7][i] = (A_UINT16)((tempValue >> 2)  & 0x3f);
        pHeaderInfo->antennaControl[8][i] = (A_UINT16)((tempValue << 4)  & 0x3f);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->antennaControl[8][i] = (A_UINT16)(((tempValue >> 12)  & 0x0f) | pHeaderInfo->antennaControl[8][i]);
        pHeaderInfo->antennaControl[9][i] = (A_UINT16)((tempValue >> 6)  & 0x3f);
        pHeaderInfo->antennaControl[10][i] = (A_UINT16)(tempValue & 0x3f);

        tempValue = pRawEeprom[offset++];
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
            tempValue = pRawEeprom[offset++];
            pHeaderInfo->ob3 = (A_UINT16)(((tempValue >> 15)  & 0x01) | pHeaderInfo->ob3);
            pHeaderInfo->db3 = (A_UINT16)((tempValue >> 12)  & 0x07);
            pHeaderInfo->ob2 = (A_UINT16)((tempValue >> 9)  & 0x07);
            pHeaderInfo->db2 = (A_UINT16)((tempValue >> 6)  & 0x07);
            pHeaderInfo->ob1 = (A_UINT16)((tempValue >> 3)  & 0x07);
            pHeaderInfo->db1 = (A_UINT16)(tempValue & 0x07);
        }

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->txEndToXLNAOn[i] = (A_UINT16)((tempValue >> 8)  & 0xff);
        pHeaderInfo->thresh62[i] = (A_UINT16)(tempValue  & 0xff);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->txEndToXPAOff[i] = (A_UINT16)((tempValue >> 8)  & 0xff);
        pHeaderInfo->txFrameToXPAOn[i] = (A_UINT16)(tempValue  & 0xff);

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->pgaDesiredSize[i] = (A_INT8)((tempValue >> 8)  & 0xff);
        pHeaderInfo->noiseFloorThresh[i] = (tempValue  & 0xff);
        if (pHeaderInfo->noiseFloorThresh[i] & 0x80) {
            pHeaderInfo->noiseFloorThresh[i] = 0 - ((pHeaderInfo->noiseFloorThresh[i] ^ 0xff) + 1);
        }

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->xlnaGain[i] = (A_UINT16)((tempValue >> 5)  & 0xff);
        pHeaderInfo->xgain[i] = (A_UINT16)((tempValue >> 1)  & 0x0f);
        pHeaderInfo->xpd[i] = (A_UINT8)(tempValue  & 0x01);
        if (version >= EEPROM_VER4_0) {
            switch (i) {
            case headerInfo11A:
                pHeaderInfo->fixedBias5 = (tempValue >> 13) & 0x1;
                break;
            case headerInfo11G:
                pHeaderInfo->fixedBias2 = (tempValue >> 13) & 0x1;
                break;
            }
        }
        if (version >= EEPROM_VER3_3) {
            tempValue = pRawEeprom[offset++];
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
            case headerInfo11A:
                pHeaderInfo->xrTargetPower5 = tempValue & 0x3F;
                break;
            }
        }
        if (version >= EEPROM_VER3_4) {
            pHeaderInfo->gainI[i] = (tempValue >> 13) & 0x07;

            tempValue = pRawEeprom[offset++];
            pHeaderInfo->gainI[i] |= (tempValue << 3) & 0x38;
            if (i == headerInfo11G) {
                pHeaderInfo->cckOfdmPwrDelta = (tempValue >> 3) & 0xFF;
    			if(version >= EEPROM_VER4_6) {
	    			pHeaderInfo->scaledCh14FilterCckDelta = (tempValue >> 11)  & 0x1f;
		    	}
            }
            if ((i == headerInfo11A) && (version >= EEPROM_VER4_0)) {
                pHeaderInfo->iqCalI[0] = (tempValue >> 8) & 0x3F;
                pHeaderInfo->iqCalQ[0] = (tempValue >> 3) & 0x1F;
            }
        } else {
            pHeaderInfo->gainI[i] = 10;
            pHeaderInfo->cckOfdmPwrDelta = TENX_OFDM_CCK_DELTA_INIT;
        }
        if (version >= EEPROM_VER4_0) {
            if (i == headerInfo11B) {
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->calPier11b[0] = fbin2freq(version, tempValue & 0xff, TRUE);
                pHeaderInfo->calPier11b[1] = fbin2freq(version, (tempValue >> 8) & 0xff, TRUE);
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->calPier11b[2] = fbin2freq(version, tempValue & 0xff, TRUE);
                if (version >= EEPROM_VER4_1) {
                    pHeaderInfo->rxtxMargin[headerInfo11B] = (tempValue >> 8) & 0x3f;
                }
            } else if (i == headerInfo11G) {
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->calPier11g[0] = fbin2freq(version, tempValue & 0xff, TRUE);
                pHeaderInfo->calPier11g[1] = fbin2freq(version, (tempValue >> 8) & 0xff, TRUE);
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->turbo2WMaxPower2 = tempValue & 0x7F;
                pHeaderInfo->xrTargetPower2 = (tempValue >> 7) & 0x3f;
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->calPier11g[2] = fbin2freq(version, tempValue & 0xff, TRUE);
                if (version >= EEPROM_VER4_1) {
                    pHeaderInfo->rxtxMargin[headerInfo11G] = (tempValue >> 8) & 0x3f;
                }
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->iqCalI[1] = (tempValue >> 5) & 0x3F;
                pHeaderInfo->iqCalQ[1] = tempValue & 0x1F;
                if (version >= EEPROM_VER4_2) {
                    tempValue = pRawEeprom[offset++];
                    pHeaderInfo->cckOfdmGainDelta = (A_INT8)(tempValue & 0xFF);

                    if (version == EEPROM_VER4_9) {
                    /*
                    ** Retrieve Dual Chain Phase Cal data from EEPROM
                    */

                    /* XXXX - Skip Modal Turbo fields - GDS 6/24/04 */
                    /* ByPass 0x11E & 0x11F fields for now. */
                    offset += 2;

                    /*
                    ** Get phase cal data for 2.412 & 2.472 GHz,
                    ** respectively.
                    ** Fetch offset 0x120. The phase cal fields are 6 bits
                    ** wide.
                    */
                    tempValue = pRawEeprom[offset++];
                    pHeaderInfo->phaseCal11g[0] = tempValue & 0x003f;
                    pHeaderInfo->phaseCal11g[1] = 
                        (tempValue & 0x0fc0) >> 6;
                    }
                }
            } else if ((i == headerInfo11A) && (version >= EEPROM_VER4_1)) {
                tempValue = pRawEeprom[offset++];
                pHeaderInfo->rxtxMargin[headerInfo11A] = tempValue & 0x3f;

                if (version == EEPROM_VER4_9) {
                    /*
                    ** Retrieve Dual Chain Phase Cal data from EEPROM
                    */

                    /* XXXX - Skip Modal Turbo fields for now - GDS 6/24/04 */
                    /* ByPass 0xE2 and 0xE3 fields for now. */
                    offset += 2;

                    /*
                    ** Get phase cal data for 5, 5.2, 5.4, 5.6, and 5.8 GHz,
                    ** respectively.
                    ** Fetch offset 0xE4. The phase cal fields are 6 bits
                    ** wide.
                    */
                    tempValue = pRawEeprom[offset++];
                    pHeaderInfo->phaseCal11a[0] = tempValue & 0x003f;
                    pHeaderInfo->phaseCal11a[1] = (tempValue & 0x0fc0) >> 6;
                    pHeaderInfo->phaseCal11a[2] = (tempValue & 0xf000) >> 12;

                    /* Fetch offset 0xE5. */
                    tempValue = pRawEeprom[offset++];
                    pHeaderInfo->phaseCal11a[2] |= (tempValue & 0x0003) << 4;
                    pHeaderInfo->phaseCal11a[3] = (tempValue & 0x00fc) >> 2;
                    pHeaderInfo->phaseCal11a[4] = (tempValue & 0x3f00) >> 8;
                }
            }
        }
    }
    if (version < EEPROM_VER3_3) {
        /* Version 3.1, 3.2 specific parameters */
        tempValue = pRawEeprom[offset++];
        pHeaderInfo->ob2GHz[0] = tempValue & 0x7;
        pHeaderInfo->db2GHz[0] = (tempValue >> 3) & 0x7;

        tempValue = pRawEeprom[offset++];
        pHeaderInfo->ob2GHz[1] = tempValue & 0x7;
        pHeaderInfo->db2GHz[1] = (tempValue >> 3) & 0x7;

    }

    /*
     * Read the conformance test limit identifiers
     * These are used to match regulatory domain testing needs with
     * the RD-specific tests that have been calibrated in the EEPROM.
     */
    offset = pHeadOff[5] - ATHEROS_EEPROM_OFFSET;
    for (i = 0; i < pHeaderInfo->numCtls; i += 2) {
        tempValue = pRawEeprom[offset++];
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
}

/**************************************************************************
 * getPcdacInterceptsFromPcdacMinMax
 *
 * Fills in the pcdac table with the correct intercepts given the min/max pcdac
 */
static void
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
static A_UINT16
fbin2freq(A_UINT16 version, A_UINT16 fbin, A_BOOL is2GHz)
{
    A_UINT16 returnValue;

    /*
     * Reserved value 0xFF provides an empty definition both as
     * an fbin and as a frequency - do not convert
     */
    if (fbin == CHANNEL_UNUSED) {
        return fbin;
    }

    if (is2GHz) {
        if (version <= EEPROM_VER3_2) {
            returnValue = (A_UINT16)(2400 + fbin);
        } else {
            returnValue = (A_UINT16)(2300 + fbin);
        }
    } else {
        if (version <= EEPROM_VER3_2) {

            returnValue = (fbin>62) ? (A_UINT16)(5100 + 10*62 + 5*(fbin-62)) : (A_UINT16)(5100 + 10*fbin);
        } else {
            returnValue = (A_UINT16)(4800 + 5*fbin);
        }
    }
    return returnValue;
}

/************** EEPROM REV 3/4 DEBUG PRINT FUNCTIONS *****************/
#if defined(DEBUG)
static void
printHeaderInfo(WLAN_DEV_INFO *pDev, EEP_MAP *pEepMap, HEADER_WMODE modeArray)
{
    A_UINT16 j;
    EEP_HEADER_INFO *pHeaderInfo = pEepMap->pEepHeader;
    A_UINT16 version = pEepMap->version;
    A_UINT32 regDmn = pDev->pHalInfo->halCapabilities.halRegDmn;

    uiPrintf("\n");
    uiPrintf(" =======================Header Information======================\n");

    uiPrintf(" |  Major Version           %2d  ", version >> 12);
    uiPrintf("|  Minor Version           %2d  |\n", version & 0xFFF);
    uiPrintf(" |-------------------------------------------------------------|\n");

    uiPrintf(" |  A Mode         %1d  ", pHeaderInfo->Amode);
    uiPrintf("|  B Mode         %1d  ", pHeaderInfo->Bmode);
    uiPrintf("|  G Mode        %1d  |\n", pHeaderInfo->Gmode);

    if(regDmn >> 15) {
        uiPrintf(" |  Country Code %03x  ", regDmn >> 15);
    } else {
        uiPrintf(" |  Reg. Domain  %03x  ", regDmn & 0xFFF);
    }

    uiPrintf("|  turbo Disable  %1d  ", pHeaderInfo->turbo5Disable);
    uiPrintf("|  RF Silent     %1d  |\n", pHeaderInfo->rfKill);
    uiPrintf(" | Turbo 2W Maximum dBm %2d | cckOfdmDelta(10x) %2d | GainI %2d   |\n",
        pHeaderInfo->turbo2WMaxPower5, pHeaderInfo->cckOfdmPwrDelta, pHeaderInfo->gainI[modeArray]);
    uiPrintf(" |-------------------------------------------------------------|\n");
    if (version >= EEPROM_VER3_3) {
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
    uiPrintf("|  Noise Threshold        %3d  |\n", pHeaderInfo->noiseFloorThresh[modeArray]);

    uiPrintf(" |  XPD Gain              0x%02x  ", pHeaderInfo->xgain[modeArray]);
    uiPrintf("|  XPD                      %1d  |\n", pHeaderInfo->xpd[modeArray]);

    uiPrintf(" |  txrx Attenuation      0x%02x  ", pHeaderInfo->txrxAtten[modeArray]);


    uiPrintf("|  Antenna control    0  0x%02X  |\n", pHeaderInfo->antennaControl[0][modeArray]);
    for(j = 1; j <= 10; j+=2) {
        uiPrintf(" |  Antenna control   %2d  0x%02X  ", j, pHeaderInfo->antennaControl[j][modeArray]);
        uiPrintf("|  Antenna control   %2d  0x%02X  |\n", j + 1, pHeaderInfo->antennaControl[j + 1][modeArray]);
    }

    uiPrintf(" |-------------------------------------------------------------|\n");
    if (modeArray == headerInfo11A) {
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
            if (modeArray == headerInfo11B) {
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
            if (modeArray == headerInfo11B) {
                uiPrintf(" |  OB_1                     %1d  ", pHeaderInfo->obFor24);
                uiPrintf("|  DB_1                     %1d  |\n", pHeaderInfo->dbFor24);
            } else {
                uiPrintf(" |  OB_1                     %1d  ", pHeaderInfo->obFor24g);
                uiPrintf("|  DB_1                     %1d  |\n", pHeaderInfo->dbFor24g);
            }

        }
    }
    uiPrintf(" ===============================================================\n");
    return;
}

static void
printChannelInfo(PCDACS_ALL_MODES *pEepromData, HEADER_WMODE mode)
{
    A_UINT16            i, j, k = 0;
    DATA_PER_CHANNEL    *pDataPerChannel;

    switch (mode) {
    case headerInfo11A:
        pDataPerChannel = pEepromData->DataPerChannel11a;
        break;

    case headerInfo11G:
        pDataPerChannel = pEepromData->DataPerChannel11g;
        break;

    case headerInfo11B:
        pDataPerChannel = pEepromData->DataPerChannel11b;
        break;

    default:
        uiPrintf("ar5513Attach: Illegal mode passed to printChannelInfo\n");
        return;
    }

    uiPrintf("\n");
    if (mode == headerInfo11A) {
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
printExpnPower5112(EEPROM_POWER_EXPN_5112 *pExpnPower, HEADER_WMODE mode)
{
    A_UINT16        i, j=0, kk;
    A_UINT16        singleXpd = 0xDEAD;
    A_CHAR          *modeString[3] = {"11A", "11B", "11G"};

    if (pExpnPower->xpdMask != 0x9) {
        for (j = 0; j < NUM_XPD_PER_CHANNEL; j++) {
            if (pExpnPower->xpdMask == (1 << j)) {
                singleXpd = j;
                break;
            }
        }
    }

    uiPrintf("======================5112 Power Calibration Information========================\n");

    uiPrintf("| XPD_Gain_mask = 0x%2x              | Number of channels for mode %s: %2d      |\n",
        pExpnPower->xpdMask, modeString[mode], pExpnPower->numChannels);
    if (pExpnPower->xpdMask != 0x9) {
        uiPrintf("| XPD_GAIN = %d                                                              |\n", j);
    }
    uiPrintf("|==============================================================================|\n");

    //print the frequency values
    uiPrintf("| freq |  pwr1  |  pwr2  |  pwr3  |  pwr4  |");
    if (pExpnPower->xpdMask == 0x9) {
        uiPrintf(" pwr1_x3| pwr2_x3| pwr3_x3| maxPow |\n");
        uiPrintf("|      | [pcd]  | [pcd]  | [pcd]  | [pcd]  | [pcd]  | [pcd]  | [pcd]  |        |\n");
    } else {
        uiPrintf("        [pcd]    [pcd]    [pcd]    [pcd]    \n");
    }

    for (i = 0; i < pExpnPower->numChannels; i++) {
        uiPrintf("|==============================================================================|\n");
        uiPrintf("| %4d |", pExpnPower->pChannels[i]);
        if (pExpnPower->xpdMask != 0x9) {
            j = singleXpd;
        } else {
            j = 0;
        }
        for (kk=0; kk < pExpnPower->pDataPerChannel[i].pDataPerXPD[j].numPcdacs; kk++) {
            uiPrintf(" %2d.%02d  |", (pExpnPower->pDataPerChannel[i].pDataPerXPD[j].pwr_t4[kk] / 4),
                (pExpnPower->pDataPerChannel[i].pDataPerXPD[j].pwr_t4[kk] % 4) * 25);
        }
        if (pExpnPower->xpdMask == 0x9) {
            for (kk=0; kk < pExpnPower->pDataPerChannel[i].pDataPerXPD[3].numPcdacs; kk++) {
                uiPrintf(" %2d.%02d  |", pExpnPower->pDataPerChannel[i].pDataPerXPD[3].pwr_t4[kk] / 4,
                    (pExpnPower->pDataPerChannel[i].pDataPerXPD[3].pwr_t4[kk] % 4) * 25);
            }
        }
        uiPrintf(" %2d.%02d  |\n", pExpnPower->pDataPerChannel[i].maxPower_t4 / 4,
            (pExpnPower->pDataPerChannel[i].maxPower_t4 % 4) * 25);
        uiPrintf("|      |");
        for (kk=0; kk<pExpnPower->pDataPerChannel[i].pDataPerXPD[j].numPcdacs; kk++) {
            uiPrintf("  [%2d]  |", pExpnPower->pDataPerChannel[i].pDataPerXPD[j].pcdac[kk]);
        }
        if (pExpnPower->xpdMask == 0x9) {
            for (kk=0; kk<pExpnPower->pDataPerChannel[i].pDataPerXPD[3].numPcdacs; kk++) {
                uiPrintf("  [%2d]  |", pExpnPower->pDataPerChannel[i].pDataPerXPD[3].pcdac[kk]);
            }
        }
        uiPrintf("        |\n");
    }
    uiPrintf("================================================================================\n");
}

static void
printTargetPowerInfo(TRGT_POWER_ALL_MODES *pPowerInfoAllModes, HEADER_WMODE mode)
{
    A_UINT16 i, k;
    TRGT_POWER_INFO     *pPowerInfo;

    uiPrintf("\n");
    if (mode == headerInfo11A) {
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
                uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr6_24 / 2,
                    (pPowerInfo[i].twicePwr6_24 % 2) * 5);
            }
            uiPrintf("|\n");

            uiPrintf("|      36      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr36 / 2,
                    (pPowerInfo[i].twicePwr36 % 2) * 5);
            }
            uiPrintf("|\n");

            uiPrintf("|      48      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr48 / 2,
                    (pPowerInfo[i].twicePwr48 % 2) * 5);
            }
            uiPrintf("|\n");

            uiPrintf("|      54      ");
            for (i = k; i < k + 4; i++) {
                uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr54 / 2,
                    (pPowerInfo[i].twicePwr54 % 2) * 5);
            }

            uiPrintf("|\n");
            uiPrintf("|==============|==============|==============|==============|==============|\n");
        }
    } else {
        if (mode == headerInfo11B) {
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

        if (mode == headerInfo11B) {
            uiPrintf("|      1       ");
        } else {
            uiPrintf("|     6-24     ");
        }

        for (i = 0; i < 2; i++) {
            uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr6_24 / 2,
                (pPowerInfo[i].twicePwr6_24 % 2) * 5);
        }
        uiPrintf("|\n");

        if (mode == headerInfo11B) {
            uiPrintf("|      2       ");
        } else {
            uiPrintf("|      36      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr36 / 2,
                (pPowerInfo[i].twicePwr36 % 2) * 5);
        }
        uiPrintf("|\n");

        if (mode == headerInfo11B) {
            uiPrintf("|      5.5     ");
        } else {
            uiPrintf("|      48      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr48 / 2,
                (pPowerInfo[i].twicePwr48 % 2) * 5);
        }
        uiPrintf("|\n");

        if (mode == headerInfo11B) {
            uiPrintf("|      11      ");
        } else {
            uiPrintf("|      54      ");
        }
        for (i = 0; i < 2; i++) {
            uiPrintf("|     %2d.%d     ", pPowerInfo[i].twicePwr54 / 2,
                (pPowerInfo[i].twicePwr54 % 2) * 5);
        }

        uiPrintf("|\n");
        uiPrintf("|==============|==============|==============|\n");

    }
}

static void
printRDEdges(RD_EDGES_POWER *pRdEdgePwrInfo, A_UINT16 *pTestGroups,
             HEADER_WMODE mode, A_UINT16 maxNumCtl, A_UINT16 version)
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
        switch (pTestGroups[i] & 0x3) {
        case 0:
        case 3:
            ctlMode = headerInfo11A;
            break;
        case 1:
            ctlMode = headerInfo11B;
            break;
        case 2:
            ctlMode = headerInfo11G;
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
                uiPrintf("| %2d.%d  ", pRdEdgePwrInfo[j].twice_rdEdgePower / 2,
                    (pRdEdgePwrInfo[j].twice_rdEdgePower % 2) * 5);
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
printEepromStruct(WLAN_DEV_INFO *pDev, EEP_MAP *pEepMap, HEADER_WMODE mode)
{
    A_UINT8 chnIdx, chainCnt;

    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN ||
        pDev->staConfig.rxChainCtrl == DUAL_CHAIN)
    {
        chainCnt = 2;
    } else {
        chainCnt = 1;
    }

    printHeaderInfo(pDev, pEepMap, mode);

    for (chnIdx = 0; chnIdx < chainCnt; chnIdx++) {
	if ((pEepMap->version >= EEPROM_VER4_0) && pEepMap->pEepHeader->eepMap) {
	    printExpnPower5112(&(pEepMap->chain[chnIdx].modePowerArray5112[mode]), mode);
	} else {
	    printChannelInfo(pEepMap->chain[chnIdx].pPcdacInfo, mode);
	}
	printTargetPowerInfo(pEepMap->chain[chnIdx].pTrgtPowerInfo, mode);
	printRDEdges(pEepMap->chain[chnIdx].pRdEdgesPower, pEepMap->pEepHeader->ctl, mode, pEepMap->pEepHeader->numCtls,
		     pEepMap->version);
    }

    return;
}

void
dump5212Eeprom(int unit)
{
    WLAN_DEV_INFO *pDev = gDrvInfo.pDev[unit];

    EEP_MAP *pEepMap;
    ASSERT(pDev->pHalInfo->pEepData);

    pEepMap = pDev->pHalInfo->pEepData;
    if (pEepMap->pEepHeader->Amode) {
        printEepromStruct(pDev, pEepMap, headerInfo11A);
    }
    if (pEepMap->pEepHeader->Bmode) {
        printEepromStruct(pDev, pEepMap, headerInfo11B);
    }
    if (pEepMap->pEepHeader->Gmode) {
        printEepromStruct(pDev, pEepMap, headerInfo11G);
    }
    return;
}

#endif /* #ifdef DEBUG */

/************ FORWARD SOFTWARE COMPATIBILITY EAR FUNCTIONS ************/
/* Globals */
int EarDebugLevel = EAR_DEBUG_OFF;

/**************************************************************************
 * ar5513EarPreParse
 *
 * Parse the entire EAR saving away the size required for each EAR register header
 * Allocation is only performed for matching version ID's.
 */
static int
ar5513EarPreParse(A_UINT16 *in, int numLocs, EAR_ALLOC *earAlloc)
{
    A_UINT16      numFound = 0;
    int           curEarLoc = 0;
    A_UINT16      numLocsConsumed;
    A_UINT16      type, regHead, tag, last;
    A_UINT16      verId;
    A_BOOL        verMaskMatch = FALSE;

    /* Record the version Id */
    verId = in[curEarLoc++];

    while (curEarLoc < numLocs) {
        /* Parse Version/Header/End */
        if (IS_VER_MASK(in[curEarLoc])) {
            verMaskMatch = IS_EAR_VMATCH(verId, in[curEarLoc]);
            curEarLoc++;
        } else if (IS_END_HEADER(in[curEarLoc])) {
            break;
        } else {
            /* Must be a register header - find number of locations consumed */
            regHead = in[curEarLoc++];

            if (IS_CM_SET(regHead)) {
                /* Bump location for channel modifier */
                curEarLoc++;
            }

            if (IS_DISABLER_SET(regHead)) {
                if (IS_PLL_SET(in[curEarLoc])) {
                    /* Bump location for PLL */
                    curEarLoc++;
                }
                /* Bump location for disabler */
                curEarLoc++;
            }

            numLocsConsumed = 0;
            /* Now into the actual register writes */
            type = (A_UINT16)((regHead & RH_TYPE_M) >> RH_TYPE_S);
            switch (type) {
            case EAR_TYPE0:
                do {
                    tag = (A_UINT16)(in[curEarLoc] & T0_TAG_M);
                    if ((tag == T0_TAG_32BIT) || (tag == T0_TAG_32BIT_LAST)) {
                        /* Full word writes */
                        numLocsConsumed += 3;
                        curEarLoc += 3;
                    } else {
                        /* Half word writes */
                        numLocsConsumed += 2;
                        curEarLoc += 2;
                    }
                }
                while ((tag != T0_TAG_32BIT_LAST) && (curEarLoc < numLocs));
                break;
            case EAR_TYPE1:
                numLocsConsumed = (A_UINT16)(((in[curEarLoc] & T1_NUM_M) * sizeof(A_UINT16)) + 3);
                curEarLoc += numLocsConsumed;
                break;
            case EAR_TYPE2:
                do {
                    last = IS_TYPE2_LAST(in[curEarLoc]);
                    if(IS_TYPE2_EXTENDED(in[curEarLoc])) {
                        numLocsConsumed += 2 + A_DIV_UP(in[curEarLoc+1], 16);
                        curEarLoc += 2 + A_DIV_UP(in[curEarLoc+1], 16);
                    } else {
                        numLocsConsumed += 2;
                        curEarLoc += 2;
                    }
                } while (!last && (curEarLoc < numLocs));
                break;
            case EAR_TYPE3:
                do {
                    last = IS_TYPE3_LAST(in[curEarLoc]);
                    numLocsConsumed += 2 + A_DIV_UP(in[curEarLoc] & 0x1F, 16);
                    curEarLoc += 2 + A_DIV_UP(in[curEarLoc] & 0x1F, 16);
                } while (!last && (curEarLoc < numLocs));
                break;
            }
            /* Only record allocation information for matching versions */
            if (verMaskMatch) {
                if (numLocsConsumed > EAR_MAX_RH_REGS) {
                    uiPrintf("ar5513EarPreParse: A register header exceeds the max allowable size\n");
                    earAlloc->numRHs = 0;
                    return 0;
                }
                earAlloc->locsPerRH[numFound] = numLocsConsumed;
                numFound++;
            }
        }
    }

    earAlloc->numRHs = numFound;

    if (EarDebugLevel >= 1) {
        uiPrintf("Number of locations parsed in preParse: %d\n", curEarLoc);
    }
    return curEarLoc;
}

/**************************************************************************
 * ar5513EarAllocate
 *
 * Now that the Ear structure size is known, allocate the EAR header and
 * the individual register headers
 */
static A_BOOL
ar5513EarAllocate(EAR_ALLOC *earAlloc, EAR_HEADER **ppEarHead)
{
    int             sizeForRHs;
    int             sizeForRegs = 0;
    int             i;
    A_UINT16        *currentAlloc;
    REGISTER_HEADER *pRH;

    /* Return if no applicable EAR exists */
    if (earAlloc->numRHs == 0) {
        return TRUE;
    }

    /* Allocate the Ear Header */
    *ppEarHead = (EAR_HEADER *)A_DRIVER_MALLOC(sizeof(EAR_HEADER));
    if (*ppEarHead == NULL) {
        uiPrintf("ar5513EarAllocate: Failed to retrieve space for EAR Header\n");
        return FALSE;
    }
    A_MEM_ZERO(*ppEarHead, sizeof(EAR_HEADER));

    /* Save number of register headers */
    (*ppEarHead)->numRHs = earAlloc->numRHs;

    /* Size the malloc for the Ear Register Headers */
    sizeForRHs = earAlloc->numRHs * sizeof(REGISTER_HEADER);
    for (i = 0; i < earAlloc->numRHs; i++) {
        sizeForRegs += earAlloc->locsPerRH[i] * sizeof(A_UINT16);
    }

    if (EarDebugLevel >= EAR_DEBUG_VERBOSE) {
        uiPrintf("ar5513EarAllocate: Size for RH's %d, size for reg data %d\n", sizeForRHs, sizeForRegs);
    }

    /* Malloc and assign the space to the RH's */
    (*ppEarHead)->pRH = (REGISTER_HEADER *) A_DRIVER_MALLOC(sizeForRegs + sizeForRHs);
    if ((*ppEarHead)->pRH == NULL) {
        uiPrintf("ar5513EarAllocate: Failed to retrieve space for EAR individual registers\n");
        return 0;
    }
    A_MEM_ZERO((*ppEarHead)->pRH, sizeForRegs + sizeForRHs);
    (*ppEarHead)->earSize = sizeForRegs + sizeForRHs;

    currentAlloc = (A_UINT16 *)(((A_UINT8 *)(*ppEarHead)->pRH) + sizeForRHs);
    for (i = 0; i < earAlloc->numRHs; i++) {
        if (EarDebugLevel >= EAR_DEBUG_EXTREME) {
             uiPrintf("ar5513EarAllocate: reg data %2d memory location 0x%08X\n", i, (A_UINT32)currentAlloc);
        }
        pRH = &((*ppEarHead)->pRH[i]);
        pRH->regs = currentAlloc;
        currentAlloc += earAlloc->locsPerRH[i];
    }
    return TRUE;
}

/**************************************************************************
 * ar5513EarCheckAndFill
 *
 * Save away EAR contents
 */
static A_BOOL
ar5513EarCheckAndFill(EAR_HEADER *earHead, A_UINT16 *in, int totalLocs, EAR_ALLOC *pEarAlloc)
{
    int             curEarLoc = 0, currentRH = 0, regLoc, i;
    A_UINT16        regHead, tag, last, currentVerMask, addr, num, numBits, startBit;
    REGISTER_HEADER *pRH, tempRH;
    A_UINT16        tempRHregs[EAR_MAX_RH_REGS], *pReg16;
    A_BOOL          verMaskUsed, verMaskMatch;

    /* Setup the temporary register header */

    /* Save the version Id */
    earHead->versionId = in[curEarLoc++];

    /* Save the first version mask */
    verMaskUsed = 0;
    verMaskMatch = IS_EAR_VMATCH(earHead->versionId, in[curEarLoc]);
    currentVerMask = in[curEarLoc++];
    if (!IS_VER_MASK(currentVerMask)) {
        uiPrintf("ar5513EarCheckAndFill: ERROR: First entry after Version ID is not a Version Mask\n");
        return FALSE;
    }

    while (curEarLoc < totalLocs) {
        /* Parse Version/Header/End */
        if (IS_VER_MASK(in[curEarLoc])) {
            if (!verMaskUsed) {
                uiPrintf("ar5513EarCheckAndFill: ERROR: Multiple version masks setup with no register headers location %d\n",
                         curEarLoc);
                return FALSE;
            }
            verMaskUsed = 0;
            verMaskMatch = IS_EAR_VMATCH(earHead->versionId, in[curEarLoc]);
            currentVerMask = in[curEarLoc++];
        } else if (IS_END_HEADER(in[curEarLoc])) {
            uiPrintf("ar5513EarCheckAndFill: ERROR: Somehow we've hit the END location but parse shouldn't let us get here. loc %d\n",
                     curEarLoc);
            return FALSE;
        } else {
            if (currentRH > earHead->numRHs) {
                uiPrintf("ar5513EarCheckAndFill: ERROR: Exceeded number of register headers found in preParse. loc %d\n",
                         curEarLoc);
                return FALSE;
            }
            A_MEM_ZERO(&tempRH, sizeof(REGISTER_HEADER));
            A_MEM_ZERO(&tempRHregs, sizeof(A_UINT16) * EAR_MAX_RH_REGS);
            tempRH.regs = tempRHregs;
            pRH = &tempRH;

            /* Must be a register header - save last version mask */
            pRH->versionMask = currentVerMask;
            regHead = in[curEarLoc++];
            pRH->modes = (A_UINT16)(regHead & RH_MODES_M);
            if ((pRH->modes != RH_ALL_MODES) && (pRH->modes & RH_RESERVED_MODES)) {
                uiPrintf("ar5513EarCheckAndFill: WARNING: Detected that reserved modes have been set. loc %d\n", curEarLoc);
            }
            pRH->type = (A_UINT16)((regHead & RH_TYPE_M) >> RH_TYPE_S);
            pRH->stage = (A_UINT16)((regHead & RH_STAGE_M) >> RH_STAGE_S);
            if (IS_CM_SET(regHead)) {
                pRH->channel = in[curEarLoc++];
                if (IS_CM_SINGLE(pRH->channel) && !IS_5GHZ_CHAN(pRH->channel) &&
                    !IS_2GHZ_CHAN(pRH->channel))
                {
                    uiPrintf("ar5513EarCheckAndFill: WARNING: Specified single channel %d is not within tunable range. loc %d\n",
                             pRH->channel & CM_SINGLE_CHAN_M, curEarLoc);
                }
            }

            if (IS_DISABLER_SET(regHead)) {
                pRH->disabler.valid = TRUE;
                pRH->disabler.disableField = in[curEarLoc++];
                if (IS_PLL_SET(pRH->disabler.disableField)) {
                    pRH->disabler.pllValue = in[curEarLoc++];
                    if (pRH->disabler.pllValue & RH_RESERVED_PLL_BITS) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Detected that reserved pll bits have been set. loc %d\n", curEarLoc);
                    }
                }
            }

            /* Now into the actual register writes */
            regLoc = 0;
            switch (pRH->type) {
            case EAR_TYPE0:
                do {
                    pRH->regs[regLoc] = in[curEarLoc++];
                    tag = (A_UINT16)(pRH->regs[regLoc] & T0_TAG_M);
                    if (!IS_VALID_ADDR(pRH->regs[regLoc] & ~T0_TAG_M)) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Detected invalid register address 0x%04X at loc %d\n",
                                 pRH->regs[regLoc] & ~T0_TAG_M, curEarLoc);
                    }
                    regLoc++;
                    if ((tag == T0_TAG_32BIT) || (tag == T0_TAG_32BIT_LAST)) {
                        /* Full word writes */
                        pRH->regs[regLoc++] = in[curEarLoc++];
                        pRH->regs[regLoc++] = in[curEarLoc++];
                    } else {
                        /* Half word writes */
                        pRH->regs[regLoc++] = in[curEarLoc++];
                    }
                } while ((tag != T0_TAG_32BIT_LAST) && (curEarLoc < totalLocs));
                break;
            case EAR_TYPE1:
                pRH->regs[regLoc] = in[curEarLoc++];
                addr = (A_UINT16)(pRH->regs[regLoc] & ~T1_NUM_M);
                num = (A_UINT16)(pRH->regs[regLoc] & T1_NUM_M);
                regLoc++;
                for (i = 0; i < num + 1; i++) {
                    /* Full word writes */
                    if (!IS_VALID_ADDR(addr + (sizeof(A_UINT32) * i))) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Detected invalid register address 0x%04X at loc %d\n",
                                 addr + (sizeof(A_UINT32) * i), curEarLoc);
                    }
                    pRH->regs[regLoc++] = in[curEarLoc++];
                    pRH->regs[regLoc++] = in[curEarLoc++];
                }
                break;
            case EAR_TYPE2:
                do {
                    pRH->regs[regLoc] = in[curEarLoc++];
                    last = IS_TYPE2_LAST(pRH->regs[regLoc]);
                    if (((pRH->regs[regLoc] & T2_BANK_M) >> T2_BANK_S) == 0) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Bank 0 update found in Type2 write at loc %d\n", curEarLoc);
                    }
                    startBit = (A_UINT16)(pRH->regs[regLoc] & T2_START_M);
                    if (IS_TYPE2_EXTENDED(pRH->regs[regLoc])) {
                        regLoc++;
                        pRH->regs[regLoc] = in[curEarLoc++];
                        if (pRH->regs[regLoc] < 12) {
                            uiPrintf("ar5513EarCheckAndFill: WARNING: Type2 Extended Write used when number of bits is under 12 at loc %d\n",
                                     curEarLoc);
                        }
                        num = A_DIV_UP(pRH->regs[regLoc], 16);
                        numBits = pRH->regs[regLoc];
                        if (startBit + numBits > MAX_ANALOG_START) {
                            uiPrintf("ar5513EarCheckAndFill: ERROR: Type2 write will exceed analog buffer limits (at loc %d)\n", curEarLoc);
                            return FALSE;
                        }
                        regLoc++;
                        for (i = 0; i < num; i++) {
                            /* Add check data exceeds num bits check? */
                            pRH->regs[regLoc++] = in[curEarLoc++];
                        }
                        if ((~((1 << (numBits % 16)) - 1) & in[curEarLoc - 1]) && ((numBits % 16) != 0)){
                            uiPrintf("ar5513EarCheckAndFill: WARNING: Type2 extended Write data exceeds number of bits specified at loc %d\n",
                                     curEarLoc);
                        }
                    } else {
                        regLoc++;
                        pRH->regs[regLoc] = in[curEarLoc++];
                        numBits = (A_UINT16)((pRH->regs[regLoc] & T2_NUMB_M) >> T2_NUMB_S);
                        if (startBit + numBits > MAX_ANALOG_START) {
                            uiPrintf("ar5513EarCheckAndFill: ERROR: Type2 write will exceed analog buffer limits (at loc %d)\n", curEarLoc);
                            return FALSE;
                        }
                        if (~((1 << numBits) - 1) &
                            (pRH->regs[regLoc] & T2_DATA_M))
                        {
                            uiPrintf("ar5513EarCheckAndFill: WARNING: Type2 Write data exceeds number of bits specified at loc %d\n",
                                     curEarLoc);
                        }
                        regLoc++;
                    }
                } while (!last && (curEarLoc < totalLocs));
                break;
            case EAR_TYPE3:
                do {
                    pRH->regs[regLoc] = in[curEarLoc++];
                    last = IS_TYPE3_LAST(pRH->regs[regLoc]);
                    num = (A_UINT16)(A_DIV_UP((pRH->regs[regLoc] & T3_NUMB_M), 16));
                    if (((pRH->regs[regLoc] & T3_START_M) >> T3_START_S) +
                        (pRH->regs[regLoc] & T3_NUMB_M) > 31)
                    {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Type3 StartBit plus Number of Bits > 31 at loc %d\n",
                                 curEarLoc);
                    }
                    if (((pRH->regs[regLoc] & T3_OPCODE_M) >> T3_OPCODE_S) > T3_MAX_OPCODE) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Type3 OpCode exceeds largest selectable opcode at loc %d\n",
                                 curEarLoc);
                    }
                    if (pRH->regs[regLoc] & T3_RESERVED_BITS) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Type3 Reserved bits used at loc %d\n",
                                 curEarLoc);
                    }
                    numBits = (A_UINT16)(pRH->regs[regLoc] & T3_NUMB_M);
                    regLoc++;
                    /* Grab Address */
                    pRH->regs[regLoc] = in[curEarLoc++];
                    if (!IS_VALID_ADDR(pRH->regs[regLoc])) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Detected invalid register address 0x%04X at loc %d\n",
                                 pRH->regs[regLoc], curEarLoc);
                    }
                    regLoc++;
                    for (i = 0; i < num; i++) {
                        pRH->regs[regLoc++] = in[curEarLoc++];
                    }
                    if ((~((1 << (numBits % 16)) - 1) & in[curEarLoc - 1]) && ((numBits % 16) != 0)) {
                        uiPrintf("ar5513EarCheckAndFill: WARNING: Type3 write data exceeds number of bits specified at loc %d\n",
                                 curEarLoc);
                    }
                } while (!last && (curEarLoc < totalLocs));
                break;
            }
            verMaskUsed = 1;
            /* Save any register headers that match the version mask*/
            if (verMaskMatch) {
                if (regLoc != pEarAlloc->locsPerRH[currentRH]) {
                    uiPrintf("ar5513EarCheckAndFill: Ear Allocation did not match Ear Usage\n");
                    return FALSE;
                }
                /*
                 * Copy stack register header to real register header on match
                 * BCOPY the regs, save away the reg16 ptr as the struct copy will overwrite it
                 * finally restore the reg16 ptr.
                 */
                A_DRIVER_BCOPY(pRH->regs, earHead->pRH[currentRH].regs, regLoc * sizeof(A_UINT16));
                pReg16 = earHead->pRH[currentRH].regs;
                earHead->pRH[currentRH] = *pRH;
                earHead->pRH[currentRH].regs = pReg16;

                currentRH++;
            }
        } /* End Register Header Parse */
    } /* End Location Reading */
    return TRUE;
}

static A_UINT16
ar5513ReadRadioChipRev(WLAN_DEV_INFO *pDev, A_UINT16 chain_base)
{
    int i;
    A_UINT32 revId;

    writePlatformReg(pDev, (chain_base + (52 << 2)), 0x00001c16);
    for (i = 0; i < 8; i++) {
        writePlatformReg(pDev, (chain_base + (32 << 2)), 0x00010000);
    }
    revId = (readPlatformReg(pDev, chain_base + (256 << 2)) >> 24) & 0xff;
    revId = ((revId & 0xf0) >> 4) | ((revId & 0x0f) << 4);

    return (A_UINT16)reverseBits(revId, 8);
}

#ifdef DEBUG
static void
showEarAlloc(EAR_ALLOC *earAlloc)
{
    int i;

    uiPrintf("Found %d register headers\n", earAlloc->numRHs);
    for (i = 0; i < earAlloc->numRHs; i++) {
        uiPrintf("Register Header %3d requires %2d 16-bit locations\n", i + 1,
                 earAlloc->locsPerRH[i]);
    }
}

static void
printEar(EAR_HEADER *earHead, EAR_ALLOC *earAlloc)
{
    REGISTER_HEADER *pRH;
    int i, j;

    uiPrintf("\n=== EAR structure dump ===\n");
    uiPrintf("Version ID: 0x%04X\n", earHead->versionId);
    uiPrintf("Number of Register Headers: %d\n", earHead->numRHs);

    for (i = 0; i < earHead->numRHs; i++) {
        pRH = &(earHead->pRH[i]);
        uiPrintf("\n=== Register Header %2d ===\n", i + 1);
        uiPrintf("Version Mask   : 0x%04X\n", pRH->versionMask);
        uiPrintf("Register Type  : %d\n", pRH->type);
        uiPrintf("Modality Mask  : ");
        printModeString(pRH->modes);
        uiPrintf("Stage to Use   : %d\n", pRH->stage);
        uiPrintf("Channel Mask   : 0x%04X", pRH->channel);
        if (IS_CM_SINGLE(pRH->channel)) {
            uiPrintf(" -> Single Channel : %d", pRH->channel & CM_SINGLE_CHAN_M);
        }
        uiPrintf("\n");
        uiPrintf("Disabler Mask  : 0x%04X", pRH->disabler.disableField);
        if (IS_PLL_SET(pRH->disabler.disableField)) {
            uiPrintf(" -> PLL Setting : 0x%x", pRH->disabler.pllValue);
        }
        uiPrintf("\n");
        for (j = 0; j < earAlloc->locsPerRH[i]; j++) {
            uiPrintf("= %2d = 0x%04X =\n", j, pRH->regs[j]);
        }
    }
}

static void
printModeString(A_UINT16 modes)
{
    const static A_CHAR *pModesArray[9] = {"11B ", "11G ", "T2 ", "11GXR ",
                                            "11A ", "NA ", "T5 ", "11AXR ", "NA "};
    int i;
    int newModes;

    if (modes == RH_ALL_MODES) {
        newModes = 0xDF;
    } else {
        newModes = modes;
    }

    for(i = 0; i < 9; i++) {
        if (newModes & (1 << i)) {
            uiPrintf("%s", pModesArray[i]);
        }
    }
    uiPrintf("\n");
}
#endif

#endif /* #ifdef BUILD_AR5513 */
