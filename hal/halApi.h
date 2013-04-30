/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines the MAC/PHY Hardware Abstraction Layer (HAL) API as well as commonly
 * used types and structs.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/halApi.h#8 $
 */

#ifndef _HAL_API_H_
#define _HAL_API_H_

#ifdef _cplusplus
extern "C" {
#endif


/* Attach Functions */
A_STATUS
halAttach(WLAN_DEV_INFO *pDev);

A_UINT32
halGetNumDevid(void);

A_UINT16
halGetDevids(A_UINT32 index);

A_STATUS
halDetach(WLAN_DEV_INFO *pDev);

A_STATUS
halFillCapabilityInfo(WLAN_DEV_INFO *pDev);

/* Reset Functions */
A_STATUS
halReset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange);

A_STATUS
halPhyDisable(WLAN_DEV_INFO *pDev);

A_STATUS
halDisable(WLAN_DEV_INFO *pDev);

A_STATUS
halPerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);

void
halSetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit);
#define MAX_TX_POWER 0xffffffff

/* Transmit Functions */

A_BOOL
halUpdateTxTrigLevel(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel);

typedef struct HalTxQueueInfo {
    enum {
        TXQ_MODE_INACTIVE = 0,
        TXQ_MODE_NORMAL,
        TXQ_MODE_BEACON,        /* implies TBTT gated queue */
        TXQ_MODE_CAB,           /* implies beacon gated queue */
        TXQ_MODE_PSPOLL         /* implies TIM gated queue */
    }           mode;
    enum {
        TXQ_FLAG_TXINT_ENABLE              = 0x0001,   /* Flag to have TXOK,TXERR Interrupts */
        TXQ_FLAG_TXDESCINT_ENABLE          = 0x0002,   /* Flag to have TXDESC     Interrupts */
        TXQ_FLAG_BACKOFF_DISABLE           = 0x0004,   /* Flag to disable Post Backoff  */
        TXQ_FLAG_COMPRESSION_ENABLE        = 0x0008,   /* Flag to compression enabled */
        TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE = 0x0010,   /* Flag to enable ready time expiry policy */
        TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE = 0x0020,   /* Flag to enable backoff while sending fragment burst */
    }           qFlags;
    A_UINT32    priority;       /* for now: hacked to map to queueNum */
    A_UINT32    aifs;
    A_UINT32    logCwMin;
    A_UINT32    logCwMax;
    A_UINT32    cbrPeriod;
    A_UINT32    cbrOverflowLimit;
    A_UINT32    burstTime;
    A_UINT32    readyTime;
    A_UINT32    physCompBuf;
} HAL_TX_QUEUE_INFO;

#define A_QUEUE_INFO_INIT(_pQ, _pAc)    do {    \
    A_MEM_ZERO(_pQ, sizeof(HAL_TX_QUEUE_INFO)); \
    (_pQ)->aifs     = (_pAc)->aifs;             \
    (_pQ)->logCwMin = (_pAc)->logCwMin;         \
    (_pQ)->logCwMax = (_pAc)->logCwMax;         \
} while (0)

int
halSetupTxQueue(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO* queueInfo);

void
halReleaseTxQueue(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
halGetTxDP(WLAN_DEV_INFO *pDev, int queueNum);

void
halSetTxDP(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp);

void
halStartTxDma(WLAN_DEV_INFO *pDev, int queueNum);

A_UINT32
halNumTxPending(WLAN_DEV_INFO *pDev, int queueNum);

void
halMonitorRxTxActivity(WLAN_DEV_INFO *pDev, int index, int display);

void
halStopTxDma(WLAN_DEV_INFO *pDev, int queueNum, int msec);

int
halGetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index);

void
halSetTxFilter(WLAN_DEV_INFO *pDev, int queueNum, int index, int value);

int
halGetTxFilterArraySize(WLAN_DEV_INFO *pDev);

void
halGetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count);

void
halSetTxFilterArray(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count);

void
halPauseTx(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause);

/* Transmit Desriptor Functions */

void
halSetupTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_UINT32 hwIndex);

A_STATUS
halProcessTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc);

A_BOOL
halGetTxDescDone(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_BOOL swap);

#if defined(DEBUG) || defined(_DEBUG)
void
halDebugPrintTxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose);
#endif

/* Receive Functions */

A_UINT32
halGetRxDP(WLAN_DEV_INFO *pDev);

void
halSetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp);

void
halEnableReceive(WLAN_DEV_INFO *pDev);

A_STATUS
halStopDmaReceive(WLAN_DEV_INFO *pDev);

void
halStartPcuReceive(WLAN_DEV_INFO *pDev);

void
halStopPcuReceive(WLAN_DEV_INFO *pDev);

void
halSetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1);

void
halMulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet);


typedef enum halRxFilterOper {
    /* The Rx Filter Operations Types have the following affect on the
     * hardware register and the software copy of the hardware register.
     *
     *                          Hardware Register   Software Copy
     *
     * HAL_RX_FILTER_INIT       Updated             Updated
     * HAL_RX_FILTER_SET        Updated             Updated
     * HAL_RX_FILTER_CLEAR      Updated             Updated
     * HAL_RX_FILTER_TEMP       Updated             unmodified
     * HAL_RX_FILTER_RESTORE    Updated             unmodified
     */

    HAL_RX_FILTER_INIT,         // Init hardware register and software copy
    HAL_RX_FILTER_SET,          // Set frame type bits
    HAL_RX_FILTER_CLEAR,        // Clear frame type bits
    HAL_RX_FILTER_TEMP,         // Temporarily set hardware register ONLY.
    HAL_RX_FILTER_RESTORE       // Restore hardware register with software copy

} HAL_RX_FILTER_OPER;

enum halRxFilterFrameTypes {
    /* Rx Filter Frame Types */
    HAL_RX_UCAST     = 0x00000001, // Common mapping! Allow unicast frames
    HAL_RX_MCAST     = 0x00000002, // Common mapping! Allow multicast frames
    HAL_RX_BCAST     = 0x00000004, // Common mapping! Allow broadcast frames
    HAL_RX_CONTROL   = 0x00000008, // Common mapping! Allow control frames
    HAL_RX_BEACON    = 0x00000010, // Common mapping! Allow beacon frames
    HAL_RX_PROM      = 0x00000020, // Common mapping! Promiscuous mode, all packets
    HAL_RX_DOUBLE_CHIRP  = 0x00000040, // Enable Double Chirp

    HAL_RX_PHY_ERR   = 0x00000040, // Non-common mapping! Allow phy errors
    HAL_RX_PHY_RADAR = 0x00000080, // Non-common mapping! Allow radar phy errors
    HAL_RX_SYNC      = 0x00000100, // Non-common mapping! Allow sync frames
    HAL_RX_XR_POLL   = 0x00000400, // Non-common mapping! Allow XR Group Polls
    HAL_RX_PROBE_REQ = 0x00000800, // Non-common mapping! Allow Probe Requests

    HAL_RX_COMMON    = ( HAL_RX_UCAST   | HAL_RX_MCAST  | HAL_RX_BCAST |
                         HAL_RX_CONTROL | HAL_RX_BEACON | HAL_RX_PROM  )
};

void
halRxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper);

A_STATUS
halProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);

#if (defined(UPSD) && defined(BUILD_AP))
void
halUpsdResponse(WLAN_DEV_INFO *pDev);
#endif

void
halSetupRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_UINT32 size);

/* Misc Functions */
typedef enum {
    /* Direct mapped to storage - must be in order */
    HAL_GET_REG_DMN,
    HAL_GET_WIRELESS_MODES,
    HAL_GET_CHAN_SPREAD_SUPPORT,
    HAL_GET_SLEEP_AFTER_BEACON_SUPPORT,
    HAL_GET_COMPRESS_SUPPORT,
    HAL_GET_BURST_SUPPORT,
    HAL_GET_FAST_FRAME_SUPPORT,
    HAL_GET_CHAP_TUNING_SUPPORT,
    HAL_GET_TURBO_G_SUPPORT,
    HAL_GET_TURBO_PRIME_SUPPORT,
    HAL_GET_DEVICE_TYPE,
    HAL_GET_XR_SUPPORT,
    HAL_GET_NUM_QUEUES,
    HAL_GET_KEY_CACHE_SIZE,
    HAL_GET_BEAMFORM_SUPPORT,
    HAL_GET_RXCOMB_SUPPORT,

    /* Multiple storage items with no direct storage map */
    HAL_GET_LOW_CHAN_EDGE,
    HAL_GET_HIGH_CHAN_EDGE,
    HAL_GET_MIC_SUPPORT,
    HAL_GET_CIPHER_SUPPORT,
    HAL_END_CAPABILITY_TYPE
} HAL_CAPABILITY_TYPE;

A_UINT32
halGetCapability(WLAN_DEV_INFO *pDev, HAL_CAPABILITY_TYPE requestType, A_UINT32 param);

A_BOOL
halGetSerialNumber(WLAN_DEV_INFO *pDev, A_CHAR *pSerialNum, A_UINT16 strLen);

A_STATUS
halSetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain);

void
halSetLedState(WLAN_DEV_INFO *pDev, A_BOOL bConnected);

void
halWriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset);

void
halGpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
halGpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

A_UINT32
halGpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio);

void
halGpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val);

void
halGpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel);

typedef struct {
    A_UINT32 nextTbtt;
    A_UINT32 nextDtim;
    A_UINT32 nextCfp;
    A_UINT32 beaconPeriod;
    A_UINT32 dtimPeriod;
    A_UINT32 cfpPeriod;
    A_UINT32 cfpDuration;
    A_UINT32 sleepDuration;
    A_UINT32 bmissThreshold;
} HAL_BEACON_TIMERS;

void
halSetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers);

void
halGetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf);

void
halResetTsf(WLAN_DEV_INFO *pDev);

void
halSetAdhocMode(WLAN_DEV_INFO *pDev);

void
halSetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *);

A_UINT32
halGetRandomSeed(WLAN_DEV_INFO *pDev);

A_BOOL
halDetectCardPresent(WLAN_DEV_INFO *pDev);

typedef enum {
    UPDATE_SW_COMMON = 1,
    UPDATE_SW_ALL,
    CLEAR_ALL_COUNTS,
    GET_COUNTERS
} HAL_MIB_CMD;

typedef struct halCounters {
    A_UINT32 txFrameCount;
    A_UINT32 rxFrameCount;
    A_UINT32 rxClearCount;
    A_UINT32 cycleCount;
    A_BOOL rxActive;
    A_BOOL txActive;
} HAL_COUNTERS;

A_UINT32
halMibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext);

typedef struct halChannelData {
    A_UINT8  clockRate;
    A_INT32  noiseFloor;
    A_UINT8  ccaThreshold;
} HAL_CHANNEL_DATA;

void
halGetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                  HAL_CHANNEL_DATA *pData);

void
halProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList);

typedef struct HalRadarPhyParams {
    A_INT32  firpwr;
    A_RSSI32 radarRssi;
    A_INT32  height;
    A_RSSI32 pulseRssi;
    A_INT32  inband;
} HAL_RADAR_PHY_PARAMS;

void
halEnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams);

void
halEnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr);

#define FRIENDLY_TURBO_RADAR_RSSI_THR                    5   /* in dB     */
#define FRIENDLY_TURBO_RADAR_RESET_INTERVAL              1   /* in seconds */
#define FRIENDLY_TURBO_RADAR_MAX_HISTORY                 500
#define FRIENDLY_TURBO_AR_REGION_WIDTH                   128
#define FRIENDLY_TURBO_AR_RSSI_DOUBLE_THRESHOLD          15  /* in dB     */
#define FRIENDLY_TURBO_AR_RSSI_THRESHOLD_STRONG_PKTS     17  /* in dB     */
#define FRIENDLY_TURBO_AR_MAX_NUM_ACK_REGIONS            9
#define FRIENDLY_TURBO_AR_ACK_DETECT_PAR_THRESHOLD       20
#define FRIENDLY_TURBO_AR_PKT_COUNT_THRESHOLD            20

void
halDisableFriendlyDetection(WLAN_DEV_INFO *pDev);

typedef enum {
    SET_NOISE_IMMUNITY_LEVEL = 1,
    SET_OFDM_WEAK_SIGNAL_DETECTION,
    SET_CCK_WEAK_SIGNAL_THR,
    SET_FIRSTEP_LEVEL,
    SET_SPUR_IMMUNITY_LEVEL
} HAL_ANI_CMD;

void
halAniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param);

A_INT32
halAniGetListenTime(WLAN_DEV_INFO *pDev);

A_RSSI32
halGetCurRssi(WLAN_DEV_INFO *pDev);

A_UINT32
halGetDefAntenna(WLAN_DEV_INFO *pDev);

void
halSetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna);

void
halSetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                    CHAN_VALUES *pChval);

void
halUpdateAntenna(WLAN_DEV_INFO *, SIB_ENTRY *, int retries, A_RSSI rssiAck, A_UINT8 curTxAnt);

void
halDmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose);

A_UINT32
halGetMacTimer3(WLAN_DEV_INFO *pDev);

A_UINT16
halSetDecompMask(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex, WLAN_PRIV_RECORD *pKey, A_BOOL en);

void
halSwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete);

A_UINT32
halGetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord);

typedef struct save_six_reg {
    A_UINT32    r1;
    A_UINT32    r2;
    A_UINT32    r3;
    A_UINT32    r4;
    A_UINT32    r5;
    A_UINT32    r6;
} SAVE_SIX_REG;

void
halGetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

void
halSetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);

/* Key Cache Functions */
void
halReserveHalKeyCacheEntries(WLAN_DEV_INFO *pDev);

void
halResetKeyCache(WLAN_DEV_INFO *pDev);

void
halResetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);

void
halSetKeyCacheEntry(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                    WLAN_MACADDR *pMacAddr, WLAN_PRIV_RECORD *pWlanPrivRecord,
                    A_BOOL bXorKey);

A_UINT16
halKeyCacheAlloc(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex, SIB_ENTRY *pSib,
                 A_UINT16 keyType, A_BOOL *pHwEncrypt);

void
halKeyCacheFree(WLAN_DEV_INFO *pDev, A_UINT16 hwIndex);


enum halPowerModes {
    AWAKE,
    FULL_SLEEP,
    NETWORK_SLEEP
};

/* Power Management Functions */
A_STATUS
halSetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip);

A_UINT32
halGetPowerMode(WLAN_DEV_INFO *pDev);

A_BOOL
halGetPowerStatus(WLAN_DEV_INFO *pDev);

void
halSetupPSPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pPSPollDesc);

void
halSetupBeaconDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt,
		   A_BOOL updtAntOnly, A_BOOL isAp);

void
halBeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp);

void
halSetupGrpPollChain(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pHead, ATHEROS_DESC *pTail, A_UINT32 hwIndex);

A_BOOL
halUseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev);

void
halSendXrChirp(WLAN_DEV_INFO *pDev);

/* Interrupt functions */

/*
 * NOTE WELL:
 * These are mapped to take advantage of the common locations for many of
 * the bits on all of the currently supported MAC chips. This is to make the ISR
 * as efficient as possible, while still abstracting HW differences. When new
 * hardware breaks this commonality this enumerated type, as well as the HAL
 * functions using it, must be modified. All values are directly mapped unless
 * commented otherwise.
 */
typedef enum HalIntType {
    HAL_INT_RX      = 0x00000001,   /* Non-common mapping */
    HAL_INT_RXEOL   = 0x00000010,
    HAL_INT_RXORN   = 0x00000020,
    HAL_INT_TX      = 0x00000040,   /* Non-common mapping */
    HAL_INT_TXDESC  = 0x00000080,   /* Non-common mapping */
    HAL_INT_TXURN   = 0x00000800,
    HAL_INT_MIB     = 0x00001000,
    HAL_INT_RXPHY   = 0x00004000,   /* Never really used as we get a RXERR interrupt also */
    HAL_INT_RXKCM   = 0x00008000,
    HAL_INT_SWBA    = 0x00010000,
    HAL_INT_BMISS   = 0x00040000,
    HAL_INT_BNR     = 0x00100000,   /* Non-common mapping */
    HAL_INT_RXCHIRP = 0x00200000,   /* RXCHIRP Interrupt */
    HAL_INT_BCNMISC = 0x00800000,   /* Venice, 'or' of misc beacon bits from ISR_S2 */
    HAL_INT_GPIO    = 0x01000000,

    /* The following are special values which don't map directly to ISR bits */
    HAL_INT_BCNTO   = 0x08000000,   /* Virtual map of BCNTO interrupt in ISR_S2 */
    HAL_INT_DTIM    = 0x20000000,   /* Virtual map of DTIM interrupt in ISR_S2 */
    HAL_INT_FATAL   = 0x40000000,   /* Non-common mapping */
    HAL_INT_GLOBAL  = 0x80000000,   /* To set/clear IER. Don't combine with other ints. */

    HAL_INT_COMMON  =  (HAL_INT_RXEOL | HAL_INT_RXORN | HAL_INT_TXURN | HAL_INT_MIB   |
                        HAL_INT_RXKCM | HAL_INT_SWBA  | HAL_INT_BMISS | HAL_INT_GPIO  |
                        HAL_INT_RXPHY),
    HAL_INT_INVALID = ~(HAL_INT_RX    | HAL_INT_RXEOL | HAL_INT_RXORN | HAL_INT_TX    |
                        HAL_INT_TXDESC| HAL_INT_TXURN | HAL_INT_MIB   | HAL_INT_RXKCM | 
                        HAL_INT_SWBA  | HAL_INT_BMISS | HAL_INT_BNR   | HAL_INT_GPIO  | 
                        HAL_INT_FATAL | HAL_INT_RXPHY | HAL_INT_GLOBAL),
    HAL_INT_NOCARD  = 0xffffffff    /* To signal that the card has been removed */
} HAL_INT_TYPE;

A_BOOL
halIsInterruptPending(WLAN_DEV_INFO *pDev);

HAL_INT_TYPE
halGetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pQueueBitMask);

void
halEnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

void
halDisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);

/* Manipulation of RF gain for temparature sensitivity */
typedef enum {
    RFGAIN_INACTIVE,
    RFGAIN_READ_REQUESTED,
    RFGAIN_NEED_CHANGE,
} RFGAIN_STATES;

RFGAIN_STATES
halGetRfgain(WLAN_DEV_INFO *pDevInfo);

#ifdef DEBUGWME
void
halDumpIFS(WLAN_DEV_INFO *pDevInfo);

void
ar52xxDumpIFS(WLAN_DEV_INFO *pDevInfo);
#endif


#ifdef _cplusplus
}
#endif

#endif /* _HAL_API_H */
