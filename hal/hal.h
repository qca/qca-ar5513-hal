/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines header objects used INTERNALLY by the HAL layer
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/hal.h#8 $
 */

#ifndef _HAL_H_
#define _HAL_H_

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlanos.h"     /* A_UINTxx, etc. */

#ifdef _cplusplus
extern "C" {
#endif

typedef enum {
    IQ_CAL_INACTIVE,
    IQ_CAL_RUNNING,
    IQ_CAL_DONE,
} IQ_CAL_STATES;


#ifdef AR5513
#define HAL_NUM_TX_QUEUES       5
#else
#define HAL_NUM_TX_QUEUES       10
#endif
#define HAL_COMP_BUF_MAX_SIZE   9216            /* 9K */
#define HAL_COMP_BUF_ALIGN_SIZE 512

/*
 * WARNING - this struct must remain A_UINT32 elements only and must exactly match
 * the order of HAL_CAPABILITY_TYPE or the halGetCapability() implementation must
 * be changed.
 */
typedef struct HalCapabilities {
    /* Direct mapped capabilities to the request type enum */
    A_UINT32  halRegDmn;
    A_UINT32  halWirelessModes;
    A_UINT32  halChanSpreadSupport;
    A_UINT32  halSleepAfterBeaconBroken;
    A_UINT32  halCompressSupport;
    A_UINT32  halBurstSupport;
    A_UINT32  halFastFramesSupport;
    A_UINT32  halChapTuningSupport;
    A_UINT32  halTurboGSupport;
    A_UINT32  halTurboPrimeSupport;
    A_UINT32  halDeviceType;
    A_UINT32  halXrSupport;
    A_UINT32  halTotalQueues;
    A_UINT32  halKeyCacheSize;
    A_UINT32  halBeamFormSupport;
    A_UINT32  halRxCombSupport;

    /* Multiple storage items depending on request type */
    A_UINT16  halLow5GhzChan;
    A_UINT16  halHigh5GhzChan;
    A_UINT16  halLow2GhzChan;
    A_UINT16  halHigh2GhzChan;
    A_BOOL    halMicAesCcmSupport;
    A_BOOL    halMicCkipSupport;
    A_BOOL    halMicTkipSupport;
    A_BOOL    halCipherAesCcmSupport;
    A_BOOL    halCipherCkipSupport;
    A_BOOL    halCipherTkipSupport;
} HAL_CAPABILITIES;

/* Storage for HAL-specific items */
typedef struct HalInfo {
    struct eepMap       *pEepData;          /* Holds all info read from EEPROM on first reset */
    struct gainValues   *pGainValues;       /* The thermal gain adjustment structure */
    A_UINT32            txQueueAllocMask;   /* Holds the allocation vector for tx queues */
    A_UINT32            txNormalIntMask;    /* Holds the Normal Interrupt bits for the Queues */
    A_UINT32            txDescIntMask;      /* Holds the Desc Interrupt bits for the Queues */
    A_UINT32            globIntRefCount;    /* Reference count for global interrupt enable */
    const struct RfHalFuncs *pRfHal;        /* Used for RF Hal */
    struct earHeader    *pEarHead;          /* All EAR information */
    void                *pAnalogBanks;      /* Analog Bank scratchpad */
    A_INT16             txPowerIndexOffset; /* Offset of transmit power table */
    A_UINT32            ofdmTxPower;        /* Tracks the nominal OFDM tx power level - mostly for probe requests */
    IQ_CAL_STATES       iqCalState;         /* Current state of IQ calibration */
    RFGAIN_STATES       rfgainState;        /* Current state of rfgain */
    A_BOOL              swSwapDesc;         /* flag indicating sw needs to swap descriptor fields */
    HAL_CAPABILITIES    halCapabilities;    /* capability values for misc small returns */
    A_CHAR              serialNumber[13];   /* Null terminated serial number */
    struct {
	A_UINT16            halKeyCacheTbl[4];  /* Reserved KeyCache indexes */
	A_UINT8             halAntCnt;          /* Antenna Count */
	A_UINT8             halAntWinCnt;       /* Antenna Window Count */
    } beaconAntCtrl;
    A_BOOL              halInit;
    A_BOOL              macReset;
    A_UINT32            pciCfg; 
    A_UINT16            chipVersion;        /* Chip version */
    A_UINT16            chipRev;            /* Chip revision */
    A_BOOL      done_synth_state_check_2_4; /* Flag to determine 2.4 synth cal
                                             *   FALSE == waiting for synth cal
                                             *   to complete.
                                             *   TRUE == completed */
    int         synth_state_2_4;            /* Integer value of rx_synth */
    int         synth_state_flag;           /* Flags, 
                                             *   b0: While set, disable tx bf updates,
                                             *       LNA off, RX open for Chain1,
                                             *       Turn on the post-LNA feed-through
                                             *       circuitry with GPIO 11 */
} HAL_INFO;

#define RX_FLIP_THRESHOLD 3 /* Count successful Tx before switching Rx Ant */

/* The device specific function list for the HW layer */
typedef struct HwFuncs {
    A_STATUS  (*hwDetach)(WLAN_DEV_INFO *pDev);
    A_STATUS  (*hwFillCapabilityInfo)(WLAN_DEV_INFO *pDev);

    /* Reset Functions */
    A_STATUS  (*hwReset)(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange);
    A_STATUS  (*hwPhyDisable)(WLAN_DEV_INFO *pDev);
    A_STATUS  (*hwDisable)(WLAN_DEV_INFO *pDev);
    A_STATUS  (*hwPerCalibration)(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);
    RFGAIN_STATES (*hwGetRfgain)(WLAN_DEV_INFO *pDev);
    void      (*hwSetTxPowerLimit)(WLAN_DEV_INFO *pDev, A_UINT32 limit);

    /* TX Functions */
    A_BOOL    (*hwUpdateTxTrigLevel)(WLAN_DEV_INFO *pDev, A_BOOL bIncTrigLevel);
    int       (*hwSetupTxQueue)(WLAN_DEV_INFO *pDev, HAL_TX_QUEUE_INFO *queueInfo);
    void      (*hwReleaseTxQueue)(WLAN_DEV_INFO *pDev, int queueNum);
    A_UINT32  (*hwGetTxDP)(WLAN_DEV_INFO *pDev, int queueNum);
    void      (*hwSetTxDP)(WLAN_DEV_INFO *pDev, int queueNum, A_UINT32 txdp);
    void      (*hwStartTxDma)(WLAN_DEV_INFO *pDev, int queueNum);
    A_UINT32  (*hwNumTxPending)(WLAN_DEV_INFO *pDev, int queueNum);
    void      (*hwStopTxDma)(WLAN_DEV_INFO *pDev, int queueNum, int msec);
    int       (*hwGetTxFilter)(WLAN_DEV_INFO *pDev, int queueNum, int index);
    void      (*hwSetTxFilter)(WLAN_DEV_INFO *pDev, int queueNum, int index, int value);
    int       (*hwGetTxFilterArraySize)(WLAN_DEV_INFO *pDev);
    void      (*hwGetTxFilterArray)(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count);
    void      (*hwSetTxFilterArray)(WLAN_DEV_INFO *pDev, A_UINT32 *pMmrArray, A_UINT32 count);
    void      (*hwPauseTx)(WLAN_DEV_INFO *pDev, A_UINT32 queueMask, A_BOOL pause);

    void      (*hwSetupTxDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc, A_UINT32 hwIndex);
    A_STATUS  (*hwProcessTxDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pTxDesc);
    A_BOOL    (*hwGetTxDescDone)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL swap);

#if defined(DEBUG) || defined(_DEBUG)
    void      (*hwDebugPrintTxDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_BOOL verbose);
#endif

    /* RX Functions */
    A_UINT32  (*hwGetRxDP)(WLAN_DEV_INFO *pDev);
    void      (*hwSetRxDP)(WLAN_DEV_INFO *pDev, A_UINT32 rxdp);
    void      (*hwEnableReceive)(WLAN_DEV_INFO *pDev);
    A_STATUS  (*hwStopDmaReceive)(WLAN_DEV_INFO *pDev);
    void      (*hwStartPcuReceive)(WLAN_DEV_INFO *pDev);
    void      (*hwStopPcuReceive)(WLAN_DEV_INFO *pDev);
    void      (*hwSetMulticastFilter)(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1);
    void      (*hwMulticastFilterIndex)(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet);
    void      (*hwRxFilter)(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper);
    A_STATUS  (*hwProcessRxDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc);
#ifdef UPSD
    void      (*hwPProcessRxDesc)(WLAN_DEV_INFO *pDev);
#endif
    void      (*hwSetupRxDesc)(ATHEROS_DESC *pDesc, A_UINT32 size);

    /* Misc Functions */
    A_STATUS  (*hwSetRegulatoryDomain)(WLAN_DEV_INFO *pDev, A_UINT16 regDomain);
    void      (*hwSetLedState)(WLAN_DEV_INFO *pDev, A_BOOL bConnected);
    void      (*hwWriteAssocid)(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset);
    void      (*hwGpioCfgInput)(WLAN_DEV_INFO *pDev, A_UINT32 gpio);
    void      (*hwGpioCfgOutput)(WLAN_DEV_INFO *pDev, A_UINT32 gpio);
    A_UINT32  (*hwGpioGet)(WLAN_DEV_INFO *pDev, A_UINT32 gpio);
    void      (*hwGpioSet)(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val);
    void      (*hwGpioSetIntr)(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel);
    void      (*hwSetStaBeaconTimers)(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers);
    void      (*hwGetTsf)(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf);
    void      (*hwResetTsf)(WLAN_DEV_INFO *pDev);
    void      (*hwSetAdhocMode)(WLAN_DEV_INFO *pDev);
    void      (*hwSetBasicRate)(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *);
    A_UINT32  (*hwGetRandomSeed)(WLAN_DEV_INFO *pDev);
    A_BOOL    (*hwDetectCardPresent)(WLAN_DEV_INFO *pDev);
    A_UINT32  (*hwMibControl)(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd,
                              void *pContext);
    void      (*hwGetChannelData)(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                                  HAL_CHANNEL_DATA *pData);
    void      (*hwProcessNoiseFloor)(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList);

    void      (*hwEnableRadarDetection)(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams);
    void      (*hwEnableFriendlyDetection)(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr);
    void      (*hwDisableFriendlyDetection)(WLAN_DEV_INFO *pDev);

    void      (*hwAniControl)(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param);
    A_INT32   (*hwAniGetListenTime)(WLAN_DEV_INFO *pDev);

    A_RSSI32  (*hwGetCurRssi)(WLAN_DEV_INFO *pDev);
    A_UINT32  (*hwGetDefAntenna)(WLAN_DEV_INFO *pDev);
    void      (*hwSetDefAntenna)(WLAN_DEV_INFO *pDev, A_UINT32 antenna);
    void      (*hwSetAntennaSwitch)(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings, CHAN_VALUES *pChval);
    void      (*hwUpdateAntenna)(WLAN_DEV_INFO *, SIB_ENTRY *, int retries, A_RSSI rssiAck, A_UINT8 curTxAnt);
    A_BOOL    (*hwUseShortSlotTime)(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev);
    void      (*hwDmaDebugDump)(WLAN_DEV_INFO *pDev, A_BOOL verbose);
    A_UINT32  (*hwGetMacTimer3)(WLAN_DEV_INFO *pDev);
    A_UINT16  (*hwSetDecompMask)(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                                 WLAN_PRIV_RECORD *pKey, A_BOOL en);
    void      (*hwSwapHwDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete);
    A_UINT32  (*hwGetHwDescWord)(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord);
    void      (*hwGetApSwitchHelper)(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);
    void      (*hwSetApSwitchHelper)(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs);
    void      (*hwSendXrChirp)(WLAN_DEV_INFO *pDev);

#if defined(DEBUG) || defined(_DEBUG)
    void      (*hwMonitorRxTxActivity)(WLAN_DEV_INFO *pDev, int index, int display);
#endif

    /* Key Cache Functions */
    A_UINT32  (*hwGetKeyCacheSize)(WLAN_DEV_INFO *pDev);
    void      (*hwReserveHalKeyCacheEntries)(WLAN_DEV_INFO *pDev);
    void      (*hwResetKeyCacheEntry)(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);
    void      (*hwSetKeyCacheEntry)(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                                    WLAN_MACADDR *pMacAddr,
                                    WLAN_PRIV_RECORD *pWlanPrivRecord,
                                    A_BOOL bXorKey);
    A_UINT16  (*hwKeyCacheAlloc)(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex,
                                 SIB_ENTRY *pSib, A_UINT16 keyType,
                                 A_BOOL *pHwEncrypt);
    void      (*hwKeyCacheFree)(WLAN_DEV_INFO *pDev, A_UINT16 keyCacheIndex);

    /* Power Management Functions */
    A_STATUS  (*hwSetPowerMode)(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip);
    A_UINT32  (*hwGetPowerMode)(WLAN_DEV_INFO *pDev);
    A_BOOL    (*hwGetPowerStatus)(WLAN_DEV_INFO *pDev);
    void      (*hwSetupPSPollDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pPSPollDesc);

    /* Beacon Functions */
    void      (*hwSetupBeaconDesc)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp);
    void      (*hwBeaconInit)(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp);
    void      (*hwSetupGrpPollChain)(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pHead, ATHEROS_DESC *pTail, A_UINT32 hwIndex);

    /* Interrupt Functions */
    A_BOOL    (*hwIsInterruptPending)(WLAN_DEV_INFO *pDev);
    HAL_INT_TYPE (*hwGetInterrupts)(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue,A_UINT32 *pQueueBitMask);
    void      (*hwEnableInterrupts)(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);
    void      (*hwDisableInterrupts)(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints);
#ifdef UPSD
    void      (*hwUpsdResponse)(WLAN_DEV_INFO *pDev);
#endif

} HW_FUNCS;

extern const char *halFrameTypeToName[];

#if DEBUG
extern int GainDebug;
#endif

#ifdef _cplusplus
}
#endif

#endif /* _HAL_H */
