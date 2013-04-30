/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Definitions for ar5211 and ar531x wlan hardware.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211.h#1 $
 */

#ifndef _AR5211_H_
#define _AR5211_H_

#ifdef _cplusplus
extern "C" {
#endif

/* Hardware-specific descriptor structures */

/* structure for Tx Control dwords */
typedef struct ar5211TxControl {
#ifdef BIG_ENDIAN
    A_UINT32    reserved1:1,        /* 31    reserved */
                encryptKeyValid:1,  /* 30    encryptKeyIndex is valid */
                interruptReq:1,     /* 29    interrupt on completion */
                antModeXmit:4,      /* 28:25 transmit antenna selection mode */
                clearDestMask:1,    /* 24    clear the Tx block bit */
                VEOL:1,             /* 23    virtual end-of-list flag */
                RTSCTSEnable:1,     /* 22    set to enable RTS/CTS */
                transmitRate:4,     /* 21:18 which rate to transmit at */
                reserved0:6,        /* 17:12 reserved */
                frameLength:12;     /* 11:0  length of the 802.11 packet */

    A_UINT32    reserved2:8,        /* 31:24 reserved */
                noAck:1,            /* 23    No ACK Flag */
                PktType:3,          /* 22:20 packet type, norm, ATIM or PS-Poll */
                encryptKeyIndex:7,  /* 19:13 index into the key cache */
                more:1,             /* 12    frame continues in next desc */
                bufferLength:12;    /* 11:0  size of buffer for this desc */
#else
    A_UINT32    frameLength:12,     /* 11:0  length of the 802.11 packet */
                reserved0:6,        /* 17:12 reserved */
                transmitRate:4,     /* 21:18 which rate to transmit at */
                RTSCTSEnable:1,     /* 22    set to enable RTS/CTS */
                VEOL:1,             /* 23    virtual end-of-list flag */
                clearDestMask:1,    /* 24    clear the Tx block bit */
                antModeXmit:4,      /* 28:25 transmit antenna selection mode */
                interruptReq:1,     /* 29    interrupt on completion */
                encryptKeyValid:1,  /* 30    encryptKeyIndex is valid */
                reserved1:1;        /* 31    reserved */
    A_UINT32    bufferLength:12,    /* 11:0  size of buffer for this descriptor */
                more:1,             /* 12    frame continues in next desc */
                encryptKeyIndex:7,  /* 19:13 index into the key cache */
                PktType:3,          /* 22:20 packet type, norm, ATIM or PS-Poll */
                noAck:1,            /* 23    No ACK Flag */
                reserved2:8;        /* 31:24 reserved */
#endif /* BIG_ENDIAN */
} AR5211_TX_CONTROL;

/* structure for Tx status dwords */
typedef volatile struct ar5211TxStatus {
#ifdef BIG_ENDIAN
    A_UINT32    sendTimestamp:16,   /* 31:16 time packet was transmitted */
                virtCollCount:4,    /* 15:12 virtual collision count */
                longRetryCount:4,   /* 11:8  long retries attempted */
                shortRetryCount:4,  /* 7:4   short retries attempted */
                filtered:1,         /* 3     frame not sent due to transmit blocking */
                fifoUnderrun:1,     /* 2     set if there was a fifo underrun */
                excessiveRetries:1, /* 1     set if there were excessive retries */
                pktTransmitOK:1;    /* 0     set if packet transmitted OK */

    A_UINT32    notUsed:10,         /* 31:22 reserved */
                dataOnAir:1,        /* 21    frame was transmitted (oahu only) */
                ackSigStrength:8,   /* 20:13 signal strength ack was received at */
                seqNum:12,          /* 12:1  sequence number assigned by hardware */
                done:1;             /* 1     HW is done with this descriptor */
#else
    A_UINT32    pktTransmitOK:1,    /* 0     set if packet transmitted OK */
                excessiveRetries:1, /* 1     set if there were excessive retries */
                fifoUnderrun:1,     /* 2     set if there was a fifo underrun */
                filtered:1,         /* 3     packet not sent due to transmit blocking */
                shortRetryCount:4,  /* 7:4   short retries attempted */
                longRetryCount:4,   /* 11:8  long retries attempted */
                virtCollCount:4,    /* 15:12 virtual collision count */
                sendTimestamp:16;   /* 31:16 time packet was transmitted */
    A_UINT32    done:1,             /* 1     HW is done with this descriptor */
                seqNum:12,          /* 12:1  sequence number assigned by hardware */
                ackSigStrength:8,   /* 20:13 signal strength ack was received at */
                dataOnAir:1,        /* 21    frame was transmitted (oahu only) */
                notUsed:10;         /* 31:21 reserved */
#endif /* BIG_ENDIAN */
} AR5211_TX_STATUS;

/* structure for Rx control dwords */
typedef struct ar5211DescRxControl {
#ifdef BIG_ENDIAN
    A_UINT32    notUsed1;           /* 31:0  reserved */
    A_UINT32    notUsed3:18,        /* 31:14 reserved */
                InterruptReq:1,     /* 13    specify mode of interrupt handling */
                notUsed2:1,         /* 12    reserved */
                bufferLength:12;    /* 11:0  length of the buffer */
#else
    A_UINT32    notUsed1;           /* 31:0  reserved */
    A_UINT32    bufferLength:12,    /* 11:0  length of the buffer */
                notUsed2:1,         /* 12    reserved */
                InterruptReq:1,     /* 13    specify mode of interrupt handling */
                notUsed3:18;        /* 31:14 reserved */
#endif
} AR5211_RX_CONTROL;

/* structure for Rx status dwords */
typedef volatile struct ar5211RxStatus {
#ifdef BIG_ENDIAN
    A_UINT32    notUsed2:2,         /* 31:30 reserved */
                rxAntenna:3,        /* 29:27 strength packet was received at */
                rxSigStrength:8,    /* 26:19 strength packet was received at */
                rxRate:4,           /* 18:15 rate packet was received at */
                notUsed1:2,         /* 14:13 reserved */
                more:1,             /* 12    set if packet continues in next descriptor */
                dataLength:12;      /* 11:0  number of bytes received into structure */
    A_UINT32    keyCacheMiss:1,     /* 31    key cache miss indication */
                rxTimestamp:15,     /* 30:16 timestamp when packet was recieved */
                keyIndex:7,         /* 15:9  index into key table */
                keyIndexValid:1,    /* 8     set if the key index is valid */
                phyError:3,         /* 7:5   set to indicate a phy error */
                decryptCRCError:1,  /* 4     set if there is an error on decryption */
                fifoOverrun:1,      /* 3     set if there is a fifo overrun */
                CRCError:1,         /* 2     set if there is a CRC error */
                pktReceivedOK:1,    /* 1     set if the packet was received OK */
                done:1;             /* 0     set when hardware is done updating the bit */
#else
    A_UINT32    dataLength:12,      /* 11:0  number of bytes received into structure */
                more:1,             /* 12    set if packet continues in next descriptor */
                notUsed1:2,         /* 14:13 reserved */
                rxRate:4,           /* 18:15 rate packet was received at */
                rxSigStrength:8,    /* 26:19 strength packet was received at */
                rxAntenna:3,        /* 29:27 strength packet was received at */
                notUsed2:2;         /* 31:30 reserved */
    A_UINT32    done:1,             /* 0     set when hardware is done updating the bit */
                pktReceivedOK:1,    /* 1     set if the packet was received OK */
                CRCError:1,         /* 2     set if there is a CRC error */
                fifoOverrun:1,      /* 3     set if there is a fifo overrun */
                decryptCRCError:1,  /* 4     set if there is an error on decryption */
                phyError:3,         /* 7:5   set to indicate a phy error */
                keyIndexValid:1,    /* 8     set if the key index is valid */
                keyIndex:7,         /* 15:9  index into key table */
                rxTimestamp:15,     /* 30:16 timestamp when packet was recieved */
                keyCacheMiss:1;     /* 31    key cache miss indication */
#endif /* BIG_ENDIAN */
} AR5211_RX_STATUS;

#define TX_CONTROL(pDesc)   ((AR5211_TX_CONTROL *)(&(pDesc)->hw.word[0]))
#define RX_CONTROL(pDesc)   ((AR5211_RX_CONTROL *)(&(pDesc)->hw.word[0]))
#define TX_STATUS(pDesc)    ((AR5211_TX_STATUS *)(&(pDesc)->hw.word[2]))
#define RX_STATUS(pDesc)    ((AR5211_RX_STATUS *)(&(pDesc)->hw.word[2]))

/* Key Cache data strucuture */

typedef struct Ar5211KeyCacheEntry {
    A_UINT32    keyVal0;
    A_UINT32    keyVal1;
    A_UINT32    keyVal2;
    A_UINT32    keyVal3;
    A_UINT32    keyVal4;
    A_UINT32    keyType;
    A_UINT32    macAddrLo;
    A_UINT32    macAddrHi;
} AR5211_KEY_CACHE_ENTRY;

#ifdef BIG_ENDIAN
typedef struct {
    A_UINT32    reserved4:16,
                keyValid:1,
                macAddrHi:15;
} MAC_ADDR_HI_KEY_VALID;
#else
typedef struct {
    A_UINT32    macAddrHi:15,
                keyValid:1,
                reserved4:16;
} MAC_ADDR_HI_KEY_VALID;
#endif /* BIG_ENDIAN */

/* DCU Transmit Filter macros */
#define CALC_MMR(dcu, idx)          ( (4 * dcu) + \
                                      (idx < 32 ? 0 : (idx < 64 ? 1 : (idx < 96 ? 2 : 3))) )
#define TXBLK_FROM_MMR(mmr)         (MAC_D_TXBLK_BASE + ((mmr & 0x1f) << 6) + ((mmr & 0x20) >> 3))
#define CALC_TXBLK_ADDR(dcu, idx)   (TXBLK_FROM_MMR(CALC_MMR(dcu, idx)))
#define CALC_TXBLK_VALUE(idx)       (1 << (idx & 0x1f))

/* MAC register values */

#define INIT_INTERRUPT_MASK         ( MAC_IMR_TXERR  | MAC_IMR_TXOK | MAC_IMR_RXORN | \
                                      MAC_IMR_RXERR  | MAC_IMR_RXOK | MAC_IMR_TXURN | \
                                      MAC_IMR_HIUERR | MAC_IMR_TXDESC )

#define INIT_CONFIG_STATUS          0x00000000
#define INIT_RSSI_THR               0x00000700  // Missed beacon counter initialized to 0x7 (max is 0xff)
#define INIT_IQCAL_LOG_COUNT_MAX    0xF
#define INIT_BCON_CNTRL_REG         0x00000000

#define INIT_SH_RETRY               10
#define INIT_LG_RETRY               10
#define INIT_SSH_RETRY              32
#define INIT_SLG_RETRY              32

/*
 * Various fifo fill before Tx start, in 64-byte units
 * i.e. put the frame in the air while still DMAing
 */
#define MIN_TX_FIFO_THRESHOLD       0x1
#define MAX_TX_FIFO_THRESHOLD       ((MAX_WLAN_BODY_SIZE / 64) + 1)
#define INIT_TX_FIFO_THRESHOLD      MIN_TX_FIFO_THRESHOLD

/* EEPROM defines for Version 2 & 3 AR5211 chips */
#define EEPROM_RFSILNT_CLKEN_OFFSET 0x0F    /* EEPROM RF Silent/Clock Run Enable */
#define EEPROM_MAC_LSW_OFFSET       0x1D    /* EEPROM Mac Low */
#define EEPROM_MAC_MID_OFFSET       0x1E    /* EEPROM Mac Mid */
#define EEPROM_MAC_MSW_OFFSET       0x1F    /* EEPROM Mac High */
#define EEPROM_PROTECT_OFFSET       0x3F    /* EEPROM Protect Bits */
#define REGULATORY_DOMAIN_OFFSET    0xBF    /* EEPROM Location of the current RD */
#define EEPROM_GROUPS_OFFSET3_2     0x100   /* groups offset for ver3.2 and earlier */
#define EEPROM_GROUPS_OFFSET3_3     0x150   /* groups offset for ver3.3 */
#define GROUP1_OFFSET               0x0     /* relative offset of group(i) to groups_offset */
#define GROUP2_OFFSET               0x5
#define GROUP3_OFFSET               0x37
#define GROUP4_OFFSET               0x46
#define GROUP5_OFFSET               0x55
#define GROUP6_OFFSET               0x65
#define GROUP7_OFFSET               0x69
#define GROUP8_OFFSET               0x6f
#define ATHEROS_EEPROM_OFFSET       0xC0    /* Atheros EEPROM defined values start at 0xC0 */
#define EEPROM_VER3                 0x3000  /* 16k EEPROM */
#define EEPROM_VER3_1               0x3001  /* Adds turbo limit, antenna gain, 16 CTL's */
                                            /* 11g info, and 2.4GHz ob/db for B & G */
#define EEPROM_VER3_2               0x3002  /* More accurate pcdac intercepts, analog chip calibration */
#define EEPROM_VER3_3               0x3003  /* Adds CTL in-band limit, 32 CTL's, frequency expansion */
#define EEPROM_VER3_4               0x3004  /* Adds gainI, 2.4 Turbo params */

/* RF silent fields in EEPROM */
#define EEP_RFSILENT_GPIO_SEL_MASK  0x001c
#define EEP_RFSILENT_GPIO_SEL_SHIFT 2
#define EEP_RFSILENT_POLARITY_MASK  0x0002
#define EEP_RFSILENT_POLARITY_SHIFT 1

/* IMPORTANT:    change EEP_SCALE and EEP_DELTA together */
#define EEP_SCALE   100         /* scale everything by this amount so get reasonable accuracy */
                                /* but don't need to do float arithmetic */
#define EEP_DELTA   10          /* SCALE/10, but don't want to do divide arithmetic each time */
#define PWR_MIN     0
#define PWR_MAX     3150        /* 31.5 * SCALE */
#define PWR_STEP    50          /* 0.5 * SCALE */
/* Keep 2 above defines together */

#define NUM_11A_EEPROM_CHANNELS     10
#define NUM_2_4_EEPROM_CHANNELS     3
#define NUM_PCDAC_VALUES            11
#define NUM_TEST_FREQUENCIES        8
#define NUM_EDGES                   8
#define NUM_INTERCEPTS              11
#define FREQ_MASK                   0x7f
#define FREQ_MASK_3_3               0xff
#define PCDAC_MASK                  0x3f
#define POWER_MASK                  0x3f
#define NON_EDGE_FLAG_MASK          0x40
#define CHANNEL_POWER_INFO          8
#define OBDB_UNSET                  0xffff

/* Used during pcdac table construction */
#define PCDAC_START                 1
#define PCDAC_STOP                  63
#define PCDAC_STEP                  1
#define PWR_TABLE_SIZE              64

/* Used during power/rate table construction */
#define NUM_RATES                   8
#define NUM_CTLS                    16
#define NUM_CTLS_3_3                32

/* Used when setting tx power limit */
#define MAX_RATE_POWER              60

/* EEPROM Modes */
enum eepModeValues {
    MODE_11A     = 0,
    MODE_OFDM_24 = 1,
    MODE_11B     = 2,
    MAX_MODE     = 2
};

typedef struct fullPcdacStruct {
    A_UINT16        channelValue;
    A_UINT16        pcdacMin;
    A_UINT16        pcdacMax;
    A_UINT16        numPcdacValues;
    A_UINT16        PcdacValues[64];
    A_INT16         PwrValues[64];      // power is 32bit since in dest it is scaled
} FULL_PCDAC_STRUCT;

typedef struct dataPerChannel {
    A_UINT16        channelValue;
    A_UINT16        pcdacMin;
    A_UINT16        pcdacMax;
    A_UINT16        numPcdacValues;
    A_UINT16        PcdacValues[NUM_PCDAC_VALUES];
    A_INT16         PwrValues[NUM_PCDAC_VALUES];    // power is 32bit since in dest it is scaled
} DATA_PER_CHANNEL;

typedef struct pcdacsEepromAllModes {
    //lla info
    A_UINT16            Channels11a[NUM_11A_EEPROM_CHANNELS];
    A_UINT16            numChannels11a;
    DATA_PER_CHANNEL    DataPerChannel11a[NUM_11A_EEPROM_CHANNELS];

    A_UINT16            numChannels2_4;
    A_UINT16            Channels2_4[NUM_2_4_EEPROM_CHANNELS];

    //11g info
    DATA_PER_CHANNEL    DataPerChannel11g[NUM_2_4_EEPROM_CHANNELS];

    //11b info
    DATA_PER_CHANNEL    DataPerChannel11b[NUM_2_4_EEPROM_CHANNELS];
} PCDACS_ALL_MODES;

/* points to the appropriate pcdac structs in the above struct based on mode */
typedef struct pcdacsEeprom {
    A_UINT16            *pChannelList;
    A_UINT16            numChannels;
    DATA_PER_CHANNEL    *pDataPerChannel;
} PCDACS_EEPROM;

typedef struct trgtPowerInfo {
    A_UINT16    twicePwr54;
    A_UINT16    twicePwr48;
    A_UINT16    twicePwr36;
    A_UINT16    twicePwr6_24;
    A_UINT16    testChannel;
} TRGT_POWER_INFO;

typedef struct trgtPowerAllModes {
    A_UINT16        numTargetPwr_11a;
    TRGT_POWER_INFO trgtPwr_11a[NUM_TEST_FREQUENCIES];
    A_UINT16        numTargetPwr_11g;
    TRGT_POWER_INFO trgtPwr_11g[3];
    A_UINT16        numTargetPwr_11b;
    TRGT_POWER_INFO trgtPwr_11b[2];
} TRGT_POWER_ALL_MODES;

typedef struct rdEdgesPower {
    A_UINT16    rdEdge;
    A_UINT16    twice_rdEdgePower;
    A_BOOL      flag;
} RD_EDGES_POWER;

typedef struct cornerCalInfo {
    A_UINT16 gSel;
    A_UINT16 pd84;
    A_UINT16 pd90;
    A_UINT16 clip;
} CORNER_CAL_INFO;

/*
 * Header Info - general parameters and
 * values set for each chipset board solution
 * that are programmed every reset
 */
typedef struct eepHeaderInfo
{
    /* General Device Parameters */
    A_UINT16 turboDisable;
    A_UINT16 rfKill;
    A_UINT16 deviceType;
    A_UINT16 turbo2WMaxPower;
    A_UINT16 Amode;
    A_UINT16 Bmode;
    A_UINT16 Gmode;
    A_UINT16 xtnd5GSupport;
    A_INT8   antennaGainMax[2];

    /* 5 GHz / 2.4 GHz CKK / 2.4 GHz OFDM common parameters */
    A_UINT16 switchSettling[3];
    A_UINT16 txrxAtten[3];
    A_UINT16 txEndToXLNAOn[3];
    A_UINT16 thresh62[3];
    A_UINT16 txEndToXPAOff[3];
    A_UINT16 txFrameToXPAOn[3];
    A_INT8   adcDesiredSize[3];     /* 8-bit signed value */
    A_INT8   pgaDesiredSize[3];     /* 8-bit signed value */
    A_INT16  noiseFloorThresh[3];
    A_UINT16 xlnaGain[3];
    A_UINT16 xgain[3];
    A_UINT16 xpd[3];
    A_UINT16 antennaControl[11][3];
    A_UINT16 falseDetectBackoff[3];
    A_UINT16 gainI[3];

    /* 5 GHz parameters */
    A_UINT16 ob1;
    A_UINT16 db1;
    A_UINT16 ob2;
    A_UINT16 db2;
    A_UINT16 ob3;
    A_UINT16 db3;
    A_UINT16 ob4;
    A_UINT16 db4;

    /* 2.4 GHz parameters */
    A_UINT16 obFor24;
    A_UINT16 dbFor24;
    A_UINT16 obFor24g;
    A_UINT16 dbFor24g;
    A_UINT16 ob2GHz[2];
    A_UINT16 db2GHz[2];
    A_UINT16 numCtls;
    A_UINT16 ctl[32];

    /* corner calibration information */
    CORNER_CAL_INFO  cornerCal;	
} EEP_HEADER_INFO;

typedef struct eepMap {
    A_UINT16         version;        /* Version field */
    A_UINT16         protect;        /* EEPROM protect field */
    EEP_HEADER_INFO  *pEepHeader;
    PCDACS_ALL_MODES *pPcdacInfo;
    TRGT_POWER_ALL_MODES  *pTrgtPowerInfo;
    RD_EDGES_POWER   *pRdEdgesPower;
} EEP_MAP;

/* Data structures initialization for gainF functions */

#define NUM_CORNER_FIX_BITS 4
#define DYN_ADJ_UP_MARGIN 15
#define DYN_ADJ_LO_MARGIN 20

typedef struct GainOptStep {
    A_INT16 paramVal[NUM_CORNER_FIX_BITS];
    A_INT32 stepGain;
    A_CHAR  stepName[16];
} GAIN_OPTIMIZATION_STEP;

typedef struct gainOptLadder {
    A_UINT32                numStepsInLadder;
    A_UINT32                defaultStepNum;
    GAIN_OPTIMIZATION_STEP  optStep[10];
} GAIN_OPTIMIZATION_LADDER;

typedef struct gainValues {
    A_UINT32                     currStepNum;
    A_UINT32                     currGain;
    A_UINT32                     targetGain;
    A_UINT32                     loTrig;
    A_UINT32                     hiTrig;
    A_BOOL                       active;
    const GAIN_OPTIMIZATION_STEP *currStep;
} GAIN_VALUES;

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_H_ */
