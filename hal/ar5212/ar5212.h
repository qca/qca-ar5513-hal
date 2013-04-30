/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Definitions for ar5212 wlan hardware.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212.h#2 $
 */

#ifndef _AR5212_H_
#define _AR5212_H_

#ifdef _cplusplus
extern "C" {
#endif

/* Hardware-specific descriptor structures */

/* structure for Tx Control dwords */
typedef struct ar5212TxControl {
#ifdef BIG_ENDIAN
    /* word 2 */
    A_UINT32    CTSEnable:1,        /* 31    Precede frame with CTS flag */
                destIdxValid:1,     /* 30    specifies whether the contents of the DestIdx field are valid */
                interruptReq:1,     /* 29    set if want an interrupt on completion */
                antModeXmit:4,      /* 28:25 transmit antenna selection mode */
                clearDestMask:1,    /* 24    set if want to clear the Tx block bit */
                VEOL:1,             /* 23    Virtual end-of-list flag */
                RTSEnable:1,        /* 22    set to enable RTS/CTS */
                transmitPwrCtrl:6,  /* 21:16 transmit power control */
                reserved0:4,        /* 15:12 reserved */
                frameLength:12;     /* 11:0  length of the 802.11 packet */

    /* word 3 */
    A_UINT32    reserved2:1,        /* 31    reserved */
                compICVLen:2,       /* 30:29 length of the frames ICV */
                compIVLen:2,        /* 28:27 length of the frames IV */
                compProc:2,         /* 26:25 compression processing indication */
                noAck:1,            /* 24    No ACK Flag */
                PktType:4,          /* 23:20 set packet type, norm, ATIM or PS-Poll */
                destIdx:7,          /* 19:13 index into which encryption key to use */
                more:1,             /* 12    set if packet continues in next descriptor */
                bufferLength:12;    /* 11:0  size of buffer for this descriptor */

    /* word 4 */
    A_UINT32    TXDataTries3:4,     /* 31:28 Num frame data exchange attempts for series 3 */
                TXDataTries2:4,     /* 27:24 Num frame data exchange attempts for series 2 */
                TXDataTries1:4,     /* 23:20 Num frame data exchange attempts for series 1 */
                TXDataTries0:4,     /* 19:16 Num frame data exchange attempts for series 0 */
                durUpdateEn:1,      /* 15    Frame duration update control */
                RTSCTSDur:15;       /* 14:0  RTS/CTS duration field value */

    /* word 5 */
    A_UINT32    reserved3:7,        /* 31:25 reserved */
                RTSCTSRate:5,       /* 24:20 RTS or CTS rate */
                TXRate3:5,          /* 19:15 Tx rate for series 3 */
                TXRate2:5,          /* 14:10 Tx rate for series 2 */
                TXRate1:5,          /* 09:05 Tx rate for series 1 */
                TXRate0:5;          /* 04:00 Tx rate for series 0 */
#else
    /* word 2 */
    A_UINT32    frameLength:12,     /* 11:0  length of the 802.11 packet */
                reserved0:4,        /* 15:12 reserved */
                transmitPwrCtrl:6,  /* 21:16 transmit power control */
                RTSEnable:1,        /* 22    set to enable RTS/CTS protocol */
                VEOL:1,             /* 23    Virtual end-of-list flag */
                clearDestMask:1,    /* 24    set if want to clear the Tx block bit */
                antModeXmit:4,      /* 28:25 transmit antenna selection mode */
                interruptReq:1,     /* 29    set if want an interrupt on completion */
                destIdxValid:1,     /* 30    specifies whether the contents of the DestIdx field are valid */
                CTSEnable:1;        /* 31    Precede frame with CTS flag */

    /* word 3 */
    A_UINT32    bufferLength:12,    /* 11:0  size of buffer for this descriptor */
                more:1,             /* 12    set if packet continues in next descriptor */
                destIdx:7,          /* 19:13 index into which encryption key to use */
                PktType:4,          /* 23:20 set packet type, norm, ATIM or PS-Poll */
                noAck:1,            /* 24    No ACK Flag */
                compProc:2,         /* 26:25 compression processing indication */
                compIVLen:2,        /* 28:27 length of the frames IV */
                compICVLen:2,       /* 30:29 length of the frames ICV */
                reserved2:1;        /* 31    reserved */

    /* word 4 */
    A_UINT32    RTSCTSDur:15,       /* 14:0  RTS/CTS duration field value */
                durUpdateEn:1,      /* 15    Frame duration update control */
                TXDataTries0:4,     /* 19:16 Num frame data exchange attempts for series 0 */
                TXDataTries1:4,     /* 23:20 Num frame data exchange attempts for series 1 */
                TXDataTries2:4,     /* 27:24 Num frame data exchange attempts for series 2 */
                TXDataTries3:4;     /* 31:28 Num frame data exchange attempts for series 3 */

    /* word 5 */
    A_UINT32    TXRate0:5,          /* 04:00 Tx rate for series 0 */
                TXRate1:5,          /* 09:05 Tx rate for series 1 */
                TXRate2:5,          /* 14:10 Tx rate for series 2 */
                TXRate3:5,          /* 19:15 Tx rate for series 3 */
                RTSCTSRate:5,       /* 24:20 RTS or CTS rate */
                reserved3:7;        /* 31:25 reserved */
#endif /* BIG_ENDIAN */
} AR5212_TX_CONTROL;

/* structure for Tx status dwords */
typedef volatile struct ar5212TxStatus {
#ifdef BIG_ENDIAN
    /* word 6 */
    A_UINT32    sendTimestamp:16,   /* 31:16 time packet was transmitted */
                virtCollCount:4,    /* 15:12 virtual collision count */
                dataFailCnt:4,      /* 11:08 Data failure count */
                RTSFailCnt:4,       /* 07:04 RTS failure count */
                filtered:1,         /* 03    packet not sent due to transmit blocking */
                fifoUnderrun:1,     /* 02    set if there was a fifo underrun */
                excessiveRetries:1, /* 01    set if there were excessive retries */
                pktTransmitOK:1;    /* 00    set if packet transmitted OK */

    /* word 7 */
    A_UINT32    notUsed:7,          /* 31:25 reserved */
                txAnt:1,            /* 24    transmit antenna indication */
                compSuccess:1,      /* 23    compression outcome indication */
                finalTSIdx:2,       /* 22:21 final Tx attempt series index */
                ackSigStrength:8,   /* 20:13 signal strength ack was received at */
                seqNum:12,          /* 12:01 sequence number assigned by hardware */
                done:1;             /* 00    set by HW when completes descriptor update */
#else
    /* word 6 */
    A_UINT32    pktTransmitOK:1,    /* 00    set if packet transmitted OK */
                excessiveRetries:1, /* 01    set if there were excessive retries */
                fifoUnderrun:1,     /* 02    set if there was a fifo underrun */
                filtered:1,         /* 03    packet not sent due to transmit blocking */
                RTSFailCnt:4,       /* 07:04 RTS failure count */
                dataFailCnt:4,      /* 11:08 Data failure count */
                virtCollCount:4,    /* 15:12 virtual collision count */
                sendTimestamp:16;   /* 31:16 time packet was transmitted */

    /* word 7 */
    A_UINT32    done:1,             /* 00    set by HW when completes descriptor update */
                seqNum:12,          /* 12:01 sequence number assigned by hardware */
                ackSigStrength:8,   /* 20:13 signal strength ack was received at */
                finalTSIdx:2,       /* 22:21 final Tx attempt series index */
                compSuccess:1,      /* 23    compression outcome indication */
                txAnt:1,            /* 24    transmit antenna indication */
                notUsed:7;          /* 31:25 reserved */
#endif /* BIG_ENDIAN */
} AR5212_TX_STATUS;

/* structure for Rx control dwords */
typedef struct ar5212DescRxControl {
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
} AR5212_RX_CONTROL;

/* structure for Rx status dwords */
typedef volatile struct ar5212RxStatus {
#ifdef BIG_ENDIAN
    /* word 4 */
    A_UINT32    rxAntenna:4,        /* 31:28 Rx antenna indication */
                rxSigStrength:8,    /* 27:20 strength packet was received at */
                rxRate:5,           /* 19:15 rate packet was received at */
                notUsed1:1,         /* 14    reserved */
                decompCRCError:1,   /* 13    decompression CRC error */
                more:1,             /* 12    set if packet continues in next descriptor */
                dataLength:12;      /* 11:0  number of bytes received into structure */

    /* word 5 */
    A_UINT32    keyCacheMiss:1,     /* 31    key cache miss indication */
                rxTimestamp:15,     /* 30:16 timestamp when packet was recieved */
                keyIndex:7,         /* 15:09 index into key table; phyErrorCode also - see below */
                keyIndexValid:1,    /* 08    set if the key index is valid; phyErrorCode also - see below */
                notUsed2:2,         /* 07:06 reserved */
                michaelError:1,     /* 05    set if there was a Michael MIC decrypt error */
                phyErrorOccured:1,  /* 04    set to indicate a phy error occured */
                decryptCRCError:1,  /* 03    set if there is an error on decryption */
                CRCError:1,         /* 02    set if there is a CRC error */
                pktReceivedOK:1,    /* 01    set if the packet was received OK */
                done:1;             /* 00    set when hardware is done updating descr */
#else
    /* word 4 */
    A_UINT32    dataLength:12,      /* 11:0  number of bytes received into structure */
                more:1,             /* 12    set if packet continues in next descriptor */
                decompCRCError:1,   /* 13    decompression CRC error */
                notUsed1:1,         /* 14    reserved */
                rxRate:5,           /* 19:15 rate packet was received at */
                rxSigStrength:8,    /* 27:20 strength packet was received at */
                rxAntenna:4;        /* 31:28 Rx antenna indication */

    /* word 5 */
    A_UINT32    done:1,             /* 00    set when hardware is done updating descr */
                pktReceivedOK:1,    /* 01    set if the packet was received OK */
                CRCError:1,         /* 02    set if there is a CRC error */
                decryptCRCError:1,  /* 03    set if there is an error on decryption */
                phyErrorOccured:1,  /* 04    set to indicate a phy error occured */
                michaelError:1,     /* 05    set if there was a Michael MIC decrypt error */
                notUsed2:2,         /* 07:06 reserved */
                keyIndexValid:1,    /* 08    set if the key index is valid; phyErrorCode also - see below */
                keyIndex:7,         /* 15:09 index into key table; phyErrorCode also - see below */
                rxTimestamp:15,     /* 30:16 timestamp when packet was recieved */
                keyCacheMiss:1;     /* 31    key cache miss indication */
#endif /* BIG_ENDIAN */
} AR5212_RX_STATUS;

/*
 * bits 15:8 of the second Rx Status word, i.e. keyIndexValid,keyIndex,
 * do double duty as phy error code if phyErrorOccured is set
 */
#define PHY_ERROR_CODE_MASK         0x0000ff00
#define PHY_ERROR_CODE_SHIFT        8
#define PHY_ERROR_CODE(_pRxStatus)  ((_pRxStatus)->phyErrorOccured                          \
    ? (((*(((A_UINT32 *)(_pRxStatus)) + 1)) & PHY_ERROR_CODE_MASK) >> PHY_ERROR_CODE_SHIFT) \
    : 0)

#define TX_CONTROL(pDesc)           ((AR5212_TX_CONTROL *)(&(pDesc)->hw.word[0]))
#define RX_CONTROL(pDesc)           ((AR5212_RX_CONTROL *)(&(pDesc)->hw.word[0]))
#define TX_STATUS(pDesc)            ((AR5212_TX_STATUS *)(&(pDesc)->hw.word[4]))
#define RX_STATUS(pDesc)            ((AR5212_RX_STATUS *)(&(pDesc)->hw.word[2]))

/* Key Cache data structure */

typedef struct Ar5212KeyCacheEntry {
    A_UINT32    keyVal0;
    A_UINT32    keyVal1;
    A_UINT32    keyVal2;
    A_UINT32    keyVal3;
    A_UINT32    keyVal4;
    A_UINT32    keyType;    /* 03 lastTxAnt, 08:04 keyCacheAsyncRateOff */
    A_UINT32    macAddrLo;
    A_UINT32    macAddrHi;
} AR5212_KEY_CACHE_ENTRY;

#ifdef BIG_ENDIAN
typedef struct {
    A_UINT32    reserved4:16,   /* 31:16 reserved */
                keyValid:1,     /* 15 key valid */
                macAddrHi:15;   /* 14:00 hi addr */
} MAC_ADDR_HI_KEY_VALID;
#else
typedef struct {
    A_UINT32    macAddrHi:15,   /* 14:00 hi addr */
                keyValid:1,     /* 15 key valid */
                reserved4:16;   /* 31:16 reserved */
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

/* RF silent fields in EEPROM */
#define EEP_RFSILENT_GPIO_SEL_MASK  0x001c
#define EEP_RFSILENT_GPIO_SEL_SHIFT 2
#define EEP_RFSILENT_POLARITY_MASK  0x0002
#define EEP_RFSILENT_POLARITY_SHIFT 1

/* Mac Versions */
enum AR5212MacVers {
    MAC_SREV_VERSION_5212   = 5,
    MAC_SREV_VERSION_2413   = 7,
};

/* Mac Revisions */
enum AR5212MacRevs {
    MAC_5312_2_0            = 2,
    MAC_5212_1_4            = 4
};

/* Baseband Revisions */
enum AR5212BasebandRevs {
    PHY_CHIP_ID_REV_2       = 0x42,  /* 5212 Rev 2 BB w. TPC fix */
    PHY_CHIP_ID_REV_3       = 0x43   /* 5212 Rev 3 5213 and up */
};

/* Analog Revisions */
enum AR5212AnalogRevs {
    RAD5111_SREV_MAJOR      = 0x10,  /* 5111 Major Rev */
    RAD2111_SREV_MAJOR      = 0x20,  /* 2111 Major Rev */
    RAD5112_SREV_MAJOR      = 0x30,  /* 5112 Major Rev */
    RAD2112_SREV_MAJOR      = 0x40,  /* 5112 Major Rev */
    RAD2413_SREV_MAJOR      = 0x50,  /* 2413 Major Rev */
    RAD5112_SREV_2_0        = 0x35,  /* 5112 Rev 2.0 */
    RAD2112_SREV_2_0        = 0x45,  /* 5112 Rev 2.0 */
};

#define IS_2413(x)           ((x)->macVersion == MAC_SREV_VERSION_2413)
#define IS_5112(x)           (((x)->analog5GhzRev & 0xF0) >= RAD5112_SREV_MAJOR)
#define IS_RAD5112_REV1(x)   ( (((x)->analog5GhzRev & 0xF0) == RAD5112_SREV_MAJOR) && (((x)->analog5GhzRev & 0x0F) < (RAD5112_SREV_2_0 & 0x0F)) )
#define IS_RADX112_REV2_0(x) ( ((x)->analog5GhzRev == RAD5112_SREV_2_0) || ((x)->analog5GhzRev == RAD2112_SREV_2_0) )

#define IS_5312_2_X(x)       ( ((x)->macVersion == MAC_SREV_VERSION_5212) && \
                               (((x)->macRev == 2) || ((x)->macRev == 7)) )

#define MAX_ANALOG_BANKS         8
#define VALID_5111_BANK_MASK  0xCF   /* Banks 0, 1, 2, 3, 6, 7 for 5111 */
#define VALID_5112_BANK_MASK  0xCE   /* Banks 1, 2, 3, 6, 7 for 5112 */

/* Channel specific macros */
#define NO_HEAVY_CLIP_CHAN_2413(x)  ((x) == 2472)

/* EEPROM defines */
#define EEPROM_RFSILNT_CLKEN_OFFSET 0x0F    /* EEPROM RF Silent/Clock Run Enable */
#define EEPROM_SIZE_LOWER           0x1B    /* EEPROM Size Info */
#define EEPROM_SIZE_UPPER           0x1C    /* EEPROM Size Info */
#define EEPROM_SIZE_UPPER_M         0xFFF0  /* Mask to get EEPROM upper 12 bits*/
#define EEPROM_MAC_LSW_OFFSET       0x1D    /* EEPROM Mac Low */
#define EEPROM_MAC_MID_OFFSET       0x1E    /* EEPROM Mac Mid */
#define EEPROM_MAC_MSW_OFFSET       0x1F    /* EEPROM Mac High */
#define EEPROM_PROTECT_OFFSET       0x3F    /* EEPROM Protect Bits */
#define EEPROM_SERIAL_NUM_OFFSET    0xB0    /* EEPROM serial number */
#define EEPROM_SERIAL_NUM_SIZE      12      /* EEPROM serial number size */
#define REGULATORY_DOMAIN_OFFSET    0xBF    /* EEPROM Location of the current RD */
#define EEPROM_CAPABILITIES_OFFSET  0xC9    /* EEPROM Location of capabilities */
#define ATHEROS_EEPROM_OFFSET       0x0C0   /* Atheros EEPROM defined values start at 0xC0 */
#define ATHEROS_EEPROM_END_DEFAULT  0x400   /* Size of the EEPROM */

#define EEPROM_VER3                 0x3000  /* 16k EEPROM */
#define EEPROM_VER3_1               0x3001  /* Adds turbo limit, antenna gain, 16 CTL's */
                                            /* 11g info, and 2.4GHz ob/db for B & G */
#define EEPROM_VER3_2               0x3002  /* More accurate pcdac intercepts, analog chip calibration */
#define EEPROM_VER3_3               0x3003  /* Adds CTL in-band limit, 32 CTL's, frequency expansion */
#define EEPROM_VER3_4               0x3004  /* Adds XR power, gainI, 2.4 Turbo params */
#define EEPROM_VER4_0               0x4000  /* Adds new 5112 cal powers and added params */
#define EEPROM_VER4_1               0x4001  /* Adds rxtxMargin board param */
#define EEPROM_VER4_2               0x4002  /* Adds cckOfdmGainDelta */
#define EEPROM_VER4_3               0x4003  /* Changes 5112 calibration data of xpd curves */
#define EEPROM_VER4_6               0x4006  /* Adds channel 14 cck ofdm adjustment */
#define EEPROM_VER4_9               0x4009  /* EEPROM EAR futureproofing */
#define EEPROM_VER5_0               0x5000  /* Adds new 2413 cal powers and added params */
#define EEPROM_VER5_1               0x5001  /* Adds capability values */

enum txCalStartingOffset {
    EEPROM_GROUPS_OFFSET3_2 = 0x100, /* groups offset for ver3.2 and earlier */
    EEPROM_GROUPS_OFFSET3_3 = 0x150, /* groups offset for ver3.3 */
    GROUP1_OFFSET           = 0x000, /* relative offset of group(i) to groups_offset */
    GROUP2_OFFSET           = 0x005,
    GROUP3_OFFSET           = 0x037,
    GROUP4_OFFSET           = 0x046,
    GROUP5_OFFSET           = 0x055,
    GROUP6_OFFSET           = 0x065,
    GROUP7_OFFSET           = 0x069,
    GROUP8_OFFSET           = 0x06f,
};

/* Capabilities Enum */
typedef enum {               
    EEPCAP_COMPRESS_DIS  = 0x0001,
    EEPCAP_AES_DIS       = 0x0002,
    EEPCAP_FASTFRAME_DIS = 0x0004,
    EEPCAP_BURST_DIS     = 0x0008,
    EEPCAP_MAXQCU_M      = 0x01F0,
    EEPCAP_MAXQCU_S      = 4,
    EEPCAP_HEAVY_CLIP_EN = 0x0200,
    EEPCAP_KC_ENTRIES_M  = 0xF000,
    EEPCAP_KC_ENTRIES_S  = 12,
} EEPROM_CAPABILITIES;

/* IMPORTANT: change EEP_SCALE and EEP_DELTA together */
#define EEP_SCALE   100         /* scale everything by this amount so get reasonable accuracy */
                                /* but don't need to do float arithmetic */
#define EEP_DELTA   10          /* SCALE/10, but don't want to do divide arithmetic each time */
#define PWR_MIN     0
#define PWR_MAX     3150        /* 31.5 * SCALE */
#define PWR_STEP    50          /* 0.5 * SCALE */
/* Keep 2 above defines together */

#define NUM_11A_EEPROM_CHANNELS        10
#define NUM_2_4_EEPROM_CHANNELS        3
#define NUM_2_4_EEPROM_CHANNELS_2413   4
#define NUM_TARGET_POWER_LOCATIONS_11B 4
#define NUM_TARGET_POWER_LOCATIONS_11G 6
#define NUM_PCDAC_VALUES               11
#define NUM_TEST_FREQUENCIES           8
#define NUM_EDGES                      8
#define NUM_INTERCEPTS                 11
#define FREQ_MASK                      0x7f
#define FREQ_MASK_3_3                  0xff
#define PCDAC_MASK                     0x3f
#define POWER_MASK                     0x3f
#define NON_EDGE_FLAG_MASK             0x40
#define CHANNEL_POWER_INFO             8
#define OBDB_UNSET                     0xffff
#define CHANNEL_UNUSED                 0xff
#define SCALE_OC_DELTA(x)              (((x) * 2) / 10)
#define TENX_OFDM_CCK_DELTA_INIT       15      /* power 1.5 dbm */
#define TENX_CH14_FILTER_CCK_DELTA_INIT 15  /* power 1.5 dbm */

/* Used during pdadc construction */
#define MAX_NUM_PDGAINS_PER_CHANNEL  4
#define NUM_PDGAINS_PER_CHANNEL      2
#define NUM_POINTS_LAST_PDGAIN       5
#define NUM_POINTS_OTHER_PDGAINS     4
#define XPD_GAIN1_GEN5               3
#define XPD_GAIN2_GEN5               1
#define MAX_PWR_RANGE_IN_HALF_DB     64
#define PD_GAIN_BOUNDARY_STRETCH_IN_HALF_DB  4

/* Used during pcdac table construction */
#define PCDAC_START                 1
#define PCDAC_STOP                  63
#define PCDAC_STEP                  1
#define PWR_TABLE_SIZE              64
#define MAX_RATE_POWER              60

/* Used during power/rate table construction */
#define NUM_RATES                   16
#define NUM_CTLS                    16
#define NUM_CTLS_3_3                32

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
    A_UINT16            Channels11g[NUM_2_4_EEPROM_CHANNELS];
    A_UINT16            Channels11b[NUM_2_4_EEPROM_CHANNELS];

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

/* == EEPROM version 4 added defines == */
#define NUM_XPD_PER_CHANNEL      4
#define NUM_POINTS_XPD0          4
#define NUM_POINTS_XPD3          3
#define IDEAL_10dB_INTERCEPT_2G  35
#define IDEAL_10dB_INTERCEPT_5G  55

/* 5112 pcdac calibration structures */
typedef struct ExpnDataPerXPD5112 {
    A_UINT16    xpd_gain;
    A_UINT16    numPcdacs;
    A_UINT16    pcdac[NUM_POINTS_XPD0];
    A_INT16     pwr_t4[NUM_POINTS_XPD0];  // or gainF
} EXPN_DATA_PER_XPD_5112;

typedef struct ExpnDataPerChannel5112 {
    A_UINT16                channelValue;
    A_INT16                 maxPower_t4;
    EXPN_DATA_PER_XPD_5112  pDataPerXPD[NUM_XPD_PER_CHANNEL];
} EXPN_DATA_PER_CHANNEL_5112;

typedef struct EepromPowerExpn5112 {
    A_UINT16                    *pChannels;
    A_UINT16                    numChannels;
    A_UINT16                    xpdMask;           // mask inicating which xpd_gains are permitted
    EXPN_DATA_PER_CHANNEL_5112  *pDataPerChannel;  //ptr to array of info held per channel
} EEPROM_POWER_EXPN_5112;

typedef struct EepromDataPerChannel5112 {
    A_UINT16        channelValue;
    A_UINT16        pcd1_xg0;
    A_INT16         pwr1_xg0;
    A_UINT16        pcd2_delta_xg0;
    A_INT16         pwr2_xg0;
    A_UINT16        pcd3_delta_xg0;
    A_INT16         pwr3_xg0;
    A_UINT16        pcd4_delta_xg0;
    A_INT16         pwr4_xg0;
    A_INT16         maxPower_t4;
    A_INT16         pwr1_xg3;  // pcdac = 20
    A_INT16         pwr2_xg3;  // pcdac = 35
    A_INT16         pwr3_xg3;  // pcdac = 63
} EEPROM_DATA_PER_CHANNEL_5112;

typedef struct EepromPower5112 {
    A_UINT16                        pChannels[NUM_11A_EEPROM_CHANNELS];
    A_UINT16                        numChannels;
    A_UINT16                        xpdMask;           // mask inicating which xpd_gains are permitted
    EEPROM_DATA_PER_CHANNEL_5112    pDataPerChannel[NUM_11A_EEPROM_CHANNELS];   //array of info held per channel
} EEPROM_POWER_5112;

/* 2413 pdadc calibration structures */
typedef struct RawDataPerPDGAIN {
    A_UINT16    pd_gain;
    A_UINT16    numVpd;
    A_UINT16    *Vpd;
    A_INT16     *pwr_t4;  // or gainF
} RAW_DATA_PER_PDGAIN_2413;

typedef struct RawDataPerChannel2413 {
    A_UINT16                    channelValue;
    A_INT16                     maxPower_t4;    
    A_UINT16                    numPdGains;       // # Pd Gains per channel
    RAW_DATA_PER_PDGAIN_2413    *pDataPerPDGain;
} RAW_DATA_PER_CHANNEL_2413;


typedef struct RawDataStruct2413 {
    A_UINT16                    *pChannels;
    A_UINT16                    numChannels;
    A_UINT16                    xpd_mask;         // mask inicating which xpd_gains are permitted   
    RAW_DATA_PER_CHANNEL_2413   *pDataPerChannel; //ptr to array of info held per channel
} RAW_DATA_STRUCT_2413;

typedef struct EepromDataPerChannel2413 {
    A_UINT16        channelValue;
    A_UINT16        numPdGains;  // number of pdGains to be stored on eeprom per channel
    A_UINT16        Vpd_I[MAX_NUM_PDGAINS_PER_CHANNEL];    
    A_INT16         pwr_I[MAX_NUM_PDGAINS_PER_CHANNEL];
    A_UINT16        Vpd_delta[NUM_POINTS_LAST_PDGAIN][MAX_NUM_PDGAINS_PER_CHANNEL];
    A_INT16         pwr_delta_t2[NUM_POINTS_LAST_PDGAIN][MAX_NUM_PDGAINS_PER_CHANNEL];
    A_INT16         maxPower_t4;
} EEPROM_DATA_PER_CHANNEL_2413;

typedef struct EepromDataStruct2413 {
    A_UINT16                        *pChannels;
    A_UINT16                        numChannels;
    A_UINT16                        xpd_mask;           // mask inicating which xpd_gains are permitted
    EEPROM_DATA_PER_CHANNEL_2413    *pDataPerChannel;   // ptr to array of info held per channel
} EEPROM_DATA_STRUCT_2413;

/* contiguous struct for passing from library to upper level software */
typedef struct EepromFullDataStruct2413 {
    A_UINT16                        numChannels11b;
    A_UINT16                        xpd_mask11b;        
    EEPROM_DATA_PER_CHANNEL_2413    pDataPerChannel11b[NUM_2_4_EEPROM_CHANNELS];
    A_UINT16                        numChannels11g;
    A_UINT16                        xpd_mask11g;    
    EEPROM_DATA_PER_CHANNEL_2413    pDataPerChannel11g[NUM_2_4_EEPROM_CHANNELS];
} EEPROM_FULL_DATA_STRUCT_2413;

typedef struct rdEdgesPower {
    A_UINT16    rdEdge;
    A_UINT16    twice_rdEdgePower;
    A_BOOL      flag;
} RD_EDGES_POWER;

typedef enum HeaderInfoMode {
    headerInfo11A,
    headerInfo11B,
    headerInfo11G
} HEADER_WMODE;

/*
 * Header Info - general parameters and
 * values set for each chipset board solution
 * that are programmed every reset
 */
typedef struct eepHeaderInfo
{
    /* General Device Parameters */
    A_UINT16 turbo5Disable;
    A_UINT16 turbo2Disable;
    A_UINT16 rfKill;
    A_UINT16 deviceType;
    A_UINT16 turbo2WMaxPower5;
    A_UINT16 turbo2WMaxPower2;
    A_UINT16 xrTargetPower5;
    A_UINT16 xrTargetPower2;
    A_UINT16 Amode;
    A_UINT16 Bmode;
    A_UINT16 Gmode;
    A_BOOL   disableXr5;
    A_BOOL   disableXr2;
    A_INT8   antennaGainMax[2];
    A_INT8   cckOfdmGainDelta;
    A_UINT16 cckOfdmPwrDelta;
    A_INT16  scaledCh14FilterCckDelta;
    A_UINT16 eepMap;
    A_UINT16 earStart;
    A_UINT16 targetPowersStart;
    A_UINT16 eepMap2PowerCalStart;
    A_UINT16 fixedBias5;
    A_UINT16 fixedBias2;
    A_BOOL   exist32kHzCrystal;
    A_UINT16 capField;

    /* 5 GHz / 2.4 GHz CKK / 2.4 GHz OFDM common parameters */
    A_UINT16 switchSettling[3];
    A_UINT16 switchSettlingTurbo[2];
    A_UINT16 txrxAtten[3];
    A_UINT16 txrxAttenTurbo[2];
    A_UINT16 txEndToXLNAOn[3];
    A_UINT16 thresh62[3];
    A_UINT16 txEndToXPAOff[3];
    A_UINT16 txFrameToXPAOn[3];
    A_INT8   adcDesiredSize[3];     /* 8-bit signed value */
    A_INT8   adcDesiredSizeTurbo[2];
    A_INT8   pgaDesiredSize[3];     /* 8-bit signed value */
    A_INT8   pgaDesiredSizeTurbo[2];
    A_INT16  noiseFloorThresh[3];
    A_UINT16 xlnaGain[3];
    A_UINT16 xgain[3];
    A_UINT16 xpd[3];
    A_UINT16 antennaControl[11][3];
    A_UINT16 falseDetectBackoff[3];
    A_UINT16 gainI[3];
    A_UINT16 rxtxMargin[3];
    A_UINT16 rxtxMarginTurbo[2];

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
    A_INT16  iqCalI[2];
    A_INT16  iqCalQ[2];
    A_UINT16 calPier11g[NUM_2_4_EEPROM_CHANNELS];
    A_UINT16 calPier11b[NUM_2_4_EEPROM_CHANNELS];

    /* CTL control */
    A_UINT16 numCtls;
    A_UINT16 ctl[32];
} EEP_HEADER_INFO;

typedef struct eepMap {
    A_UINT16                version;        /* Version field */
    A_UINT16                protect;        /* EEPROM protect field */
    EEP_HEADER_INFO         *pEepHeader;
    PCDACS_ALL_MODES        *pPcdacInfo;
    TRGT_POWER_ALL_MODES    *pTrgtPowerInfo;
    RD_EDGES_POWER          *pRdEdgesPower;
    EEPROM_POWER_EXPN_5112  modePowerArray5112[3];
    RAW_DATA_STRUCT_2413    *pRawDataset2413[3];
} EEP_MAP;

/* Data structures initialization for gainF functions */

#define NUM_CORNER_FIX_BITS      4
#define NUM_CORNER_FIX_BITS_5112 7
#define DYN_ADJ_UP_MARGIN        15
#define DYN_ADJ_LO_MARGIN        20
#define MAX_GAIN_STEPS           10
#define PHY_PROBE_CCK_CORRECTION 5
#define CCK_OFDM_GAIN_DELTA      15

enum GAIN_PARAMS {
    GP_TXCLIP,
    GP_PD90,
    GP_PD84,
    GP_GSEL
};

enum GAIN_PARAMS_5112 {
    GP_MIXGAIN_OVR,
    GP_PWD_138,
    GP_PWD_137,
    GP_PWD_136,
    GP_PWD_132,
    GP_PWD_131,
    GP_PWD_130,
};

typedef struct GainOptStep {
    A_INT16 paramVal[NUM_CORNER_FIX_BITS_5112];
    A_INT32 stepGain;
    A_CHAR  stepName[16];
} GAIN_OPTIMIZATION_STEP;

typedef struct gainOptLadder {
    A_UINT32                numStepsInLadder;
    A_UINT32                defaultStepNum;
    GAIN_OPTIMIZATION_STEP  optStep[MAX_GAIN_STEPS];
} GAIN_OPTIMIZATION_LADDER;

typedef struct gainValues {
    A_UINT32                     currStepNum;
    A_UINT32                     currGain;
    A_UINT32                     targetGain;
    A_UINT32                     loTrig;
    A_UINT32                     hiTrig;
    A_UINT32                     gainFCorrection;
    A_BOOL                       active;
    const GAIN_OPTIMIZATION_STEP *currStep;
} GAIN_VALUES;

/* RF HAL structures */
typedef struct RfHalFuncs {
    A_STATUS  (*ar5212SetChannel)(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval);
    void      (*ar5212SetRfRegs)(WLAN_DEV_INFO *pDev, EEP_HEADER_INFO *pHeaderInfo,
                  CHAN_VALUES *pChval, A_UINT16 modesIndex, A_UINT16 *rfXpdGain);
    void      (*ar5212SetPowerTable)(WLAN_DEV_INFO *pDev, A_UINT16 *pPcdacTable, A_INT16 *pPowerMin,
                  A_INT16 *pPowerMax, CHAN_VALUES *pChval, A_UINT16 *rfXpdGain);
} RF_HAL_FUNCS;

/* EAR definitions and structures */
#define EAR_VERSION_SUPPORT        0xEA02

extern int EarDebugLevel;
#define MAX_NUM_REGISTER_HEADERS   256
#define MAX_EAR_LOCATIONS          1024
#define A_DIV_UP(x, y) (((x) + (y) - 1) / (y))
#define EAR_MAX_RH_REGS            256

typedef struct disablerModifier {
    A_BOOL   valid;
    A_UINT16 disableField;
    A_UINT16 pllValue;
} DISABLER_MODIFIER;

typedef struct registerHeader {
    A_UINT16 versionMask;
    A_UINT16 channel;
    A_UINT16 stage;
    A_UINT16 type;
    A_UINT16 modes;
    DISABLER_MODIFIER disabler;
    A_UINT16 *regs;
} REGISTER_HEADER;

typedef struct earHeader {
    A_UINT16 versionId;
    A_UINT16 numRHs;
    int      earSize;
    REGISTER_HEADER *pRH;
} EAR_HEADER;

typedef struct headerAlloc {
    A_UINT16 numRHs;
    A_UINT16 locsPerRH[MAX_NUM_REGISTER_HEADERS];
} EAR_ALLOC;

typedef enum {
    EAR_TYPE0,
    EAR_TYPE1,
    EAR_TYPE2,
    EAR_TYPE3
} earTypesEnum;

enum earWirelessMode {
    EAR_11B   = 0x001,
    EAR_11G   = 0x002,
    EAR_2T    = 0x004,
    EAR_11A   = 0x010,
    EAR_XR    = 0x020,
    EAR_5T    = 0x040,
};

enum earStages {
    EAR_STAGE_ANALOG_WRITE,
    EAR_STAGE_PRE_PHY_ENABLE,
    EAR_STAGE_POST_RESET,
    EAR_STAGE_POST_PER_CAL
};

enum earMasksAndShifts {
    VERSION_MASK_M   = 0x7FFF,
    RH_MODES_M       = 0x01FF,
    RH_TYPE_S        = 9,
    RH_TYPE_M        = 0x0600,
    RH_STAGE_S       = 11,
    RH_STAGE_M       = 0x1800,
    CM_SINGLE_CHAN_M = 0x7FFF,
    T0_TAG_M         = 0x0003,
    T1_NUM_M         = 0x0003,
    T2_BANK_S        = 13,
    T2_BANK_M        = 0xE000,
    T2_COL_S         = 10,
    T2_COL_M         = 0x0C00,
    T2_START_M       = 0x01FF,
    T2_NUMB_S        = 12,
    T2_NUMB_M        = 0xF000,
    T2_DATA_M        = 0x0FFF,
    T3_NUMB_M        = 0x001F,
    T3_START_S       = 5,
    T3_START_M       = 0x03E0,
    T3_OPCODE_S      = 10,
    T3_OPCODE_M      = 0x1C00,
};

enum earMiscDefines {
    RH_RESERVED_MODES    = 0x0188,
    RH_ALL_MODES         = 0x01FF,
    RH_RESERVED_PLL_BITS = 0xFF00,
    T0_TAG_32BIT         = 0,
    T0_TAG_16BIT_LOW     = 1,
    T0_TAG_16BIT_HIGH    = 2,
    T0_TAG_32BIT_LAST    = 3,
    T3_MAX_OPCODE        = 6,
    T3_RESERVED_BITS     = 0x6000,
    MAX_ANALOG_START     = 320,
};

enum earType3OpCodes {
    T3_OC_REPLACE,
    T3_OC_ADD,
    T3_OC_SUB,
    T3_OC_MUL,
    T3_OC_XOR,
    T3_OC_OR,
    T3_OC_AND
};

typedef enum {
    EAR_DEBUG_OFF,
    EAR_DEBUG_BASIC,
    EAR_DEBUG_VERBOSE,
    EAR_DEBUG_EXTREME
} EAR_DEBUG_LEVELS;

typedef enum {
    TXPOWER_DEBUG_OFF,
    TXPOWER_DEBUG_BASIC,
    TXPOWER_DEBUG_VERBOSE,
    TXPOWER_DEBUG_EXTREME
} TXPOWER_DEBUG_LEVELS;

typedef enum {
    EAR_LC_RF_WRITE,           /* Modifies or disables analog writes */
    EAR_LC_PHY_ENABLE,         /* Modifies any register(s) before PHY enable */
    EAR_LC_POST_RESET,         /* Modifies any register(s) at the end of Reset */
    EAR_LC_POST_PER_CAL,       /* Modifies any register(s) at the end of periodic cal */
    EAR_LC_PLL,                /* Changes the PLL value in chipReset */
    EAR_LC_RESET_OFFSET,       /* Enable/disable offset in reset */
    EAR_LC_RESET_NF,           /* Enable/disable NF in reset */
    EAR_LC_RESET_IQ,           /* Enable/disable IQ in reset */
    EAR_LC_PER_FIXED_GAIN,     /* Enable/disable fixed gain */
    EAR_LC_PER_GAIN_CIRC,      /* Enable/disable gain circulation */
    EAR_LC_PER_NF              /* Enable/disable periodic cal NF */
} EAR_LOC_CHECK;

/* EAR Macros */
#define IS_SPUR_CHAN(x)     ( (((x) % 32) != 0) && ((((x) % 32) < 10) || (((x) % 32) > 22)) )

#define IS_VER_MASK(x)      (((x) >> 15) & 1)
#define IS_END_HEADER(x)    ((x) == 0)

#define IS_CM_SET(x)        (((x) >> 13) & 1)
#define IS_CM_SPUR(x)       (((x) >> 14) & 1)
#define IS_CM_SINGLE(x)     (((x) >> 15) & 1)
#define IS_DISABLER_SET(x)  (((x) >> 14) & 1)
#define IS_PLL_SET(x)       (((x) >> 15) & 1)
#define IS_TYPE2_EXTENDED(x) (((x) >> 9) & 1)
#define IS_TYPE2_LAST(x)    (((x) >> 12) & 1)
#define IS_TYPE3_LAST(x)    (((x) >> 15) & 1)
#define IS_5GHZ_CHAN(x)     ((((x) & CM_SINGLE_CHAN_M) > 4980) && (((x) & CM_SINGLE_CHAN_M) < 6100))
#define IS_2GHZ_CHAN(x)     ((((x) & CM_SINGLE_CHAN_M) > 2300) && (((x) & CM_SINGLE_CHAN_M) < 2700))
#define IS_VALID_ADDR(x)    ((((x) & 0x3) == 0) && \
                             (((x) <= 0x100) || ((x) >= 0x400 && (x) <= 0xc00) || \
                             ((x) >= 0x1000 && (x) <= 0x1500) || ((x) >= 0x4000 && (x) <= 0x4080) || \
                             ((x) >= 0x6000 && (x) <= 0x6020) || ((x) >= 0x8000 && (x) <= 0x9000) || \
                             ((x) >= 0x9800 && (x) <= 0x9D00) || ((x) >= 0xA000 && (x) <= 0xA300)))
#define IS_EAR_VMATCH(x, y) (A_BOOL)(((x) > EAR_VERSION_SUPPORT) && \
                             ((1 << ((x) - EAR_VERSION_SUPPORT)) & ((y) & VERSION_MASK_M)))

typedef enum {                  /* values associated with Registry key "enable32KHzClock" */
    AUTO_32KHZ,                 /* eeprom's value of 'exist32kHzCrystal' enables 32KHz clock */
    USE_32KHZ,                  /* use 32KHz (even if eeprom indicates it's not present) */
    DONT_USE_32KHZ              /* don't use 32KHz (even if eeprom indicates it is present) */
} ENABLE_32KHZ;

#ifdef _cplusplus
}
#endif

#endif /* _AR5212_H_ */
