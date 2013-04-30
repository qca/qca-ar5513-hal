/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 */

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Phy.c#1 $"

#include "wlandrv.h"
#include "wlanPhy.h"
#include "ar5212Phy.h"

#ifdef BUILD_AR5212
#define TURBO_PRIME 1

#ifdef TURBO_PRIME
static RATE_TABLE ar5212_TurboPrimeARateTable = {
    16,  /* number of rates */
    { -1 },
    {/*                                                              short            ctrl  RssiAck  RssiAck long Preamble short Preamble  */
     /*              valid                  Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration */
     /*   6 Mb */ {  TRUE, WLAN_PHY_OFDM,   6000,  5400,   0x0b,    0x00, (0x80|12),   0,       2,       1,     60,       60},
     /*   9 Mb */ {  TRUE, WLAN_PHY_OFDM,   9000,  7800,   0x0f,    0x00,        18,   0,       3,       1,     60,       60},
     /*  12 Mb */ {  TRUE, WLAN_PHY_OFDM,  12000, 10000,   0x0a,    0x00, (0x80|24),   2,       4,       2,     48,       48},
     /*  18 Mb */ {  TRUE, WLAN_PHY_OFDM,  18000, 13900,   0x0e,    0x00,        36,   2,       6,       2,     48,       48},
     /*  24 Mb */ {  TRUE, WLAN_PHY_OFDM,  24000, 17300,   0x09,    0x00, (0x80|48),   4,      10,       3,     44,       44},
     /*  36 Mb */ {  TRUE, WLAN_PHY_OFDM,  36000, 23000,   0x0d,    0x00,        72,   4,      14,       3,     44,       44},
     /*  48 Mb */ {  TRUE, WLAN_PHY_OFDM,  48000, 27400,   0x08,    0x00,        96,   4,      19,       3,     44,       44},
     /*  54 Mb */ {  TRUE, WLAN_PHY_OFDM,  54000, 29300,   0x0c,    0x00,       108,   4,      23,       3,     44,       44},
     /*   6 Mb */ {  TRUE, WLAN_PHY_TURBO,  6000, 10600,   0x0b,    0x00, (0x80|12),   8,       2,       1,     34,       34},
     /*   9 Mb */ {  TRUE, WLAN_PHY_TURBO,  9000, 15600,   0x0f,    0x00,        18,   8,       3,       1,     34,       34},
     /*  12 Mb */ {  TRUE, WLAN_PHY_TURBO, 12000, 19400,   0x0a,    0x00, (0x80|24),  10,       4,       2,     30,       30},
     /*  18 Mb */ {  TRUE, WLAN_PHY_TURBO, 18000, 26900,   0x0e,    0x00,        36,  10,       6,       2,     30,       30},
     /*  24 Mb */ {  TRUE, WLAN_PHY_TURBO, 24000, 33200,   0x09,    0x00, (0x80|48),  12,      10,       3,     26,       26},
     /*  36 Mb */ {  TRUE, WLAN_PHY_TURBO, 36000, 43600,   0x0d,    0x00,        72,  12,      14,       3,     26,       26},
     /*  48 Mb */ {  TRUE, WLAN_PHY_TURBO, 48000, 51300,   0x08,    0x00,        96,  12,      19,       3,     26,       26},
     /*  54 Mb */ {  TRUE, WLAN_PHY_TURBO, 54000, 55100,   0x0c,    0x00,       108,  12,      23,       3,     26,       26},
    },
    100, /* probe interval */
    50,  /* rssi reduce interval */
    5,   /* 36 Mbps for 11a */
    11,  /* 18 Mbps (36 Mbps) for 108a */
    50,  /* packet count threshold */
    7,   /* initial rateMax (index) */
    8    /* # of turboRates */
};
#else
static RATE_TABLE ar5212_11aRateTable = {
    8,  /* number of rates */
    { -1 },
    {/*                                                             short            ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid                 Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /*   6 Mb */ {  TRUE, WLAN_PHY_OFDM,  6000,  5400,   0x0b,    0x00, (0x80|12),   0,       2,       1,      60,        60},
     /*   9 Mb */ {  TRUE, WLAN_PHY_OFDM,  9000,  7800,   0x0f,    0x00,        18,   0,       3,       1,      60,        60},
     /*  12 Mb */ {  TRUE, WLAN_PHY_OFDM, 12000, 10000,   0x0a,    0x00, (0x80|24),   2,       4,       2,      48,        48},
     /*  18 Mb */ {  TRUE, WLAN_PHY_OFDM, 18000, 13900,   0x0e,    0x00,        36,   2,       6,       2,      48,        48},
     /*  24 Mb */ {  TRUE, WLAN_PHY_OFDM, 24000, 17300,   0x09,    0x00, (0x80|48),   4,      10,       3,      44,        44},
     /*  36 Mb */ {  TRUE, WLAN_PHY_OFDM, 36000, 23000,   0x0d,    0x00,        72,   4,      14,       3,      44,        44},
     /*  48 Mb */ {  TRUE, WLAN_PHY_OFDM, 48000, 27400,   0x08,    0x00,        96,   4,      19,       3,      44,        44},
     /*  54 Mb */ {  TRUE, WLAN_PHY_OFDM, 54000, 29300,   0x0c,    0x00,       108,   4,      23,       3,      44,        44},
    },
    100, /* probe interval */
    50,  /* rssi reduce interval */
    0,
    0, 
    0,
    7,   /* initial rateMax (index) */
    0,   /* # of turboRates */
    0,
    0,
    0
};
#endif

static RATE_TABLE ar5212_TurboRateTable = {
    8,  /* number of rates */
    { -1 },
    {/*                                                              short            ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid                  Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /*   6 Mb */ {  TRUE, WLAN_PHY_TURBO,  6000,  5400,   0x0b,    0x00, (0x80|12),   0,       2,       1,     34,         34},
     /*   9 Mb */ {  TRUE, WLAN_PHY_TURBO,  9000,  7800,   0x0f,    0x00,        18,   0,       4,       1,     34,         34},
     /*  12 Mb */ {  TRUE, WLAN_PHY_TURBO, 12000, 10000,   0x0a,    0x00, (0x80|24),   2,       7,       2,     30,         30},
     /*  18 Mb */ {  TRUE, WLAN_PHY_TURBO, 18000, 13900,   0x0e,    0x00,        36,   2,       9,       2,     30,         30},
     /*  24 Mb */ {  TRUE, WLAN_PHY_TURBO, 24000, 17300,   0x09,    0x00, (0x80|48),   4,      14,       3,     26,         26},
     /*  36 Mb */ {  TRUE, WLAN_PHY_TURBO, 36000, 23000,   0x0d,    0x00,        72,   4,      17,       3,     26,         26},
     /*  48 Mb */ {  TRUE, WLAN_PHY_TURBO, 48000, 27400,   0x08,    0x00,        96,   4,      22,       3,     26,         26},
     /*  54 Mb */ {  TRUE, WLAN_PHY_TURBO, 54000, 29300,   0x0c,    0x00,       108,   4,      26,       3,     26,         26},
    },
    100, /* probe interval */
    50,  /* rssi reduce interval */
    0,
    0, 
    0,
    7,   /* initial rateMax (index) */
    8,   /* # of turboRates */
    0,
    0,
    0
};

#ifdef TURBO_PRIME
static RATE_TABLE ar5212_TurboPrimeGRateTable = {
    19,  /* number of rates */
    { -1 },
    {/*                                                                short           ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid  phy              Kbps  uKbps   rateCode Preamble dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /*   1 Mb */ {  TRUE,  WLAN_PHY_CCK,    1000,   900,  0x1b,    0x00,         2,   0,      0,      1,       314,         314},
     /*   2 Mb */ {  TRUE,  WLAN_PHY_CCK,    2000,  1900,  0x1a,    0x04,         4,   1,      1,      1,       258,         162},
     /* 5.5 Mb */ {  TRUE,  WLAN_PHY_CCK,    5500,  4900,  0x19,    0x04,        11,   2,      2,      2,       223,         127},
     /*  11 Mb */ {  TRUE,  WLAN_PHY_CCK,   11000,  8100,  0x18,    0x04,        22,   3,      3,      2,       213,         117},
     /*   6 Mb */ {  FALSE, WLAN_PHY_OFDM,   6000,  5400,  0x0b,    0x00,        12,   4,      2,      1,        60,          60},
     /*   9 Mb */ {  FALSE, WLAN_PHY_OFDM,   9000,  7800,  0x0f,    0x00,        18,   4,      3,      1,        60,          60},
     /*  12 Mb */ {  TRUE,  WLAN_PHY_OFDM,  12000, 10100,  0x0a,    0x00,        24,   6,      4,      1,        48,          48},
     /*  18 Mb */ {  TRUE,  WLAN_PHY_OFDM,  18000, 14100,  0x0e,    0x00,        36,   6,      6,      2,        48,          48},
     /*  24 Mb */ {  TRUE,  WLAN_PHY_OFDM,  24000, 17700,  0x09,    0x00,        48,   8,     10,      3,        44,          44},
     /*  36 Mb */ {  TRUE,  WLAN_PHY_OFDM,  36000, 23700,  0x0d,    0x00,        72,   8,     14,      3,        44,          44},
     /*  48 Mb */ {  TRUE,  WLAN_PHY_OFDM,  48000, 27400,  0x08,    0x00,        96,   8,     19,      3,        44,          44},
     /*  54 Mb */ {  TRUE,  WLAN_PHY_OFDM,  54000, 29300,  0x0c,    0x00,       108,   8,     23,      3,        44,          44},
     /*   6 Mb */ {  TRUE,  WLAN_PHY_TURBO,  6000, 10600,  0x0b,    0x00, (0x80|12),   12,     2,      1,        34,          34},
     /*  12 Mb */ {  TRUE,  WLAN_PHY_TURBO, 12000, 19400,  0x0a,    0x00,        24,   13,     4,      1,        30,          30},
     /*  18 Mb */ {  TRUE,  WLAN_PHY_TURBO, 18000, 26900,  0x0e,    0x00,        36,   13,     6,      2,        30,          30},
     /*  24 Mb */ {  TRUE,  WLAN_PHY_TURBO, 24000, 33200,  0x09,    0x00, (0x80|48),   15,    10,      3,        26,          26},
     /*  36 Mb */ {  TRUE,  WLAN_PHY_TURBO, 36000, 43600,  0x0d,    0x00,        72,   15,    14,      3,        26,          26},
     /*  48 Mb */ {  TRUE,  WLAN_PHY_TURBO, 48000, 51300,  0x08,    0x00,        96,   15,    19,      3,        26,          26},
     /*  54 Mb */ {  TRUE,  WLAN_PHY_TURBO, 54000, 55100,  0x0c,    0x00,       108,   15,    23,      3,        26,          26},
    },
    100, /* probe interval */
    50,  /* rssi reduce interval */
    9,   /* 36 Mbps for 11g */
    14,  /* 18 Mbps (36 Mbps) for 108g */
    50,  /* packet count threshold */
    11,  /* initial rateMax (index) */
    7,   /* # of turboRates */
    0,
    0,
    0
};
#else
/* Venice TODO: roundUpRate() is broken when the rate table does not represent rates
 * in increasing order  e.g.  5.5, 11, 6, 9.    
 * An average rate of 6 Mbps will currently map to 11 Mbps. 
 */
static RATE_TABLE ar5212_11gRateTable = {
    12,  /* number of rates */
    { -1 },
    {/*                                                              short            ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid                  Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /*   1 Mb */ {  TRUE,  WLAN_PHY_CCK,   1000,   900,  0x1b,    0x00,         2,   0,       0,       1,       314,       314},
     /*   2 Mb */ {  TRUE,  WLAN_PHY_CCK,   2000,  1900,  0x1a,    0x04,         4,   1,       1,       1,       258,       162},
     /* 5.5 Mb */ {  TRUE,  WLAN_PHY_CCK,   5500,  4900,  0x19,    0x04,        11,   2,       2,       2,       223,       127},
     /*  11 Mb */ {  TRUE,  WLAN_PHY_CCK,  11000,  8100,  0x18,    0x04,        22,   3,       3,       2,       213,       117},
     /*   6 Mb */ {  FALSE, WLAN_PHY_OFDM, 6000,   5400,  0x0b,    0x00,        12,   4,       2,       1,        60,        60},
     /*   9 Mb */ {  FALSE, WLAN_PHY_OFDM, 9000,   7800,  0x0f,    0x00,        18,   4,       3,       1,        60,        60},
     /*  12 Mb */ {  TRUE,  WLAN_PHY_OFDM, 12000, 10000,  0x0a,    0x00,        24,   6,       4,       1,        48,        48},
     /*  18 Mb */ {  TRUE,  WLAN_PHY_OFDM, 18000, 13900,  0x0e,    0x00,        36,   6,       6,       2,        48,        48},
     /*  24 Mb */ {  TRUE,  WLAN_PHY_OFDM, 24000, 17300,  0x09,    0x00,        48,   8,      10,       3,        44,        44},
     /*  36 Mb */ {  TRUE,  WLAN_PHY_OFDM, 36000, 23000,  0x0d,    0x00,        72,   8,      14,       3,        44,        44},
     /*  48 Mb */ {  TRUE,  WLAN_PHY_OFDM, 48000, 27400,  0x08,    0x00,        96,   8,      19,       3,        44,        44},
     /*  54 Mb */ {  TRUE,  WLAN_PHY_OFDM, 54000, 29300,  0x0c,    0x00,       108,   8,      23,       3,        44,        44},
    },
    100, /* probe interval */
    50,  /* rssi reduce interval */
    0,
    0, 
    0,
    11,  /* initial rateMax (index) */
    0,   /* # of turboRates */
    0,
    0,
    0
};
#endif

static RATE_TABLE ar5212_11bRateTable = {
    4,  /* number of rates */
    { -1 },
    {/*                                                             short            ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid                 Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /*   1 Mb */ {  TRUE,  WLAN_PHY_CCK,  1000,  900,  0x1b,    0x00, (0x80| 2),   0,       0,       1,        314,        314},
     /*   2 Mb */ {  TRUE,  WLAN_PHY_CCK,  2000, 1800,  0x1a,    0x04, (0x80| 4),   1,       1,       1,        258,        162},
     /* 5.5 Mb */ {  TRUE,  WLAN_PHY_CCK,  5500, 4300,  0x19,    0x04, (0x80|11),   1,       2,       2,        258,        162},
     /*  11 Mb */ {  TRUE,  WLAN_PHY_CCK, 11000, 7100,  0x18,    0x04, (0x80|22),   1,       4,     100,        258,        162},
    },
    200, /* probe interval */
    100, /* rssi reduce interval */
    0,
    0, 
    0,
    3,   /* initial rateMax (index) */
    0,   /* # of turboRates */
    0,
    0,
    0
};

static RATE_TABLE ar5212_XrRateTable = {
    13,  /* number of rates */
    { -1 },
    {/*                                                              short            ctrl  RssiAck  RssiAck long Preamble short Preamble*/
     /*              valid                  Kbps  uKbps  rateCode Preamble  dot11Rate Rate ValidMin DeltaMin  ACK Duration   ACK Duration*/
     /* 1/4 Mb */ {  TRUE, WLAN_PHY_XR,     250,   200,   0x03,    0x00,         1,   0,      -9,       1,     612,       612},
     /* 0.5 Mb */ {  TRUE, WLAN_PHY_XR,     500,   400,   0x07,    0x00,         1,   1,      -8,       4,     457,       457},
     /*   1 Mb */ {  TRUE, WLAN_PHY_XR,    1000,   900,   0x02,    0x00,         2,   2,      -4,       2,     228,       228},
     /*   2 Mb */ {  TRUE, WLAN_PHY_XR,    2000,  1800,   0x06,    0x00,         4,   3,      -2,       1,     160,       160},
     /*   3 Mb */ {  TRUE, WLAN_PHY_XR,    3000,  2700,   0x01,    0x00,         6,   4,      -1,       1,     140,       140},
     /*   6 Mb */ {  TRUE, WLAN_PHY_OFDM,  6000,  5400,   0x0b,    0x00, (0x80|12),   5,       2,       1,      60,        60},
     /*   9 Mb */ {  TRUE, WLAN_PHY_OFDM,  9000,  7800,   0x0f,    0x00,        18,   5,       3,       1,      60,        60},
     /*  12 Mb */ {  TRUE, WLAN_PHY_OFDM, 12000, 10000,   0x0a,    0x00, (0x80|24),   7,       4,       2,      48,        48},
     /*  18 Mb */ {  TRUE, WLAN_PHY_OFDM, 18000, 13900,   0x0e,    0x00,        36,   7,       6,       2,      48,        48},
     /*  24 Mb */ {  TRUE, WLAN_PHY_OFDM, 24000, 17300,   0x09,    0x00, (0x80|48),   9,      10,       3,      44,        44},
     /*  36 Mb */ {  TRUE, WLAN_PHY_OFDM, 36000, 23000,   0x0d,    0x00,        72,   9,      14,       3,      44,        44},
     /*  48 Mb */ {  TRUE, WLAN_PHY_OFDM, 48000, 27400,   0x08,    0x00,        96,   9,      19,       3,      44,        44},
     /*  54 Mb */ {  TRUE, WLAN_PHY_OFDM, 54000, 29300,   0x0c,    0x00,       108,   9,      23,       3,      44,        44},
    },
    200, /* probe interval */
    100, /* rssi reduce interval */
    0,
    0, 
    0,
    12,  /* initial rateMax (index) */
    0,   /* # of turboRates */
    6,
    7,
    50
};

void
ar5212AttachRateTables(WLAN_DEV_INFO *pDev)
{
    /*
     * Attach device specific rate tables; for ar5212.
     * 11a static turbo and 11g static turbo share the same table.
     * Dynamic turbo uses combined rate table.
     */
    pDev->hwRateTable[WIRELESS_MODE_TURBO] = &ar5212_TurboRateTable;
    pDev->hwRateTable[WIRELESS_MODE_11b]   = &ar5212_11bRateTable;
#ifdef TURBO_PRIME
    pDev->hwRateTable[WIRELESS_MODE_11a]   = &ar5212_TurboPrimeARateTable;
    pDev->hwRateTable[WIRELESS_MODE_11g]   = &ar5212_TurboPrimeGRateTable;
#else
    pDev->hwRateTable[WIRELESS_MODE_11a]   = &ar5212_11aRateTable;
    pDev->hwRateTable[WIRELESS_MODE_11g]   = &ar5212_11gRateTable;
#endif
    pDev->hwRateTable[WIRELESS_MODE_108g]  = &ar5212_TurboRateTable;
    pDev->hwRateTable[WIRELESS_MODE_XR]    = &ar5212_XrRateTable;
#if defined(XR_HACKERY)
#ifdef TURBO_PRIME
    pDev->hwRateTable[WIRELESS_MODE_XR]    = &ar5212_TurboPrimeARateTable;
#else
    pDev->hwRateTable[WIRELESS_MODE_XR]    = &ar5212_11aRateTable;
#endif
#endif  
}

#endif
