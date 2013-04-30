/*
 * Copyright © 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines header objects used EXTERNALLY by the WLAN and WME layers
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/halTxQID.h#2 $
 */

#ifndef _HALTXQID_H_
#define _HALTXQID_H_

/* NOTE: Changes to AC<->TXQ mapping must propagate to AC_TO_QNUM and QNUM_TO_AC macros */

#if defined(BUILD_AR5513)

#ifdef AR5513_QOS

#define TXQ_ID_FOR_DATA         0       /* default for AP or STA */
#define TXQ_ID_FOR_PSPOLL       0       /* STA only */
#define TXQ_ID_FOR_XR_DATA      0       /* AP only */
#define TXQ_ID_FOR_XR_GRPPOLL   0       /* AP only */
#define TXQ_ID_FOR_XR_BEACON    0       /* AP only */
#define TXQ_ID_FOR_AC0          0       /* WME AC0 (AC_BK), AP or STA */
#define TXQ_ID_FOR_AC1          0       /* WME AC1 (AC_BE), AP or STA */
#define TXQ_ID_FOR_AC2          0       /* WME AC2 (AC_VI), AP or STA */
#define TXQ_ID_FOR_AC3          0       /* WME AC3 (AC_VO), AP or STA */
#define TXQ_ID_FOR_VIDEO        1       /* AP or STA */
#define TXQ_ID_FOR_CONTROL      2       /* AP or STA */
#define TXQ_ID_FOR_UPSD         0       /* AP only */
#define TXQ_ID_FOR_GBURST       5
#define TXQ_ID_FOR_GPRS         0       /* AP only */
#define TXQ_ID_FOR_HCF          0       /* AP only */
#define TXQ_ID_FOR_CAB          3
#define TXQ_ID_FOR_BEACON       4

#else

#define TXQ_ID_FOR_DATA         0       /* default for AP or STA */
#define TXQ_ID_FOR_PSPOLL       1       /* STA only */
#define TXQ_ID_FOR_XR_DATA      1       /* AP only */
#define TXQ_ID_FOR_XR_GRPPOLL   2       /* AP only */
#define TXQ_ID_FOR_XR_BEACON    2       /* AP only */
#define TXQ_ID_FOR_AC0          0       /* WME AC0 (AC_BK), AP or STA */
#define TXQ_ID_FOR_AC1          1       /* WME AC1 (AC_BE), AP or STA */
#define TXQ_ID_FOR_AC2          2       /* WME AC2 (AC_VI), AP or STA */
#define TXQ_ID_FOR_AC3          2       /* WME AC3 (AC_VO), AP or STA */
#define TXQ_ID_FOR_VIDEO        2       /* AP or STA */
#define TXQ_ID_FOR_UPSD         2       /* AP only */
#define TXQ_ID_FOR_GBURST       2
#define TXQ_ID_FOR_GPRS         2       /* AP only */
#define TXQ_ID_FOR_HCF          2       /* AP only */
#define TXQ_ID_FOR_CAB          3
#define TXQ_ID_FOR_BEACON       4

#endif /* AR5513_QOS */

#define QNUM_TO_AC(_q) ( \
    ((_q) == 0) ? ACI_BK : \
    ((_q) == 2) ? ACI_Vi : \
    ((_q) == 2) ? ACI_Vo : \
    ((_q) == 1) ? ACI_BE :  \
    ACI_BE)

#else

#define TXQ_ID_FOR_DATA         0       /* default for AP or STA */
#define TXQ_ID_FOR_PSPOLL       1       /* STA only */
#define TXQ_ID_FOR_XR_DATA      1       /* AP only */
#define TXQ_ID_FOR_XR_GRPPOLL   2       /* AP only */
#define TXQ_ID_FOR_XR_BEACON    3       /* AP only */
#define TXQ_ID_FOR_AC0          0       /* WME AC0 (AC_BK), AP or STA */
#define TXQ_ID_FOR_AC1          3       /* WME AC1 (AC_BE), AP or STA */
#define TXQ_ID_FOR_AC2          4       /* WME AC2 (AC_VI), AP or STA */
#define TXQ_ID_FOR_AC3          5       /* WME AC3 (AC_VO), AP or STA */
#define TXQ_ID_FOR_VIDEO        4       /* AP or STA */
#define TXQ_ID_FOR_UPSD         6       /* AP only */
#define TXQ_ID_FOR_GBURST       7
#define TXQ_ID_FOR_GPRS         7       /* AP only */
#define TXQ_ID_FOR_HCF          7       /* AP only */
#define TXQ_ID_FOR_CAB          8
#define TXQ_ID_FOR_BEACON       9

#define QNUM_TO_AC(_q) ( \
    ((_q) == 0) ? ACI_BK : \
    ((_q) == 4) ? ACI_Vi : \
    ((_q) == 5) ? ACI_Vo : \
    ((_q) == 3) ? ACI_BE :  \
    ACI_BE)

#endif /* ! BUILD_AR5513 */

#define AC_TO_QNUM(_ac) ( \
    ((_ac) == ACI_BK) ? TXQ_ID_FOR_AC0 : \
    ((_ac) == ACI_Vi) ? TXQ_ID_FOR_AC2 : \
    ((_ac) == ACI_Vo) ? TXQ_ID_FOR_AC3 : \
    ((_ac) == ACI_BE) ? TXQ_ID_FOR_AC1 : \
    TXQ_ID_FOR_AC1)

#endif /* _HALTXQID_H_ */
