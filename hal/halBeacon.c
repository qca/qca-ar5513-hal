/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * TODO: comment
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halBeacon.c#3 $"

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO struct */
#include "halApi.h"     /* fn prototypes */
#include "hal.h"        /* HW_FUNCS struct */
#include "wlanchannel.h"


/**************************************************************
 * halSetupBeaconDesc
 *
 * This routine is used by both the AP and client.  It constructs the beacon
 * descriptor based on HW requirements.
 *
 * Assumptions: frameLen does not include the FCS
 */
void
halSetupBeaconDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_INT32 beaconCnt, A_BOOL updtAntOnly, A_BOOL isAp)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwSetupBeaconDesc);

    pDev->pHwFunc->hwSetupBeaconDesc(pDev, pDesc, beaconCnt, updtAntOnly, isAp);
}

/**************************************************************
 * halBeaconInit
 *
 * Initializes the HW registers required to send beacons.
 */
void
halBeaconInit(WLAN_DEV_INFO *pDev, A_UINT32 tsf, A_BOOL isAp)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwBeaconInit);

    pDev->pHwFunc->hwBeaconInit(pDev, tsf, isAp);
}

/**************************************************************
 * halSetupGrpPollChain
 *
 * This routine is used by both the AP and client.  It sets up the
 * group poll chain for XR
 */
void
halSetupGrpPollChain(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pHead, ATHEROS_DESC *pTail, A_UINT32 hwIndex)
{
    ASSERT(pDev && pDev->pHwFunc);

    if (pDev->pHwFunc->hwSetupGrpPollChain) {
        pDev->pHwFunc->hwSetupGrpPollChain(pDev, pHead, pTail, hwIndex);
    }
}

