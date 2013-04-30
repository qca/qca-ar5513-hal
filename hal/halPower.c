/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halPower.c#1 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"


/**************************************************************
 * halSetPowerMode
 *
 * Set power mgt to the requested mode, and conditionally set
 * the chip as well
 * Returns A_OK if chip is successfully forced awake.
 */
A_STATUS
halSetPowerMode(WLAN_DEV_INFO *pDev, A_UINT32 powerRequest, A_BOOL setChip)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwSetPowerMode);

    return pDev->pHwFunc->hwSetPowerMode(pDev, powerRequest, setChip);
}

/**************************************************************
 * halGetPowerMode
 *
 * Get and return current power setting from the chip
 */
A_UINT32
halGetPowerMode(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwGetPowerMode);

    return pDev->pHwFunc->hwGetPowerMode(pDev);
}

/**************************************************************
 * halGetPowerStatus
 *
 * Return the current sleep state of the chip
 * TRUE = sleeping
 */
A_BOOL
halGetPowerStatus(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwGetPowerStatus);

    return pDev->pHwFunc->hwGetPowerStatus(pDev);
}

/**************************************************************
 * halSetupPSPollDesc
 *
 * Initialize for PS-Polls
 */
void
halSetupPSPollDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pPSPollDesc)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwSetupPSPollDesc);

    pDev->pHwFunc->hwSetupPSPollDesc(pDev, pPSPollDesc);
}
