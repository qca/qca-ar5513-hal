/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halReset.c#1 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

/**************************************************************
 * halPeriodicCal
 *
 * Readjusts hardware to environmental changes such as
 * temperature, noise, etc.  Performs channel "correctness" check.
 *
 */
A_STATUS
halPerCalibration(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChval)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwPerCalibration);
    return pDev->pHwFunc->hwPerCalibration(pDev, pChval);
}

/**************************************************************
 * halReset
 *
 * halReset requires a successful halInit.  halReset soft-resets
 * all chips present and then reloads their initialization
 * information based on the chips' selected operation mode.
 *
 */
A_STATUS
halReset(WLAN_DEV_INFO *pDev, WLAN_SERVICE serviceType, CHAN_VALUES *pChval, A_BOOL bChannelChange)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwReset);
    return pDev->pHwFunc->hwReset(pDev, serviceType, pChval, bChannelChange);
}

/**************************************************************
 * halDisable
 *
 * Disable the chip by putting it into reset
 */
A_STATUS
halDisable(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwDisable);
    return pDev->pHwFunc->hwDisable(pDev);
}

/**************************************************************
 * halPhyDisable
 *
 * Used by RFKIll to turn off all radio capabilities.
 */
A_STATUS
halPhyDisable(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwPhyDisable);
    return pDev->pHwFunc->hwPhyDisable(pDev);
}

#if DEBUG
int GainDebug = 0;
#endif

/**************************************************************
 * halGetRfgain
 *
 * Get the Rf Gain state and inform the RfGain that a packet has been
 * sent.
 */
RFGAIN_STATES
halGetRfgain(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGetRfgain);
    return pDev->pHwFunc->hwGetRfgain(pDev);
}

/**************************************************************
 * halSetTxPowerLimit
 *
 * Sets a limit on the overall output power.  Used for dynamic
 * transmit power control and the like.
 *
 * NOTE: The power passed in is in units of 0.5 dBm.
 */
void
halSetTxPowerLimit(WLAN_DEV_INFO *pDev, A_UINT32 limit)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwSetTxPowerLimit);
    pDev->pHwFunc->hwSetTxPowerLimit(pDev, limit);
}


