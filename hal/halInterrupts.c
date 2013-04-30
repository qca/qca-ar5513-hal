/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * Hardware-abstracted functions used to deal with interrupts.  These functions
 * are primarily called in the driver code residing above the HAL layer.
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halInterrupts.c#1 $"

#include "wlantype.h"   /* A_UINTxx, etc. */
#include "wlandrv.h"    /* WLAN_DEV_INFO struct */
#include "halApi.h"     /* fn prototypes */
#include "hal.h"        /* HW_FUNCS struct */


/**************************************************************
 * halIsInteruptPending
 *
 * Checks to see if an interrupt is pending on our NIC
 *
 * Returns: TRUE    if an interrupt is pending
 *          FALSE   if not
 */
A_BOOL
halIsInterruptPending(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwIsInterruptPending);

    return pDev->pHwFunc->hwIsInterruptPending(pDev);
}

/**************************************************************
 * halGetInterrupts
 *
 * Reads the Interrupt Status Register value from the NIC, thus deasserting
 * the interrupt line, and returns both the masked and unmasked mapped ISR
 * values.  The value returned is mapped to abstract the hw-specific bit
 * locations in the Interrupt Status Register.
 *
 * Returns: A hardware-abstracted bitmap of all non-masked-out
 *          interrupts pending, as well as an unmasked value
 */
HAL_INT_TYPE
halGetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pDescQueueBitMask)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwGetInterrupts);

    return pDev->pHwFunc->hwGetInterrupts(pDev, pUnmaskedValue, pDescQueueBitMask);
}

/**************************************************************
 * halEnableInterrupts
 *
 * Atomically enables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in "ints".
 */
VOID
halEnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwEnableInterrupts);
    ASSERT((ints & HAL_INT_INVALID) == 0);

    pDev->pHwFunc->hwEnableInterrupts(pDev, ints);
}

/**************************************************************
 * halDisableInterrupts
 *
 * Atomically disables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in "ints".
 */
VOID
halDisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwDisableInterrupts);
    ASSERT((ints & HAL_INT_INVALID) == 0);

    pDev->pHwFunc->hwDisableInterrupts(pDev, ints);
}

