/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halReceive.c#1 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

/**************************************************************
 * halGetRxDP
 *
 * Get the RXDP.
 */
A_UINT32
halGetRxDP(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwGetRxDP(pDev);
}

/**************************************************************
 * halSetRxDP
 *
 * Set the RxDP.
 */
void
halSetRxDP(WLAN_DEV_INFO *pDev, A_UINT32 rxdp)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetRxDP(pDev, rxdp);
}

/**************************************************************
 * halEnableReceive
 *
 * Set Transmit Enable bits for the "main" data queue.
 */
void
halEnableReceive(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwEnableReceive(pDev);
}

/**************************************************************
 * halStopDmaReceive
 *
 * Stop Transmit at the DMA engine
 */
A_STATUS
halStopDmaReceive(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    return pDev->pHwFunc->hwStopDmaReceive(pDev);
}

/**************************************************************
 * halStartPcuReceive
 *
 * Start Transmit at the PCU engine (unpause receive)
 */
void
halStartPcuReceive(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwStartPcuReceive(pDev);
}

/**************************************************************
 * halStopPcuReceive
 *
 * Stop Transmit at the PCU engine (pause receive)
 */
void
halStopPcuReceive(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwStopPcuReceive(pDev);
}


/**************************************************************
 * halSetMulticastFilter
 *
 * Set multicast filter 0 (lower 32-bits)
 *               filter 1 (upper 32-bits)
 */
void
halSetMulticastFilter(WLAN_DEV_INFO *pDev, A_UINT32 filter0, A_UINT32 filter1)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwSetMulticastFilter(pDev, filter0, filter1);
}

/**************************************************************
 * halMulticastFilterIndex
 *
 * Clear/Set multicast filter by index
 */
void
halMulticastFilterIndex(WLAN_DEV_INFO *pDev, A_UINT32 index, A_BOOL bSet)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);

    pDev->pHwFunc->hwMulticastFilterIndex(pDev, index, bSet);
}

/**************************************************************
 * halRxFilter
 *
 * Set the RxFilter in HW and its associated pDev value
 */
void
halRxFilter(WLAN_DEV_INFO *pDev, A_UINT32 bits, HAL_RX_FILTER_OPER oper)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwRxFilter);
    pDev->pHwFunc->hwRxFilter(pDev, bits, oper);
}


A_STATUS
halProcessRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc)
{
    ASSERT(pDev);
    ASSERT(pDesc);

    return pDev->pHwFunc->hwProcessRxDesc(pDev, pDesc);
}

void
halSetupRxDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, A_UINT32 size)
{
    ASSERT(pDev);
    ASSERT(pDesc);

    pDev->pHwFunc->hwSetupRxDesc(pDesc, size);
}
#ifdef UPSD
void
halUpsdResponse(WLAN_DEV_INFO *pDev)
{
    pDev->pHwFunc->hwUpsdResponse(pDev);
}
#endif
