/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Connects reset Reg Vectors, EEPROM Data, and device Functions to pDev
 */

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/halAttach.c#5 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

#include "halDevId.h"

#ifdef BUILD_AR5211
#include "ar5211/ar5211Attach.h"
#endif

#ifdef BUILD_AR5212
#include "ar5212/ar5212Attach.h"
#endif

#ifdef BUILD_AR5513
#include "ar5513/ar5513Attach.h"
#endif

typedef struct DeviceAttachData {
    A_UINT16 deviceID;
    A_STATUS (*hwAttach)(WLAN_DEV_INFO *, A_UINT16);
} DEVICE_ATTACH_DATA;

static DEVICE_ATTACH_DATA ar5kAttachData[] = {
#ifdef BUILD_AR5211
    {AR5211_DEVID,      ar5211Attach},
    {AR5311_DEVID,      ar5211Attach},
    {AR5312_DEVID,      ar5211Attach},
    {AR5211_FPGA11B,    ar5211Attach},
    {AR5211_DEFAULT,    ar5211Attach},
#endif

#ifdef BUILD_AR5212
    {AR5212_DEVID,      ar5212Attach},
    {AR5212_DEFAULT,    ar5212Attach},
    {AR5312_DEVID,      ar5212Attach},
    {AR5212_DEVID_0014, ar5212Attach},
    {AR5212_DEVID_0015, ar5212Attach},
    {AR5212_DEVID_0016, ar5212Attach},
    {AR5212_DEVID_0017, ar5212Attach},
    {AR5212_DEVID_0018, ar5212Attach},
    {AR5212_DEVID_0019, ar5212Attach},
    {AR5212_DEVID_001A, ar5212Attach},
#endif

#ifdef BUILD_AR5513
    {AR5513_DEVID,      ar5513Attach},
    {AR5513_DEVID_0013, ar5513Attach},
    {AR5513_DEVID_ff18, ar5513Attach},
#if defined(FALCON_EMUL)
    {AR5513_DEVID_fb40, ar5513Attach},
#endif /* FALCON_EMUL */
    {AR5513_DEFAULT,    ar5513Attach},
#endif

};

const static A_UINT32 NumDevids = (sizeof(ar5kAttachData) / sizeof(DEVICE_ATTACH_DATA));

typedef struct macRevData {
    A_UINT16 macRev;
    A_UINT16 deviceID;
    A_UINT32 macSrevReg;
} MAC_REV_DATA;

#define MAC_SREV_MASK   0xff

static MAC_REV_DATA ar5kMacRevData[] = {
#ifdef BUILD_AR5211
    {AR5211_SREV_1_0, AR5211_DEVID, AR5211_SREV_REG},
    {AR5211_SREV_1_1, AR5211_DEVID, AR5211_SREV_REG},
#endif

#ifdef BUILD_AR5212
    {AR5212_SREV_1_0, AR5212_DEVID, AR5212_SREV_REG},
    {AR5212_SREV_1_1, AR5212_DEVID, AR5212_SREV_REG},
    {AR5212_SREV_1_3, AR5212_DEVID, AR5212_SREV_REG},
    {AR5212_SREV_1_4, AR5212_DEVID, AR5212_SREV_REG},
    {AR5213_SREV_1_0, AR5212_DEVID, AR5213_SREV_REG},
#endif

#ifdef BUILD_AR5513
    {AR5513_SREV_1_0, AR5513_DEVID, AR5513_SREV_REG},
#endif

};

const static A_UINT32 NumMacRev = (sizeof(ar5kMacRevData) / sizeof(MAC_REV_DATA));

static void halGetDeviceId(WLAN_DEV_INFO *pDev);

/**************************************************************
 * halGetDeviceId
 *
 * halGetDeviceId is actual private function in the HAL layer.
 * It's used to set the DeviceID for the WLAN.  For stations
 * running in NT4.0, the DeviceID may not be available and
 * would have to be derived from the MAC_SREV value.
 */
void
halGetDeviceId(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);

    if (pDev->pciInfo.DeviceID) {
        return;
    }
#ifdef NDIS40_MINIPORT
    {
        A_UINT32 macRev;
        A_UINT32 count;

        for (count=0; count < NumMacRev; count++) {
            macRev = A_REG_RD(pDev, ar5kMacRevData[count].macSrevReg) & MAC_SREV_MASK;
            if (macRev == ar5kMacRevData[count].macRev) {
                pDev->pciInfo.DeviceID = ar5kMacRevData[count].deviceID;
                break;
            }
        }
    }
#endif
}

/**************************************************************
 * halAttach
 *
 * HalAttach is the first call into the HAL layer.  The HAL
 * Attach detects device chip revisions, initializes the hwLayer
 * function list, reads EEPROM information,
 * selects reset vectors, and performs a short self test.
 * Any failures will return an error that should cause a hardware
 * disable.
 */

A_STATUS
halAttach(WLAN_DEV_INFO *pDev)
{
    A_STATUS      status;
    unsigned int  i;
    A_UINT32      numDeviceIDs;

    ASSERT(pDev);

    halGetDeviceId(pDev);

    /* Find a matching DeviceID attach routine */
    numDeviceIDs = (sizeof(ar5kAttachData)/sizeof(DEVICE_ATTACH_DATA));
    for (i = 0; i < numDeviceIDs; i++) {
        if (pDev->pciInfo.DeviceID == ar5kAttachData[i].deviceID) {
            break;
        }
    }
    if (i == numDeviceIDs) {
        uiPrintf("halAttach: Failed to find an attachable driver for devid 0x%04X\n", pDev->pciInfo.DeviceID);
        return A_DEVICE_NOT_FOUND;
    }

    pDev->pHalInfo = (HAL_INFO *) A_DRIVER_MALLOC(sizeof(*pDev->pHalInfo));
    if (pDev->pHalInfo == NULL) {
        uiPrintf("halAttach: Error allocating memory for info struct\n");
        return A_ERROR;
    }
    A_MEM_ZERO(pDev->pHalInfo, sizeof(HAL_INFO));

    /* Call the device specific attach function */
    status = ar5kAttachData[i].hwAttach(pDev, pDev->pciInfo.DeviceID);

    /* If unsuccessful, free any allocated memory */
    if (status != A_OK) {
        A_DRIVER_FREE(pDev->pHalInfo, sizeof(HAL_INFO));
        pDev->pHalInfo = NULL;
    }

    return status;
}

/**************************************************************
 * halDetach
 *
 * Remove HAL layer allocated structures
 */
A_STATUS
halDetach(WLAN_DEV_INFO *pDev)
{
    A_STATUS status;

    ASSERT(pDev);

    if (pDev->pHalInfo == NULL) {
        return A_ERROR;
    }

    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwDetach);
    status = pDev->pHwFunc->hwDetach(pDev);

    /* Free HAL info struct */
    A_DRIVER_FREE(pDev->pHalInfo, sizeof(HAL_INFO));
    pDev->pHalInfo = NULL;

    return status;
}

/**************************************************************
 * halFillCapabilityInfo
 *
 * Request the HAL to recache the static capability info (which
 * is changing).
 */
A_STATUS
halFillCapabilityInfo(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwDetach);
    return pDev->pHwFunc->hwFillCapabilityInfo(pDev);
}

/**************************************************************
 * halGetNumDevid
 *
 * Returns the number of devids in the table
 */
A_UINT32
halGetNumDevid(void)
{
    return NumDevids;
}

/**************************************************************
 * halGetDevids
 *
 * Return the devid at the given index
 */
A_UINT16
halGetDevids(A_UINT32 index)
{
    ASSERT(index < NumDevids);

    return ar5kAttachData[index].deviceID;
}
