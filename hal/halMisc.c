/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 */

#ident "ACI $Id: //depot/sw/branches/AV_dev/src/hal/halMisc.c#3 $"

#include "wlantype.h"
#include "wlandrv.h"
#include "wlanchannel.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"

/**************************************************************
 * halGetCapability
 *
 * Returns the software state for the given hardware capability
 * request.
 */
A_UINT32
halGetCapability(WLAN_DEV_INFO *pDev, HAL_CAPABILITY_TYPE requestType, A_UINT32 param)
{
    HAL_CAPABILITIES *pCap;
    A_UINT32         result = 0;

    ASSERT(pDev);
    pCap = &(pDev->pHalInfo->halCapabilities);
    
    ASSERT(requestType < HAL_END_CAPABILITY_TYPE);
    ASSERT(pCap);

    switch (requestType) {
    case HAL_GET_REG_DMN:
    case HAL_GET_WIRELESS_MODES:
    case HAL_GET_CHAN_SPREAD_SUPPORT:
    case HAL_GET_SLEEP_AFTER_BEACON_SUPPORT:
    case HAL_GET_COMPRESS_SUPPORT:
    case HAL_GET_BURST_SUPPORT:
    case HAL_GET_FAST_FRAME_SUPPORT:
    case HAL_GET_CHAP_TUNING_SUPPORT:
    case HAL_GET_TURBO_G_SUPPORT:
    case HAL_GET_TURBO_PRIME_SUPPORT:
    case HAL_GET_DEVICE_TYPE:
    case HAL_GET_XR_SUPPORT:
    case HAL_GET_NUM_QUEUES:
    case HAL_GET_KEY_CACHE_SIZE:
    case HAL_GET_BEAMFORM_SUPPORT:
    case HAL_GET_RXCOMB_SUPPORT:
        result = ((A_UINT32 *)pCap)[requestType];
        break;

    case HAL_GET_LOW_CHAN_EDGE:
        if (IS_CHAN_5GHZ(param)) {
            result = pCap->halLow5GhzChan;
        } else {
            result = pCap->halLow2GhzChan;
        }
        break;

    case HAL_GET_HIGH_CHAN_EDGE:
        if (IS_CHAN_5GHZ(param)) {
            result = pCap->halHigh5GhzChan;
        } else {
            result = pCap->halHigh2GhzChan;
        }
        break;

    case HAL_GET_MIC_SUPPORT:
        switch (param) {
        case PRIV_KEY_TYPE_CKIP:
            result = pCap->halMicCkipSupport;
            break;
        case PRIV_KEY_TYPE_TKIP:
            result = pCap->halMicTkipSupport;
            break;
        case PRIV_KEY_TYPE_AES_CCM:
            result = pCap->halMicAesCcmSupport;
            break;
        case PRIV_KEY_TYPE_TKIP_SW:
            result = FALSE;
            break;
        default:
            result = TRUE;
            break;
        }
        break;

    case HAL_GET_CIPHER_SUPPORT:
        switch (param) {
        case PRIV_KEY_TYPE_CKIP:
            result = pCap->halCipherCkipSupport;
            break;
        case PRIV_KEY_TYPE_TKIP:
            result = pCap->halCipherTkipSupport;
            break;
        case PRIV_KEY_TYPE_AES_CCM:
            result = pCap->halCipherAesCcmSupport;
            break;
        case PRIV_KEY_TYPE_TKIP_SW:
            result = FALSE;
            break;
        default:
            result = TRUE;
            break;
        }
        break;
    default:
        ASSERT(0);
        break;
    }

    return result;
}

/**************************************************************
 * halGetSerialNumber
 *
 * Copy Hal Serial Number into provided string.
 * Returns TRUE if the value was copied.
 */
A_BOOL
halGetSerialNumber(WLAN_DEV_INFO *pDev, A_CHAR *pSerialNum, A_UINT16 strLen)
{
    if (strLen < sizeof(pDev->pHalInfo->serialNumber)) {
        return FALSE;
    }
    A_BCOPY(pDev->pHalInfo->serialNumber, pSerialNum, sizeof(pDev->pHalInfo->serialNumber));
    return TRUE;
}

/**************************************************************
 * halSetRegulatoryDomain
 *
 * Changes the Regulatory domain in the PROM.
 * Returns A_OK for a successful change.
 */
A_STATUS
halSetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwSetRegulatoryDomain);
    return pDev->pHwFunc->hwSetRegulatoryDomain(pDev, regDomain);
}

/**************************************************************
 * halSetLedState
 *
 * Change LED state based on WLAN connection status
 */
void
halSetLedState(WLAN_DEV_INFO *pDev, A_BOOL bConnected)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwSetLedState);
    pDev->pHwFunc->hwSetLedState(pDev, bConnected);
}

/**************************************************************
 * halWriteAssocid
 *
 * Setup the association id (duh)
 */
void
halWriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwWriteAssocid);
    pDev->pHwFunc->hwWriteAssocid(pDev, bssid, assocId, timOffset);
}

void
halGpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGpioCfgInput);
    pDev->pHwFunc->hwGpioCfgInput(pDev, gpio);
}

void
halGpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGpioCfgOutput);
    pDev->pHwFunc->hwGpioCfgOutput(pDev, gpio);
}

A_UINT32
halGpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGpioGet);
    return pDev->pHwFunc->hwGpioGet(pDev, gpio);
}

void
halGpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGpioSet);
    pDev->pHwFunc->hwGpioSet(pDev, gpio, val);
}

void
halGpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGpioSetIntr);
    pDev->pHwFunc->hwGpioSetIntr(pDev, gpio, ilevel);
}

void
halSetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwSetStaBeaconTimers);
    pDev->pHwFunc->hwSetStaBeaconTimers(pDev, pTimers);
}

void
halGetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwGetTsf);
    pDev->pHwFunc->hwGetTsf(pDev, tsf);
}

void
halResetTsf(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwResetTsf);
    pDev->pHwFunc->hwResetTsf(pDev);
}

void
halSetAdhocMode(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwSetAdhocMode);
    pDev->pHwFunc->hwSetAdhocMode(pDev);
}

/**************************************************************
 * halSetBasicRate
 *
 * Modify the basic rate
 */
void
halSetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *pSet)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    pDev->pHwFunc->hwSetBasicRate(pDev, pSet);
}

/**************************************************************
 * halGetRandomSeed
 *
 * Grab a semi-random value from hardware registers - may not
 * change often
 */
A_UINT32
halGetRandomSeed(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    return pDev->pHwFunc->hwGetRandomSeed(pDev);
}

/**************************************************************
 * halDetectCardPresent
 *
 * Detect if our card is present
 */
A_BOOL
halDetectCardPresent(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwDetectCardPresent);
    return pDev->pHwFunc->hwDetectCardPresent(pDev);
}

/**************************************************************
 * halMibControl
 *
 */
A_UINT32
halMibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    if (pDev->pHwFunc->hwMibControl) {
        return pDev->pHwFunc->hwMibControl(pDev, cmd, pContext);
    }

    return 0;
}

/**************************************************************
 * halGetChannelData
 *
 * For the given channel, return constants for noise floor,
 * cca threshold, and clock rate.
 */
void
halGetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                  HAL_CHANNEL_DATA *pData)
{
    if (pDev->pHwFunc->hwGetChannelData) {
        pDev->pHwFunc->hwGetChannelData(pDev, pChan, pData);
    }
}

/**************************************************************
 * halProcessNoiseFloor
 *
 * Take the given channel list and process all valid raw noise floors
 * into the dBm noise floor values.
 */
void
halProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwProcessNoiseFloor);
    pDev->pHwFunc->hwProcessNoiseFloor(pDev, pCList);
}

/**************************************************************
 * halEnableRadarDetection
 *
 * Enable radar detection in the hardware - may later be changed
 * to modify radar "sensitivity"
 */
void
halEnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwEnableRadarDetection);
    pDev->pHwFunc->hwEnableRadarDetection(pDev, pPhyParams);
}

/**************************************************************
 * halEnableFriendlyDetection
 *
 * Enable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
halEnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwEnableFriendlyDetection);
    pDev->pHwFunc->hwEnableFriendlyDetection(pDev, detectionRSSIThr);
}

/**************************************************************
 * halDisableFriendlyDetection
 *
 * Disable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
halDisableFriendlyDetection(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    ASSERT(pDev->pHwFunc->hwDisableFriendlyDetection);
    pDev->pHwFunc->hwDisableFriendlyDetection(pDev);
}

/**************************************************************
 * halAniControl
 *
 * Control Adaptive Noise Immunity Parameters
 */
void
halAniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param)
{
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    if (pDev->pHwFunc->hwAniControl) {
        pDev->pHwFunc->hwAniControl(pDev, cmd, param);
    }
}

A_INT32
halAniGetListenTime(WLAN_DEV_INFO *pDev) {
    ASSERT(pDev);
    ASSERT(pDev->pHwFunc);
    if (pDev->pHwFunc->hwAniGetListenTime) {
        return pDev->pHwFunc->hwAniGetListenTime(pDev);
    } else {
        return 0;
    }
}

A_RSSI32
halGetCurRssi(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    return pDev->pHwFunc->hwGetCurRssi(pDev);
}

A_UINT32
halGetDefAntenna(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev);
    return pDev->pHwFunc->hwGetDefAntenna(pDev);
}

void
halSetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna)
{
    ASSERT(pDev);
    pDev->pHwFunc->hwSetDefAntenna(pDev, antenna);
}

/**************************************************************
 * halSetAntennaSwitch
 *
 */
void
halSetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                    CHAN_VALUES *pChval)
{
    ASSERT(pDev);
    pDev->pHwFunc->hwSetAntennaSwitch(pDev, settings, pChval);    
}

void
halUpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                 A_RSSI rssiAck, A_UINT8 curTxAnt)
{
    ASSERT(pDev && pSib);
    pDev->pHwFunc->hwUpdateAntenna(pDev, pSib, retries, rssiAck, curTxAnt);
}

/**************************************************************
 * halUseShortSlotTime
 *
 * Initializes the HW registers required to send beacons.
 */
A_BOOL
halUseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwUseShortSlotTime);

    return pDev->pHwFunc->hwUseShortSlotTime(pDev, en, prev);
}

void
halDmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwDmaDebugDump);

    pDev->pHwFunc->hwDmaDebugDump(pDev, verbose);
}

A_UINT32
halGetMacTimer3(WLAN_DEV_INFO *pDev)
{
    ASSERT(pDev && pDev->pHwFunc && pDev->pHwFunc->hwGetMacTimer3);
    return pDev->pHwFunc->hwGetMacTimer3(pDev);
}

/**************************************************************
 * halSwapHwDesc
 *
 * swap the hw fields of the descriptor
 */
void
halSwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete)
{

    ASSERT(pDev && pDev->pHwFunc);
    
    if (pDev->pHwFunc->hwSwapHwDesc) {
        pDev->pHwFunc->hwSwapHwDesc(pDev, pDesc, pTail, complete);
    }
}

/**************************************************************
 * halGetHwDescWord
 *
 * get the field of the descriptor
 */
A_UINT32
halGetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord)
{

    ASSERT(pDev && pDev->pHwFunc);

    return pDev->pHwFunc->hwGetHwDescWord(pDev, hwDescWord);
}

/**************************************************************
 * halGetApSwitchHelper
 *
 * Get some Beacon relevant registers.
 */
void
halGetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{

    ASSERT(pDev && pDev->pHwFunc);

    if (pDev->pHwFunc->hwGetApSwitchHelper) {
        pDev->pHwFunc->hwGetApSwitchHelper(pDev, pRegs);
    }
}

/**************************************************************
 * halSetApSwitchHelper
 *
 * Restore some Beacon relevant registers.
 */
void
halSetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{

    ASSERT(pDev && pDev->pHwFunc);

    if (pDev->pHwFunc->hwSetApSwitchHelper) {
        pDev->pHwFunc->hwSetApSwitchHelper(pDev, pRegs);
    }
}

/**************************************************************
 * halSendXrChirp
 *
 * Send a double chirp
 */
void
halSendXrChirp(WLAN_DEV_INFO *pDev)
{

    ASSERT(pDev && pDev->pHwFunc);

    if (pDev->pHwFunc->hwSendXrChirp) {
        pDev->pHwFunc->hwSendXrChirp(pDev);
    }
}
