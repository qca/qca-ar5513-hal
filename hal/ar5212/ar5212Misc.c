/*
 *  Copyright (c) 2000-2003 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific device miscellaneous functions including hardware queries,
 *  EEPROM routines, gpio funcs, beacon creation, ...
 */

#ifdef BUILD_AR5212

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5212/ar5212Misc.c#3 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"
#include "halDevId.h"
#include "wlanchannel.h"

/* Headers for HW private items */
#include "ar5212Reg.h"
#include "ar5212Misc.h"
#include "ar5212Reset.h"
#include "ar5212Power.h"
#include "ar5212Interrupts.h"
#include "ar5212.h"

#include "pktlog.h"

#if defined(AR531X)
#include "ar531xreg.h"
#endif

static A_STATUS
ar5212EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data);

static A_INT16
ar5212GetNfAdjust(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan);

static A_BOOL
ar5212IsSlowSleepClockAllowed(WLAN_DEV_INFO *pDev);

#define SLOT_TIME_20 880
#define SLOT_TIME_09 396

/**************************************************************
 * ar5212SetRegulatoryDomain
 *
 * Attempt to change the cards operating regulatory domain to the given value
 * Returns: A_EINVAL for an unsupported regulatory domain.
 *          A_HARDWARE for an unwritable EEPROM or bad EEPROM version
 */
A_STATUS
ar5212SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain)
{
    A_STATUS status = A_OK;
    EEP_MAP *pEep;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pEep = pDev->pHalInfo->pEepData;
    if (pDev->pHalInfo->halCapabilities.halRegDmn != regDomain) {
        if (!(pEep->protect & EEPROM_PROTECT_WP_128_191))
        {
            if(wlanIsRegCcValid(pDev)) {
                status = ar5212EepromWrite(pDev, REGULATORY_DOMAIN_OFFSET, regDomain);
                if (status == A_OK) {
                    pDev->pHalInfo->halCapabilities.halRegDmn = regDomain;
                } else {
                    uiPrintf("Failed to write EEPROM\n");
                    status = A_HARDWARE;
                }
            } else {
                uiPrintf("EEPROM does not contain matching information for this country setting\n");
                status = A_EINVAL;
            }
        } else {
            uiPrintf("Protect bits set or incorrect EEPROM version\n");
            status = A_HARDWARE;
        }
    }

    if (status != A_OK) {
        uiPrintf("Cannot change current EEPROM Country/Reg 0x%02X to given value 0x%02X\n",
                 pDev->pHalInfo->halCapabilities.halRegDmn, regDomain);
    }

    return status;
}

/**************************************************************
 * ar5212GetRfKill
 *
 * Accessor to get rfkill info from private EEPROM structures
 */
A_BOOL
ar5212GetRfKill(WLAN_DEV_INFO *pDev)
{
    A_UINT16 rfsilent;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    if (pDev->pHalInfo->pEepData->pEepHeader->rfKill) {
        ar5212EepromRead(pDev, EEPROM_RFSILNT_CLKEN_OFFSET, &rfsilent);
        pDev->rfSilent.gpioSelect = (rfsilent & EEP_RFSILENT_GPIO_SEL_MASK) >> EEP_RFSILENT_GPIO_SEL_SHIFT;
        pDev->rfSilent.polarity = (rfsilent & EEP_RFSILENT_POLARITY_MASK) >> EEP_RFSILENT_POLARITY_SHIFT;
        pDev->rfSilent.eepEnabled = TRUE;
    } else {
        pDev->rfSilent.gpioSelect = pDev->rfSilent.polarity = 0;
        pDev->rfSilent.eepEnabled = FALSE;
    }
    return (A_BOOL)pDev->pHalInfo->pEepData->pEepHeader->rfKill;
}

/**************************************************************
 * ar5212GetMacAddr
 *
 * Attempt to get the MAC address from the wireless EEPROM
 * Returns: A_HARDWARE for an unreadable EEPROM
 */
A_STATUS
ar5212GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac)
{
#if defined(PCI_INTERFACE)
    A_UINT32 total = 0;
    A_UINT16 half;

    if (ar5212EepromRead(pDev, EEPROM_MAC_MSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[1] = half & 0xff;
    mac->octets[0] = half >> 8;

    if (ar5212EepromRead(pDev, EEPROM_MAC_MID_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[3] = half & 0xff;
    mac->octets[2] = half >> 8;
    if (ar5212EepromRead(pDev, EEPROM_MAC_LSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[5] = half & 0xff;
    mac->octets[4] = half >> 8;

    if ((total == 0) || total == (3 * 0xffff)) {
        uiPrintf("ar5212GetMacAddr: EEPROM MAC is all 0's or all F's\n");
        return A_HARDWARE;
    }
    return A_OK;

ee_error:
    uiPrintf("ar5212GetMacAddr: EEPROM read failed\n");
    return A_HARDWARE;
#elif defined(AR531X)
    /* The ar5312 may store the WLAN mac addr in the system board config */
    if (pDev->devno == 0) {
        A_BCOPY(sysBoardData.wlan0Mac, mac->octets, 6);
    } else {
        A_BCOPY(sysBoardData.wlan1Mac, mac->octets, 6);
    }
    return A_OK;
#endif
}

/**************************************************************
 * ar5212EepromRead
 *
 * Read 16 bits of data from offset into *data
 */
A_STATUS
ar5212EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 status;
    int to = 10000;     /* 10ms timeout */

    writePlatformReg(pDev, MAC_EEPROM_ADDR, offset);
    writePlatformReg(pDev, MAC_EEPROM_CMD, MAC_EEPROM_CMD_READ);

    while (to > 0) {
        udelay(1);
        status = readPlatformReg(pDev, MAC_EEPROM_STS);
        if (status & MAC_EEPROM_STS_READ_COMPLETE) {
            status = readPlatformReg(pDev, MAC_EEPROM_DATA);
            *data = (A_UINT16)(status & 0xffff);
            return A_OK;
        }

        if (status & MAC_EEPROM_STS_READ_ERROR) {
            uiPrintf("ar5212EepromRead: eeprom read error at offset %d.\n", offset);
            return A_HARDWARE;
        }
        to--;
    }
    uiPrintf("ar5212EepromRead: eeprom read timeout at offset %d.\n", offset);
    return A_HARDWARE;
#elif defined(AR531X)
    /* radio configuration data is stored in the system flash */
    *data = sysFlashConfigRead(FLC_RADIOCFG, (offset * 2) + 1) |
            (sysFlashConfigRead(FLC_RADIOCFG, offset * 2) << 8);
    return A_OK;
#endif /* PLATFORM */
}

/**************************************************************
 * ar5212EepromWrite
 *
 * Write 16 bits of data from data into offset
 */
A_STATUS
ar5212EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 status;
    int to = 15000;     /* 15ms timeout */

    /* Send write data */
    writePlatformReg(pDev, MAC_EEPROM_ADDR, offset);
    writePlatformReg(pDev, MAC_EEPROM_DATA, data);
    writePlatformReg(pDev, MAC_EEPROM_CMD, MAC_EEPROM_CMD_WRITE);

    while (to > 0) {
        udelay(1);
        status = readPlatformReg(pDev, MAC_EEPROM_STS);
        if (status & MAC_EEPROM_STS_WRITE_COMPLETE) {
            return A_OK;
        }

        if (status & MAC_EEPROM_STS_WRITE_ERROR)    {
            uiPrintf("ar5212EepromWrite: eeprom write error at offset %d.\n", offset);
            return A_HARDWARE;;
        }
        to--;
    }

    uiPrintf("ar5212EepromWrite: eeprom write timeout at offset %d.\n", offset);
    return A_HARDWARE;
#elif defined(AR531X)
    char str[2];

    /* Radio configuration data is stored in system flash (in reverse endian) */
    str[0] = (data >> 8) & 0xff;
    str[1] = data & 0xff;
    sysFlashConfigWrite(FLC_RADIOCFG, offset<<1, str, 2);

    return A_OK;
#endif /* PLATFORM */
}

/**************************************************************************
 * ar5212EnableRfKill
 *
 * Called if RfKill is supported (according to EEPROM).  Set the interrupt and
 * GPIO values so the ISR can disable RF on a switch signal.  Assumes GPIO port
 * and interrupt polarity are set prior to call.
 */
void
ar5212EnableRfKill(WLAN_DEV_INFO *pDev)
{
    /* TODO - can this really be above the hal on the GPIO interface for
     * TODO - the client only?
     */
#if !defined(BUILD_AP)  /* AP uses GPIOs for LEDs and buttons */

    /* Configure the desired GPIO port for input and enable baseband rf silence */
    ar5212GpioCfgInput(pDev, pDev->rfSilent.gpioSelect);
    writePlatformReg(pDev, PHY_BASE, readPlatformReg(pDev, PHY_BASE) | 0x00002000);
    /*
     * If radio disable switch connection to GPIO bit x is enabled
     * program GPIO interrupt.
     * If rfkill bit on eeprom is 1, setupeeprommap routine has already
     * verified that it is a later version of eeprom, it has a place for
     * rfkill bit and it is set to 1, indicating that GPIO bit x hardware
     * connection is present.
     */

    if (pDev->rfSilent.polarity == ar5212GpioGet(pDev, pDev->rfSilent.gpioSelect)) {
        /* switch already closed, set to interrupt upon open */
        ar5212GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, !pDev->rfSilent.polarity);
    } else {
        ar5212GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, pDev->rfSilent.polarity);
    }
#endif /* !BUILD_AP */
}

/**************************************************************
 * ar5212GpioCfgOutput
 *
 * Configure GPIO Output lines
 */
void
ar5212GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg =  readPlatformReg(pDev, MAC_GPIOCR);
//    reg &= ~(MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT));
    reg |= MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT);

    writePlatformReg(pDev, MAC_GPIOCR, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5212GpioCfgInput
 *
 * Configure GPIO Input lines
 */
void
ar5212GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg =  readPlatformReg(pDev, MAC_GPIOCR);
    reg &= ~(MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT));
    reg |= MAC_GPIOCR_0_CR_N << (gpio * MAC_GPIOCR_CR_SHIFT);

    writePlatformReg(pDev, MAC_GPIOCR, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5212GpioSet
 *
 * Once configured for I/O - set output lines
 */
void
ar5212GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg =  readPlatformReg(pDev, MAC_GPIODO);
    reg &= ~(1 << gpio);
    reg |= (val&1) << gpio;

    writePlatformReg(pDev, MAC_GPIODO, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5212GpioGet
 *
 * Once configured for I/O - get input lines
 */
A_UINT32
ar5212GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg =  readPlatformReg(pDev, MAC_GPIODI);
    reg = ((reg & MAC_GPIOD_MASK) >> gpio) & 0x1;

    return reg;
#else /* !PCI_INTERFACE */
    return 0;
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5212GpioSetIntr
 *
 * Set the desired GPIO Interrupt
 */
void
ar5212GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel)
{
#if defined(PCI_INTERFACE)
    A_UINT32    reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg = readPlatformReg(pDev, MAC_GPIOCR);
    /* clear the bits that we will modify */
    reg &= ~((MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT)) | MAC_GPIOCR_INT_MASK | MAC_GPIOCR_INT_EN | MAC_GPIOCR_INT_SELH);
    reg |= MAC_GPIOCR_INT_EN | (gpio << MAC_GPIOCR_INT_SHIFT) | (MAC_GPIOCR_0_CR_N << (gpio * MAC_GPIOCR_CR_SHIFT));
    if (ilevel) {
        reg |= MAC_GPIOCR_INT_SELH;
    }
    /* don't need to change anything for low level interrupt. */
    writePlatformReg(pDev, MAC_GPIOCR, reg);
    /* change the interrupt mask */
    writePlatformReg(pDev, MAC_IMR, readPlatformReg(pDev, MAC_IMR) | MAC_IMR_GPIO);
    pDev->MaskReg = readPlatformReg(pDev, MAC_IMR);
#endif /* PCI_INTERFACE */
}

/******************************************************************
 * ar5212SetLedState
 *
 * Change the LED blinking pattern to correspond to the connectivity.
 *   Normal blink when connected, alternate blink when not.
 */
void
ar5212SetLedState(WLAN_DEV_INFO *pDev, A_BOOL bConnected)
{
#if defined(PCI_INTERFACE)
    A_UINT32 val;

    val = bConnected ?
          MAC_PCICFG_ASSOC_STATUS_ASSOCIATED : MAC_PCICFG_ASSOC_STATUS_PENDING;

    if (pDev->pHalInfo->halCapabilities.halDeviceType != ATH_DEV_TYPE_MINIPCI) {
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, ASSOC_STATUS, val);
    }

#elif defined(AR531X)
    A_UINT32 reg = sysRegRead(AR531X_PCICFG) & ~ASSOC_STATUS_M;

    if (bConnected) {
        reg |= ASSOC_STATUS_ASSOCIATED;
    } else {
        reg |= ASSOC_STATUS_PENDING;
    }
    sysRegWrite(AR531X_PCICFG, reg);
#endif /* PLATFORM */
}

/**************************************************************************
 * ar5212WriteAssocid - Change association related fields programmed into the hardware.
 *
 * Writing a valid BSSID to the hardware effectively enables the hardware
 * to synchronize its TSF to the correct beacons and receive frames coming
 * from that BSSID. It is called by the SME JOIN operation.
 */
void
ar5212WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset)
{
    A_UINT32    ulAddressLow, ulAddressHigh;

    if (bssid != NULL) {
        ulAddressLow  = cpu2le32(bssid->st.word);
        ulAddressHigh = cpu2le16(bssid->st.half);
    } else {
        ulAddressLow = ulAddressHigh = 0;       /* default for no paramter */
    }

    A_REG_WR(pDev, MAC_BSS_ID0, ulAddressLow);
    A_REG_WR(pDev, MAC_BSS_ID1, ulAddressHigh);

    /* Workaround for a hardware bug in Oahu. Write a 0 associd to prevent
     * hardware from parsing beyond end of TIM element. It will avoid
     * sending spurious PS-polls. Beacon/TIM processing done in software
     */
    A_REG_RMW_FIELD(pDev, MAC_BSS_ID1, AID, 0);

    if (assocId != 0) {
        /* Now set the TIM_OFFSET */
        A_REG_RMW_FIELD(pDev, MAC_BEACON, TIM, timOffset ? timOffset + 4 : 0);
    }
}


/**************************************************************
 * ar5212SetStaBeaconTimers
 *
 *  Sets all the beacon related bits on the h/w for stations
 *  i.e. initializes the corresponding h/w timers;
 *  also tells the h/w whether to anticipate PCF beacons
 */
void
ar5212SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers)
{
    /* if the AP will do PCF */
    if (pTimers->cfpDuration) {
        /* tell the h/w that the associated AP is PCF capable */
        A_REG_SET_BIT(pDev, MAC_STA_ID1, PCF);

        /* set CFP_PERIOD(1.024ms) register */
        A_REG_WR(pDev, MAC_CFP_PERIOD, pTimers->cfpPeriod);

        /* set CFP_DUR(1.024ms) register to max cfp duration */
        A_REG_WR(pDev, MAC_CFP_DUR, pTimers->cfpDuration);

        /* set TIMER2(128us) to anticipated time of next CFP */
        A_REG_WR(pDev, MAC_TIMER2, pTimers->nextCfp * 8);
    } else {
        /* tell the h/w that the associated AP is not PCF capable */
        A_REG_CLR_BIT(pDev, MAC_STA_ID1, PCF);
    }


    /* set the TIMER0(1.024ms) register to anticipated time of the next beacon */
    A_REG_WR(pDev, MAC_TIMER0, pTimers->nextTbtt);

    /*
     * Start the beacon timers by setting the BEACON register to the beacon
     * interval; also write the tim offset which we should know by now.  The
     * code, in ar5211WriteAssocid, also sets the tim offset once the AID is
     * known which can be left as such for now.
     */
    A_REG_RMW_FIELD(pDev, MAC_BEACON, PERIOD, pTimers->beaconPeriod);
    A_REG_RMW_FIELD(pDev, MAC_BEACON, TIM,
            pDev->bssDescr->timOffset ? pDev->bssDescr->timOffset + 4 : 0);


    /*
     * Configure the BMISS interrupt
     */
    ASSERT(pTimers->bmissThreshold <= (MAC_RSSI_THR_BM_THR_M >> MAC_RSSI_THR_BM_THR_S));

    /* First disable the interrupt */
    ar5212DisableInterrupts(pDev, HAL_INT_GLOBAL);
    pDev->MaskReg &= ~MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);

    /* Clear any pending BMISS interrupt so far */
    pDev->globISRReg &= ~MAC_IMR_BMISS;

    A_REG_RMW_FIELD(pDev, MAC_RSSI_THR, BM_THR, pTimers->bmissThreshold);

    /* Now enable the interrupt */
    pDev->MaskReg |= MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);
    ar5212EnableInterrupts(pDev, HAL_INT_GLOBAL);

    /*
     * Quiet Time configuration for TGh
     * For now: hw test mode only
     */
    if (pDev->staConfig.quietDuration) {
        uiPrintf("Quiet Period start @ %d for %d (every %d)\n",
                 pTimers->nextTbtt + pDev->staConfig.quietOffset,
                 pDev->staConfig.quietDuration,
                 pTimers->beaconPeriod);
        A_REG_WR_FIELD(pDev, MAC_QUIET2, QUIET_PERIOD, pTimers->beaconPeriod);
        A_REG_RMW_FIELD(pDev, MAC_QUIET2, QUIET_DURATION, pDev->staConfig.quietDuration);

        A_REG_WR_FIELD(pDev, MAC_QUIET1, NEXT_QUIET, pTimers->nextTbtt + pDev->staConfig.quietOffset);
        if (pDev->staConfig.quietAckCtsAllow) {
            A_REG_SET_BIT(pDev, MAC_QUIET1, QUIET_ACK_CTS_ENABLE);
        }
        A_REG_SET_BIT(pDev, MAC_QUIET1, QUIET_ENABLE);
    }

    /*
     * 5211 beacons timers on the station were used for power
     * save operation (waking up in anticipation of a beacon)
     * and any CFP function; 5212 and up does sleep/power-save timers
     * differently - so this is the right place to set them up;
     * don't think the beacon timers are used by venice sta hw
     * for any useful purpose anymore
     * Setup venice's sleep related timers
     * Current implementation assumes sw processing of beacons -
     *   assuming an interrupt is generated every beacon which
     *   causes the hardware to become awake until the sw tells
     *   it to go to sleep again; beacon timeout is to allow for
     *   beacon jitter; cab timeout is max time to wait for cab
     *   after seeing the last DTIM or MORE CAB bit
     */
#define CAB_TIMEOUT_VAL     10 /* in TU */
#define BEACON_TIMEOUT_VAL  10 /* in TU */
#define SLEEP_SLOP          3  /* in TU */

    /*
     * For max powersave mode we may want to sleep for longer than a
     * beacon period and not want to receive all beacons; modify the
     * timers accordingly; make sure to align the next TIM to the
     * next DTIM if we decide to wake for DTIMs only
     */
    if (pTimers->sleepDuration > pTimers->beaconPeriod) {
        ASSERT(A_ROUNDUP(pTimers->sleepDuration, pTimers->beaconPeriod) == pTimers->sleepDuration);
        pTimers->beaconPeriod = pTimers->sleepDuration;
    }
    if (pTimers->sleepDuration > pTimers->dtimPeriod) {
        ASSERT(A_ROUNDUP(pTimers->sleepDuration, pTimers->dtimPeriod) == pTimers->sleepDuration);
        pTimers->dtimPeriod = pTimers->sleepDuration;
    }
    ASSERT(pTimers->beaconPeriod <= pTimers->dtimPeriod);
    if (pTimers->beaconPeriod == pTimers->dtimPeriod) {
        pTimers->nextTbtt = pTimers->nextDtim;
    }

#if 0
    uiPrintf("ar5212SetStaBeaconTimers: setting next DTIM for %d\n", pTimers->nextDtim);
    uiPrintf("ar5212SetStaBeaconTimers: setting next beacon for %d\n", pTimers->nextTbtt);
    uiPrintf("ar5212SetStaBeaconTimers: setting beacon period to %d\n", pTimers->beaconPeriod);
    uiPrintf("ar5212SetStaBeaconTimers: setting DTIM period to %d\n", pTimers->dtimPeriod);
#endif

    A_REG_WR_FIELD(pDev, MAC_SLEEP1, NEXT_DTIM, (pTimers->nextDtim - SLEEP_SLOP) * 8);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP1, CAB_TIMEOUT, CAB_TIMEOUT_VAL);
    A_REG_SET_BIT2(pDev, MAC_SLEEP1, ASSUME_DTIM, ENH_SLEEP_ENABLE);
    A_REG_WR_FIELD(pDev, MAC_SLEEP2, NEXT_TIM, (pTimers->nextTbtt - SLEEP_SLOP) * 8);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP2, BEACON_TIMEOUT, BEACON_TIMEOUT_VAL);
    A_REG_WR_FIELD(pDev, MAC_SLEEP3, TIM_PERIOD, pTimers->beaconPeriod);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP3, DTIM_PERIOD, pTimers->dtimPeriod);
}

/******************************************************************
 * ar5212GetTsf
 *
 * Get the current hardware tsf for stamlme
 */
void
ar5212GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf)
{
    A_UINT32    high;           /* Timestamp high order 32 bits */

    tsf->low  = readPlatformReg(pDev, MAC_TSF_L32);
    high      = readPlatformReg(pDev, MAC_TSF_U32);    
    tsf->high = readPlatformReg(pDev, MAC_TSF_U32);
    if (tsf->high != high) {    /* Counter rollover? */
        tsf->low = readPlatformReg(pDev, MAC_TSF_L32);        
    }
}

/******************************************************************
 * ar5212ResetTsf
 *
 * Reset the current hardware tsf for stamlme
 */
void
ar5212ResetTsf(WLAN_DEV_INFO *pDev)
{
    A_REG_SET_BIT(pDev, MAC_BEACON, RESET_TSF);
    /*
     * workaround for hw bug! when resetting the TSF, write twice to the
     * corresponding register; each write to the RESET_TSF bit toggles
     * the internal signal to cause a reset of the TSF - but if the signal
     * is left high, it will reset the TSF on the next chip reset also!
     * writing the bit an even number of times fixes this issue
     */
    A_REG_SET_BIT(pDev, MAC_BEACON, RESET_TSF);
}

/******************************************************************
 * ar5212SetAdhocMode
 *
 * Set adhoc mode for stamlme
 */
void
ar5212SetAdhocMode(WLAN_DEV_INFO *pDev)
{
    A_UINT32 id1 = readPlatformReg(pDev, MAC_STA_ID1);

    writePlatformReg(pDev, MAC_CFG,
                     readPlatformReg(pDev, MAC_CFG) | MAC_CFG_AP_ADHOC_INDICATION);

    /* Update antenna mode */
    id1 &= ~(MAC_STA_ID1_USE_DEFANT | MAC_STA_ID1_DEFANT_UPDATE);
    writePlatformReg(pDev, MAC_STA_ID1, id1 |
                     MAC_STA_ID1_AD_HOC |
                     MAC_STA_ID1_RTS_USE_DEF);

    /* 11g Ad-Hoc always uses long slot time */
    if (IS_CHAN_G(pDev->staConfig.pChannel->channelFlags)) {
        writePlatformReg(pDev, MAC_D_GBL_IFS_SLOT, SLOT_TIME_20);
        PKTLOG_CHANGE_SLOT_TIME(pDev, SLOT_TIME_20);
        pDev->staConfig.shortSlotTime = FALSE;
        pDev->defaultStaConfig.shortSlotTime = FALSE;
        pDev->bssDescr->capabilityInfo.shortSlotTime = 0;
        pDev->localSta->capInfo.shortSlotTime = 0;
        pDev->useShortSlotTime = FALSE;
    }
}

/**************************************************************
 * ar5212SetBasicRate
 *
 * Set or clear hardware basic rate bit
 * Set harfware basic rate set if basic rate is found
 * and basic rate is equal or less than 2Mbps
 */
void
ar5212SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *pSet)
{
    int     i;
    A_UINT8 rset, xset = 0;


    if (IS_CHAN_CCK(pDev->staConfig.phwChannel->channelFlags) == FALSE) {
        return;
    }

    for (i = 0; i < pSet->length; i++) {
        rset = pSet->rates[i];
        if (rset & 0x80) {              /* Basic rate defined? */
            rset = rset & 0x7f;
            xset = (rset >= xset) ? rset : xset;
        }
    }

    /* If basic rate is found to be equal or less than 2Mbps, then set the hw bit */
    if (xset && xset/2 <= 2) {
        /* tell the h/w that the associated AP is running basic rate */
        A_REG_SET_BIT(pDev, MAC_STA_ID1, BASE_RATE_11B);
        return;
    }

    /* tell the h/w that the associated AP is not running basic rate */
    A_REG_CLR_BIT(pDev, MAC_STA_ID1, BASE_RATE_11B);
}

/**************************************************************
 * ar5212GetRandomSeed
 *
 * Grab a semi-random value from hardware registers - may not
 * change often
 */

A_UINT32
ar5212GetRandomSeed(WLAN_DEV_INFO *pDev)
{
    A_UINT32 seed, nf;

    nf = (readPlatformReg(pDev, PHY_BASE+(25<<2)) >> 19) & 0x1ff;
    if (nf & 0x100) {
        nf = 0 - ((nf ^ 0x1ff) + 1);
    }

    seed = readPlatformReg(pDev, MAC_TSF_U32) ^ readPlatformReg(pDev, MAC_TSF_L32) ^
        nf;

    return seed;
}

/**************************************************************
 * ar5212DetectCardPresent
 *
 * Detect if our card is present
 */

A_BOOL
ar5212DetectCardPresent(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
    A_UINT32 value;
    A_UINT16  macVersion, macRev;

    /* Read the SREV register and compare to what we read at initialization */
    value     = readPlatformReg(pDev, MAC_SREV) & MAC_SREV_ID_M;
    macVersion= (A_UINT16) (value >> MAC_SREV_ID_S);
    macRev    = (A_UINT16) (value & MAC_SREV_REVISION_M);

    if ( (pDev->macVersion == macVersion) && (pDev->macRev == macRev) ) {
        return TRUE;
    } else {
        return FALSE;
    }
#else
    return TRUE;                        /* the spirit mac is not going anywhere */
#endif
}

/**************************************************************
 * ar5212MibControl
 *
 * Control of MIB Counters
 */
A_UINT32
ar5212MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext)
{
    switch (cmd) {
    case UPDATE_SW_ALL:
    case UPDATE_SW_COMMON: {
        WLAN_STATS     *pStats  = &pDev->localSta->stats;

        pStats->AckRcvFailures += readPlatformReg(pDev, MAC_ACK_FAIL);
        pStats->RtsFailCnt     += readPlatformReg(pDev, MAC_RTS_FAIL);
        pStats->FcsFailCnt     += readPlatformReg(pDev, MAC_FCS_FAIL);
        pStats->RtsSuccessCnt  += readPlatformReg(pDev, MAC_RTS_OK);
        pStats->CompCPC0Cnt    += readPlatformReg(pDev, MAC_CPC_0);
        pStats->CompCPC1Cnt    += readPlatformReg(pDev, MAC_CPC_1);
        pStats->CompCPC2Cnt    += readPlatformReg(pDev, MAC_CPC_2);
        pStats->CompCPC3Cnt    += readPlatformReg(pDev, MAC_CPC_3);

        if (UPDATE_SW_ALL == cmd) {
            /* Read MIB variables that change less frequently. */
            pStats->RxBeacons += readPlatformReg(pDev, MAC_BEACON_CNT);
        }
        break;
    }

    case CLEAR_ALL_COUNTS: {
        A_UINT32 regVal;

        regVal = readPlatformReg(pDev, MAC_MIBC);
        writePlatformReg(pDev, MAC_MIBC, regVal | MAC_MIBC_CMC);
        writePlatformReg(pDev, MAC_MIBC, regVal & ~MAC_MIBC_CMC);
        break;
    }

    case GET_COUNTERS: {
        HAL_COUNTERS *pCnts;

        if (NULL == pContext) {
            break;
        }

        pCnts               = (HAL_COUNTERS *) pContext;
        pCnts->txFrameCount = readPlatformReg(pDev, MAC_TFCNT);
        pCnts->rxFrameCount = readPlatformReg(pDev, MAC_RFCNT);
        pCnts->rxClearCount = readPlatformReg(pDev, MAC_RCCNT);
        pCnts->cycleCount   = readPlatformReg(pDev, MAC_CCCNT);
        pCnts->txActive     = (readPlatformReg(pDev, MAC_TFCNT) ==
                               pCnts->txFrameCount) ? FALSE : TRUE;
        pCnts->rxActive     = (readPlatformReg(pDev, MAC_RFCNT) ==
                               pCnts->rxFrameCount) ? FALSE : TRUE;
    }

    default:
        break;
    }

    return 0;
}

/*
 * Look up tables for mode dependent clock rate and noise floor
 */
                                     /* 11a Turbo  11b  11g  108g  XR */
static const A_UINT8  CLOCK_RATE[]  = { 40,  80,   22,  44,   88,  40};
static const A_INT16  NOISE_FLOOR[] = {-96, -93,  -98, -96,  -93, -96};

/**************************************************************
 * ar5212GetChannelData
 *
 * Get hw data specific to the current channel
 */
void
ar5212GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
                     HAL_CHANNEL_DATA *pData)
{
    WIRELESS_MODE mode;

    mode = wlanCFlagsToWirelessMode(pDev, pChan->channelFlags);
    ASSERT(mode < WIRELESS_MODE_MAX);

    pData->clockRate    = CLOCK_RATE[mode];
    pData->noiseFloor   = NOISE_FLOOR[mode];
    pData->ccaThreshold = (A_UINT8) A_REG_RD_FIELD(pDev, PHY_CCA, THRESH62);
}

/**************************************************************
 * ar5212ProcessNoiseFloor
 *
 * Take the given channel list and process all valid raw noise floors
 * into the dBm noise floor values.
 *
 * Though our device has no reference for a dBm noise floor, we perform
 * a relative minimization of NF's based on the lowest NF found across a
 * channel scan.
 */
void
ar5212ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList)
{
    int i;
    A_INT16 correct2 = 0, correct5 = 0, lowest2 = 0, lowest5 = 0, tempFinalNf;
    CHAN_VALUES *pChan;
    WIRELESS_MODE mode;

    /* 
     * Find the lowest 2GHz and 5GHz noise floor values after adjusting
     * for statistically recorded NF/channel deviation.
     */
    for (i = 0; i < pCList->listSize; i++) {
        pChan = &(pCList->chanArray[i]);
        mode = wlanCFlagsToWirelessMode(pDev, pChan->channelFlags);
        ASSERT(mode < WIRELESS_MODE_MAX);

        if ( !(pChan->channelFlags & CHANNEL_DONT_SCAN) &&
             (pChan->rawNoiseFloor < 0) )
        {
            if (IS_CHAN_5GHZ(pChan->channelFlags)) {
                tempFinalNf = pChan->rawNoiseFloor + NOISE_FLOOR[mode] + ar5212GetNfAdjust(pDev, pChan);
                if (tempFinalNf < lowest5) { 
                    lowest5 = tempFinalNf;
                    correct5 = NOISE_FLOOR[mode] - (pChan->rawNoiseFloor + ar5212GetNfAdjust(pDev, pChan));
                }
            } else {
                tempFinalNf = pChan->rawNoiseFloor + NOISE_FLOOR[mode];
                if (tempFinalNf < lowest2) { 
                    lowest2 = tempFinalNf;
                    correct2 = NOISE_FLOOR[mode] - pChan->rawNoiseFloor;
                }
            }
        }
    }

    /* Correct the channels to reach the expected NF value */
    for (i = 0; i < pCList->listSize; i++) {
        pChan = &(pCList->chanArray[i]);

        if ( !(pChan->channelFlags & CHANNEL_DONT_SCAN) &&
             (pChan->rawNoiseFloor < 0) )
        {
            /* Apply correction factor */
            if (IS_CHAN_5GHZ(pChan->channelFlags)) {
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + ar5212GetNfAdjust(pDev, pChan) + correct5;
            } else {
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + correct2;
            }
        }
    }
}


/**************************************************************
 * ar5212GetNfAdjust
 *
 * Adjust NF based on statistical values for 5GHz frequencies.
 */
static A_INT16
ar5212GetNfAdjust(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan)
{
    A_UINT16      chan = pChan->channel;
    A_INT16       adjust = 0;
    const A_INT16 *pAdjust;
    const A_INT16 Adjust5112[10] = {1, 0, 2, 4, 5, 7, 8, 9, 10, 11};
    const A_INT16 Adjust5111[10] = {5, 3, 1, 0, 0, 1, 2, 3, 4, 6};

    if (IS_5112(pDev) || IS_2413(pDev)) {
        pAdjust = Adjust5112;
    } else {
        pAdjust = Adjust5111;
    }

    if (chan > 5790) {
        adjust = pAdjust[9];
    } else if (chan > 5730) {
        adjust = pAdjust[8];
    } else if (chan > 5690) {
        adjust = pAdjust[7];
    } else if (chan > 5660) {
        adjust = pAdjust[6];
    } else if (chan > 5610) {
        adjust = pAdjust[5];
    } else if (chan > 5530) {
        adjust = pAdjust[4];
    } else if (chan > 5450) {
        adjust = pAdjust[3];
    } else if (chan >= 5380) {
        adjust = pAdjust[2];
    } else if (chan >= 5210) {
        adjust = pAdjust[1];
    } else {
        adjust = pAdjust[0];
    }

    return adjust;
}

/**************************************************************
 * ar5212IsHwCipherSupported
 *
 * Detect if Cipher requested is implemented in HW.
 */
A_BOOL
ar5212IsHwCipherSupported(WLAN_DEV_INFO *pDev, A_UINT32 keyType)
{
    switch (keyType) {
    case PRIV_KEY_TYPE_WEP:
    case PRIV_KEY_TYPE_TKIP:
    case PRIV_KEY_TYPE_NULL:
        return TRUE;
    case PRIV_KEY_TYPE_AES_CCM:
        /* CCM non-compliant up to rev 3 */
        return (pDev->macRev > 3);
    default:
        return FALSE;
    }
}

/**************************************************************
 * ar5212EnableRadarDetection
 *
 * Enable radar detection in the hardware - may later be changed
 * to modify radar "sensitivity"
 */
void
ar5212EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams)
{
    A_UINT32 val;
    if (pPhyParams) {
        /* New params supplied by caller */
        val = A_FIELD_VALUE(PHY_RADAR_0, FIRPWR, pPhyParams->firpwr) |
              A_FIELD_VALUE(PHY_RADAR_0, RRSSI, pPhyParams->radarRssi) |
              A_FIELD_VALUE(PHY_RADAR_0, HEIGHT, pPhyParams->height) |
              A_FIELD_VALUE(PHY_RADAR_0, PRSSI, pPhyParams->pulseRssi) |
              A_FIELD_VALUE(PHY_RADAR_0, INBAND, pPhyParams->inband);
        writePlatformReg(pDev, PHY_RADAR_0, val);
    }

    /* Now enable pulse detection */
    A_REG_SET_BIT(pDev, PHY_RADAR_0, EN);

    /*
     * Fast antenna diversity for strong signal disturbs radar detection of 1-2 us pulses.
     * Enable antenna diversity but disable strong signal aspect of it.
     */

    /* enable_ant_fast_div = 1 */
    A_REG_SET_BIT(pDev, PHY_CCK_DETECT, BB_ENABLE_ANT_FAST_DIV);
    /* ant_fast_div_gc_limit = 1 */
    A_REG_WR_FIELD(pDev, PHY_RESTART, FAST_DIV_GC, 1);
}

/**************************************************************
 * ar5212EnableFriendlyDetection
 *
 * Enable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
ar5212EnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr)
{
    /* Now enable pulse detection */
    detectionRSSIThr &= 0x3f;    /* only 6 bits available */
    A_REG_RMW_FIELD (pDev, PHY_RADAR_0, RRSSI, detectionRSSIThr);
    A_REG_SET_BIT(pDev, PHY_RADAR_0, EN);
}

/**************************************************************
 * ar5212DisableFriendlyDetection
 *
 * Disable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
ar5212DisableFriendlyDetection(WLAN_DEV_INFO *pDev)
{
    /* Now disable pulse detection */
    A_REG_CLR_BIT(pDev, PHY_RADAR_0, EN);
}

/**************************************************************
 * ar5212AniControl
 *
 * Control Adaptive Noise Immunity Parameters
 */
void
ar5212AniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param)
{
#define TABLE_SIZE(_table) (sizeof(_table)/sizeof((_table)[0]))
    typedef int TABLE[];

    switch (cmd) {
    case SET_NOISE_IMMUNITY_LEVEL: {
        unsigned int level = param;

        if (IS_5312_2_X(pDev)) {
            /* for Freedom, whose sigma-delta ADC has higher noise */
            const TABLE totalSizeDesired = { -41, -41, -48, -48, -48 };
            const TABLE coarseHigh       = { -18, -18, -16, -14, -12 };
            const TABLE coarseLow        = { -56, -56, -60, -60, -60 };
            const TABLE firpwr           = { -72, -72, -75, -78, -80 };

            ASSERT(level < TABLE_SIZE(totalSizeDesired));

            A_REG_RMW_FIELD(pDev, PHY_DESIRED_SZ, TOT_DES, totalSizeDesired[level]);
            A_REG_RMW_FIELD(pDev, PHY_AGC_CTL1, COARSE_HIGH, coarseHigh[level]);
            A_REG_RMW_FIELD(pDev, PHY_AGC_CTL1, COARSE_LOW, coarseLow[level]);
            A_REG_RMW_FIELD(pDev, PHY_FIND_SIG, FIRPWR, firpwr[level]);
        } else {
            const TABLE totalSizeDesired = { -55, -55, -55, -55, -62 };
            const TABLE coarseHigh       = { -14, -14, -14, -14, -12 };
            const TABLE coarseLow        = { -64, -64, -64, -64, -70 };
            const TABLE firpwr           = { -78, -78, -78, -78, -80 };

            ASSERT(level < TABLE_SIZE(totalSizeDesired));

            A_REG_RMW_FIELD(pDev, PHY_DESIRED_SZ, TOT_DES, totalSizeDesired[level]);
            A_REG_RMW_FIELD(pDev, PHY_AGC_CTL1, COARSE_HIGH, coarseHigh[level]);
            A_REG_RMW_FIELD(pDev, PHY_AGC_CTL1, COARSE_LOW, coarseLow[level]);
            A_REG_RMW_FIELD(pDev, PHY_FIND_SIG, FIRPWR, firpwr[level]);
        }

        break;
    }

    case SET_OFDM_WEAK_SIGNAL_DETECTION: {
        const TABLE m1ThreshLow   = { 127,   50 };
        const TABLE m2ThreshLow   = { 127,   40 };
        const TABLE m1Thresh      = { 127, 0x4d };
        const TABLE m2Thresh      = { 127, 0x40 };
        const TABLE m2CountThr    = {  31,   16 };
        const TABLE m2CountThrLow = {  63,   48 };

        unsigned int on = param ? 1 : 0;

        A_REG_RMW_FIELD(pDev, PHY_SFCORR_LOW, M1_THRESH_LOW, m1ThreshLow[on]);
        A_REG_RMW_FIELD(pDev, PHY_SFCORR_LOW, M2_THRESH_LOW, m2ThreshLow[on]);
        A_REG_RMW_FIELD(pDev, PHY_SFCORR, M1_THRESH, m1Thresh[on]);
        A_REG_RMW_FIELD(pDev, PHY_SFCORR, M2_THRESH, m2Thresh[on]);
        A_REG_RMW_FIELD(pDev, PHY_SFCORR, M2COUNT_THR, m2CountThr[on]);
        A_REG_RMW_FIELD(pDev, PHY_SFCORR_LOW, M2COUNT_THR_LOW, m2CountThrLow[on]);

        if (on) {
            A_REG_SET_BIT(pDev, PHY_SFCORR_LOW, USE_SELF_CORR_LOW);
        } else {
            A_REG_CLR_BIT(pDev, PHY_SFCORR_LOW, USE_SELF_CORR_LOW);
        }

        break;
    }

    case SET_CCK_WEAK_SIGNAL_THR: {
        const TABLE weakSigThrCck = { 8, 6 };

        unsigned int high = param ? 1 : 0;

        A_REG_RMW_FIELD(pDev, PHY_CCK_DETECT, WEAK_SIG_THR_CCK, weakSigThrCck[high]);

        break;
    }

    case SET_FIRSTEP_LEVEL: {
        const TABLE firstep = { 0, 4, 8 };

        unsigned int level = param;

        ASSERT(level < TABLE_SIZE(firstep));

        A_REG_RMW_FIELD(pDev, PHY_FIND_SIG, FIRSTEP, firstep[level]);

        break;
    }

    case SET_SPUR_IMMUNITY_LEVEL: {
        const TABLE cycpwrThr1 = { 2, 4, 6, 8, 10, 12, 14, 16 };

        unsigned int level = param;

        ASSERT(level < TABLE_SIZE(cycpwrThr1));

        A_REG_RMW_FIELD(pDev, PHY_TIMING5, CYCPWR_THR1, cycpwrThr1[level]);

        break;
    }

    default:
        ASSERT(0);
    }

    return;
}

/**************************************************************
 * ar5212AniGetListenTime
 *
 * Get listening time for Adaptive Noise Immunity
 * Return -1 if counter wrap-around occurs
 */

/* convert HW counter values to ms using 11g clock rate, good enough for 11a and Turbo */
#define CLOCK_RATE  44000

A_INT32
ar5212AniGetListenTime(WLAN_DEV_INFO *pDevInfo)
{
    A_INT32  listenTime;                    // return value (-1 means invalid)

    A_UINT32 txFrameCount = A_REG_RD(pDevInfo, MAC_TFCNT);
    A_UINT32 rxFrameCount = A_REG_RD(pDevInfo, MAC_RFCNT);
    A_UINT32 cycleCount   = A_REG_RD(pDevInfo, MAC_CCCNT); // must read this last!

    if (pDevInfo->cycleCount == 0 || pDevInfo->cycleCount > cycleCount) {
        listenTime = -1; // return invalid if first call or wrap-around
    } else {
        listenTime = (cycleCount - txFrameCount - rxFrameCount - pDevInfo->cycleCount
            + pDevInfo->txFrameCount + pDevInfo->rxFrameCount) / CLOCK_RATE;
    }

    pDevInfo->cycleCount   = cycleCount;    // update reg values
    pDevInfo->txFrameCount = txFrameCount;
    pDevInfo->rxFrameCount = rxFrameCount;

    return listenTime;
}

static INLINE void
ar5212DumpRegSet(WLAN_DEV_INFO* pDev, int first, int last)
{
    int i;

    for (i = first; i <= last; i += 4) {
        uiPrintf("=== 0x%04X: 0x%08lX\n", i, readPlatformReg(pDev, i));
    }
}

/**************************************************************
 * ar5212DumpRegisters
 *
 * Print out a bunch of HW registers.  DO NOT CALL FROM ISR ON AP.
 */
void
ar5212DumpRegisters(WLAN_DEV_INFO *pDev)
{
    uiPrintf("MAC Registers\n");
    ar5212DumpRegSet(pDev, 0x0008, 0x00b4);
    uiPrintf("\nQCU Registers\n");
    ar5212DumpRegSet(pDev, 0x0800, 0x0a40);
    uiPrintf("\nDCU Registers\n");
    ar5212DumpRegSet(pDev, 0x1000, 0x10F0);
    ar5212DumpRegSet(pDev, 0x1230, 0x1230);
    uiPrintf("\nPCI Registers\n");
    ar5212DumpRegSet(pDev, 0x4000, 0x4030);
    uiPrintf("\nEeprom Registers\n");
    ar5212DumpRegSet(pDev, 0x6000, 0x6010);
    uiPrintf("\nPCU Registers\n");
    ar5212DumpRegSet(pDev, 0x8000, 0x8058);
    uiPrintf("\nBB Registers\n");
    ar5212DumpRegSet(pDev, 0x9800, 0x9878);
    ar5212DumpRegSet(pDev, 0x9900, 0x995C);
    ar5212DumpRegSet(pDev, 0x9C00, 0x9C1C);
}

/******************************************************************
 * ar5212GetCurRssi
 *
 * Get the rssi of frame curently being received.
 */
A_RSSI32
ar5212GetCurRssi(WLAN_DEV_INFO *pDev)
{
    A_INT8 rssi = (A_INT8)(readPlatformReg(pDev, PHY_CURRENT_RSSI) & 0xff);
    return (A_INT32)(rssi < 0 ? 0 : rssi);
}

A_UINT32
ar5212GetDefAntenna(WLAN_DEV_INFO *pDev)
{
    return (readPlatformReg(pDev, MAC_DEF_ANTENNA) & 0x7);
}

void
ar5212SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna)
{
    writePlatformReg(pDev, MAC_DEF_ANTENNA, (antenna & 0x7));
    /*
     * This section only needed by fast diversity code.
     * Assumed to be protected by the caller of ar5212SetDefAntenna()
     * if necessary.
     */
    pDev->cachedDefAnt = (A_UINT8)(antenna & 0x7);
    pDev->countOtherRxAnt = 0;
}

void
ar5212SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                       CHAN_VALUES *pChval)
{
    A_UINT32        antSwitchA,
                    antSwitchB;
    EEP_HEADER_INFO *pHeaderInfo    = pDev->pHalInfo->pEepData->pEepHeader;
    WLAN_STA_CONFIG *pConfig        = &pDev->staConfig;
    int             arrayMode       = 0;

    switch (pChval->channelFlags & CHANNEL_ALL) {
    case CHANNEL_A:
    case CHANNEL_T:
    case CHANNEL_XR_A:
    case CHANNEL_XR_T:
        arrayMode = 0;
        break;
    case CHANNEL_B:
        arrayMode = 1;
        break;
    case CHANNEL_G:
    case CHANNEL_108G:
    case CHANNEL_XR_G:
        arrayMode = 2;
        break;
    default:
        ASSERT(0);
        break;
    }

    antSwitchA =  pHeaderInfo->antennaControl[1][arrayMode]        |
                 (pHeaderInfo->antennaControl[2][arrayMode] << 6)  |
                 (pHeaderInfo->antennaControl[3][arrayMode] << 12) |
                 (pHeaderInfo->antennaControl[4][arrayMode] << 18) |
                 (pHeaderInfo->antennaControl[5][arrayMode] << 24);
    antSwitchB =  pHeaderInfo->antennaControl[6][arrayMode]        |
                 (pHeaderInfo->antennaControl[7][arrayMode] << 6)  |
                 (pHeaderInfo->antennaControl[8][arrayMode] << 12) |
                 (pHeaderInfo->antennaControl[9][arrayMode] << 18) |
                 (pHeaderInfo->antennaControl[10][arrayMode] << 24);

    /*
     * For fixed antenna, give the same setting for both switch banks
     */
    if (ANTENNA_FIXED_A == settings) {
        antSwitchB = antSwitchA;
    } else if (ANTENNA_FIXED_B == settings) {
        antSwitchA = antSwitchB;
    } else {
        ASSERT(ANTENNA_CONTROLLABLE == settings);
    }
    pConfig->diversityControl = settings;

    writePlatformReg(pDev, ANT_SWITCH_TABLE1, antSwitchA);
    writePlatformReg(pDev, ANT_SWITCH_TABLE2, antSwitchB);
}

void
ar5212UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt)
{
    struct TxRateCtrl_s *pRc;

    pRc = &pSib->txRateCtrl;

    /* curTxAnt is determined from tx descriptor on Venice */
    pSib->stats.AntCnt[curTxAnt]++;

    if (pSib->antTx != curTxAnt) {
        /*
         * Hw does AABBAA on transmit attempts, and has flipped on this transmit.
         */
        pSib->antTx = curTxAnt; /* 0 or 1 */
        pSib->stats.AntSwCnt++;
        pRc->antFlipCnt = 1;

#ifndef BUILD_AP
        /*
         * Update rx ant (default) to this transmit antenna if:
         *   1. The very first try on the other antenna succeeded and
         *      with a very good ack rssi.
         *   2. Or if we find ourselves succeeding for RX_FLIP_THRESHOLD
         *      consecutive transmits on the other antenna;
         * NOTE that the default antenna is preserved across a chip reset
         * by the hal software
         */
        if (!pDev->useFastDiversity &&
            retries == 2
            && rssiAck >= pRc->rssiLast + 2)
        {
            ar5212SetDefAntenna(pDev, curTxAnt ? 2 : 1);
        }
    } else {
        if (!pDev->useFastDiversity &&
            pRc->antFlipCnt < RX_FLIP_THRESHOLD)
        {
            pRc->antFlipCnt++;
            if (pRc->antFlipCnt == RX_FLIP_THRESHOLD) {
                ar5212SetDefAntenna(pDev, curTxAnt ? 2 : 1);
            }
        }
#endif
    }
}

/*
 * ar5212UseShortSlotTime - set short time 'en' != 'prev' and
 *                          9 us if 'en' == TRUE else 20 us.
 *                          Returns 'en'.
 */
A_BOOL
ar5212UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev)
{
    WLAN_CFLAGS cf = pDev->staConfig.pChannel->channelFlags;

    if ((IS_CHAN_G(cf) || IS_CHAN_XR_G(cf)) && en != prev) {
        if (en) {
            writePlatformReg(pDev, MAC_D_GBL_IFS_SLOT, SLOT_TIME_09);
            PKTLOG_CHANGE_SLOT_TIME(pDev, SLOT_TIME_09);
        } else {
            writePlatformReg(pDev, MAC_D_GBL_IFS_SLOT, SLOT_TIME_20);
            PKTLOG_CHANGE_SLOT_TIME(pDev, SLOT_TIME_20);
        }
    }
    return en;
}

void
ar5212DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose)
{
    if (!verbose) {
        return;
    }

#ifdef BUILD_AP
    isrPrintf("Dma Debug Reg 0 0x%08x    Reg 1 0x%08x\n", 
              A_REG_RD(pDev, MAC_DMADBG_0),A_REG_RD(pDev, MAC_DMADBG_1));
    isrPrintf("Dma Debug Reg 2 0x%08x    Reg 3 0x%08x\n", 
              A_REG_RD(pDev, MAC_DMADBG_2),A_REG_RD(pDev, MAC_DMADBG_3));
    isrPrintf("Dma Debug Reg 4 0x%08x    Reg 5 0x%08x\n", 
              A_REG_RD(pDev, MAC_DMADBG_4),A_REG_RD(pDev, MAC_DMADBG_5));
    isrPrintf("Dma Debug Reg 6 0x%08x    Reg 7 0x%08x\n", 
              A_REG_RD(pDev, MAC_DMADBG_6),A_REG_RD(pDev, MAC_DMADBG_7));
#else
    uiPrintf("Dma Debug Reg 0 0x%08x    Reg 1 0x%08x\n", 
             A_REG_RD(pDev, MAC_DMADBG_0),A_REG_RD(pDev, MAC_DMADBG_1));
    uiPrintf("Dma Debug Reg 2 0x%08x    Reg 3 0x%08x\n", 
             A_REG_RD(pDev, MAC_DMADBG_2),A_REG_RD(pDev, MAC_DMADBG_3));
    uiPrintf("Dma Debug Reg 4 0x%08x    Reg 5 0x%08x\n", 
             A_REG_RD(pDev, MAC_DMADBG_4),A_REG_RD(pDev, MAC_DMADBG_5));
    uiPrintf("Dma Debug Reg 6 0x%08x    Reg 7 0x%08x\n", 
             A_REG_RD(pDev, MAC_DMADBG_6),A_REG_RD(pDev, MAC_DMADBG_7));
#endif
}

/**************************************************************
 * ar5212SetSleepRegisters
 *
 * If slow clock exists, use it to lower power consumption during sleep.
 * Set sleep registers correctly for refclk or slow clock operation as
 * they are not all set by the INI.
 * Note: If sleep clock is set to slow mode, delays on accessing certain
 *       baseband registers (27-31, 124-127) are required.
 *
 * enSlowClockSleep: select whether to use slowClock sleeping or refclk
 */
void
ar5212SetSleepRegisters(WLAN_DEV_INFO *pDev, A_BOOL enSlowClockSleep)
{
    if (ar5212IsSlowSleepClockAllowed(pDev) && enSlowClockSleep) {
        /*
         * If this card has an external 32 KHz crystal,
         * enable clocks to be turned OFF in BB during sleep
         * and also enable turning OFF 32MHz/40MHz Refclk
         * from A2.
         */
        A_REG_WR(pDev, PHY_SLEEP_CTR_CONTROL, 0x1f); //reg 28 =  9870
        A_REG_WR(pDev, PHY_M_SLEEP,           0x03); //reg 124 = 99f0

        /* Set USEC32 to 1 */
        A_REG_RMW_FIELD(pDev, MAC_USEC, 32, 1);
        /* Set TSF Increment for 32 KHz */
        A_REG_WR_FIELD(pDev, MAC_TSF_PARM, INCREMENT, 61);

#if defined(PCI_INTERFACE)
        /* # Set sleep clock rate to 32 KHz. */
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_SEL, 1);
#endif

        /* Set power down sequence for 5112/2413 */
        A_REG_WR(pDev, PHY_REFCLKPD, 14);

        if (IS_2413(pDev)) {
            A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x34);
            A_REG_WR(pDev, PHY_SLEEP_SCAL,        0x0d);
            A_REG_WR(pDev, PHY_REFCLKDLY,         0xA0);
#if defined(PCI_INTERFACE)
            A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_RATE_IND, 2);
#endif
        } else if (IS_5112(pDev)) {
            A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x0d); //reg 29 =  9874
            A_REG_WR(pDev, PHY_SLEEP_SCAL,        0x0c); //reg 30 =  9878
            A_REG_WR(pDev, PHY_REFCLKDLY,         0x05); //reg 125 = 99f4
#if defined(PCI_INTERFACE)
            A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_RATE_IND, 0x3);
#endif
        } else {
            uiPrintf("Slow sleep clock is not supported with this device\n");
            ASSERT(0);
        }
    } else {
        /* Set sleep clock rate back to refclk. */
#if defined(PCI_INTERFACE)
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_RATE_IND, 0);
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_SEL, 0);
#endif

        /* Set TSF Increment for refclk */
        A_REG_WR_FIELD(pDev, MAC_TSF_PARM, INCREMENT, 1);
        if (IS_2413(pDev) || IS_5112(pDev)) {
            A_REG_RMW_FIELD(pDev, MAC_USEC, 32, 39);
        } else {
            A_REG_RMW_FIELD(pDev, MAC_USEC, 32, 31);
        }

        /*
         * Restore BB registers to power-on defaults
         */
        A_REG_WR(pDev, PHY_SLEEP_CTR_CONTROL, 0x1f);
        A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x7f);

        if (IS_5312_2_X(pDev)) {
            /* Set ADC/DAC select values */
            A_REG_WR(pDev, PHY_SLEEP_SCAL, 0x04);

        } else {
            A_REG_WR(pDev, PHY_SLEEP_SCAL, 0x0e);
            /*
             * WAR - 99f0, 99f4, 99f8 cannot be written in 5312 as they alias 
             * to other register addresses.
             */
            A_REG_WR(pDev, PHY_M_SLEEP,           0x0c);
            A_REG_WR(pDev, PHY_REFCLKDLY,         0xff);
            if (IS_2413(pDev) || IS_5112(pDev)) {
                A_REG_WR(pDev, PHY_REFCLKPD, 14);
            } else {
                A_REG_WR(pDev, PHY_REFCLKPD, 18);
            }
        }
    }
}

/**************************************************************
 * ar5212IsSlowSleepClockAllowed
 *
 * If slow clock exists, turn it off and turn back on the refclk
 *
 * Bug 9975: Enable the slow sleep clock only when the hardware
 * can goto sleep. Currently hardware can goto sleep when 
 * power save mode is enabled or radio is disabled
 */
static A_BOOL
ar5212IsSlowSleepClockAllowed(WLAN_DEV_INFO *pDev)
{
#ifndef BUILD_AP
    struct eepMap   *pEepData = pDev->pHalInfo->pEepData;

    if ( ((pDev->staConfig.sleepMode != POWERMGT_CAM) ||
         (pDev->rfSilent.swRadioDisable == TRUE) ||
         (pDev->rfSilent.hwRadioDisable == TRUE)) &&
         ( (pDev->staConfig.enable32KHzClock == USE_32KHZ) ||
         ((pDev->staConfig.enable32KHzClock == AUTO_32KHZ) &&
         pEepData->pEepHeader->exist32kHzCrystal) ) )
    {
        return TRUE;
    }
#endif
    return FALSE;
}

/**************************************************************
 * ar5212SwapHwDesc
 *
 * Swap hardware fields in the descriptor for compression
 */
void
ar5212SwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete)
{

    if (pDev->pHalInfo->swSwapDesc) {
        ATHEROS_DESC        *pTempDesc  = pDesc;
        ATHEROS_DESC        *pFirstDesc = pDesc;

        do {
            pTempDesc = pFirstDesc;
            do {
                if (!complete) {
                    pTempDesc->nextPhysPtr = cpu2le32(pTempDesc->nextPhysPtr);
                }
                pTempDesc->bufferPhysPtr = cpu2le32(pTempDesc->bufferPhysPtr);
                pTempDesc->hw.word[0]    = cpu2le32(pTempDesc->hw.word[0]);
                pTempDesc->hw.word[1]    = cpu2le32(pTempDesc->hw.word[1]);
                pTempDesc->hw.word[2]    = cpu2le32(pTempDesc->hw.word[2]); 
                pTempDesc->hw.word[3]    = cpu2le32(pTempDesc->hw.word[3]); 
                if (complete) {
                    pTempDesc->hw.word[4]    = cpu2le32(pTempDesc->hw.word[4]); 
                    pTempDesc->hw.word[5]    = cpu2le32(pTempDesc->hw.word[5]);
                }
                pTempDesc = pTempDesc->pNextVirtPtr;
            } while (pTempDesc != pFirstDesc->pTxLastDesc->pNextVirtPtr);
            pFirstDesc = pFirstDesc->pTxLastDesc->pNextVirtPtr;
        } while (pFirstDesc != pTail->pNextVirtPtr);
    }    
}

/**************************************************************
 * ar5212GetHwDescWord
 *
 * Get hardware specified word in the descriptor for compression
 */
A_UINT32
ar5212GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord)
{
    return (pDev->pHalInfo->swSwapDesc ? cpu2le32(hwDescWord) : hwDescWord);
}


/**************************************************************
 * ar5212GetApSwitchHelper
 *
 * Read some Beacon relevant registers in anticipation of saving
 * them for future restoration.
 */
void
ar5212GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    if (pRegs) {
        pRegs->r1 = readPlatformReg(pDev, MAC_TIMER0);
        pRegs->r2 = readPlatformReg(pDev, MAC_TIMER1);
        pRegs->r3 = readPlatformReg(pDev, MAC_TIMER2);
        pRegs->r4 = readPlatformReg(pDev, MAC_QUIET1);
        pRegs->r5 = readPlatformReg(pDev, MAC_QUIET2);
    }

    /* TODO: temp for debug */
    ar5212MacStop(pDev);
}

/**************************************************************
 * ar5212SetApSwitchHelper
 *
 * Restore some Beacon relevant registers.
 */
void
ar5212SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    ASSERT(pRegs);

    writePlatformReg(pDev, MAC_TIMER0, pRegs->r1);
    writePlatformReg(pDev, MAC_TIMER1, pRegs->r2);
    writePlatformReg(pDev, MAC_TIMER2, pRegs->r3);
    writePlatformReg(pDev, MAC_QUIET1, pRegs->r4);
    writePlatformReg(pDev, MAC_QUIET2, pRegs->r5);

    /* Enable SWBA interrupt. */
    ar5212EnableInterrupts(pDev, HAL_INT_SWBA);
    ar5212EnableInterrupts(pDev, HAL_INT_TXDESC);

    /* Set the Beacon Control register */
    writePlatformReg(pDev, MAC_BEACON, pDev->bssDescr->beaconInterval | MAC_BEACON_EN);
}

/*
 * Send a double chirp. Write a zero followed by one. 
 * Couple of gotchas with the double chirp:
 *   - There cannot be any other activity (like sending normal probes
 *     while sending double chirps, infact the transmit queue has to
 *     be empty before sending double chirp. This is because the chirp
 *     could stomp the packets in the queue
 *
 *  - we don't have to ensure that the register will go back to zero,
 *    at the end of the routine (because the h/w would ensure this)
 *
 *  - if the caller calls this function to send double chirp in
 *    succession, h/w is going to ignore the second double chirp
 *
 *  - Also, as the assert in the code indicates that the chip
 *    cannot be put in polled mode for sending the double chirp
 *
 */ 
void
ar5212SendXrChirp(WLAN_DEV_INFO *pDev)
{
#if 0
    unsigned int x;
    int wait = 2;
#endif

    ASSERT((readPlatformReg(pDev, MAC_XRMODE) & MAC_XRMODE_XR_WAIT_FOR_POLL) == 0);

    if (A_REG_RD(pDev, MAC_XRCRP_SEND_CHIRP) == 1) {
        uiPrintf("ar5212SendXrChirp: double chirp in progress, send chirp ignored \n");
        return;
    }

    A_REG_RMW(pDev, MAC_XRCRP, MAC_XRCRP_SEND_CHIRP, 0);
    A_REG_RMW(pDev, MAC_XRCRP, MAC_XRCRP_SEND_CHIRP, 1);

#if 0
    /* wait for 400 microseconds to make sure that a chirp has been sent */
    do {
        
	udelay(200);
        x = readPlatformReg(pDev, MAC_XRCRP);

    } while (--wait && ((x & 1) == 1));

    ASSERT((x & 1) == 0);
#endif
}

#endif /* #ifdef BUILD_AR5212 */
