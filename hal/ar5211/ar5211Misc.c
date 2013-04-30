/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific device miscellaneous functions including hardware queries,
 *  EEPROM routines, gpio funcs, beacon creation, ...
 */

#ifdef BUILD_AR5211

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Misc.c#3 $"

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "halApi.h"
#include "hal.h"
#include "ui.h"
#include "halUtil.h"
#include "wlanchannel.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Misc.h"
#include "ar5211Reset.h"
#include "ar5211Power.h"
#include "ar5211Interrupts.h"
#include "ar5211.h"

#if defined(AR531X)
#include "ar531xreg.h"
#endif

static A_STATUS
ar5211EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data);

static A_INT16
ar5211GetNfAdjust(CHAN_VALUES *pChan);

/**************************************************************
 * ar5211SetRegulatoryDomain
 *
 * Attempt to change the cards operating regulatory domain to the given value
 * Returns: A_EINVAL for an unsupported regulatory domain.
 *          A_HARDWARE for an unwritable EEPROM or bad EEPROM version
 */
A_STATUS
ar5211SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain)
{
    A_STATUS status = A_OK;
    EEP_MAP *pEep;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pEep = pDev->pHalInfo->pEepData;
    if (pDev->pHalInfo->halCapabilities.halRegDmn != regDomain) {
        if (!(pEep->protect & EEPROM_PROTECT_WP_128_191))
        {
            if(wlanIsRegCcValid(pDev)) {
                status = ar5211EepromWrite(pDev, REGULATORY_DOMAIN_OFFSET, regDomain);
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
 * ar5211GetRfKill
 *
 * Accessor to get rfkill info from private EEPROM structures
 */
A_BOOL
ar5211GetRfKill(WLAN_DEV_INFO *pDev)
{
    A_UINT16 rfsilent;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    if (pDev->pHalInfo->pEepData->pEepHeader->rfKill) {
        ar5211EepromRead(pDev, EEPROM_RFSILNT_CLKEN_OFFSET, &rfsilent);
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
 * ar5211GetMacAddr
 *
 * Attempt to get the MAC address from the wireless EEPROM
 * Returns: A_HARDWARE for an unreadable EEPROM
 */
A_STATUS
ar5211GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac)
{
#if defined(PCI_INTERFACE)
    A_UINT32 total = 0;
    A_UINT16 half;

    if (ar5211EepromRead(pDev, EEPROM_MAC_MSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[1] = half & 0xff;
    mac->octets[0] = half >> 8;

    if (ar5211EepromRead(pDev, EEPROM_MAC_MID_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[3] = half & 0xff;
    mac->octets[2] = half >> 8;
    if (ar5211EepromRead(pDev, EEPROM_MAC_LSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[5] = half & 0xff;
    mac->octets[4] = half >> 8;

    if ((total == 0) || total == (3 * 0xffff)) {
        uiPrintf("ar5211GetMacAddr: EEPROM MAC is all 0's or all F's\n");
        return A_HARDWARE;
    }
    return A_OK;

ee_error:
    uiPrintf("ar5211GetMacAddr: EEPROM read failed\n");
    return A_HARDWARE;
#elif defined(AR531X)
    /* The ar531x may store the WLAN mac addr in the system board config */
    if (pDev->devno == 0) {
        A_BCOPY(sysBoardData.wlan0Mac, mac->octets, 6);
    } else {
        A_BCOPY(sysBoardData.wlan1Mac, mac->octets, 6);
    }
    return A_OK;
#endif
}

/**************************************************************
 * ar5211EepromRead
 *
 * Read 16 bits of data from offset into *data
 */
A_STATUS
ar5211EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 status;
    int to = 10000;     /* 10ms timeout */

    A_REG_WR(pDev, MAC_EEPROM_ADDR, offset);
    A_REG_WR(pDev, MAC_EEPROM_CMD, MAC_EEPROM_CMD_READ);

    while (to > 0) {
        udelay(1);
        status = A_REG_RD(pDev, MAC_EEPROM_STS);
        if (status & MAC_EEPROM_STS_READ_COMPLETE) {
            status = readPlatformReg(pDev, MAC_EEPROM_DATA);
            *data = (A_UINT16)(status & 0xffff);
            return A_OK;
        }

        if (status & MAC_EEPROM_STS_READ_ERROR) {
            uiPrintf("ar5211EepromRead: eeprom read error at offset %d.\n", offset);
            return A_HARDWARE;
        }
        to--;
    }
    uiPrintf("ar5211EepromRead: eeprom read timeout at offset %d.\n", offset);
    return A_HARDWARE;
#elif defined(AR531X)
    /* radio configuration data is stored in the system flash */
    *data = sysFlashConfigRead(FLC_RADIOCFG, (offset * 2) + 1) |
            (sysFlashConfigRead(FLC_RADIOCFG, offset * 2) << 8);
    return A_OK;
#endif /* PLATFORM */
}

/**************************************************************
 * ar5211EepromWrite
 *
 * Write 16 bits of data from data into offset
 */
A_STATUS
ar5211EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 status;
    int to = 15000;     /* 15ms timeout */

    /* Send write data */
    A_REG_WR(pDev, MAC_EEPROM_ADDR, offset);
    A_REG_WR(pDev, MAC_EEPROM_DATA, data);
    A_REG_WR(pDev, MAC_EEPROM_CMD, MAC_EEPROM_CMD_WRITE);

    while (to > 0) {
        udelay(1);
        status = A_REG_RD(pDev, MAC_EEPROM_STS);
        if (status & MAC_EEPROM_STS_WRITE_COMPLETE) {
            return A_OK;
        }

        if (status & MAC_EEPROM_STS_WRITE_ERROR)    {
            uiPrintf("ar5211EepromWrite: eeprom write error at offset %d.\n", offset);
            return A_HARDWARE;;
        }
        to--;
    }

    uiPrintf("ar5211EepromWrite: eeprom write timeout at offset %d.\n", offset);
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
 * ar5211EnableRfKill
 *
 * Called if RfKill is supported (according to EEPROM).  Set the interrupt and
 * GPIO values so the ISR can disable RF on a switch signal.  Assumes GPIO port
 * and interrupt polarity are set prior to call.
 */
void
ar5211EnableRfKill(WLAN_DEV_INFO *pDev)
{
    /* TODO - can this really be above the hal on the GPIO interface for
     * TODO - the client only?
     */
#if !defined(BUILD_AP)  /* AP uses GPIOs for LEDs and buttons */

    /* Configure the desired GPIO port for input and enable baseband rf silence */
    ar5211GpioCfgInput(pDev, pDev->rfSilent.gpioSelect);
    A_REG_RMW(pDev, PHY_BASE, 0x00002000, 0);

    /*
     * If radio disable switch connection to GPIO bit x is enabled
     * program GPIO interrupt.
     * If rfkill bit on eeprom is 1, setupeeprommap routine has already
     * verified that it is a later version of eeprom, it has a place for
     * rfkill bit and it is set to 1, indicating that GPIO bit x hardware
     * connection is present.
     */

    if (pDev->rfSilent.polarity == ar5211GpioGet(pDev, pDev->rfSilent.gpioSelect)) {
        /* switch already closed, set to interrupt upon open */
        ar5211GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, !pDev->rfSilent.polarity);
    } else {
        ar5211GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, pDev->rfSilent.polarity);
    }
#endif /* !BUILD_AP */
}

/**************************************************************
 * ar5211GpioCfgOutput
 *
 * Configure GPIO Output lines
 */
void
ar5211GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg  = A_REG_RD(pDev, MAC_GPIOCR);
    reg |= MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT);

    A_REG_WR(pDev, MAC_GPIOCR, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5211GpioCfgInput
 *
 * Configure GPIO Input lines
 */
void
ar5211GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg  = A_REG_RD(pDev, MAC_GPIOCR);
    reg &= ~(MAC_GPIOCR_0_CR_A << (gpio * MAC_GPIOCR_CR_SHIFT));
    reg |= MAC_GPIOCR_0_CR_N << (gpio * MAC_GPIOCR_CR_SHIFT);

    A_REG_WR(pDev, MAC_GPIOCR, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5211GpioSet
 *
 * Once configured for I/O - set output lines
 */
void
ar5211GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg =  A_REG_RD(pDev, MAC_GPIODO);
    reg &= ~(1 << gpio);
    reg |= (val&1) << gpio;

    A_REG_WR(pDev, MAC_GPIODO, reg);
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5211GpioGet
 *
 * Once configured for I/O - get input lines
 */
A_UINT32
ar5211GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
{
#if defined(PCI_INTERFACE)
    A_UINT32 reg;

    ASSERT(gpio < MAC_NUM_GPIO);

    reg = A_REG_RD(pDev, MAC_GPIODI);
    reg = ((reg & MAC_GPIOD_MASK) >> gpio) & 0x1;

    return reg;
#else /* !PCI_INTERFACE */
    return 0;
#endif /* PCI_INTERFACE */
}

/**************************************************************
 * ar5211GpioSetIntr
 *
 * Set the desired GPIO Interrupt
 */
void
ar5211GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel)
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
    A_REG_WR(pDev, MAC_GPIOCR, reg);
    /* change the interrupt mask */
    A_REG_SET_BIT(pDev, MAC_IMR, GPIO);
    pDev->MaskReg = readPlatformReg(pDev, MAC_IMR);
#endif /* PCI_INTERFACE */
}

/******************************************************************
 * ar5211SetLedState
 *
 * Change the LED blinking pattern to correspond to the connectivity
 */
void
ar5211SetLedState(WLAN_DEV_INFO *pDev, A_BOOL bConnected)
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
 * ar5211WriteAssocid - Change association related fields programmed into the hardware.
 *
 * Writing a valid BSSID to the hardware effectively enables the hardware
 * to synchronize its TSF to the correct beacons and receive frames coming
 * from that BSSID. It is called by the SME JOIN operation.
 */
void
ar5211WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset)
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
 * ar5211SetStaBeaconTimers
 *
 *  Sets all the beacon related bits on the h/w for stations
 *  i.e. initializes the corresponding h/w timers;
 *  also tells the h/w whether to anticipate PCF beacons
 */
void
ar5211SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers)
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
    ar5211DisableInterrupts(pDev, HAL_INT_GLOBAL);
    pDev->MaskReg &= ~MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);

    /* Clear any pending BMISS interrupt so far */
    pDev->globISRReg &= ~MAC_IMR_BMISS;

    A_REG_RMW_FIELD(pDev, MAC_RSSI_THR, BM_THR, pTimers->bmissThreshold);

    /* Now enable the interrupt */
    pDev->MaskReg |= MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);
    ar5211EnableInterrupts(pDev, HAL_INT_GLOBAL);

    /*
     * Set the sleep duration register - in 128us units
     */
#if defined(PCI_INTERFACE)
#define SLEEP_SLOP 3  /* in TU */
    pTimers->sleepDuration -= SLEEP_SLOP;
    A_REG_RMW_FIELD(pDev, MAC_SCR, SLDUR, pTimers->sleepDuration * 8);
#endif
}

/******************************************************************
 * ar5211GetTsf
 *
 * Get the current hardware tsf for stamlme
 */
void
ar5211GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf)
{
    A_UINT32    high;           /* Timestamp high order 32 bits */
    
    tsf->low  = A_REG_RD(pDev, MAC_TSF_L32);
    high      = A_REG_RD(pDev, MAC_TSF_U32);    
    tsf->high = A_REG_RD(pDev, MAC_TSF_U32);
    if (tsf->high != high) {    /* Counter rollover? */
        tsf->low = A_REG_RD(pDev, MAC_TSF_L32);
    }
}

/******************************************************************
 * ar5211ResetTsf
 *
 * Reset the current hardware tsf for stamlme
 */
void
ar5211ResetTsf(WLAN_DEV_INFO *pDev)
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
 * ar5211SetAdhocMode
 *
 * Set adhoc mode for stamlme
 */
void
ar5211SetAdhocMode(WLAN_DEV_INFO *pDev)
{
    A_REG_SET_BIT(pDev, MAC_CFG, AP_ADHOC_INDICATION);

    /* Update antenna mode */
    A_REG_SET_BIT2(pDev, MAC_STA_ID1, AD_HOC, RTS_USE_DEF);
    A_REG_CLR_BIT2(pDev, MAC_STA_ID1, USE_DEFANT, DEFANT_UPDATE);
}

/**************************************************************
 * ar5211SetBasicRate
 *
 * Set or clear hardware basic rate bit
 * Set harfware basic rate set if basic rate is found
 * and basic rate is equal or less than 2Mbps
 */
void
ar5211SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *pSet)
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
 * ar5211GetRandomSeed
 *
 * Grab a semi-random value from hardware registers - may not
 * change often
 */

A_UINT32
ar5211GetRandomSeed(WLAN_DEV_INFO *pDev)
{
    A_UINT32 seed, nf;

    nf = (A_REG_RD(pDev, PHY_BASE+(25<<2)) >> 19) & 0x1ff;
    if (nf & 0x100) {
        nf = 0 - ((nf ^ 0x1ff) + 1);
    }

    seed = A_REG_RD(pDev, MAC_TSF_U32) ^ A_REG_RD(pDev, MAC_TSF_L32) ^ nf;

    return seed;
}

/**************************************************************
 * ar5211DetectCardPresent
 *
 * Detect if our card is present
 */

A_BOOL
ar5211DetectCardPresent(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
    A_UINT32 value;
    A_UINT16 macVersion, macRev;

    /* Read the SREV register and compare to what we read at initialization */
    value      = A_REG_RD(pDev, MAC_SREV) & MAC_SREV_ID_M;
    macVersion = (A_UINT16) (value >> MAC_SREV_ID_S);
    macRev     = (A_UINT16) (value & MAC_SREV_REVISION_M);

    if (pDev->macVersion == macVersion && pDev->macRev == macRev) {
        return TRUE;
    } else {
        return FALSE;
    }
#else
    return TRUE; // the spirit mac is not going anywhere
#endif
}

/**************************************************************
 * ar5211MibControl
 *
 * Control of MIB Counters
 */
A_UINT32
ar5211MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext)
{
    switch (cmd) {
    case UPDATE_SW_ALL:
    case UPDATE_SW_COMMON: {
        WLAN_STATS     *pStats  = &pDev->localSta->stats;

        pStats->AckRcvFailures += readPlatformReg(pDev, MAC_ACK_FAIL);
        pStats->RtsFailCnt     += readPlatformReg(pDev, MAC_RTS_FAIL);
        pStats->FcsFailCnt     += readPlatformReg(pDev, MAC_FCS_FAIL);
        pStats->RtsSuccessCnt  += readPlatformReg(pDev, MAC_RTS_OK);

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

    case GET_COUNTERS:
        break;

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
 * ar5211GetChannelData
 *
 * Get hw data specific to the current channel
 */
void
ar5211GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
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
 * ar5211ProcessNoiseFloor
 *
 * Take the given channel list and process all valid raw noise floors
 * into the dBm noise floor values.
 *
 * Though our device has no reference for a dBm noise floor, we perform
 * a relative minimization of NF's based on the lowest NF found across a
 * channel scan.
 */
void
ar5211ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList)
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
                tempFinalNf = pChan->rawNoiseFloor + NOISE_FLOOR[mode] + ar5211GetNfAdjust(pChan);
                if (tempFinalNf < lowest5) { 
                    lowest5 = tempFinalNf;
                    correct5 = NOISE_FLOOR[mode] - (pChan->rawNoiseFloor + ar5211GetNfAdjust(pChan));
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
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + correct5;
            } else {
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + correct2;
            }
        }

    }


}


/**************************************************************
 * ar5211GetNfAdjust
 *
 * Adjust NF based on statistical values for 5GHz frequencies.
 */
static A_INT16
ar5211GetNfAdjust(CHAN_VALUES *pChan)
{
    A_UINT16 chan = pChan->channel;
    A_INT16  adjust = 0;

    if (chan < 5210) {
        adjust = 1;
    } else if (chan > 5790) {
        adjust = 11;
    } else if (chan > 5730) {
        adjust = 10;
    } else if (chan > 5690) {
        adjust = 9;
    } else if (chan > 5660) {
        adjust = 8;
    } else if (chan > 5610) {
        adjust = 7;
    } else if (chan > 5530) {
        adjust = 5;
    } else if (chan > 5450) {
        adjust = 4;
    } else if (chan >= 5380) {
        adjust = 2;
    }

    /* Placeholder for 5111's less severe adjustment required */
    adjust = adjust / 3;

    return adjust;
}

/**************************************************************
 * ar5211EnableRadarDetection
 *
 * Enable radar detection in the hardware - may later be changed
 * to modify radar "sensitivity"
 */
void
ar5211EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams)
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

}

static INLINE void
ar5211DumpRegSet(WLAN_DEV_INFO* pDev, int first, int last)
{
    int i;

    for (i = first; i <= last; i += 4) {
        uiPrintf("=== 0x%04X: 0x%08lX\n", i, readPlatformReg(pDev, i));
    }
}

/**************************************************************
 * ar52211DumpRegisters
 *
 * Print out a bunch of HW registers.  DO NOT CALL FROM ISR ON AP.
 */
void
ar5211DumpRegisters(WLAN_DEV_INFO *pDev)
{
    uiPrintf("MAC Registers\n");
    ar5211DumpRegSet(pDev, 0x0008, 0x00b4);
    uiPrintf("\nQCU Registers\n");
    ar5211DumpRegSet(pDev, 0x0800, 0x0a40);
    uiPrintf("\nDCU Registers\n");
    ar5211DumpRegSet(pDev, 0x1000, 0x10F0);
    ar5211DumpRegSet(pDev, 0x1230, 0x1230);
    uiPrintf("\nEEP Registers\n");
    ar5211DumpRegSet(pDev, 0x6000, 0x6010);
    uiPrintf("\nPCU Registers\n");
    ar5211DumpRegSet(pDev, 0x8000, 0x8058);
    uiPrintf("\nBB Registers\n");
    ar5211DumpRegSet(pDev, 0x9800, 0x9878);
    ar5211DumpRegSet(pDev, 0x9900, 0x995C);
    ar5211DumpRegSet(pDev, 0x9C00, 0x9C1C);
}

/******************************************************************
 * ar5211GetCurRssi
 *
 * Get the rssi of frame curently being received.
 */
A_RSSI32
ar5211GetCurRssi(WLAN_DEV_INFO *pDev)
{
    return (readPlatformReg(pDev, PHY_CURRENT_RSSI) & 0xff);
}

A_UINT32
ar5211GetDefAntenna(WLAN_DEV_INFO *pDev)
{   
    return (readPlatformReg(pDev, MAC_DEF_ANTENNA) & 0x7);
}   

void
ar5211SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna)
{
    writePlatformReg(pDev, MAC_DEF_ANTENNA, (antenna & 0x7));
}

void
ar5211SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
                       CHAN_VALUES *pChval)
{
    A_UINT32        antSwitchA,
                    antSwitchB;
    EEP_HEADER_INFO *pHeaderInfo    = pDev->pHalInfo->pEepData->pEepHeader;
    WLAN_STA_CONFIG *pConfig        = &pDev->staConfig;
    int             arrayMode       = 0;

    switch (pChval->channelFlags &
            (CHANNEL_OFDM | CHANNEL_CCK | CHANNEL_5GHZ | CHANNEL_2GHZ))
    {
    case CHANNEL_A:
        arrayMode = 0;
        break;
    case CHANNEL_B:
        arrayMode = 1;
        break;
    case CHANNEL_PUREG:
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
ar5211UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt)
{
    struct TxRateCtrl_s *pRc;

    pRc = &pSib->txRateCtrl;

    pRc->antFlipCnt++;
    if (retries == 2 || retries == 3) {
        /*
         * this implies that the pkt was transmitted using other
         * than the default antenna; hw does AABBAA on transmit
         * attempts
         */
        pSib->stats.AntCnt[!pSib->antTx]++;

#ifdef BUILD_AP
        /*
         * on AP the hw has automatically changed the antenna
         * corresponding to this station
         */
        pSib->antTx = !pSib->antTx;
        pSib->stats.AntSwCnt++;
#else
        /*
         * flip the default antenna going forward if the very
         * first try on the other antenna succeeded and that
         * too with a very good ack rssi.. or if we find ourselves
         * continuously succeeding on the other antenna; if
         * deciding to flip also make sure to stick to the new
         * one for a few frames going forward; NOTE that the
         * default antenna is preserved across a chip reset by
         * the hal software
         */
        if ((pRc->antFlipCnt >= 0 && retries == 2 && rssiAck >= pRc->rssiLast + 2)
            || (pRc->antFlipCnt >= RX_FLIP_THRESHOLD))
        {
            /*
             * relookup the default antenna as we may have gotten out
             * of sync - as in the case of adhoc; getting out of sync
             * is OK for keeping AntCnt stats, but won't do for flipping
             * the default antenna here!
             */
            pSib->antTx = (ar5211GetDefAntenna(pDev) == 1) ? 0 : 1;
            pSib->antTx = !pSib->antTx;
            ar5211SetDefAntenna(pDev, pSib->antTx ? 2 : 1);
            pSib->stats.AntSwCnt++;
            pRc->antFlipCnt = -20;
        }
#endif
    } else {
        pSib->stats.AntCnt[pSib->antTx]++;
        if (pRc->antFlipCnt > 0) {
            pRc->antFlipCnt = 0;
        }
    }
}

/* Does nothing on 5211 */
A_BOOL
ar5211UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev)
{
    return FALSE;
}

void
ar5211DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose)
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
 * ar5211GetHwDescWord
 *
 * Get hardware specified word in the descriptor for compression
 */
A_UINT32
ar5211GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord)
{
    return hwDescWord;
}

/**************************************************************
 * ar5211GetApSwitchHelper
 *
 * Read some Beacon relevant registers in anticipation of saving
 * them for future restoration.
 */
void
ar5211GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    if (pRegs) {
        pRegs->r1 = readPlatformReg(pDev, MAC_TIMER0);
        pRegs->r2 = readPlatformReg(pDev, MAC_TIMER1);
        pRegs->r3 = readPlatformReg(pDev, MAC_TIMER2);
        pRegs->r4 = 0;
        pRegs->r5 = 0;
    }
}

/**************************************************************
 * ar5211SetApSwitchHelper
 *
 * Restore some Beacon relevant registers.
 */
void
ar5211SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    ASSERT(pRegs);

    writePlatformReg(pDev, MAC_TIMER0, pRegs->r1);
    writePlatformReg(pDev, MAC_TIMER1, pRegs->r2);
    writePlatformReg(pDev, MAC_TIMER2, pRegs->r3);

    /* Enable SWBA interrupt. */
    ar5211EnableInterrupts(pDev, HAL_INT_SWBA);
    ar5211EnableInterrupts(pDev, HAL_INT_TXDESC);

    /* Set the Beacon Control register */
    writePlatformReg(pDev, MAC_BEACON, pDev->bssDescr->beaconInterval | MAC_BEACON_EN);
}

#endif /* #ifdef BUILD_AR5211 */

#ifdef DEBUGWME
void
ar52xxDumpIFS(WLAN_DEV_INFO *pDev)
{
int q, reg, val;
unsigned cwmin, cwmax, aifs;

    uiPrintf("Q Setup\n");
    reg = 0x01040;
    for(q=0; q<8; q++) {
        val = readPlatformReg(pDev, reg);
        cwmin = val&0x3ff;
        val>>=10;
        cwmax = val&0x3ff;
        val>>=10;
        aifs = val&0xff;
        uiPrintf("%d: %d\t%d\t%d\n", q, cwmin, cwmax, aifs);
        reg += 4;
    }
    ar5211DumpRegSet(pDev, 0x1100, 0x1120);
}
#endif

