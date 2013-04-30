/*
 *  Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 *  Chips-specific device miscellaneous functions including hardware queries,
 *  EEPROM routines, gpio funcs, beacon creation, ...
 */

#ifdef BUILD_AR5513

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Misc.c#11 $"

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
#include "ar5513MacReg.h"
#include "ar5513Misc.h"
#include "ar5513Reset.h"
#include "ar5513Power.h"
#include "ar5513Interrupts.h"
#include "ar5513Mac.h"

#include "pktlog.h"

#if defined(AR531X)
#include "ar531xreg.h"
#endif

#if defined(AR5513)
#include "ar5513reg.h"
#endif /* AR5513 */

static A_INT16
ar5513GetNfAdjust(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan);

#ifndef BUILD_AP
static A_BOOL
ar5513IsSlowSleepClockAllowed(WLAN_DEV_INFO *pDev);
#endif

#define SLOT_TIME_20 880
#define SLOT_TIME_09 396

/**************************************************************
 * ar5513InitFrmTypeCapTable
 *
 * Initialize AR5513 Frame Type Capabilities Table
 *
 * Returns: 
 */
A_STATUS
ar5513InitFrmTypeCapTable(WLAN_DEV_INFO *pDev)
{
    int i;
    A_STATUS status      = A_OK;
    A_UINT32 frmTypeBits = 0;

    /*
    ** Set Beam Form bits in Frame Type Capabilities Table
    ** for each 802.11 packet type. The one exception is 
    ** BFCOEF_TX_ENABLE_MCAST that will NOT be enabled.
    */

    /*
     * Further, as part of the AR5513 E2.0 2.4 synth workaround,
     * TX bit are set until a valid phase is calculated.
     */

    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN) {
        frmTypeBits = MAC_FTC_BF_RX_UPDT_NORM |
                      MAC_FTC_BF_RX_UPDT_SELF |
                      (pDev->pHalInfo->synth_state_flag ? 0 : MAC_FTC_BF_TX_ENB_NORM) |
                      (pDev->pHalInfo->synth_state_flag ? 0 : MAC_FTC_BF_TX_ENB_SELF) |
                      (pDev->pHalInfo->synth_state_flag ? 0 : MAC_FTC_BF_TX_ENB_GEN);
    } else {
        frmTypeBits = 0;
    }

    for (i=0; i < (MAC_FRM_TYPE_CAP_SIZE * 4); i += 4) {
        writePlatformReg(pDev,MAC_FRM_TYPE_CAP_TBL + i, frmTypeBits);
    }

    return status;
}

/**************************************************************
 * ar5513SetRegulatoryDomain
 *
 * Attempt to change the cards operating regulatory domain to the given value
 * Returns: A_EINVAL for an unsupported regulatory domain.
 *          A_HARDWARE for an unwritable EEPROM or bad EEPROM version
 */
A_STATUS
ar5513SetRegulatoryDomain(WLAN_DEV_INFO *pDev, A_UINT16 regDomain)
{
    A_STATUS status = A_OK;
    EEP_MAP *pEep;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);

    pEep = pDev->pHalInfo->pEepData;
    if (pDev->pHalInfo->halCapabilities.halRegDmn != regDomain) {
        if (!(pEep->protect & EEPROM_PROTECT_WP_128_191))
        {
            if(wlanIsRegCcValid(pDev)) {
                status = ar5513EepromWrite(pDev, REGULATORY_DOMAIN_OFFSET, regDomain);
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
 * ar5513GetRfKill
 *
 * Accessor to get rfkill info from private EEPROM structures
 */
A_BOOL
ar5513GetRfKill(WLAN_DEV_INFO *pDev)
{
    A_UINT16 rfsilent;

    ASSERT(pDev->pHalInfo && pDev->pHalInfo->pEepData);
    if (pDev->pHalInfo->pEepData->pEepHeader->rfKill) {
        ar5513EepromRead(pDev, EEPROM_RFSILNT_CLKEN_OFFSET, &rfsilent);
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
 * ar5513GetMacAddr
 *
 * Attempt to get the MAC address from the wireless EEPROM
 * Returns: A_HARDWARE for an unreadable EEPROM
 */
A_STATUS
ar5513GetMacAddr(WLAN_DEV_INFO *pDev, WLAN_MACADDR *mac)
{
#if defined(PCI_INTERFACE)
    A_UINT32 total = 0;
    A_UINT16 half;

    if (ar5513EepromRead(pDev, EEPROM_MAC_MSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[1] = half & 0xff;
    mac->octets[0] = half >> 8;

    if (ar5513EepromRead(pDev, EEPROM_MAC_MID_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[3] = half & 0xff;
    mac->octets[2] = half >> 8;
    if (ar5513EepromRead(pDev, EEPROM_MAC_LSW_OFFSET, &half) != A_OK) {
        goto ee_error;
    }
    total += half;
    mac->octets[5] = half & 0xff;
    mac->octets[4] = half >> 8;

    if ((total == 0) || total == (3 * 0xffff)) {
        uiPrintf("ar5513GetMacAddr: EEPROM MAC is all 0's or all F's\n");
        return A_HARDWARE;
    }
    return A_OK;

ee_error:
    uiPrintf("ar5513GetMacAddr: EEPROM read failed\n");
    return A_HARDWARE;
#elif defined(AR531X) || defined(AR5513)
    /* The ar5312 may store the WLAN mac addr in the system board config */
    sysLanEnetAddrGet("ar", pDev->devno, mac->octets);
    return A_OK;
#endif
}

/**************************************************************
 * ar5513EepromRead
 *
 * Read 16 bits of data from offset into *data
 */
A_STATUS
ar5513EepromRead(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 *data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 temp, sector;
    int to = 10000;
    int fDone = FALSE;

// Long - Really should use a static instead of reading signature 
// for every 2 byte read
#define CB63_NDIS 
#if defined(AV10_NDIS)

    writePlatformReg(pDev, MAC_SPI_AO, MAC_SPI_AO_RD_SIG);
    writePlatformReg(pDev, MAC_SPI_CS, MAC_SPI_CS_RD_SIGN);

    while (to > 0) {
	    if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
	        udelay(1);
	        to--;
	    }
	    else {
	        fDone = TRUE;
	        break;
	    }
    }

    if (fDone == TRUE) {
	    temp = readPlatformReg(pDev, MAC_SPI_D) & 0xff;

	    switch (temp) {
	    case 0x13:  // 1MB Flash
	        sector = 0xf0000;
	        break;
	    case 0x14:  // 2MB Flash
	        sector = 0x1f0000;
	        break;
	    case 0x15:  // 4MB Flash
	        sector = 0x3f0000;
	        break;
	    default:
	        return A_HARDWARE;
	    }
    }
    else {
	    uiPrintf("ar5513EepromRead: eeprom read error at offset %d\n",offset);
	    return A_HARDWARE;
    }

    temp = MAC_SPI_AO_RD_DATA | ((sector | (offset << 1)) << 8);
    writePlatformReg(pDev, MAC_SPI_AO, temp);
    writePlatformReg(pDev, MAC_SPI_CS, MAC_SPI_CS_RD_2BYTES);

    while (to > 0) {
	    if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
	        udelay(1);
	        to--;
	    }
	    else {
	        fDone = TRUE;
	        break;
	    }
    }

    if (fDone == TRUE) {
	    temp = readPlatformReg(pDev, MAC_SPI_D);
	    /*
	    **  XXXX - GDS 07/17/2004
	    **  Temporarily Swap bytes as Work-Around for Falcon 1.0 Silicon.
	    **  MIPS CPU is executing to prevent NMI from flooding x86 CPU
	    */
	    *data = (A_UINT16) (((temp & 0xff) << 8) | ((temp & 0xff00) >> 8));
        return A_OK;
    }
    else {
	    uiPrintf("ar5513EepromRead: eeprom read error at offset %d\n",offset);
	    return A_HARDWARE;
    }
#elif defined(CB63_NDIS)

    temp = MAC_SPI_EEPROM_RD | ((offset * 2) << 16);
    writePlatformReg(pDev, MAC_SPI_AO, temp);
    writePlatformReg(pDev, MAC_SPI_CS, (MAC_SPI_CS_SPI_FORCE_SZ_16 << MAC_SPI_CS_SPI_AUTO_SZ_S) 
                            | MAC_SPI_CS_START 
                            | ((2 << MAC_SPI_CS_RX_BYTE_CNT_S) & MAC_SPI_CS_RX_BYTE_CNT_M)
                            | ((3 << MAC_SPI_CS_TX_BYTE_CNT_S) & MAC_SPI_CS_TX_BYTE_CNT_M));

    while (to > 0) {
	    if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
	        udelay(1);
	        to--;
	    }
	    else {
	        fDone = TRUE;
	        break;
	    }
    }

    if (fDone == TRUE) {
	    temp = readPlatformReg(pDev, MAC_SPI_D);
	    *data = (A_UINT16) (temp & 0xffff);
        return A_OK;
    }
    else {
	    uiPrintf("ar5513EepromRead: eeprom read error at offset %d\n",offset);
	    return A_HARDWARE;
    }
#else
#error "define AV10_NDIS or CB63_NDIS to define Board" 
#endif

#elif defined(AR531X) || defined(AR5513)
    /* radio configuration data is stored in the system flash */
    *data = sysFlashConfigRead(FLC_RADIOCFG, (offset * 2) + 1) |
            (sysFlashConfigRead(FLC_RADIOCFG, offset * 2) << 8);
    return A_OK;
#endif /* PLATFORM */
}


#define TIMEOUT_SPI_CS                  10000   /* us */
#define TIMEOUT_SPI_WRITE_IN_PROGRESS   10000   /* us */

/**************************************************************
 * ar5513EepromWrite
 *
 * Write 16 bits of data from data into offset
 */
A_STATUS
ar5513EepromWrite(WLAN_DEV_INFO *pDev, A_UINT32 offset, A_UINT16 data)
{
#if defined(PCI_INTERFACE)
    A_UINT32 temp, sector;
    int to;
    int fDone;
    int fWriteInProgress;
    int toWriteInProgress;
    int byteCount;
    A_UINT32 writeAddr, writeData;

// Long - Really should use a static instead of reading signature 
// for every 2 byte read
#define CB63_NDIS 
#if defined(AV10_NDIS)
    /* TODO */
    ASSERT(0); 
#elif defined(CB63_NDIS)

    /* Write two bytes */
    for (byteCount = 0; byteCount < 2; byteCount ++) {

        /* Enable writes */
        writePlatformReg(pDev, MAC_SPI_AO, MAC_SPI_EEPROM_WR_EN);
        writePlatformReg(pDev, MAC_SPI_D, 0);
        writePlatformReg(pDev, MAC_SPI_CS, (MAC_SPI_CS_SPI_FORCE_SZ_16 << MAC_SPI_CS_SPI_AUTO_SZ_S) 
                                | MAC_SPI_CS_START 
                                | ((1 << MAC_SPI_CS_TX_BYTE_CNT_S) & MAC_SPI_CS_TX_BYTE_CNT_M));

        to = TIMEOUT_SPI_CS;
        fDone = FALSE;
        while (to > 0) {
            if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
                udelay(1);
                to--;
            }
            else {
                fDone = TRUE;
                break;
            }
        }

        if (fDone == FALSE) {
            uiPrintf("ar5513EepromWrite: Could not enable write\n");
            return A_HARDWARE;
        }


        /* Write (one byte at a time) */
        writeData = (data >> (8 * byteCount)) & 0xFF;
        writeAddr = (((2 * offset) + byteCount) << 16) & 0xFFFF0000;
        temp = MAC_SPI_EEPROM_WR_STATUS | writeAddr | (writeData << 8);
        writePlatformReg(pDev, MAC_SPI_AO, temp);
        writePlatformReg(pDev, MAC_SPI_D, 0);
        writePlatformReg(pDev, MAC_SPI_CS, (MAC_SPI_CS_SPI_FORCE_SZ_16 << MAC_SPI_CS_SPI_AUTO_SZ_S) 
                                | MAC_SPI_CS_START 
                                | ((4 << MAC_SPI_CS_TX_BYTE_CNT_S) & MAC_SPI_CS_TX_BYTE_CNT_M));

        to = TIMEOUT_SPI_CS;
        fDone = FALSE;
        while (to > 0) {
            if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
                udelay(1);
                to--;
            }
            else {
                fDone = TRUE;
                break;
            }
        }

        if (fDone == FALSE) {
            uiPrintf("ar5513EepromWrite: Could not write\n");
            return A_HARDWARE;
        }

        /* Read status register (wait for write to finish) */
        fWriteInProgress = TRUE;
        toWriteInProgress = TIMEOUT_SPI_WRITE_IN_PROGRESS;

        while (fWriteInProgress && (toWriteInProgress > 0)) {
            writePlatformReg(pDev, MAC_SPI_AO, MAC_SPI_EEPROM_RD_STATUS);
            writePlatformReg(pDev, MAC_SPI_D, 0);
            writePlatformReg(pDev, MAC_SPI_CS, (MAC_SPI_CS_SPI_FORCE_SZ_16 << MAC_SPI_CS_SPI_AUTO_SZ_S) 
                                    | MAC_SPI_CS_START 
                                    | ((1 << MAC_SPI_CS_RX_BYTE_CNT_S) & MAC_SPI_CS_RX_BYTE_CNT_M)
                                    | ((1 << MAC_SPI_CS_TX_BYTE_CNT_S) & MAC_SPI_CS_TX_BYTE_CNT_M));

            to = TIMEOUT_SPI_CS;
            fDone = FALSE;
            while (to > 0) {
                if (readPlatformReg(pDev, MAC_SPI_CS) & MAC_SPI_CS_BUSY) {
                    udelay(1);
                    to--;
                }
                else {
                    fDone = TRUE;
                    break;
                }
            }

            if (fDone == TRUE) {
                temp = readPlatformReg(pDev, MAC_SPI_D);
                fWriteInProgress = temp & 0x1;
            } else {
                uiPrintf("ar5513EepromWrite: Could not read status register\n");
                return A_HARDWARE;
            }

            toWriteInProgress--;
        }

        if (fWriteInProgress) {
            uiPrintf("ar5513EepromWrite: write did not finish\n");
            return A_HARDWARE;
        }
    }  

    return A_OK;
#else
#error "define AV10_NDIS or CB63_NDIS to define Board" 
#endif
#elif defined(AR531X) || defined(AR5513)
    char str[2];

    /* Radio configuration data is stored in system flash (in reverse endian) */
    str[0] = (data >> 8) & 0xff;
    str[1] = data & 0xff;
    sysFlashConfigWrite(FLC_RADIOCFG, offset<<1, str, 2);

    return A_OK;
#endif /* PLATFORM */
}

/**************************************************************************
 * ar5513EnableRfKill
 *
 * Called if RfKill is supported (according to EEPROM).  Set the interrupt and
 * GPIO values so the ISR can disable RF on a switch signal.  Assumes GPIO port
 * and interrupt polarity are set prior to call.
 */
void
ar5513EnableRfKill(WLAN_DEV_INFO *pDev)
{
    /* TODO - can this really be above the hal on the GPIO interface for
     * TODO - the client only?
     */
#if !defined(BUILD_AP)  /* AP uses GPIOs for LEDs and buttons */

    /* Configure the desired GPIO port for input and enable baseband rf silence */
    ar5513GpioCfgInput(pDev, pDev->rfSilent.gpioSelect);
    writePlatformReg(pDev, PHY_BASE, readPlatformReg(pDev, PHY_BASE) | 0x00002000);
    /*
     * If radio disable switch connection to GPIO bit x is enabled
     * program GPIO interrupt.
     * If rfkill bit on eeprom is 1, setupeeprommap routine has already
     * verified that it is a later version of eeprom, it has a place for
     * rfkill bit and it is set to 1, indicating that GPIO bit x hardware
     * connection is present.
     */

    if (pDev->rfSilent.polarity == ar5513GpioGet(pDev, pDev->rfSilent.gpioSelect)) {
        /* switch already closed, set to interrupt upon open */
        ar5513GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, !pDev->rfSilent.polarity);
    } else {
        ar5513GpioSetIntr(pDev, pDev->rfSilent.gpioSelect, pDev->rfSilent.polarity);
    }
#endif /* !BUILD_AP */
}

/**************************************************************
 * ar5513GpioCfgOutput
 *
 * Configure GPIO Output lines
 */
void
ar5513GpioCfgOutput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
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
 * ar5513GpioCfgInput
 *
 * Configure GPIO Input lines
 */
void
ar5513GpioCfgInput(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
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
 * ar5513GpioSet
 *
 * Once configured for I/O - set output lines
 */
void
ar5513GpioSet(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 val)
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
 * ar5513GpioGet
 *
 * Once configured for I/O - get input lines
 */
A_UINT32
ar5513GpioGet(WLAN_DEV_INFO *pDev, A_UINT32 gpio)
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
 * ar5513GpioSetIntr
 *
 * Set the desired GPIO Interrupt
 */
void
ar5513GpioSetIntr(WLAN_DEV_INFO *pDev, A_UINT32 gpio, A_UINT32 ilevel)
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
 * ar5513SetLedState
 *
 * Change the LED blinking pattern to correspond to the connectivity.
 *   Normal blink when connected, alternate blink when not.
 */
void
ar5513SetLedState(WLAN_DEV_INFO *pDev, A_BOOL bConnected)
{
#if defined(PCI_INTERFACE)
#ifdef BUILD_AR5513
    A_UINT32 val;

    val = bConnected ?
          MAC_PCICFG_ASSOC_STATUS_ASSOCIATED : MAC_PCICFG_ASSOC_STATUS_PENDING;

    pDev->pHalInfo->pciCfg = (pDev->pHalInfo->pciCfg & 
                              ~(MAC_PCICFG_ASSOC_STATUS_M)) | 
                    (((val) << (MAC_PCICFG_ASSOC_STATUS_S)) & (MAC_PCICFG_ASSOC_STATUS_M));
    A_REG_WR(pDev, MAC_PCICFG,  pDev->pHalInfo->pciCfg);
#else
    A_UINT32 val;

    val = bConnected ?
          MAC_PCICFG_ASSOC_STATUS_ASSOCIATED : MAC_PCICFG_ASSOC_STATUS_PENDING;

    A_REG_RMW_FIELD(pDev, MAC_PCICFG, ASSOC_STATUS, val);
#endif

#elif defined(AR531X)
    A_UINT32 reg = sysRegRead(AR531X_PCICFG) & ~ASSOC_STATUS_M;

    if (bConnected) {
        reg |= ASSOC_STATUS_ASSOCIATED;
    } else {
        reg |= ASSOC_STATUS_PENDING;
    }
    sysRegWrite(AR531X_PCICFG, reg);

#elif defined(AR5513)
    A_UINT32 reg = sysRegRead(AR5513_PCI_CFG) & ~AR5513_PCI_CFG_ASSOC_STATUS_M;

    if (bConnected) {
        reg |= AR5513_PCI_CFG_ASSOC_ASSOCIATED;
    } else {
        reg |= AR5513_PCI_CFG_ASSOC_PENDING;
    }
    sysRegWrite(AR5513_PCI_CFG, reg);
#endif /* PLATFORM */
}

/**************************************************************************
 * ar5513WriteAssocid - Change association related fields programmed into the hardware.
 *
 * Writing a valid BSSID to the hardware effectively enables the hardware
 * to synchronize its TSF to the correct beacons and receive frames coming
 * from that BSSID. It is called by the SME JOIN operation.
 */
void
ar5513WriteAssocid(WLAN_DEV_INFO *pDev, WLAN_MACADDR *bssid, A_UINT16 assocId, A_UINT16 timOffset)
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
 * ar5513SetStaBeaconTimers
 *
 *  Sets all the beacon related bits on the h/w for stations
 *  i.e. initializes the corresponding h/w timers;
 *  also tells the h/w whether to anticipate PCF beacons
 */
void
ar5513SetStaBeaconTimers(WLAN_DEV_INFO *pDev, HAL_BEACON_TIMERS *pTimers)
{
    /* Disable global interrupts */
    ar5513DisableInterrupts(pDev, HAL_INT_GLOBAL);

    /* Mask BMISS interrupt */
    pDev->MaskReg &= ~MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);

    /* Clear any pending BMISS interrupt so far */
    pDev->globISRReg &= ~MAC_IMR_BMISS;

    /*
     * Configure the BMISS interrupt
     */
    ASSERT(pTimers->bmissThreshold <= (MAC_RSSI_THR_BM_THR_M >> MAC_RSSI_THR_BM_THR_S));

    A_REG_RMW_FIELD(pDev, MAC_RSSI_THR, BM_THR, pTimers->bmissThreshold);

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
     * Oahu beacons timers on the station were used for power
     * save operation (waking up in anticipation of a beacon)
     * and any CFP function; Venice does sleep/power-save timers
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
    uiPrintf("ar5513SetStaBeaconTimers: setting next DTIM for %d\n", pTimers->nextDtim);
    uiPrintf("ar5513SetStaBeaconTimers: setting next beacon for %d\n", pTimers->nextTbtt);
    uiPrintf("ar5513SetStaBeaconTimers: setting beacon period to %d\n", pTimers->beaconPeriod);
    uiPrintf("ar5513SetStaBeaconTimers: setting DTIM period to %d\n", pTimers->dtimPeriod);
#endif

    A_REG_WR_FIELD(pDev, MAC_SLEEP1, NEXT_DTIM, (pTimers->nextDtim - SLEEP_SLOP) * 8);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP1, CAB_TIMEOUT, CAB_TIMEOUT_VAL);

#ifdef AR5513_CCC
    /* 
     * Coordinated channel change utilizes the beacon timeout interrupt.
     * ASSUME_DTIM = 1 prevents the BCNTO interrupt from occurring
     * Set ASSUME_DTIM=0 as a workaround.
     * Side effect will be that upon missing a DTIM beacon,
     * the hardware will not wait for CAB traffic.
     */
    A_REG_SET_BIT(pDev, MAC_SLEEP1, ENH_SLEEP_ENABLE);
#else 
    A_REG_SET_BIT2(pDev, MAC_SLEEP1, ASSUME_DTIM, ENH_SLEEP_ENABLE);
#endif
    A_REG_WR_FIELD(pDev, MAC_SLEEP2, NEXT_TIM, (pTimers->nextTbtt - SLEEP_SLOP) * 8);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP2, BEACON_TIMEOUT, BEACON_TIMEOUT_VAL);
    A_REG_WR_FIELD(pDev, MAC_SLEEP3, TIM_PERIOD, pTimers->beaconPeriod);
    A_REG_RMW_FIELD(pDev, MAC_SLEEP3, DTIM_PERIOD, pTimers->dtimPeriod);

    /* Clear pending BMISS interrupt in ISR register */
    writePlatformReg(pDev, MAC_ISR, MAC_IMR_BMISS); // cleared on write

    /* Unmask BMISS interrupt */
    pDev->MaskReg |= MAC_IMR_BMISS;
    A_REG_WR(pDev, MAC_IMR, pDev->MaskReg);

    /* Enable global interrupts */
    ar5513EnableInterrupts(pDev, HAL_INT_GLOBAL);
}

/******************************************************************
 * ar5513GetTsf
 *
 * Get the current hardware tsf for stamlme
 */
void
ar5513GetTsf(WLAN_DEV_INFO *pDev, WLAN_TIMESTAMP *tsf)
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
 * ar5513ResetTsf
 *
 * Reset the current hardware tsf for stamlme
 */
void
ar5513ResetTsf(WLAN_DEV_INFO *pDev)
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
 * ar5513SetAdhocMode
 *
 * Set adhoc mode for stamlme
 */
void
ar5513SetAdhocMode(WLAN_DEV_INFO *pDev)
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
 * ar5513SetBasicRate
 *
 * Set or clear hardware basic rate bit
 * Set harfware basic rate set if basic rate is found
 * and basic rate is equal or less than 2Mbps
 */
void
ar5513SetBasicRate(WLAN_DEV_INFO *pDev, WLAN_RATE_SET *pSet)
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
 * ar5513GetRandomSeed
 *
 * Grab a semi-random value from hardware registers - may not
 * change often
 */

A_UINT32
ar5513GetRandomSeed(WLAN_DEV_INFO *pDev)
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
 * ar5513DetectCardPresent
 *
 * Detect if our card is present
 */

A_BOOL
ar5513DetectCardPresent(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
#ifndef BUILD_AR5513
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
    A_UINT32 value;
    A_UINT16  chipVersion, chipRev;

    /* Read the RST_SREV register and compare to what we read at initialization */
    value     = readPlatformReg(pDev, RST_SREV) & RST_SREV_M;
    chipVersion= (A_UINT16) (value >> RST_SREV_VERSION_S);
    chipRev    = (A_UINT16) (value & RST_SREV_REVISION_M);

    if ( (pDev->pHalInfo->chipVersion == chipVersion) && (pDev->pHalInfo->chipRev == chipRev) ) {
        return TRUE;
    } else {
        return FALSE;
    }
#endif
#else
    return TRUE;                        /* the spirit mac is not going anywhere */
#endif
}

/**************************************************************
 * ar5513MibControl
 *
 * Control of MIB Counters
 */
A_UINT32
ar5513MibControl(WLAN_DEV_INFO *pDev, HAL_MIB_CMD cmd, void *pContext)
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
 * ar5513GetChannelData
 *
 * Get hw data specific to the current channel
 */
void
ar5513GetChannelData(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan,
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
 * ar5513ProcessNoiseFloor
 *
 * Take the given channel list and process all valid raw noise floors
 * into the dBm noise floor values.
 *
 * Though our device has no reference for a dBm noise floor, we perform
 * a relative minimization of NF's based on the lowest NF found across a
 * channel scan.
 */
void
ar5513ProcessNoiseFloor(WLAN_DEV_INFO *pDev, WLAN_CHANNEL_LIST *pCList)
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
                tempFinalNf = pChan->rawNoiseFloor + NOISE_FLOOR[mode] + ar5513GetNfAdjust(pDev, pChan);
                if (tempFinalNf < lowest5) { 
                    lowest5 = tempFinalNf;
                    correct5 = NOISE_FLOOR[mode] - (pChan->rawNoiseFloor + ar5513GetNfAdjust(pDev, pChan));
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
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + ar5513GetNfAdjust(pDev, pChan) + correct5;
            } else {
                pChan->finalNoiseFloor = pChan->rawNoiseFloor + correct2;
            }
        }
    }
}


/**************************************************************
 * ar5513GetNfAdjust
 *
 * Adjust NF based on statistical values for 5GHz frequencies.
 */
static A_INT16
ar5513GetNfAdjust(WLAN_DEV_INFO *pDev, CHAN_VALUES *pChan)
{
    A_UINT16      chan = pChan->channel;
    A_INT16       adjust = 0;
    const A_INT16 *pAdjust;
    const A_INT16 Adjust5112[10] = {1, 0, 2, 4, 5, 7, 8, 9, 10, 11};
    const A_INT16 Adjust5111[10] = {5, 3, 1, 0, 0, 1, 2, 3, 4, 6};

    if (IS_5112(pDev)) {
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
 * ar5513IsHwCipherSupported
 *
 * Detect if Cipher requested is implemented in HW.
 */
A_BOOL
ar5513IsHwCipherSupported(WLAN_DEV_INFO *pDev, A_UINT32 keyType)
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
 * ar5513EnableRadarDetection
 *
 * Enable radar detection in the hardware - may later be changed
 * to modify radar "sensitivity"
 */
void
ar5513EnableRadarDetection(WLAN_DEV_INFO *pDev, HAL_RADAR_PHY_PARAMS *pPhyParams)
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
 * ar5513EnableFriendlyDetection
 *
 * Enable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
ar5513EnableFriendlyDetection(WLAN_DEV_INFO *pDev, A_UINT8 detectionRSSIThr)
{
    /* Now enable pulse detection */
    detectionRSSIThr &= 0x3f;    /* only 6 bits available */
    A_REG_RMW_FIELD (pDev, PHY_RADAR_0, RRSSI, detectionRSSIThr);
    A_REG_SET_BIT(pDev, PHY_RADAR_0, EN);
}

/**************************************************************
 * ar5513DisableFriendlyDetection
 *
 * Disable 11g detection (radar) for Friendly Turbo in the hardware 
 */
void
ar5513DisableFriendlyDetection(WLAN_DEV_INFO *pDev)
{
    /* Now disable pulse detection */
    A_REG_CLR_BIT(pDev, PHY_RADAR_0, EN);
}

/**************************************************************
 * ar5513AniControl
 *
 * Control Adaptive Noise Immunity Parameters
 */
void
ar5513AniControl(WLAN_DEV_INFO *pDev, HAL_ANI_CMD cmd, int param)
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
 * ar5513AniGetListenTime
 *
 * Get listening time for Adaptive Noise Immunity
 * Return -1 if counter wrap-around occurs
 */

/* convert HW counter values to ms using 11g clock rate, good enough for 11a and Turbo */
#define CLOCK_RATE  44000

A_INT32
ar5513AniGetListenTime(WLAN_DEV_INFO *pDevInfo)
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
ar5513DumpRegSet(WLAN_DEV_INFO* pDev, int first, int last)
{
    int i;

    for (i = first; i <= last; i += 4) {
        uiPrintf("=== 0x%04X: 0x%08lX\n", i, readPlatformReg(pDev, i));
    }
}

/**************************************************************
 * ar5513DumpRegisters
 *
 * Print out a bunch of HW registers.  DO NOT CALL FROM ISR ON AP.
 */
void
ar5513DumpRegisters(WLAN_DEV_INFO *pDev)
{
    uiPrintf("MAC Registers\n");
    ar5513DumpRegSet(pDev, 0x0008, 0x00b4);
    uiPrintf("\nQCU Registers\n");
    ar5513DumpRegSet(pDev, 0x0800, 0x0a40);
    uiPrintf("\nDCU Registers\n");
    ar5513DumpRegSet(pDev, 0x1000, 0x10F0);
    ar5513DumpRegSet(pDev, 0x1230, 0x1230);
    uiPrintf("\nPCI Registers\n");
    ar5513DumpRegSet(pDev, 0x4000, 0x4030);
    uiPrintf("\nEeprom Registers\n");
    ar5513DumpRegSet(pDev, 0x6000, 0x6010);
    uiPrintf("\nPCU Registers\n");
    ar5513DumpRegSet(pDev, 0x8000, 0x8058);
    uiPrintf("\nBB Registers\n");
    ar5513DumpRegSet(pDev, 0x9800, 0x9878);
    ar5513DumpRegSet(pDev, 0x9900, 0x995C);
    ar5513DumpRegSet(pDev, 0x9C00, 0x9C1C);
}

/******************************************************************
 * ar5513GetCurRssi
 *
 * Get the rssi of frame curently being received.
 */
A_RSSI32
ar5513GetCurRssi(WLAN_DEV_INFO *pDev)
{
    A_INT8 rssi = (A_INT8)(readPlatformReg(pDev, PHY_CURRENT_RSSI) & 0xff);
    return (A_INT32)(rssi < 0 ? 0 : rssi);
}

A_UINT32
ar5513GetDefAntenna(WLAN_DEV_INFO *pDev)
{
    return (readPlatformReg(pDev, MAC_DEF_ANTENNA) & 0x7);
}

void
ar5513SetDefAntenna(WLAN_DEV_INFO *pDev, A_UINT32 antenna)
{
    writePlatformReg(pDev, MAC_DEF_ANTENNA, (antenna & 0x7));
}

void
ar5513SetAntennaSwitch(WLAN_DEV_INFO *pDev, ANTENNA_CONTROL settings,
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

    if (pDev->staConfig.txChainCtrl == DUAL_CHAIN) {
        writePlatformReg(pDev, ANT_SWITCH_TABLE1_CH0, antSwitchA);
        writePlatformReg(pDev, ANT_SWITCH_TABLE2_CH0, antSwitchB);

        /* Workaround for 2.4 Synth problem on AR5513 E2.0 chip */
        if (pDev->pHalInfo->synth_state_flag & 1) {
            antSwitchA &= ~0x240;   /* LNA off, RX open for CH1 */
            antSwitchB &= ~0x280;   /* LNA off, RX open for CH1 */
        }

        writePlatformReg(pDev, ANT_SWITCH_TABLE1_CH1, antSwitchA);
        writePlatformReg(pDev, ANT_SWITCH_TABLE2_CH1, antSwitchB);
    } else {
        writePlatformReg(pDev, ANT_SWITCH_TABLE1, antSwitchA);
        writePlatformReg(pDev, ANT_SWITCH_TABLE2, antSwitchB);
    }
}

void
ar5513UpdateAntenna(WLAN_DEV_INFO *pDev, SIB_ENTRY *pSib, int retries,
                    A_RSSI rssiAck, A_UINT8 curTxAnt)
{
    return;
}

/*
 * ar5513UseShortSlotTime - set short time 'en' != 'prev' and
 *                          9 us if 'en' == TRUE else 20 us.
 *                          Returns 'en'.
 */
A_BOOL
ar5513UseShortSlotTime(WLAN_DEV_INFO *pDev, A_BOOL en, A_BOOL prev)
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
ar5513DmaDebugDump(WLAN_DEV_INFO *pDev, A_BOOL verbose)
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
 * ar5513SetupSleepRegisters
 *
 * If slow clock exists, use it to lower power consumption during sleep
 * set sleep registers correctly for refclk or slow clock operation
 * Note: If clock is set to 32 KHz, delays on accessing certain
 *       baseband registers (27-31, 124-127) are required.
 *
 */
void
ar5513SetupSleepRegisters(WLAN_DEV_INFO *pDev)
{
#ifndef BUILD_AP
    if (ar5513IsSlowSleepClockAllowed(pDev)) {
        /*
         * If this card has an external 32 KHz crystal,
         * enable clocks to be turned OFF in BB during sleep
         * and also enable turning OFF 32MHz/40MHz Refclk
         * from A2.
         */
        A_REG_WR(pDev, PHY_SLEEP_CTR_CONTROL, 0x1f); //reg 28 =  9870
        A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x0d); //reg 29 =  9874
        A_REG_WR(pDev, PHY_SLEEP_SCAL,        0x0c); //reg 30 =  9878
        A_REG_WR(pDev, PHY_M_SLEEP,           0x03); //reg 124 = 99f0
        A_REG_WR(pDev, PHY_REFCLKDLY,         0x05); //reg 125 = 99f4
        if (IS_5112(pDev)) {
            A_REG_WR(pDev, PHY_REFCLKPD,      0x14); //reg 126 = 99f8
        } else {
            A_REG_WR(pDev, PHY_REFCLKPD,      0x18); //reg 126 = 99f8
        }

        /* Set USEC32 to 1 */
        A_REG_RMW_FIELD(pDev, MAC_USEC, 32, 1);
        /* Set TSF Increment for 32 KHz */
        A_REG_WR(pDev, MAC_TSF_PARM, 61);

#ifdef BUILD_AR5513
        /* # Set sleep clock rate to 32 KHz. */
        pDev->pHalInfo->pciCfg = (pDev->pHalInfo->pciCfg & 
                                  ~(MAC_PCICFG_SLEEP_CLK_SEL_M)) | 
                        (((1) << (MAC_PCICFG_SLEEP_CLK_SEL_S)) & (MAC_PCICFG_SLEEP_CLK_SEL_M));
        A_REG_WR(pDev, MAC_PCICFG,  pDev->pHalInfo->pciCfg);
        
        pDev->pHalInfo->pciCfg = (pDev->pHalInfo->pciCfg & 
                                  ~(MAC_PCICFG_SLEEP_CLK_RATE_IND_M)) | 
                        (((0x3) << (MAC_PCICFG_SLEEP_CLK_RATE_IND_S)) & (MAC_PCICFG_SLEEP_CLK_RATE_IND_M));
        A_REG_WR(pDev, MAC_PCICFG,  pDev->pHalInfo->pciCfg);
#else
        /* # Set sleep clock rate to 32 KHz. */
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_SEL, 1);
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_RATE_IND, 0x3);
#endif

    } else {
#endif
        A_REG_WR(pDev, PHY_SLEEP_CTR_CONTROL, 0x1f);
        A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x7f);

        if (IS_5312_2_X(pDev)) {
            /* Set ADC/DAC select values */
            A_REG_WR(pDev, PHY_SLEEP_SCAL, 0x00000004);
        } else {
            A_REG_WR(pDev, PHY_SLEEP_SCAL,        0x0e);
            A_REG_WR(pDev, PHY_M_SLEEP,           0x0c);
            A_REG_WR(pDev, PHY_REFCLKDLY,         0xff);
            if (IS_5112(pDev)) {
                A_REG_WR(pDev, PHY_REFCLKPD,      0x14); //reg 126 = 99f8
            } else {
                A_REG_WR(pDev, PHY_REFCLKPD,      0x18); //reg 126 = 99f8
            }
        }

#ifndef BUILD_AP
    }
#endif
}

/**************************************************************
 * ar5513RestoreSleepRegisters
 *
 * If slow clock exists, turn it off and turn back on the refclk
 * set sleep registers to match refclk operation
 */
void
ar5513RestoreSleepRegisters(WLAN_DEV_INFO *pDev)
{
#ifndef BUILD_AP
    if (ar5513IsSlowSleepClockAllowed(pDev)) {
        /* # Set sleep clock rate back to 32 MHz. */

#ifdef BUILD_AR5513
        /* # Set sleep clock rate to 32 KHz. */
        pDev->pHalInfo->pciCfg = (pDev->pHalInfo->pciCfg & 
                                  ~(MAC_PCICFG_SLEEP_CLK_SEL_M)) | 
                        (((0) << (MAC_PCICFG_SLEEP_CLK_SEL_S)) & (MAC_PCICFG_SLEEP_CLK_SEL_M));
        A_REG_WR(pDev, MAC_PCICFG,  pDev->pHalInfo->pciCfg);
        
        pDev->pHalInfo->pciCfg = (pDev->pHalInfo->pciCfg & 
                                  ~(MAC_PCICFG_SLEEP_CLK_RATE_IND_M)) | 
                        (((0) << (MAC_PCICFG_SLEEP_CLK_RATE_IND_S)) & (MAC_PCICFG_SLEEP_CLK_RATE_IND_M));
        A_REG_WR(pDev, MAC_PCICFG,  pDev->pHalInfo->pciCfg);
#else
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_RATE_IND, 0);
        A_REG_RMW_FIELD(pDev, MAC_PCICFG, SLEEP_CLK_SEL, 0);
#endif

        /* # Set TSF Increment for 32 MHz */
        A_REG_WR(pDev, MAC_TSF_PARM, 1);
         /* # Set USEC32 to 1 */
        A_REG_RMW_FIELD(pDev, MAC_USEC, 32, 31);

        /*
         * Restore BB registers to power-on defaults
         */
        A_REG_WR(pDev, PHY_SLEEP_CTR_CONTROL, 0x1f);
        A_REG_WR(pDev, PHY_SLEEP_CTR_LIMIT,   0x7f);
        if (IS_5312_2_X(pDev)) {
            /* Set ADC/DAC select values */
            A_REG_WR(pDev, PHY_SLEEP_SCAL, 0x00000004);
        } else {
            A_REG_WR(pDev, PHY_SLEEP_SCAL,        0x0e);
            A_REG_WR(pDev, PHY_M_SLEEP,           0x0c);
            A_REG_WR(pDev, PHY_REFCLKDLY,         0xff);
            if (IS_5112(pDev)) {
                A_REG_WR(pDev, PHY_REFCLKPD,      0x14); //reg 126 = 99f8
            } else {
                A_REG_WR(pDev, PHY_REFCLKPD,      0x18); //reg 126 = 99f8
            }
        }
    }
#endif
}

#ifndef BUILD_AP
/**************************************************************
 * ar5513IsSlowSleepClockAllowed
 *
 * If slow clock exists, turn it off and turn back on the refclk
 *
 * Bug 9975: Enable the 32 KHz clock only when the hardware
 * can goto sleep. Currently hardware can goto sleep when 
 * power save mode is enabled or radio is disabled
 */
static A_BOOL
ar5513IsSlowSleepClockAllowed(WLAN_DEV_INFO *pDev)
{
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
     return FALSE;
}
#endif

/**************************************************************
 * ar5513SwapHwDesc
 *
 * Swap hardware fields in the descriptor for compression
 */
void
ar5513SwapHwDesc(WLAN_DEV_INFO *pDev, ATHEROS_DESC *pDesc, ATHEROS_DESC *pTail, A_BOOL complete)
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
                pTempDesc->hw.word[4]    = cpu2le32(pTempDesc->hw.word[4]); 
                pTempDesc->hw.word[5]    = cpu2le32(pTempDesc->hw.word[5]); 
                pTempDesc->hw.word[6]    = cpu2le32(pTempDesc->hw.word[6]); 
                pTempDesc->hw.word[7]    = cpu2le32(pTempDesc->hw.word[7]); 
                if (complete) {
                    pTempDesc->hw.word[8]    = cpu2le32(pTempDesc->hw.word[8]); 
                    pTempDesc->hw.word[9]    = cpu2le32(pTempDesc->hw.word[9]);
                    pTempDesc->hw.word[10]    = cpu2le32(pTempDesc->hw.word[10]);
                }
                pTempDesc = pTempDesc->pNextVirtPtr;
            } while (pTempDesc != pFirstDesc->pTxLastDesc->pNextVirtPtr);
            pFirstDesc = pFirstDesc->pTxLastDesc->pNextVirtPtr;
        } while (pFirstDesc != pTail->pNextVirtPtr);
    }    
}

/**************************************************************
 * ar5513GetHwDescWord
 *
 * Get hardware specified word in the descriptor for compression
 */
A_UINT32
ar5513GetHwDescWord(WLAN_DEV_INFO *pDev, A_UINT32 hwDescWord)
{
    return (pDev->pHalInfo->swSwapDesc ? cpu2le32(hwDescWord) : hwDescWord);
}


/**************************************************************
 * ar5513GetApSwitchHelper
 *
 * Read some Beacon relevant registers in anticipation of saving
 * them for future restoration.
 */
void
ar5513GetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    if (pRegs) {
        pRegs->r1 = readPlatformReg(pDev, MAC_TIMER0);
        pRegs->r2 = readPlatformReg(pDev, MAC_TIMER1);
        pRegs->r3 = readPlatformReg(pDev, MAC_TIMER2);
        pRegs->r4 = readPlatformReg(pDev, MAC_QUIET1);
        pRegs->r5 = readPlatformReg(pDev, MAC_QUIET2);
    }

    /* TODO: temp for debug */
    ar5513MacStop(pDev);
}

/**************************************************************
 * ar5513SetApSwitchHelper
 *
 * Restore some Beacon relevant registers.
 */
void
ar5513SetApSwitchHelper(WLAN_DEV_INFO *pDev, SAVE_SIX_REG *pRegs)
{
    ASSERT(pRegs);

    writePlatformReg(pDev, MAC_TIMER0, pRegs->r1);
    writePlatformReg(pDev, MAC_TIMER1, pRegs->r2);
    writePlatformReg(pDev, MAC_TIMER2, pRegs->r3);
    writePlatformReg(pDev, MAC_QUIET1, pRegs->r4);
    writePlatformReg(pDev, MAC_QUIET2, pRegs->r5);

    /* Enable SWBA interrupt. */
    ar5513EnableInterrupts(pDev, HAL_INT_SWBA);
    ar5513EnableInterrupts(pDev, HAL_INT_TXDESC);

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
ar5513SendXrChirp(WLAN_DEV_INFO *pDev)
{
#if 0
    unsigned int x;
    int wait = 2;
#endif

    ASSERT((readPlatformReg(pDev, MAC_XRMODE) & MAC_XRMODE_XR_WAIT_FOR_POLL) == 0);

    if (A_REG_RD(pDev, MAC_XRCRP_SEND_CHIRP) == 1) {
        uiPrintf("ar5513SendXrChirp: double chirp in progress, send chirp ignored \n");
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

#endif /* #ifdef BUILD_AR5513 */
