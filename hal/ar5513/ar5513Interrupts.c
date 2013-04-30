/*
 * Copyright (c) 2003-2004 Atheros Communications, Inc., All Rights Reserved
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513Interrupts.c#4 $
 */

#ifdef BUILD_AR5513

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "ui.h"
#include "halApi.h"
#include "hal.h"

/* Headers for HW private items */
#include "ar5513MacReg.h"
#include "ar5513Interrupts.h"
#include "ar5513Mac.h"

/**************************************************************
 * ar5513IsInteruptPending
 *
 * Checks to see if an interrupt is pending on our NIC
 *
 * Returns: TRUE    if an interrupt is pending
 *          FALSE   if not
 */
A_BOOL
ar5513IsInterruptPending(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
#ifndef BUILD_AR5513
    /* 
     *  Some platforms trigger our ISR before applying power to
     *  the card, so make sure the INTPEND is really 1, not 0xffffffff.
     */
    A_UINT32 intPend = readPlatformReg(pDev, MAC_INTPEND);
    if (intPend == MAC_INTPEND_TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
#else
    A_UINT32 intEnable = readPlatformReg(pDev, MAC_IER);
    if (intEnable & MAC_IER_ENABLE) {
        return TRUE;
    } else {
        return FALSE;
    }
#endif
#elif defined(AR531X) || defined(AR5513)
    return TRUE;
#endif
}

/**************************************************************
 * ar5513GetInterrupts
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
ar5513GetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue, A_UINT32 *pDescQueueBitMask)
{
    A_UINT32        isr, isrS2 = 0, maskedIsr;
    HAL_INT_TYPE    ints = 0, maskedInts = 0;

    *pDescQueueBitMask = 0;
    isr = readPlatformReg(pDev, MAC_ISR);
    
    if (isr == 0xffffffff) {
        return HAL_INT_NOCARD;
    }

    maskedIsr = isr & pDev->MaskReg;
    ASSERT(maskedIsr);

    /* Mask out non-common interrupts.  These will be added in below. */
    maskedInts = maskedIsr & HAL_INT_COMMON;

   if (
#ifdef PCI_INTERFACE
        (maskedIsr & MAC_ISR_HIUERR) ||
#endif
        (maskedIsr & MAC_ISR_BCNMISC))
    {
        isrS2 = readPlatformReg(pDev, MAC_ISR_S2);
    }
#ifdef PCI_INTERFACE
    if (maskedIsr & MAC_ISR_HIUERR) {
        if (isrS2 & MAC_ISR_S2_MCABT) {
            isrPrintf("hwGetInterrupts: Bus Master Cycle Abort Error!\n");
        }
        if (isrS2 & MAC_ISR_S2_SSERR) {
            isrPrintf("hwGetInterrupts: Bus Signalled System Error!\n");
        }
        if (isrS2 & MAC_ISR_S2_DPERR) {
            isrPrintf("hwGetInterrupts: Bus Parity Error!\n");
        }
        maskedInts |= HAL_INT_FATAL;
    }
#endif

    if (maskedIsr & (MAC_ISR_RXOK | MAC_ISR_RXERR)) {
        maskedInts |= HAL_INT_RX;
    }
    if (maskedIsr & (MAC_ISR_TXOK | MAC_ISR_TXERR)) {
        maskedInts |= HAL_INT_TX;
    }

    if (maskedIsr & MAC_ISR_TXDESC) {
        maskedInts |= HAL_INT_TXDESC;
    }

    if ((maskedIsr & MAC_ISR_BCNMISC) && (isrS2 & MAC_ISR_S2_DTIM)) {
        maskedInts |= HAL_INT_DTIM;
    }

#ifdef AR5513_CCC
    if ((maskedIsr & MAC_ISR_BCNMISC) && (isrS2 & MAC_ISR_S2_BCNTO)) {
        maskedInts |= HAL_INT_BCNTO;
    }
#endif
    if (maskedIsr & MAC_ISR_RXCHIRP) {
        maskedInts |= HAL_INT_RXCHIRP;
    }
    /*
     * Receive Overuns happen quite frequently in Venice/Hainan based MAC 
     * when compression and fastframes are enabled as reported in 
     * bug 9208. Since the bug report also concludes that RXORN is 
     * not fatal anymore, software now uses it as information and 
     * carries on with its normal operation without doing a chip reset 
     */  
#if 0
    /* Receive overrun is usually non-fatal on Oahu/Spirit.
     *
     * BUT early silicon had a bug which causes the rx to fail and the chip
     * must be reset.
     *
     * AND even AR531X v3.1 and AR5211 v4.2 silicon seems to have a problem
     * with RXORN which hangs the receive path and requires a chip reset
     * to proceed (see bug 3996). So for now, we force a hardware reset in
     * all cases.
     */
    if (isr & MAC_ISR_RXORN) {
        isrPrintf("hwGetInterrupts: Receive FIFO Overrun Error!\n");
        maskedInts |= HAL_INT_FATAL;
    }
#endif
    /*
     * Now we take care of the unmasked interrupts.  We need to return certain
     * unmasked interrupts or we'll "lose" them after reading the ISR register.
     */
    ints = isr & HAL_INT_COMMON;

    if (isr & (MAC_ISR_RXOK | MAC_ISR_RXERR)) {
        ints |= HAL_INT_RX;
    }

    if (isr & (MAC_ISR_TXOK | MAC_ISR_TXERR)) {
        ints |= HAL_INT_TX;
    }

    if (isr & MAC_ISR_TXDESC) {
        ints |= HAL_INT_TXDESC;
        *pDescQueueBitMask = A_REG_RD_FIELD(pDev, MAC_ISR_S0, QCU_TXDESC);

        /*
         * Disable the interrupt caused by the queue by writing ones
         * to the secondary ISR.  This avoids the race condition
         * mentioned below.
         */
        A_REG_WR_FIELD(pDev, MAC_ISR_S0, QCU_TXDESC, *pDescQueueBitMask);

        /*
         * This avoids a race condition where a new TXDESC interrupt
         * could come in between reading the ISR and clearing the interrupt
         * via the primary ISR.  We therefore clear the interrupt via
         * the secondary, which avoids this race.
         */
        isr = isr & ~MAC_ISR_TXDESC;
    }
    /*
     * Clear the interrupts we've read by writing back ones in these locations
     * to the primary ISR, TXDESC excepted (see above).
     */
    writePlatformReg(pDev, MAC_ISR, isr);
#ifndef NDIS_HW  
    /* Flush the write to the Register */
    (void)readPlatformReg(pDev, MAC_ISR); 
#endif

#if AR_PB32
    sysPciIntrAck();
#endif

    /* Return the unmasked value. */
    *pUnmaskedValue = ints;

    /* Return the masked value. */
    return maskedInts;
}

/**************************************************************
 * ar5513EnableInterrupts
 *
 * Atomically enables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in ints.
 */
void
ar5513EnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
{
    A_UINT32    mask;
    A_BOOL      intsEnabled;
#if AR_PB32
    int         intKey = 0;
#endif

    if (pDev->powerMgmt.powerState == D3_STATE && 0) {
        A_UINT32 rdData;
        /*
         * We just wake up from D3 mode, need repogram DSL register for interrupt
         */
        uiPrintf ("wake up from D3 and reporgram DSL interrupt register\n");
        writePlatformReg(pDev, RST_CIMR, WMAC_INTERRUPT_MASK | WMAC_POLL_INTERRUPT_MASK);
        writePlatformReg(pDev, RST_IF_CTL, ENABLE_PCI_INTERFACE | PCI_CLIENT_INT_ENABLE);
        rdData = readPlatformReg(pDev, PCI_MCFG);
        rdData = rdData | ADDRESS_SHIFT;
        writePlatformReg(pDev, PCI_MCFG, rdData);
    }

    if (ints & HAL_INT_GLOBAL) {
        /* Assert that no other ints are combined with the global bit. */
        ASSERT((ints & ~HAL_INT_GLOBAL) == 0);

        ASSERT((pDev->pHalInfo->globIntRefCount) != 0);

        if(pDev->pHalInfo->globIntRefCount > 0) {
            pDev->pHalInfo->globIntRefCount--;
        }
        if (pDev->pHalInfo->globIntRefCount == 0) {
            writePlatformReg(pDev, MAC_IER, MAC_IER_ENABLE);
        }
        return;
    }

    /*
     * We have to disable interrupts here before reading & modifying
     * the mask so that the ISR does not modify the mask out from under us.
     */
    intsEnabled = (readPlatformReg(pDev, MAC_IER) == MAC_IER_ENABLE);
    if (intsEnabled) {
#if AR_PB32
        /*
         * On PB32, we'd have to do a sysPciIntrAck after
         * disabling interrupts on the PCI device.  But we'd
         * have to intLock/intUnlock around the whole sequence.
         * Rather than incur this extra expense, we choose to
         * simply disable interrupts at the processor rather
         * than at the device.
         */
        intKey = intLock();
#else
        writePlatformReg(pDev, MAC_IER, MAC_IER_DISABLE);
#ifndef NDIS_HW  
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
#endif
#endif
    }

    mask = pDev->MaskReg;

    /* Handle interrupts not common between platforms first. */

    if (ints & HAL_INT_TX) {
        mask |= MAC_IMR_TXOK | MAC_IMR_TXERR;
    }

    if (ints & HAL_INT_RX) {
        mask |= MAC_IMR_RXOK | MAC_IMR_RXERR;
    }

    if (ints & HAL_INT_TXDESC) {
        mask |= MAC_IMR_TXDESC;
    }

    /* Now handle the common interrupts. */
    ints &= HAL_INT_COMMON;
    mask |= ints;

    /* Update shadow first before writing out the new IMR */
    pDev->MaskReg = mask;
    writePlatformReg(pDev, MAC_IMR, mask);
#ifndef NDIS_HW  
    (void)readPlatformReg(pDev, MAC_IMR);   /* flush write to HW */
#endif

    /* Re-enable interrupts if they were enabled before. */
    if (intsEnabled) {
#if AR_PB32
        intUnlock(intKey);
#else
        writePlatformReg(pDev, MAC_IER, MAC_IER_ENABLE);
#endif
    }
}

/**************************************************************
 * ar5513DisableInterrupts
 *
 * Atomically disables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in "ints".
 */
void
ar5513DisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
{
    A_UINT32    mask;
    A_BOOL      intsEnabled;
#if AR_PB32
    int         intKey = 0;
#endif

    if (ints & HAL_INT_GLOBAL) {
#if AR_PB32
        intKey = intLock();
#endif
        /* Assert that no other ints are combined with the global bit. */
        ASSERT((ints & ~HAL_INT_GLOBAL) == 0);

        pDev->pHalInfo->globIntRefCount++;
        writePlatformReg(pDev, MAC_IER, MAC_IER_DISABLE);
#ifndef NDIS_HW  
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
#endif
#if AR_PB32
        sysPciIntrAck();
        intUnlock(intKey);
#endif
        return;
    }
    /*
     * We have to disable interrupts here before reading & modifying
     * the mask so that the ISR does not modify the mask out from under us.
     */
    intsEnabled = (readPlatformReg(pDev, MAC_IER) == MAC_IER_ENABLE);
    if (intsEnabled) {
#if AR_PB32
        intKey = intLock();
#else
        writePlatformReg(pDev, MAC_IER, MAC_IER_DISABLE);
#ifndef NDIS_HW  
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
#endif
#endif
    }
    mask = pDev->MaskReg;

    /* Handle bits not common across different HW platforms */
    
    if (ints & HAL_INT_TX) {
        mask &= ~(MAC_IMR_TXOK | MAC_IMR_TXERR);
    }

    if (ints & HAL_INT_RX) {
        mask &= ~(MAC_IMR_RXOK | MAC_IMR_RXERR);
    }

    if (ints & HAL_INT_TXDESC) {
        mask &= ~(MAC_IMR_TXDESC);
    }
    /* Handle common bits */
    
    ints &= HAL_INT_COMMON;
    mask &= ~ints;

    /* Write the new IMR and store off our SW copy. */
    writePlatformReg(pDev, MAC_IMR, mask);
#ifndef NDIS_HW  
    (void)readPlatformReg(pDev, MAC_IMR);   /* flush write to HW */
#endif
#if AR_PB32
    sysPciIntrAck();
#endif

    pDev->MaskReg = mask;

    /* Re-enable interrupts if they were enabled before. */
    if (intsEnabled) {
#if AR_PB32
        intUnlock(intKey);
#else
        writePlatformReg(pDev, MAC_IER, MAC_IER_ENABLE);
#endif
    }
}

#endif /* BUILD_AR5513 */
