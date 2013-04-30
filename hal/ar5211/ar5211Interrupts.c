/*
 * Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 * TODO: comment
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Interrupts.c#1 $
 */

#ifdef BUILD_AR5211

/* Standard HAL Headers */
#include "wlantype.h"
#include "wlandrv.h"
#include "ui.h"
#include "halApi.h"
#include "hal.h"

/* Headers for HW private items */
#include "ar5211Reg.h"
#include "ar5211Interrupts.h"
#include "ar5211.h"

/**************************************************************
 * ar5211IsInteruptPending
 *
 * Checks to see if an interrupt is pending on our NIC
 *
 * Returns: TRUE    if an interrupt is pending
 *          FALSE   if not
 */
A_BOOL
ar5211IsInterruptPending(WLAN_DEV_INFO *pDev)
{
#if defined(PCI_INTERFACE)
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
#elif defined(AR531X)
    return TRUE;
#endif
}

/**************************************************************
 * ar5211GetInterrupts
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
ar5211GetInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE *pUnmaskedValue,A_UINT32 *pDescQueueBitMask)
{
    A_UINT32        isr, maskedIsr;
    HAL_INT_TYPE    ints = 0, maskedInts = 0;

    *pDescQueueBitMask = 0;
    isr = readPlatformReg(pDev, MAC_ISR_RAC);

    if (isr == 0xffffffff) {
        return HAL_INT_NOCARD;
    }

#if AR_PB32
    sysPciIntrAck();
#endif

    maskedIsr = isr & pDev->MaskReg;
    ASSERT(maskedIsr);

    /* Mask out non-common interrupts.  These will be added in below. */
    maskedInts = maskedIsr & HAL_INT_COMMON;

#ifdef PCI_INTERFACE
    if (maskedIsr & MAC_ISR_HIUERR) {
        A_UINT32 temp = readPlatformReg(pDev, MAC_ISR_S2_S);

        if (temp & MAC_ISR_S2_MCABT) {
            isrPrintf("hwGetInterrupts: Bus Master Cycle Abort Error!\n");
        }
        if (temp & MAC_ISR_S2_SSERR) {
            isrPrintf("hwGetInterrupts: Bus Signalled System Error!\n");
        }
        if (temp & MAC_ISR_S2_DPERR) {
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
        *pDescQueueBitMask = A_REG_RD_FIELD(pDev, MAC_ISR_S0_S, QCU_TXDESC) ;
    }

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
    }

    /* Return the unmasked value. */
    *pUnmaskedValue = ints;

    /* Return the masked value. */
    return maskedInts;
}

/**************************************************************
 * ar5211EnableInterrupts
 *
 * Atomically enables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in ints.
 */
void
ar5211EnableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
{
    A_UINT32    mask;
    A_BOOL      intsEnabled;
#if AR_PB32
    int         intKey = 0;
#endif
    A_BOOL      txInt;

    if (ints & HAL_INT_GLOBAL) {
        /* Assert that no other ints are combined with the global bit. */
        ASSERT((ints & ~HAL_INT_GLOBAL) == 0);

        ASSERT(pDev->pHalInfo->globIntRefCount!=0);

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
        intKey = intLock();
#else
        writePlatformReg(pDev, MAC_IER, MAC_IER_DISABLE);
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
#endif
    }

    mask = pDev->MaskReg;

    /* Handle interrupts not common between platforms first. */

    /*
     * Build the value of mask, update pDev->MaskReg
     * and then only modify mask registers in hw.
     * This avoids a race condition with interrupt
     * being delivered too quickly.
     */
    txInt = FALSE;
    if (ints & HAL_INT_TX) {
        txInt = TRUE;
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

    /* Update shadow copy first before writing the IMR */
    pDev->MaskReg = mask;
    if (txInt) {
        A_UINT32 txMask;

        txMask = pDev->pHalInfo->txNormalIntMask;
        A_REG_RMW_FIELD(pDev, MAC_IMR_S0, QCU_TXOK, txMask);  /* TXOK */
        A_REG_RMW_FIELD(pDev, MAC_IMR_S1, QCU_TXERR, txMask); /* TXERR, not TXEOL */
        txMask |= MAC_IMR_S2_MCABT | MAC_IMR_S2_SSERR | MAC_IMR_S2_DPERR;
        writePlatformReg(pDev, MAC_IMR_S2, txMask); /* TXURN, et al */
    }
    if (ints & HAL_INT_TXDESC) {

        A_REG_RMW_FIELD(pDev, MAC_IMR_S0, QCU_TXDESC,
                        pDev->pHalInfo->txDescIntMask);  /* TXOK */
    }
    writePlatformReg(pDev, MAC_IMR, mask);
    (void)readPlatformReg(pDev, MAC_IMR);    /* flush write to HW */

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
 * ar5211DisableInterrupts
 *
 * Atomically disables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in "ints".
 */
void
ar5211DisableInterrupts(WLAN_DEV_INFO *pDev, HAL_INT_TYPE ints)
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
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
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
        (void)readPlatformReg(pDev, MAC_IER);   /* flush write to HW */
#endif
    }

    mask = pDev->MaskReg;

    /* Handle bits not common across different HW platforms */
    
    if (ints & HAL_INT_TX) {
        A_REG_RMW_FIELD(pDev, MAC_IMR_S0, QCU_TXOK, 0);  /* TXOK */
        A_REG_RMW_FIELD(pDev, MAC_IMR_S1, QCU_TXERR, 0); /* TXERR */
        mask &= ~(MAC_IMR_TXOK | MAC_IMR_TXERR);
    }

    if (ints & HAL_INT_RX) {
        mask &= ~(MAC_IMR_RXOK | MAC_IMR_RXERR);
    }
    if (ints & HAL_INT_TXDESC) {
        A_REG_RMW_FIELD(pDev, MAC_IMR_S0, QCU_TXDESC,0);
        mask &= ~(MAC_IMR_TXDESC);
    }

    /* Handle common bits */
    
    ints &= HAL_INT_COMMON;
    mask &= ~ints;

    /* Write the new IMR and store off our SW copy. */
    writePlatformReg(pDev, MAC_IMR, mask);
    (void)readPlatformReg(pDev, MAC_IMR); /* flush write to HW */
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


#endif /* BUILD_AR5211 */
