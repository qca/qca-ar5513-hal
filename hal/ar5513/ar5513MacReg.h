/*
 * Copyright 2003-2004 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Register definitions for Atheros AR5513 chipset
 *
 * Supports the following MAC revisions
 *      5    - Venice   (5212)
 *
 * A maximum of 10 QCUs and 10 DCUs are supported to provide
 * compatibility across all MAC revisions.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5513/ar5513MacReg.h#12 $
 */

#ifndef _AR5513_REG_H_
#define _AR5513_REG_H_

#ifdef _cplusplus
extern "C" {
#endif

enum ar5513Registers {
/* DMA Control and Interrupt Registers */
    MAC_CR               = 0x0008, // MAC Control Register - only write values of 1 have effect
    MAC_CR_RXE            = 0x00000004, // Receive enable
    MAC_CR_RXD            = 0x00000020, // Receive disable
    MAC_CR_SWI            = 0x00000040, // One-shot software interrupt

    MAC_RXDP             = 0x000C, // MAC receive queue descriptor pointer

    MAC_CFG              = 0x0014, // MAC configuration and status register
    MAC_CFG_SWTD          = 0x00000001, // byteswap tx descriptor words
    MAC_CFG_SWTB          = 0x00000002, // byteswap tx data buffer words
    MAC_CFG_SWRD          = 0x00000004, // byteswap rx descriptor words
    MAC_CFG_SWRB          = 0x00000008, // byteswap rx data buffer words
    MAC_CFG_SWRG          = 0x00000010, // byteswap register access data words
    MAC_CFG_AP_ADHOC_INDICATION = 0x00000020, // AP/adhoc indication (0-AP, 1-Adhoc)
    MAC_CFG_PHOK          = 0x00000100, // PHY OK status
    MAC_CFG_CLK_GATE_DIS  = 0x00000400, // Clock gating disable
#if defined(PCI_INTERFACE)
    MAC_CFG_EEBS          = 0x00000200, // EEPROM busy
    MAC_CFG_PCI_MASTER_REQ_Q_THRESH_M       = 0x00060000, // Mask of PCI core master request queue full threshold
    MAC_CFG_PCI_MASTER_REQ_Q_THRESH_S       = 17        , // Shift for PCI core master request queue full threshold
#endif /* PCI_INTERFACE */

    MAC_IER              = 0x0024, // MAC Interrupt enable register
    MAC_IER_ENABLE        = 0x00000001, // Global interrupt enable
    MAC_IER_DISABLE       = 0x00000000, // Global interrupt disable

    MAC_DMASIZE_4B        = 0x00000000, // DMA size 4 bytes (TXCFG + RXCFG)
    MAC_DMASIZE_8B        = 0x00000001, // DMA size 8 bytes
    MAC_DMASIZE_16B       = 0x00000002, // DMA size 16 bytes
    MAC_DMASIZE_32B       = 0x00000003, // DMA size 32 bytes
    MAC_DMASIZE_64B       = 0x00000004, // DMA size 64 bytes
    MAC_DMASIZE_128B      = 0x00000005, // DMA size 128 bytes
    MAC_DMASIZE_256B      = 0x00000006, // DMA size 256 bytes
    MAC_DMASIZE_512B      = 0x00000007, // DMA size 512 bytes

    MAC_TXCFG            = 0x0030, // MAC tx DMA size config register
    MAC_FTRIG_M           = 0x000003F0, // Mask for Frame trigger level
    MAC_FTRIG_S           = 4         , // Shift for Frame trigger level
    MAC_FTRIG_IMMED       = 0x00000000, // bytes in PCU TX FIFO before air
    MAC_FTRIG_64B         = 0x00000010, // default
    MAC_FTRIG_128B        = 0x00000020,
    MAC_FTRIG_192B        = 0x00000030,
    MAC_FTRIG_256B        = 0x00000040, // 5 bits total

    MAC_RXCFG            = 0x0034, // MAC rx DMA size config register
    MAC_RXCFG_CHIRP       = 0x00000008, // Only double chirps
    MAC_RXCFG_ZLFDMA      = 0x00000010, // Enable DMA of zero-length frame
    MAC_RXCFG_EN_JUM      = 0x00000020, // Enable jumbo rx descriptors
    MAC_RXCFG_WR_JUM      = 0x00000040, // Wrap jumbo rx descriptors


    MAC_JUMBO_LAST       = 0x0038, // Jumbo descriptor last address

    MAC_MIBC             = 0x0040, // MAC MIB control register
    MAC_MIBC_COW          = 0x00000001, // counter overflow warning
    MAC_MIBC_FMC          = 0x00000002, // freeze MIB counters
    MAC_MIBC_CMC          = 0x00000004, // clear MIB counters
    MAC_MIBC_MCS          = 0x00000008, // MIB counter strobe, increment all

    MAC_TOPS             = 0x0044, // MAC timeout prescale count
    MAC_TOPS_MASK         = 0x0000FFFF, // Mask for timeout prescale

    MAC_RXNPTO           = 0x0048, // MAC no frame received timeout
    MAC_RXNPTO_MASK       = 0x000003FF, // Mask for no frame received timeout

    MAC_TXNPTO           = 0x004C, // MAC no frame trasmitted timeout
    MAC_TXNPTO_MASK       = 0x000003FF, // Mask for no frame transmitted timeout
    MAC_TXNPTO_QCU_MASK   = 0x000FFC00, // Mask indicating the set of QCUs
                                        // for which frame completions will cause
                                        // a reset of the no frame transmitted timeout

    MAC_RPGTO            = 0x0050, // MAC receive frame gap timeout
    MAC_RPGTO_MASK        = 0x000003FF, // Mask for receive frame gap timeout

    MAC_RPCNT            = 0x0054, // MAC receive frame count limit
    MAC_RPCNT_MASK        = 0x0000001F, // Mask for receive frame count limit

    MAC_MACMISC           = 0x0058, // MAC miscellaneous control/status register
    MAC_MACMISC_DMA_OBS_M    = 0x000001E0, // Mask for DMA observation bus mux select
    MAC_MACMISC_DMA_OBS_S    = 5         , // Shift for DMA observation bus mux select
    MAC_MACMISC_MISC_OBS_M   = 0x00000E00, // Mask for MISC observation bus mux select
    MAC_MACMISC_MISC_OBS_S   = 9         , // Shift for MISC observation bus mux select
    MAC_MACMISC_MAC_OBS_BUS_LSB_M   = 0x00007000, // Mask for MAC observation bus mux select (lsb)
    MAC_MACMISC_MAC_OBS_BUS_LSB_S   = 12        , // Shift for MAC observation bus mux select (lsb)
    MAC_MACMISC_MAC_OBS_BUS_MSB_M   = 0x00038000, // Mask for MAC observation bus mux select (msb)
    MAC_MACMISC_MAC_OBS_BUS_MSB_S   = 15        , // Shift for MAC observation bus mux select (msb)

/* Interrupt Status Registers */
/*
* New to Venice:
*  Only the bits in this register (ISR_P) and the
*  primary interrupt mask register (IMR_P) control whether the MAC's
*  INTA# output is asserted.  The bits in the several secondary
*  interrupt status/mask registers control what bits are set in the
*  primary interrupt status register; however, the IMR_S* registers
*  DO NOT determine whether INTA# is asserted.  That is, INTA# is
*  asserted only when the logical AND of ISR_P amd IMR_P is
*  non-zero.  The secondary interrupt mask/status registers affect
*  what bits are set in ISR_P, but they do not directly affect
*  whether INTA# is asserted.
*/
    MAC_ISR             = 0x0080, // MAC Primary interrupt status register
    MAC_ISR_RXOK          = 0x00000001, // At least one frame received sans errors
    MAC_ISR_RXDESC        = 0x00000002, // Receive interrupt request
    MAC_ISR_RXERR         = 0x00000004, // Receive error interrupt
    MAC_ISR_RXNOPKT       = 0x00000008, // No frame received within timeout clock
    MAC_ISR_RXEOL         = 0x00000010, // Received descriptor empty interrupt
    MAC_ISR_RXORN         = 0x00000020, // Receive FIFO overrun interrupt
    MAC_ISR_TXOK          = 0x00000040, // Transmit okay interrupt
    MAC_ISR_TXDESC        = 0x00000080, // Transmit interrupt request
    MAC_ISR_TXERR         = 0x00000100, // Transmit error interrupt
    MAC_ISR_TXNOPKT       = 0x00000200, // No frame transmitted interrupt
    MAC_ISR_TXEOL         = 0x00000400, // Transmit descriptor empty interrupt
    MAC_ISR_TXURN         = 0x00000800, // Transmit FIFO underrun interrupt
    MAC_ISR_MIB           = 0x00001000, // MIB interrupt - see MIBC
    MAC_ISR_SWI           = 0x00002000, // Software interrupt
    MAC_ISR_RXPHY         = 0x00004000, // PHY receive error interrupt
    MAC_ISR_RXKCM         = 0x00008000, // Key-cache miss interrupt
    MAC_ISR_SWBA          = 0x00010000, // Software beacon alert interrupt
    MAC_ISR_BRSSI         = 0x00020000, // Beacon threshold interrupt
    MAC_ISR_BMISS         = 0x00040000, // Beacon missed interrupt
    MAC_ISR_HIUERR        = 0x00080000, // An unexpected bus error has occurred
    MAC_ISR_BNR           = 0x00100000, // Beacon not ready interrupt
    MAC_ISR_RXCHIRP       = 0x00200000, // Phy received a 'chirp'
    MAC_ISR_BCNMISC       = 0x00800000, // In venice, 'or' of TIM, CABEND, DTIMSYNC, BCNTO, CABTO, DTIM bits from ISR_S2
    MAC_ISR_TIM           = 0x00800000, // TIM interrupt
    MAC_ISR_GPIO          = 0x01000000, // GPIO Interrupt
    MAC_ISR_QCBROVF       = 0x02000000, // QCU CBR overflow interrupt
    MAC_ISR_QCBRURN       = 0x04000000, // QCU CBR underrun interrupt
    MAC_ISR_QTRIG         = 0x08000000, // QCU scheduling trigger interrupt
    MAC_ISR_RESV0         = 0xF0000000, // Reserved

    MAC_ISR_S0              = 0x0084, // MAC Secondary interrupt status register 0
    MAC_ISR_S0_QCU_TXOK_M    = 0x000003FF, // Mask for TXOK (QCU 0-9)
    MAC_ISR_S0_QCU_TXOK_S    = 0,          // Shift for TXOK (QCU 0-9)
    MAC_ISR_S0_QCU_TXDESC_M  = 0x03FF0000, // Mask for TXDESC (QCU 0-9)
    MAC_ISR_S0_QCU_TXDESC_S  = 16        , // Shift for TXDESC (QCU 0-9)


    MAC_ISR_S1             = 0x0088, // MAC Secondary interrupt status register 1
    MAC_ISR_S1_QCU_TXERR_M  = 0x000003FF, // Mask for TXERR (QCU 0-9)
    MAC_ISR_S1_QCU_TXERR_S  = 0,          // Shift for TXERR (QCU 0-9)
    MAC_ISR_S1_QCU_TXEOL_M  = 0x03FF0000, // Mask for TXEOL (QCU 0-9)
    MAC_ISR_S1_QCU_TXEOL_S  = 16        , // Shift for TXEOL (QCU 0-9)

    MAC_ISR_S2             = 0x008c, // MAC Secondary interrupt status register 2
    MAC_ISR_S2_QCU_TXURN_M  = 0x000003FF, // Mask for TXURN (QCU 0-9)
    MAC_ISR_S2_MCABT        = 0x00010000, // Master cycle abort interrupt
    MAC_ISR_S2_SSERR        = 0x00020000, // SERR interrupt
    MAC_ISR_S2_DPERR        = 0x00040000, // PCI bus parity error
    MAC_ISR_S2_TIM          = 0x01000000, // TIM
    MAC_ISR_S2_CABEND       = 0x02000000, // CABEND
    MAC_ISR_S2_DTIMSYNC     = 0x04000000, // DTIMSYNC
    MAC_ISR_S2_BCNTO        = 0x08000000, // BCNTO
    MAC_ISR_S2_CABTO        = 0x10000000, // CABTO
    MAC_ISR_S2_DTIM         = 0x20000000, // DTIM
    MAC_ISR_S2_RESV0        = 0xE0F8FC00, // Reserved

    MAC_ISR_S3             = 0x0090, // MAC Secondary interrupt status register 3
    MAC_ISR_S3_QCU_QCBROVF_M  = 0x000003FF, // Mask for QCBROVF (QCU 0-9)
    MAC_ISR_S3_QCU_QCBRURN_M  = 0x03FF0000, // Mask for QCBRURN (QCU 0-9)

    MAC_ISR_S4             = 0x0094, // MAC Secondary interrupt status register 4
    MAC_ISR_S4_QCU_QTRIG_M  = 0x000003FF, // Mask for QTRIG (QCU 0-9)
    MAC_ISR_S4_RESV0        = 0xFFFFFC00, // Reserved

/* Interrupt Mask Registers */
/*
 * Note for venice:
 *  Only the bits in this register control whether the MAC's INTA#
 *  output will be asserted.  The bits in the several secondary
 *  interrupt mask registers control what bits get set in the
 *  primary interrupt status register; however, the IMR_S* registers
 *   DO NOT determine whether INTA# is asserted.
 */
    MAC_IMR             = 0x00a0, // MAC Primary interrupt mask register
    MAC_IMR_RXOK          = 0x00000001, // At least one frame received sans errors
    MAC_IMR_RXDESC        = 0x00000002, // Receive interrupt request
    MAC_IMR_RXERR         = 0x00000004, // Receive error interrupt
    MAC_IMR_RXNOPKT       = 0x00000008, // No frame received within timeout clock
    MAC_IMR_RXEOL         = 0x00000010, // Received descriptor empty interrupt
    MAC_IMR_RXORN         = 0x00000020, // Receive FIFO overrun interrupt
    MAC_IMR_TXOK          = 0x00000040, // Transmit okay interrupt
    MAC_IMR_TXDESC        = 0x00000080, // Transmit interrupt request
    MAC_IMR_TXERR         = 0x00000100, // Transmit error interrupt
    MAC_IMR_TXNOPKT       = 0x00000200, // No frame transmitted interrupt
    MAC_IMR_TXEOL         = 0x00000400, // Transmit descriptor empty interrupt
    MAC_IMR_TXURN         = 0x00000800, // Transmit FIFO underrun interrupt
    MAC_IMR_MIB           = 0x00001000, // MIB interrupt - see MIBC
    MAC_IMR_SWI           = 0x00002000, // Software interrupt
    MAC_IMR_RXPHY         = 0x00004000, // PHY receive error interrupt
    MAC_IMR_RXKCM         = 0x00008000, // Key-cache miss interrupt
    MAC_IMR_SWBA          = 0x00010000, // Software beacon alert interrupt
    MAC_IMR_BRSSI         = 0x00020000, // Beacon threshold interrupt
    MAC_IMR_BMISS         = 0x00040000, // Beacon missed interrupt
    MAC_IMR_HIUERR        = 0x00080000, // An unexpected bus error has occurred
    MAC_IMR_BNR           = 0x00100000, // BNR interrupt
    MAC_IMR_RXCHIRP       = 0x00200000, // RXCHIRP interrupt
    MAC_IMR_BCNMISC       = 0x00800000, // Venice: BCNMISC
    MAC_IMR_TIM           = 0x00800000, // TIM interrupt
    MAC_IMR_GPIO          = 0x01000000, // GPIO Interrupt
    MAC_IMR_QCBROVF       = 0x02000000, // QCU CBR overflow interrupt
    MAC_IMR_QCBRURN       = 0x04000000, // QCU CBR underrun interrupt
    MAC_IMR_QTRIG         = 0x08000000, // QCU scheduling trigger interrupt
    MAC_IMR_RESV0         = 0xF0000000, // Reserved

    MAC_IMR_S0             = 0x00a4, // MAC Secondary interrupt mask register 0
    MAC_IMR_S0_QCU_TXOK_M    = 0x000003FF, // Mask for TXOK (QCU 0-9)
    MAC_IMR_S0_QCU_TXOK_S    = 0,          // Shift for TXOK (QCU 0-9)
    MAC_IMR_S0_QCU_TXDESC_M  = 0x03FF0000, // Mask for TXDESC (QCU 0-9)
    MAC_IMR_S0_QCU_TXDESC_S  = 16        , // Shift for TXDESC (QCU 0-9)

    MAC_IMR_S1             = 0x00a8, // MAC Secondary interrupt mask register 1
    MAC_IMR_S1_QCU_TXERR_M  = 0x000003FF, // Mask for TXERR (QCU 0-9)
    MAC_IMR_S1_QCU_TXERR_S  = 0,          // Shift for TXERR (QCU 0-9)
    MAC_IMR_S1_QCU_TXEOL_M  = 0x03FF0000, // Mask for TXEOL (QCU 0-9)
    MAC_IMR_S1_QCU_TXEOL_S  = 16        , // Shift for TXEOL (QCU 0-9)

    MAC_IMR_S2             = 0x00ac, // MAC Secondary interrupt mask register 2
    MAC_IMR_S2_QCU_TXURN_M  = 0x000003FF, // Mask for TXURN (QCU 0-9)
    MAC_IMR_S2_QCU_TXURN_S  = 0,          // Shift for TXURN (QCU 0-9)
    MAC_IMR_S2_MCABT        = 0x00010000, // Master cycle abort interrupt
    MAC_IMR_S2_SSERR        = 0x00020000, // SERR interrupt
    MAC_IMR_S2_DPERR        = 0x00040000, // PCI bus parity error
    MAC_IMR_S2_TIM          = 0x01000000, // TIM
    MAC_IMR_S2_CABEND       = 0x02000000, // CABEND
    MAC_IMR_S2_DTIMSYNC     = 0x04000000, // DTIMSYNC
    MAC_IMR_S2_BCNTO        = 0x08000000, // BCNTO
    MAC_IMR_S2_CABTO        = 0x10000000, // CABTO
    MAC_IMR_S2_DTIM         = 0x20000000, // DTIM
    MAC_IMR_S2_RESV0        = 0xE0F8FC00, // Reserved

    MAC_IMR_S3             = 0x00b0, // MAC Secondary interrupt mask register 3
    MAC_IMR_S3_QCU_QCBROVF_M  = 0x000003FF, // Mask for QCBROVF (QCU 0-9)
    MAC_IMR_S3_QCU_QCBRURN_M  = 0x03FF0000, // Mask for QCBRURN (QCU 0-9)
    MAC_IMR_S3_QCU_QCBRURN_S  = 16        , // Shift for QCBRURN (QCU 0-9)

    MAC_IMR_S4             = 0x00b4, // MAC Secondary interrupt mask register 4
    MAC_IMR_S4_QCU_QTRIG_M  = 0x000003FF, // Mask for QTRIG (QCU 0-9)
    MAC_IMR_S4_RESV0        = 0xFFFFFC00, // Reserved

/* Interrupt status registers (read-and-clear access, secondary shadow copies) */
    MAC_ISR_RAC            = 0x00c0, // MAC Primary interrupt status register,
                                     // read-and-clear access
    MAC_ISR_S0_S           = 0x00c4, // MAC Secondary interrupt status register 0,
                                     // shadow copy
    MAC_ISR_S0_S_QCU_TXOK_M    = 0x000003FF, // Mask for TXOK (QCU 0-9)
    MAC_ISR_S0_S_QCU_TXOK_S    = 0,          // Shift for TXOK (QCU 0-9)
    MAC_ISR_S0_S_QCU_TXDESC_M  = 0x03FF0000, // Mask for TXDESC (QCU 0-9)
    MAC_ISR_S0_S_QCU_TXDESC_S  = 16        , // Shift for TXDESC (QCU 0-9)
    
    MAC_ISR_S1_S           = 0x00c8, // MAC Secondary interrupt status register 1,
                                     // shadow copy
    MAC_ISR_S1_S_QCU_TXERR_M  = 0x000003FF, // Mask for TXERR (QCU 0-9)
    MAC_ISR_S1_S_QCU_TXERR_S  = 0,          // Shift for TXERR (QCU 0-9)
    MAC_ISR_S1_S_QCU_TXEOL_M  = 0x03FF0000, // Mask for TXEOL (QCU 0-9)
    MAC_ISR_S1_S_QCU_TXEOL_S  = 16        , // Shift for TXEOL (QCU 0-9)
    
    MAC_ISR_S2_S           = 0x00cc, // MAC Secondary interrupt status register 2,
                                     // shadow copy
    MAC_ISR_S3_S           = 0x00d0, // MAC Secondary interrupt status register 3,
                                     // shadow copy
    MAC_ISR_S4_S           = 0x00d4, // MAC Secondary interrupt status register 4,
                                     // shadow copy
    MAC_DMADBG_0           = 0x00e0, // MAC DMA Debug Registers
    MAC_DMADBG_1           = 0x00e4,
    MAC_DMADBG_2           = 0x00e8,
    MAC_DMADBG_3           = 0x00ec,
    MAC_DMADBG_4           = 0x00f0,
    MAC_DMADBG_5           = 0x00f4,
    MAC_DMADBG_6           = 0x00f8,
    MAC_DMADBG_7           = 0x00fc,

/* QCU registers */
    MAC_NUM_QCU      = 5,
    MAC_QCU_0        = 0x0001,
    MAC_QCU_1        = 0x0002,
    MAC_QCU_2        = 0x0004,
    MAC_QCU_3        = 0x0008,
    MAC_QCU_4        = 0x0010,
    MAC_QCU_5        = 0x0020,
    MAC_QCU_6        = 0x0040,
    MAC_QCU_7        = 0x0080,
    MAC_QCU_8        = 0x0100,
    MAC_QCU_9        = 0x0200,

    MAC_Q0_TXDP           = 0x0800, // MAC Transmit Queue descriptor pointer
    MAC_Q1_TXDP           = 0x0804, // MAC Transmit Queue descriptor pointer
    MAC_Q2_TXDP           = 0x0808, // MAC Transmit Queue descriptor pointer
    MAC_Q3_TXDP           = 0x080c, // MAC Transmit Queue descriptor pointer
    MAC_Q4_TXDP           = 0x0810, // MAC Transmit Queue descriptor pointer
    MAC_Q5_TXDP           = 0x0814, // MAC Transmit Queue descriptor pointer
    MAC_Q6_TXDP           = 0x0818, // MAC Transmit Queue descriptor pointer
    MAC_Q7_TXDP           = 0x081c, // MAC Transmit Queue descriptor pointer
    MAC_Q8_TXDP           = 0x0820, // MAC Transmit Queue descriptor pointer
    MAC_Q9_TXDP           = 0x0824, // MAC Transmit Queue descriptor pointer

    MAC_Q_TXE             = 0x0840, // MAC Transmit Queue enable
    MAC_Q_TXE_M              = 0x000003FF, // Mask for TXE (QCU 0-9)

    MAC_Q_TXD             = 0x0880, // MAC Transmit Queue disable
    MAC_Q_TXD_M              = 0x000003FF, // Mask for TXD (QCU 0-9)

    MAC_Q0_CBRCFG         = 0x08c0, // MAC CBR configuration
    MAC_Q1_CBRCFG         = 0x08c4, // MAC CBR configuration
    MAC_Q2_CBRCFG         = 0x08c8, // MAC CBR configuration
    MAC_Q3_CBRCFG         = 0x08cc, // MAC CBR configuration
    MAC_Q4_CBRCFG         = 0x08d0, // MAC CBR configuration
    MAC_Q5_CBRCFG         = 0x08d4, // MAC CBR configuration
    MAC_Q6_CBRCFG         = 0x08d8, // MAC CBR configuration
    MAC_Q7_CBRCFG         = 0x08dc, // MAC CBR configuration
    MAC_Q8_CBRCFG         = 0x08e0, // MAC CBR configuration
    MAC_Q9_CBRCFG         = 0x08e4, // MAC CBR configuration
    MAC_Q_CBRCFG_INTERVAL_M   = 0x00FFFFFF, // Mask for CBR interval (us)
    MAC_Q_CBRCFG_INTERVAL_S   = 0,          // Shift for CBR interval (us)
    MAC_Q_CBRCFG_OVF_THRESH_M = 0xFF000000, // Mask for CBR overflow threshold
    MAC_Q_CBRCFG_OVF_THRESH_S = 24,         // Shift for CBR overflow threshold

    MAC_Q0_RDYTIMECFG         = 0x0900, // MAC ReadyTime configuration
    MAC_Q1_RDYTIMECFG         = 0x0904, // MAC ReadyTime configuration
    MAC_Q2_RDYTIMECFG         = 0x0908, // MAC ReadyTime configuration
    MAC_Q3_RDYTIMECFG         = 0x090c, // MAC ReadyTime configuration
    MAC_Q4_RDYTIMECFG         = 0x0910, // MAC ReadyTime configuration
    MAC_Q5_RDYTIMECFG         = 0x0914, // MAC ReadyTime configuration
    MAC_Q6_RDYTIMECFG         = 0x0918, // MAC ReadyTime configuration
    MAC_Q7_RDYTIMECFG         = 0x091c, // MAC ReadyTime configuration
    MAC_Q8_RDYTIMECFG         = 0x0920, // MAC ReadyTime configuration
    MAC_Q9_RDYTIMECFG         = 0x0924, // MAC ReadyTime configuration
    MAC_Q_RDYTIMECFG_DURATION_M = 0x00FFFFFF, // Mask for ReadyTime duration (us)
    MAC_Q_RDYTIMECFG_DURATION_S = 0,          // Shift for ReadyTime duration (us)
    MAC_Q_RDYTIMECFG_EN         = 0x01000000, // ReadyTime enable

    MAC_Q_ONESHOTARM_SC       = 0x0940,     // MAC OneShotArm set control
    MAC_Q_ONESHOTARM_SC_M     = 0x000003FF, // Mask for MAC_Q_ONESHOTARM_SC (QCU 0-9)
    MAC_Q_ONESHOTARM_SC_RESV0 = 0xFFFFFC00, // Reserved

    MAC_Q_ONESHOTARM_CC       = 0x0980,     // MAC OneShotArm clear control
    MAC_Q_ONESHOTARM_CC_M     = 0x000003FF, // Mask for MAC_Q_ONESHOTARM_CC (QCU 0-9)
    MAC_Q_ONESHOTARM_CC_RESV0 = 0xFFFFFC00, // Reserved

    MAC_Q0_MISC         = 0x09c0, // MAC Miscellaneous QCU settings
    MAC_Q1_MISC         = 0x09c4, // MAC Miscellaneous QCU settings
    MAC_Q2_MISC         = 0x09c8, // MAC Miscellaneous QCU settings
    MAC_Q3_MISC         = 0x09cc, // MAC Miscellaneous QCU settings
    MAC_Q4_MISC         = 0x09d0, // MAC Miscellaneous QCU settings
    MAC_Q5_MISC         = 0x09d4, // MAC Miscellaneous QCU settings
    MAC_Q6_MISC         = 0x09d8, // MAC Miscellaneous QCU settings
    MAC_Q7_MISC         = 0x09dc, // MAC Miscellaneous QCU settings
    MAC_Q8_MISC         = 0x09e0, // MAC Miscellaneous QCU settings
    MAC_Q9_MISC         = 0x09e4, // MAC Miscellaneous QCU settings
    MAC_Q_MISC_FSP_M                   = 0x0000000F, // Mask for Frame Scheduling Policy
    MAC_Q_MISC_FSP_ASAP                = 0         , // ASAP
    MAC_Q_MISC_FSP_CBR                 = 1         , // CBR
    MAC_Q_MISC_FSP_DBA_GATED           = 2         , // DMA Beacon Alert gated
    MAC_Q_MISC_FSP_TIM_GATED           = 3         , // TIM gated
    MAC_Q_MISC_FSP_BEACON_SENT_GATED   = 4         , // Beacon-sent-gated
    MAC_Q_MISC_ONE_SHOT_EN             = 0x00000010, // OneShot enable
    MAC_Q_MISC_CBR_INCR_DIS1           = 0x00000020, // Disable CBR expired counter incr (empty q)
    MAC_Q_MISC_CBR_INCR_DIS0           = 0x00000040, // Disable CBR expired counter incr (empty beacon q)
    MAC_Q_MISC_BEACON_USE              = 0x00000080, // Beacon use indication
    MAC_Q_MISC_CBR_EXP_CNTR_LIMIT_EN   = 0x00000100, // CBR expired counter limit enable
    MAC_Q_MISC_RDYTIME_EXP_POLICY      = 0x00000200, // Enable TXE cleared on ReadyTime expired or VEOL
    MAC_Q_MISC_RESET_CBR_EXP_CTR       = 0x00000400, // Reset CBR expired counter
    MAC_Q_MISC_DCU_EARLY_TERM_REQ      = 0x00000800, // DCU frame early termination request control
    MAC_Q_MISC_QCU_COMP_EN             = 0x00001000, // QCU frame compression enable
    MAC_Q_MISC_RESV0                   = 0xFFFFF000, // Reserved

    MAC_Q0_STS         = 0x0a00, // MAC Miscellaneous QCU status
    MAC_Q1_STS         = 0x0a04, // MAC Miscellaneous QCU status
    MAC_Q2_STS         = 0x0a08, // MAC Miscellaneous QCU status
    MAC_Q3_STS         = 0x0a0c, // MAC Miscellaneous QCU status
    MAC_Q4_STS         = 0x0a10, // MAC Miscellaneous QCU status
    MAC_Q5_STS         = 0x0a14, // MAC Miscellaneous QCU status
    MAC_Q6_STS         = 0x0a18, // MAC Miscellaneous QCU status
    MAC_Q7_STS         = 0x0a1c, // MAC Miscellaneous QCU status
    MAC_Q8_STS         = 0x0a20, // MAC Miscellaneous QCU status
    MAC_Q9_STS         = 0x0a24, // MAC Miscellaneous QCU status
    MAC_Q_STS_PEND_FR_CNT_M        = 0x00000003, // Mask for Pending Frame Count
    MAC_Q_STS_RESV0                = 0x000000FC, // Reserved
    MAC_Q_STS_CBR_EXP_CNT_M        = 0x0000FF00, // Mask for CBR expired counter
    MAC_Q_STS_RESV1                = 0xFFFF0000, // Reserved

    MAC_Q_RDYTIMESHDN    = 0x0a40,     // MAC ReadyTimeShutdown status
    MAC_Q_RDYTIMESHDN_M  = 0x000003FF, // Mask for ReadyTimeShutdown status (QCU 0-9)

    MAC_Q_CBBS           = 0xb00,          // Compression buffer base select
    MAC_Q_CBBS_M           = 0x0000007F,   // mask for MAC_Q_CBBS
    MAC_Q_CBBS_RESV0       = 0xFFFFFF80,   // Reserved

    MAC_Q_CBBA           = 0xb04,          // Compression buffer base access
    MAC_Q_CBBA_M           = 0xFFFFFE00,   // mask for MAC_Q_CBBA
    MAC_Q_CBBA_RESV0       = 0x000001FF,   // Reserved
     
    MAC_Q_CBC            = 0xb08,          // Compression buffer configuration
    MAC_Q_CBC_SIZE_M       = 0x0000000F,   // mask for compression buffer size
    MAC_Q_CBC_RESV0        = 0xFFFFFFF0,   // Reserved

/* DCU registers */
    MAC_NUM_DCU      = 5,
    MAC_DCU_0        = 0x0001,
    MAC_DCU_1        = 0x0002,
    MAC_DCU_2        = 0x0004,
    MAC_DCU_3        = 0x0008,
    MAC_DCU_4        = 0x0010,
    MAC_DCU_5        = 0x0020,
    MAC_DCU_6        = 0x0040,
    MAC_DCU_7        = 0x0080,
    MAC_DCU_8        = 0x0100,
    MAC_DCU_9        = 0x0200,

    MAC_D0_QCUMASK     = 0x1000, // MAC QCU Mask
    MAC_D1_QCUMASK     = 0x1004, // MAC QCU Mask
    MAC_D2_QCUMASK     = 0x1008, // MAC QCU Mask
    MAC_D3_QCUMASK     = 0x100c, // MAC QCU Mask
    MAC_D4_QCUMASK     = 0x1010, // MAC QCU Mask
    MAC_D5_QCUMASK     = 0x1014, // MAC QCU Mask
    MAC_D6_QCUMASK     = 0x1018, // MAC QCU Mask
    MAC_D7_QCUMASK     = 0x101c, // MAC QCU Mask
    MAC_D8_QCUMASK     = 0x1020, // MAC QCU Mask
    MAC_D9_QCUMASK     = 0x1024, // MAC QCU Mask
    MAC_D_QCUMASK_M       = 0x000003FF, // Mask for QCU Mask (QCU 0-9)
    MAC_D_QCUMASK_RESV0   = 0xFFFFFC00, // Reserved

    MAC_D0_LCL_IFS     = 0x1040, // MAC DCU-specific IFS settings
    MAC_D1_LCL_IFS     = 0x1044, // MAC DCU-specific IFS settings
    MAC_D2_LCL_IFS     = 0x1048, // MAC DCU-specific IFS settings
    MAC_D3_LCL_IFS     = 0x104c, // MAC DCU-specific IFS settings
    MAC_D4_LCL_IFS     = 0x1050, // MAC DCU-specific IFS settings
    MAC_D5_LCL_IFS     = 0x1054, // MAC DCU-specific IFS settings
    MAC_D6_LCL_IFS     = 0x1058, // MAC DCU-specific IFS settings
    MAC_D7_LCL_IFS     = 0x105c, // MAC DCU-specific IFS settings
    MAC_D8_LCL_IFS     = 0x1060, // MAC DCU-specific IFS settings
    MAC_D9_LCL_IFS     = 0x1064, // MAC DCU-specific IFS settings
    MAC_D_LCL_IFS_CWMIN_M     = 0x000003FF, // Mask for CW_MIN
    MAC_D_LCL_IFS_CWMIN_S     = 0,          // Shift for CW_MIN
    MAC_D_LCL_IFS_CWMAX_M     = 0x000FFC00, // Mask for CW_MAX
    MAC_D_LCL_IFS_CWMAX_S     = 10        , // Shift for CW_MAX
    MAC_D_LCL_IFS_AIFS_M      = 0x0FF00000, // Mask for AIFS
    MAC_D_LCL_IFS_AIFS_S      = 20        , // Shift for AIFS
    /*
     *  Note:  even though this field is 8 bits wide, the
     *  maximum supported AIFS value is 0xfc.  Setting the AIFS value
     *  to 0xfd, 0xfe, or 0xff will not work correctly and will cause
     *  the DCU to hang.
     */
    MAC_D_LCL_IFS_RESV0       = 0xF0000000, // Reserved

    MAC_D0_RETRY_LIMIT     = 0x1080, // MAC Retry limits
    MAC_D1_RETRY_LIMIT     = 0x1084, // MAC Retry limits
    MAC_D2_RETRY_LIMIT     = 0x1088, // MAC Retry limits
    MAC_D3_RETRY_LIMIT     = 0x108c, // MAC Retry limits
    MAC_D4_RETRY_LIMIT     = 0x1090, // MAC Retry limits
    MAC_D5_RETRY_LIMIT     = 0x1094, // MAC Retry limits
    MAC_D6_RETRY_LIMIT     = 0x1098, // MAC Retry limits
    MAC_D7_RETRY_LIMIT     = 0x109c, // MAC Retry limits
    MAC_D8_RETRY_LIMIT     = 0x10a0, // MAC Retry limits
    MAC_D9_RETRY_LIMIT     = 0x10a4, // MAC Retry limits
    MAC_D_RETRY_LIMIT_FR_SH_M     = 0x0000000F, // Mask for frame short retry limit
    MAC_D_RETRY_LIMIT_FR_SH_S     = 0,          // Shift for frame short retry limit
    MAC_D_RETRY_LIMIT_FR_LG_M     = 0x000000F0, // Mask for frame long retry limit
    MAC_D_RETRY_LIMIT_FR_LG_S     = 4,          // Shift for frame long retry limit
    MAC_D_RETRY_LIMIT_STA_SH_M    = 0x00003F00, // Mask for station short retry limit
    MAC_D_RETRY_LIMIT_STA_SH_S    = 8,          // Shift for station short retry limit
    MAC_D_RETRY_LIMIT_STA_LG_M    = 0x000FC000, // Mask for station short retry limit
    MAC_D_RETRY_LIMIT_STA_LG_S    = 14,         // Shift for station short retry limit
    MAC_D_RETRY_LIMIT_RESV0       = 0xFFF00000, // Reserved

    MAC_D0_CHNTIME     = 0x10c0, // MAC ChannelTime settings
    MAC_D1_CHNTIME     = 0x10c4, // MAC ChannelTime settings
    MAC_D2_CHNTIME     = 0x10c8, // MAC ChannelTime settings
    MAC_D3_CHNTIME     = 0x10cc, // MAC ChannelTime settings
    MAC_D4_CHNTIME     = 0x10d0, // MAC ChannelTime settings
    MAC_D5_CHNTIME     = 0x10d4, // MAC ChannelTime settings
    MAC_D6_CHNTIME     = 0x10d8, // MAC ChannelTime settings
    MAC_D7_CHNTIME     = 0x10dc, // MAC ChannelTime settings
    MAC_D8_CHNTIME     = 0x10e0, // MAC ChannelTime settings
    MAC_D9_CHNTIME     = 0x10e4, // MAC ChannelTime settings
    MAC_D_CHNTIME_DUR_M       = 0x000FFFFF, // Mask for ChannelTime duration (us)
    MAC_D_CHNTIME_DUR_S       = 0,          // Shift for ChannelTime duration (us)
    MAC_D_CHNTIME_EN          = 0x00100000, // ChannelTime enable
    MAC_D_CHNTIME_RESV0       = 0xFFE00000, // Reserved

    MAC_D0_MISC        = 0x1100, // MAC Miscellaneous DCU-specific settings
    MAC_D1_MISC        = 0x1104, // MAC Miscellaneous DCU-specific settings
    MAC_D2_MISC        = 0x1108, // MAC Miscellaneous DCU-specific settings
    MAC_D3_MISC        = 0x110c, // MAC Miscellaneous DCU-specific settings
    MAC_D4_MISC        = 0x1110, // MAC Miscellaneous DCU-specific settings
    MAC_D5_MISC        = 0x1114, // MAC Miscellaneous DCU-specific settings
    MAC_D6_MISC        = 0x1118, // MAC Miscellaneous DCU-specific settings
    MAC_D7_MISC        = 0x111c, // MAC Miscellaneous DCU-specific settings
    MAC_D8_MISC        = 0x1120, // MAC Miscellaneous DCU-specific settings
    MAC_D9_MISC        = 0x1124, // MAC Miscellaneous DCU-specific settings
    MAC_D_MISC_BKOFF_THRESH_M      = 0x0000003F, // Mask for Backoff threshold setting
    MAC_D_MISC_RETRY_CNT_RESET_EN  = 0x00000040, // End of tx series station RTS/data failure count reset policy
    MAC_D_MISC_CW_RESET_EN         = 0x00000080, // End of tx series CW reset enable
    MAC_D_MISC_FRAG_WAIT_EN        = 0x00000100, // Fragment Starvation Policy
    MAC_D_MISC_FRAG_BKOFF_EN       = 0x00000200, // Backoff during a frag burst
    MAC_D_MISC_HCF_POLL_EN         = 0x00000800, // HFC poll enable
    MAC_D_MISC_CW_BKOFF_EN         = 0x00001000, // Use binary exponential CW backoff
    MAC_D_MISC_VIR_COL_HANDLING_M  = 0x0000C000, // Mask for Virtual collision handling policy
    MAC_D_MISC_VIR_COL_HANDLING_S  = 14,         // Shift for Virtual collision handling policy
    MAC_D_MISC_VIR_COL_HANDLING_DEFAULT = 0    , // Normal
    MAC_D_MISC_VIR_COL_HANDLING_IGNORE  = 1    , // Ignore
    MAC_D_MISC_BEACON_USE          = 0x00010000, // Beacon use indication
    MAC_D_MISC_ARB_LOCKOUT_CNTRL_M = 0x00060000, // Mask for DCU arbiter lockout control
    MAC_D_MISC_ARB_LOCKOUT_CNTRL_S = 17        , // Shift for DCU arbiter lockout control
    MAC_D_MISC_ARB_LOCKOUT_CNTRL_NONE     = 0  , // No lockout
    MAC_D_MISC_ARB_LOCKOUT_CNTRL_INTRA_FR = 1  , // Intra-frame
    MAC_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL   = 2  , // Global
    MAC_D_MISC_ARB_LOCKOUT_IGNORE  = 0x00080000, // DCU arbiter lockout ignore control
    MAC_D_MISC_SEQ_NUM_INCR_DIS    = 0x00100000, // Sequence number increment disable
    MAC_D_MISC_POST_FR_BKOFF_DIS   = 0x00200000, // Post-frame backoff disable
    MAC_D_MISC_VIT_COL_CW_BKOFF_EN = 0x00400000, // Virtual coll. handling policy
    MAC_D_MISC_BLOWN_IFS_RETRY_EN  = 0x00800000, // Initiate Retry procedure on Blown IFS
    MAC_D_MISC_RESV0               = 0xFF000000, // Reserved

    MAC_D0_SEQNUM      = 0x1140, // MAC Frame sequence number control/status
    MAC_D1_SEQNUM      = 0x1144, // MAC Frame sequence number control/status
    MAC_D2_SEQNUM      = 0x1148, // MAC Frame sequence number control/status
    MAC_D3_SEQNUM      = 0x114c, // MAC Frame sequence number control/status
    MAC_D4_SEQNUM      = 0x1150, // MAC Frame sequence number control/status
    MAC_D5_SEQNUM      = 0x1154, // MAC Frame sequence number control/status
    MAC_D6_SEQNUM      = 0x1158, // MAC Frame sequence number control/status
    MAC_D7_SEQNUM      = 0x115c, // MAC Frame sequence number control/status
    MAC_D8_SEQNUM      = 0x1160, // MAC Frame sequence number control/status
    MAC_D9_SEQNUM      = 0x1164, // MAC Frame sequence number control/status
    MAC_D_SEQNUM_M     = 0x00000FFF, // Mask for value of sequence number
    MAC_D_SEQNUM_RESV0  = 0xFFFFF000, // Reserved

    MAC_D_GBL_IFS_SIFS        = 0x1030, // MAC DCU-global IFS settings: SIFS duration
    MAC_D_GBL_IFS_SIFS_M       = 0x0000FFFF, // Mask for SIFS duration (core clocks)
    MAC_D_GBL_IFS_SIFS_RESV0   = 0xFFFFFFFF, // Reserved

    MAC_D_GBL_IFS_SLOT        = 0x1070, // MAC DCU-global IFS settings: slot duration
    MAC_D_GBL_IFS_SLOT_M       = 0x0000FFFF, // Mask for Slot duration (core clocks)
    MAC_D_GBL_IFS_SLOT_RESV0   = 0xFFFF0000, // Reserved

    MAC_D_GBL_IFS_EIFS        = 0x10b0, // MAC DCU-global IFS settings: EIFS duration
    MAC_D_GBL_IFS_EIFS_M       = 0x0000FFFF, // Mask for Slot duration (core clocks)
    MAC_D_GBL_IFS_EIFS_RESV0   = 0xFFFF0000, // Reserved

    MAC_D_GBL_IFS_MISC        = 0x10f0, // MAC DCU-global IFS settings: Miscellaneous
    MAC_D_GBL_IFS_MISC_LFSR_SLICE_SEL_M     = 0x00000007, // Mask forLFSR slice select
    MAC_D_GBL_IFS_MISC_TURBO_MODE           = 0x00000008, // Turbo mode indication
    MAC_D_GBL_IFS_MISC_SIFS_DURATION_USEC_M = 0x000003F0, // Mask for SIFS duration (us)
    MAC_D_GBL_IFS_MISC_USEC_DURATION_M      = 0x000FFC00, // Mask for microsecond duration
    MAC_D_GBL_IFS_MISC_DCU_ARBITER_DLY_M    = 0x00300000, // Mask for DCU arbiter delay
    MAC_D_GBL_IFS_MISC_RESV0                = 0xFFC00000, // Reserved

    MAC_D_TXBLK_BASE           = 0x00001038,
    MAC_D_TXBLK_WRITE_BITMASK_M  = 0x0000FFFF,   // Mask for bitmask
    MAC_D_TXBLK_WRITE_BITMASK_S  = 0,            // Shift for bitmask
    MAC_D_TXBLK_WRITE_SLICE_M    = 0x000F0000,   // Mask for slice
    MAC_D_TXBLK_WRITE_SLICE_S    = 16,           // Shift for slice
    MAC_D_TXBLK_WRITE_DCU_M      = 0x00F00000,   // Mask for DCU number
    MAC_D_TXBLK_WRITE_DCU_S      = 20,           // Shift for DCU number
    MAC_D_TXBLK_WRITE_COMMAND_M  = 0x0F000000,   // Mask for command
    MAC_D_TXBLK_WRITE_COMMAND_S  = 24,           // Shift for command

    MAC_D_FPCTL                = 0x1230,         // DCU frame prefetch settings
    MAC_D_FPCTL_DCU_M            = 0x0000000F,   // Mask for DCU for which prefetch is enabled
    MAC_D_FPCTL_DCU_S            = 0,            // Shift for DCU for which prefetch is enabled
    MAC_D_FPCTL_PREFETCH_EN      = 0x00000010,   // Enable prefetch for normal (non-burst) operation
    MAC_D_FPCTL_BURST_PREFETCH_M = 0x00007FE0,   // Mask for Burst frame prefetch per DCU
    MAC_D_FPCTL_BURST_PREFETCH_S = 5,            // Shift for Burst frame prefetch per DCU

    MAC_D_TXPSE                = 0x1270, // MAC DCU transmit pause control/status
    MAC_D_TXPSE_CTRL_M          = 0x000003FF, // Mask of DCUs to pause (DCUs 0-9)
    MAC_D_TXPSE_RESV0           = 0x0000FC00, // Reserved
    MAC_D_TXPSE_STATUS          = 0x00010000, // Transmit pause status
    MAC_D_TXPSE_RESV1           = 0xFFFE0000, // Reserved

#if defined(PCI_INTERFACE)
// DMA & PCI Registers in PCI space (usable during sleep)
    MAC_RC               = 0x4000, // Warm reset control register
    MAC_RC_MAC            = 0x00000001, // MAC reset
    MAC_RC_BB             = 0x00000002, // Baseband reset
    MAC_RC_RESV0          = 0x00000004, // Reserved
    MAC_RC_RESV1          = 0x00000008, // Reserved
    MAC_RC_PCI            = 0x00000010, // PCI-core reset

    MAC_SCR              = 0x4004, // Sleep control register
    MAC_SCR_SLDUR_M       = 0x0000ffff, // sleep duration mask, units of 128us
    MAC_SCR_SLDUR_S       = 0,          // sleep duration shift
    MAC_SCR_SLMODE_M      = 0x00030000, // sleep mode mask
    MAC_SCR_SLMODE_S      = 16,         // sleep mode bits shift
    MAC_SLMODE_FWAKE      = 0,          // force wake
    MAC_SLMODE_FSLEEP     = 1,          // force sleep
    MAC_SLMODE_NORMAL     = 2,          // sleep logic normal operation
    MAC_SCR_SLDTP         = 0x00040000, // sleep duration timing policy
    MAC_SCR_SLDWP         = 0x00080000, // sleep duration write policy
    MAC_SCR_SLEPOL        = 0x00100000, // sleep policy mode

    MAC_INTPEND          = 0x4008, // Interrupt Pending register
    MAC_INTPEND_TRUE      = 0x00000001, // interrupt pending

    MAC_SFR              = 0x400C, // Sleep force register
    MAC_SFR_SLEEP         = 0x00000001, // force sleep

    MAC_PCICFG           = 0x4010, // PCI configuration register
    MAC_PCICFG_SLEEP_CLK_SEL_M       = 0x00000002, // Mask for sleep clock select
    MAC_PCICFG_SLEEP_CLK_SEL_S       = 1,          // Shift for sleep clock select
    MAC_PCICFG_CLKRUNEN   = 0x00000004, // enable PCI CLKRUN function
    MAC_PCICFG_EEPROM_SIZE_M = 0x00000018, // Mask for EEPROM size
    MAC_PCICFG_EEPROM_SIZE_S =   3,        // Mask for EEPROM size
    MAC_PCICFG_EEPROM_SIZE_4K =  0,        // EEPROM size 4 Kbit
    MAC_PCICFG_EEPROM_SIZE_8K =  1,        // EEPROM size 8 Kbit
    MAC_PCICFG_EEPROM_SIZE_16K = 2,        // EEPROM size 16 Kbit
    MAC_PCICFG_EEPROM_SIZE_FAILED = 3,      // Failure
    MAC_PCICFG_ASSOC_STATUS_M  = 0x00000060,// Mask for Association Status
    MAC_PCICFG_ASSOC_STATUS_S  = 5,         // Shift for Association Status
    MAC_PCICFG_ASSOC_STATUS_NONE =         0,
    MAC_PCICFG_ASSOC_STATUS_PENDING =      1,
    MAC_PCICFG_ASSOC_STATUS_ASSOCIATED =   2,
    MAC_PCICFG_PCI_BUS_SEL_M         = 0x00000380, // Mask for PCI observation bus mux select
    MAC_PCICFG_DIS_CBE_FIX           = 0x00000400, // Disable fix for bad PCI CBE# generation
    MAC_PCICFG_SL_INTEN              = 0x00000800, // enable interrupt line assertion when asleep
    MAC_PCICFG_RESV0                 = 0x00001000, // Reserved
    MAC_PCICFG_SL_INPEN              = 0x00002000, // Force asleep when an interrupt is pending
    MAC_PCICFG_RESV1                 = 0x0000C000, // Reserved
    MAC_PCICFG_SPWR_DN               = 0x00010000, // mask for sleep/awake indication
    MAC_PCICFG_LED_MODE_M            = 0x000E0000, // Mask for LED mode select
    MAC_PCICFG_LED_BLINK_THRESHOLD_M = 0x00700000, // Mask for LED blink threshold select
    MAC_PCICFG_LED_SLOW_BLINK_MODE   = 0x00800000, // LED slowest blink rate mode
    MAC_PCICFG_SLEEP_CLK_RATE_IND_M  = 0x03000000, // Mask for sleep clock rate indication
    MAC_PCICFG_SLEEP_CLK_RATE_IND_S  = 24,         // Shift for sleep clock rate indication
    MAC_PCICFG_RESV2                 = 0xFC000000, // Reserved

    MAC_NUM_GPIO         =6, // Six numbered 0 to 5.

    MAC_GPIOCR           = 0x4014, // GPIO control register
    MAC_GPIOCR_CR_SHIFT   = 2         , // Each CR is 2 bits
    MAC_GPIOCR_0_CR_N     = 0x00000000, // Input only mode for GPIODO[0]
    MAC_GPIOCR_0_CR_0     = 0x00000001, // Output only if GPIODO[0] = 0
    MAC_GPIOCR_0_CR_1     = 0x00000002, // Output only if GPIODO[0] = 1
    MAC_GPIOCR_0_CR_A     = 0x00000003, // Always output
    MAC_GPIOCR_1_CR_N     = 0x00000000, // Input only mode for GPIODO[1]
    MAC_GPIOCR_1_CR_0     = 0x00000004, // Output only if GPIODO[1] = 0
    MAC_GPIOCR_1_CR_1     = 0x00000008, // Output only if GPIODO[1] = 1
    MAC_GPIOCR_1_CR_A     = 0x0000000C, // Always output
    MAC_GPIOCR_2_CR_N     = 0x00000000, // Input only mode for GPIODO[2]
    MAC_GPIOCR_2_CR_0     = 0x00000010, // Output only if GPIODO[2] = 0
    MAC_GPIOCR_2_CR_1     = 0x00000020, // Output only if GPIODO[2] = 1
    MAC_GPIOCR_2_CR_A     = 0x00000030, // Always output
    MAC_GPIOCR_3_CR_N     = 0x00000000, // Input only mode for GPIODO[3]
    MAC_GPIOCR_3_CR_0     = 0x00000040, // Output only if GPIODO[3] = 0
    MAC_GPIOCR_3_CR_1     = 0x00000080, // Output only if GPIODO[3] = 1
    MAC_GPIOCR_3_CR_A     = 0x000000C0, // Always output
    MAC_GPIOCR_4_CR_N     = 0x00000000, // Input only mode for GPIODO[4]
    MAC_GPIOCR_4_CR_0     = 0x00000100, // Output only if GPIODO[4] = 0
    MAC_GPIOCR_4_CR_1     = 0x00000200, // Output only if GPIODO[4] = 1
    MAC_GPIOCR_4_CR_A     = 0x00000300, // Always output
    MAC_GPIOCR_5_CR_N     = 0x00000000, // Input only mode for GPIODO[5]
    MAC_GPIOCR_5_CR_0     = 0x00000400, // Output only if GPIODO[5] = 0
    MAC_GPIOCR_5_CR_1     = 0x00000800, // Output only if GPIODO[5] = 1
    MAC_GPIOCR_5_CR_A     = 0x00000C00, // Always output
    MAC_GPIOCR_INT_SHIFT  = 12        , // Interrupt select field shifter
    MAC_GPIOCR_INT_MASK   = 0x00007000, // Interrupt select field mask
    MAC_GPIOCR_INT_SEL0   = 0x00000000, // Select Interrupt Pin GPIO_0
    MAC_GPIOCR_INT_SEL1   = 0x00001000, // Select Interrupt Pin GPIO_1
    MAC_GPIOCR_INT_SEL2   = 0x00002000, // Select Interrupt Pin GPIO_2
    MAC_GPIOCR_INT_SEL3   = 0x00003000, // Select Interrupt Pin GPIO_3
    MAC_GPIOCR_INT_SEL4   = 0x00004000, // Select Interrupt Pin GPIO_4
    MAC_GPIOCR_INT_SEL5   = 0x00005000, // Select Interrupt Pin GPIO_5
    MAC_GPIOCR_INT_EN     = 0x00008000, // Enable GPIO Interrupt
    MAC_GPIOCR_INT_SELL   = 0x00000000, // Generate Interrupt if selected pin is low
    MAC_GPIOCR_INT_SELH   = 0x00010000, // Generate Interrupt if selected pin is high

    MAC_GPIODO           = 0x4018, // GPIO data output access register
    MAC_GPIODI           = 0x401C, // GPIO data input access register
    MAC_GPIOD_MASK        = 0x0000002F, // Mask for reading or writing GPIO data regs

    MAC_SREV             = 0x4020, // Silicon Revision register
    MAC_SREV_ID_M         = 0x000000FF, // Mask to read SREV info
    
    MAC_SCR_ALIAS         = 0x4024, // Sleep control register
    MAC_SCR_ALIAS_SLMODE_M= 0x00030000, // sleep mode mask
    MAC_SCR_ALIAS_SLMODE_S= 16,         // sleep mode bits shift
#else
    MAC_PCICFG_EEPROM_SIZE_16K = 2,    // EEPROM size 16 Kbit

#if defined(AR5513)

    /*
    **  !!!!  HARDWARE WORK-AROUND FIX for Falcon Emulation and 1.0 !!!!
    **  Added by Gerald Stanton 01/16/2004
    **
    **  IMPORTANT:  The WLAN0_MAC Sleep Control registers must
    **  be cleared prior to writing the RESET bit for the WLAN MAC.
    **  The delays are important as well!!
    */

    MAC_SCR              = 0x4004, // Sleep control register
    MAC_SCR_SLDUR_M       = 0x0000ffff, // sleep duration mask, units of 128us
    MAC_SCR_SLDUR_S       = 0,          // sleep duration shift
    MAC_SCR_SLMODE_M      = 0x00030000, // sleep mode mask
    MAC_SCR_SLMODE_S      = 16,         // sleep mode bits shift
    MAC_SLMODE_FWAKE      = 0,          // force wake
    MAC_SLMODE_FSLEEP     = 1,          // force sleep
    MAC_SLMODE_NORMAL     = 2,          // sleep logic normal operation
    MAC_SCR_SLDTP         = 0x00040000, // sleep duration timing policy
    MAC_SCR_SLDWP         = 0x00080000, // sleep duration write policy
    MAC_SCR_SLEPOL        = 0x00100000, // sleep policy mode

    /*
    **  END OF HARDWARE WORK-AROUND FIX for Falcon Emulation and 1.0
    */

#endif /* AR5513 */

#endif /* PCI_INTERFACE */
    MAC_SREV_ID_S         = 4,          // Mask to shift Major Rev Info
    MAC_SREV_REVISION_M   = 0x0000000F, // Mask for Chip revision level
    MAC_SREV_VERSION_M    = 0x000000F0, // Mask for Chip version indication
    MAC_SREV_VERSION_VENICE = 5,

    MAC_TXEPOST             = 0x4028, // TXE write posting resgister
    /* TODO: fill in as req */

    MAC_QSM                 = 0x402C, // QCU sleep mask
    /* TODO: fill in as req */

#if defined(PCI_INTERFACE)
/* EEPROM Registers in the MAC */
    MAC_SPI_CS            = 0x17000, // SPI Control/Status register
    
    MAC_SPI_CS_TX_BYTE_CNT_M  = 0x0000000F, 
    MAC_SPI_CS_TX_BYTE_CNT_S  = 0,          
    MAC_SPI_CS_RX_BYTE_CNT_M  = 0x000000F0, 
    MAC_SPI_CS_RX_BYTE_CNT_S  = 4,          
    MAC_SPI_CS_START          = 0x00000100, 
    MAC_SPI_CS_BUSY           = 0x00010000, 
    MAC_SPI_CS_SPI_ADDR_SZ_M  = 0x00060000, 
    MAC_SPI_CS_SPI_ADDR_SZ_S  = 17,          
    MAC_SPI_CS_SPI_ADDR_16    = 0,
    MAC_SPI_CS_SPI_ADDR_24    = 1,
    MAC_SPI_CS_SPI_AUTO_SZ_M  = 0x00180000, 
    MAC_SPI_CS_SPI_AUTO_SZ_S  = 19,          
    MAC_SPI_CS_SPI_AUTO_SZ    = 0,
    MAC_SPI_CS_SPI_FORCE_SZ_16= 1,
    MAC_SPI_CS_SPI_FORCE_SZ_24= 2,

    MAC_SPI_CS_RD_SIGN        = 0x114,   // SPI Read Signature
    MAC_SPI_CS_RD_2BYTES      = 0x124,   // SPI Read 2 Byte Command

    MAC_SPI_AO            = 0x17004, // SPI Address/Opcode register
    MAC_SPI_AO_RD_DATA        = 0x03, // SPI Read Data Opcode
    MAC_SPI_AO_RD_SIG         = 0xab, // SPI Read Electronic Signature Opcode

    /* Atmel EEPROM Opcode */
    MAC_SPI_EEPROM_WR_STATUS  = 0x02,
    MAC_SPI_EEPROM_RD_STATUS  = 0x05,
    MAC_SPI_EEPROM_WR         = 0x02,
    MAC_SPI_EEPROM_RD         = 0x03,
    MAC_SPI_EEPROM_WR_EN      = 0x06,
    MAC_SPI_EEPROM_WR_DIS     = 0x04,

    MAC_SPI_D             = 0x17008, // SPI Data register
#endif /* PCI_INTERFACE */

    // Protect Bits RP is read protect, WP is write protect
    EEPROM_PROTECT_RP_0_31    = 0x0001,
    EEPROM_PROTECT_WP_0_31    = 0x0002,
    EEPROM_PROTECT_RP_32_63   = 0x0004,
    EEPROM_PROTECT_WP_32_63   = 0x0008,
    EEPROM_PROTECT_RP_64_127  = 0x0010,
    EEPROM_PROTECT_WP_64_127  = 0x0020,
    EEPROM_PROTECT_RP_128_191 = 0x0040,
    EEPROM_PROTECT_WP_128_191 = 0x0080,
    EEPROM_PROTECT_RP_192_207 = 0x0100,
    EEPROM_PROTECT_WP_192_207 = 0x0200,
    EEPROM_PROTECT_RP_208_223 = 0x0400,
    EEPROM_PROTECT_WP_208_223 = 0x0800,
    EEPROM_PROTECT_RP_224_239 = 0x1000,
    EEPROM_PROTECT_WP_224_239 = 0x2000,
    EEPROM_PROTECT_RP_240_255 = 0x4000,
    EEPROM_PROTECT_WP_240_255 = 0x8000,

// MAC PCU Registers
    MAC_STA_ID0          = 0x8000, // MAC station ID0 register - low 32 bits
    MAC_STA_ID1          = 0x8004, // MAC station ID1 register - upper 16 bits
    MAC_STA_ID1_SADH_MASK       = 0x0000FFFF, // Mask for upper 16 bits of MAC addr
    MAC_STA_ID1_STA_AP          = 0x00010000, // Device is AP
    MAC_STA_ID1_AD_HOC          = 0x00020000, // Device is ad-hoc
    MAC_STA_ID1_PWR_SAV         = 0x00040000, // Power save reporting in self-generated frames
    MAC_STA_ID1_KSRCHDIS        = 0x00080000, // Key search disable
    MAC_STA_ID1_PCF             = 0x00100000, // Observe PCF
    MAC_STA_ID1_USE_DEFANT      = 0x00200000, // Use default antenna
    MAC_STA_ID1_DEFANT_UPDATE   = 0x00400000, // Update default antenna w/ TX antenna
    MAC_STA_ID1_RTS_USE_DEF     = 0x00800000, // Use default antenna to send RTS
    MAC_STA_ID1_ACKCTS_6MB      = 0x01000000, // Use 6Mb/s rate for ACK & CTS
    MAC_STA_ID1_BASE_RATE_11B   = 0x02000000, // Use 11b base rate for ACK & CTS
    MAC_STA_ID1_SECTOR_SELF_GEN = 0x04000000, // Use default antenna for self generated frames
    MAC_STA_ID1_CRPT_MIC_ENABLE = 0x08000000, // Enable Michael
    MAC_STA_ID1_KSRCH_MODE      = 0x10000000, // Look-up unique key when keyID != 0

    MAC_BSS_ID0          = 0x8008, // MAC BSSID low 32 bits
    MAC_BSS_ID1          = 0x800C, // MAC BSSID upper 16 bits / AID
    MAC_BSS_ID1_U16_M     = 0x0000FFFF, // Mask for upper 16 bits of BSSID
    MAC_BSS_ID1_AID_M     = 0xFFFF0000, // Mask for association ID
    MAC_BSS_ID1_AID_S     = 16        , // Shift for association ID

    MAC_SLOT_TIME        = 0x8010, // MAC Time-out after a collision
    MAC_SLOT_TIME_MASK    = 0x000007FF, // Slot time mask

    MAC_TIME_OUT         = 0x8014, // MAC ACK & CTS time-out
    MAC_TIME_OUT_ACK_M    = 0x00001FFF, // Mask for ACK time-out
    MAC_TIME_OUT_ACK_S    = 0,
    MAC_TIME_OUT_CTS_M    = 0x1FFF0000, // Mask for CTS time-out
    MAC_TIME_OUT_CTS_S    = 16,

    MAC_RSSI_THR         = 0x8018, // MAC Beacon RSSI warning and missed beacon threshold
    MAC_RSSI_THR_MASK     = 0x000000FF, // Mask for Beacon RSSI warning threshold
    MAC_RSSI_THR_BM_THR_M = 0x0000FF00, // Mask for Missed beacon threshold
    MAC_RSSI_THR_BM_THR_S = 8         , // Shift for Missed beacon threshold

    MAC_USEC             = 0x801c, // MAC transmit latency register
    MAC_USEC_M            = 0x0000007F, // Mask for clock cycles in 1 usec
    MAC_USEC_32_M         = 0x00003F80, // Mask for number of 32MHz clock cycles in 1 usec
    MAC_USEC_32_S         = 7         , // Shift for number of 32MHz clock cycles in 1 usec
    MAC_USEC_TX_LAT_M     = 0x007FC000, // Mask for tx latency to start of SIGNAL (usec)
    MAC_USEC_TX_LAT_S     = 14        , // Shift for tx latency to start of SIGNAL (usec)
    MAC_USEC_RX_LAT_M     = 0x1F800000, // Mask for rx latency to start of SIGNAL (usec)
    MAC_USEC_RX_LAT_S     = 23        , // Shift for rx latency to start of SIGNAL (usec)

    MAC_BEACON           = 0x8020, // MAC beacon control value/mode bits
    MAC_BEACON_PERIOD_M    = 0x0000FFFF, // Beacon period mask in TU/msec
    MAC_BEACON_PERIOD_S    = 0,          // Shift for byte offset of PERIOD start
    MAC_BEACON_TIM_M       = 0x007F0000, // Mask for byte offset of TIM start
    MAC_BEACON_TIM_S       = 16        , // Shift for byte offset of TIM start
    MAC_BEACON_EN          = 0x00800000, // beacon enable
    MAC_BEACON_RESET_TSF   = 0x01000000, // Clears TSF to 0

    MAC_CFP_PERIOD       = 0x8024, // MAC CFP Interval (TU/msec)
    MAC_TIMER0           = 0x8028, // MAC Next beacon time (TU/msec)
    MAC_TIMER1           = 0x802c, // MAC DMA beacon alert time (1/8 TU)
    MAC_TIMER2           = 0x8030, // MAC Software beacon alert (1/8 TU)
    MAC_TIMER3           = 0x8034, // MAC ATIM window time

    MAC_CFP_DUR          = 0x8038, // MAC maximum CFP duration in TU

    MAC_RX_FILTER        = 0x803C, // MAC receive filter register
    MAC_RX_FILTER_ALL     = 0x00000000, // Disallow all frames
    MAC_RX_UCAST          = 0x00000001, // Allow unicast frames
    MAC_RX_MCAST          = 0x00000002, // Allow multicast frames
    MAC_RX_BCAST          = 0x00000004, // Allow broadcast frames
    MAC_RX_CONTROL        = 0x00000008, // Allow control frames
    MAC_RX_BEACON         = 0x00000010, // Allow beacon frames
    MAC_RX_PROM           = 0x00000020, // Promiscuous mode, all packets
    MAC_RX_XR_POLL        = 0x00000040, // Allow XR Poll frames
    MAC_RX_PROBE_REQ      = 0x00000080, // Allow probe request frames
    MAC_RX_SYNC           = 0x00000100, // Allow sync frames

    MAC_MCAST_FIL0       = 0x8040, // MAC multicast filter lower 32 bits
    MAC_MCAST_FIL1       = 0x8044, // MAC multicast filter upper 32 bits

    MAC_DIAG_SW          = 0x8048, // MAC PCU control register
    MAC_DIAG_CACHE_ACK    = 0x00000001, // disable ACK when no valid key found
    MAC_DIAG_ACK_DIS      = 0x00000002, // disable ACK generation
    MAC_DIAG_CTS_DIS      = 0x00000004, // disable CTS generation
    MAC_DIAG_ENCRYPT_DIS  = 0x00000008, // disable encryption
    MAC_DIAG_DECRYPT_DIS  = 0x00000010, // disable decryption
    MAC_DIAG_RX_DIS       = 0x00000020, // disable receive
    MAC_DIAG_CORR_FCS     = 0x00000080, // corrupt FCS
    MAC_DIAG_CHAN_INFO    = 0x00000100, // dump channel info
    MAC_DIAG_EN_SCRAMSD   = 0x00000200, // enable fixed scrambler seed
#if defined(AR5513)
    MAC_DIAG_FORCE_RX_CLEAR_HIGH = 0x00100000, // force rx_clear high
    MAC_DIAG_IGNORE_VIRT_CS = 0x00200000, // ignore virtual carrier sense
    MAC_DIAG_FORCE_CH_IDLE_HIGH = 0x00400000, // force channel idle high
#endif
    MAC_DIAG_SCRAM_SEED_M = 0x0001FC00, // Mask for fixed scrambler seed
    MAC_DIAG_SCRAM_SEED_S = 10       , // Shift for fixed scrambler seed
    MAC_DIAG_FRAME_NV0    = 0x00020000, // accept frames of non-zero protocol version
    MAC_DIAG_OBS_PT_SEL_M = 0x000C0000, // Mask for observation point select
    MAC_DIAG_OBS_PT_SEL_S = 18       , // Shift for observation point select

    MAC_TSF_L32          = 0x804c, // MAC local clock lower 32 bits
    MAC_TSF_U32          = 0x8050, // MAC local clock upper 32 bits

    MAC_TST_ADDAC        = 0x8054, // ADDAC test register
    MAC_DEF_ANTENNA      = 0x8058, // default antenna register
    MAC_DEF_ANT_CHN_SEL  = 0x4,    // Default Ant Chain Select bit
    MAC_DEF_ANT_CHN0_ANT = 0x1,    // Def Ant Chain 0 Antenna Select bit
    MAC_DEF_ANT_CHN1_ANT = 0x2,    // Def Ant Chain 1 Antenna Select bit

/* AR5513 Rev 2, 5 and up require 0x8060 for correct AES operation */
    MAC_SEQ_MASK         = 0x8060, // MAC AES mute mask

    MAC_LAST_TSTP        = 0x8080, // MAC Time stamp of the last beacon received
    MAC_NAV              = 0x8084, // MAC current NAV value
    MAC_RTS_OK           = 0x8088, // MAC RTS exchange success counter
    MAC_RTS_FAIL         = 0x808c, // MAC RTS exchange failure counter
    MAC_ACK_FAIL         = 0x8090, // MAC ACK failure counter
    MAC_FCS_FAIL         = 0x8094, // FCS check failure counter
    MAC_BEACON_CNT       = 0x8098, // Valid beacon counter

/* AR5513 new features here */
    MAC_XRMODE          = 0x80c0,           // Extended range mode
    MAC_XRMODE_XR_POLL_TYPE_M    = 0x00000003, // poll type mask
    MAC_XRMODE_XR_POLL_TYPE_S    = 0,
    MAC_XRMODE_XR_POLL_SUBTYPE_M = 0x0000003c, // poll type mask
    MAC_XRMODE_XR_POLL_SUBTYPE_S = 2,
    MAC_XRMODE_XR_WAIT_FOR_POLL  = 0x00000080, // wait for poll, sta only
    MAC_XRMODE_XR_FRAME_HOLD_M   = 0xfff00000, // cycles hold for chirps
    MAC_XRMODE_XR_FRAME_HOLD_S   = 20,

    MAC_XRDEL           = 0x80c4,           // Extended range delay
    MAC_XRDEL_SLOT_DELAY_M       = 0x0000ffff, // cycles
    MAC_XRDEL_SLOT_DELAY_S       = 0,
    MAC_XRDEL_CHIRP_DATA_DELAY_M = 0xffff0000, // cycles
    MAC_XRDEL_CHIRP_DATA_DELAY_S = 16,

    MAC_XRTO            = 0x80c8,           // Extended range timeout
    MAC_XRTO_CHIRP_TO_M          = 0x0000ffff, // cycles
    MAC_XRTO_CHIRP_TO_S          = 0,
    MAC_XRTO_POLL_TO_M           = 0xffff0000, // cycles
    MAC_XRTO_POLL_TO_S           = 16,

    MAC_XRCRP           = 0x80cc,           // Extended range chirp
    MAC_XRCRP_SEND_CHIRP         = 0x00000001, // generate stand alone chirp
    MAC_XRCRP_CHIRP_GAP_M        = 0xffff0000, // cycles
    MAC_XRCRP_CHIRP_GAP_S        = 16,

    MAC_XRSTMP          = 0x80d0,           // Extended range stomp
    MAC_XRSTMP_TX_STOMP          = 0x00000001, // transmit stomp on receive cmd
    MAC_XRSTMP_RX_ABORT          = 0x00000002, // stomp current receive enable
    MAC_XRSTMP_RSSI_THRESH_M     = 0x0000ff00, // threshold for tx stomp
    MAC_XRSTMP_RSSI_THRESH_S     = 8,

    MAC_SLEEP1          = 0x80d4,           // Enhanced sleep control 1
    MAC_SLEEP1_NEXT_DTIM_M       = 0x0007ffff, // Absolute time(1/8TU) for next dtim mask
    MAC_SLEEP1_NEXT_DTIM_S       = 0,          // Absolute time(1/8TU) for next dtim shift
    MAC_SLEEP1_ASSUME_DTIM       = 0x00080000, // Assume DTIM present on missed beacon
    MAC_SLEEP1_ENH_SLEEP_ENABLE  = 0x00100000, // Enables Venice sleep logic
    MAC_SLEEP1_CAB_TIMEOUT_M     = 0xff000000, // Cab timeout(TU) mask
    MAC_SLEEP1_CAB_TIMEOUT_S     = 24,         // Cab timeout(TU) shift

    MAC_SLEEP2          = 0x80d8,           // Enhanced sleep control 2
    MAC_SLEEP2_NEXT_TIM_M        = 0x0007ffff, // Absolute time(1/8TU) for next tim/beacon mask
    MAC_SLEEP2_NEXT_TIM_S        = 0,          // Absolute time(1/8TU) for next tim/beacon shift
    MAC_SLEEP2_BEACON_TIMEOUT_M  = 0xff000000, // Beacon timeout(TU) mask
    MAC_SLEEP2_BEACON_TIMEOUT_S  = 24,         // Beacon timeout(TU) shift

    MAC_SLEEP3          = 0x80dc,           // Enhanced sleep control 3
    MAC_SLEEP3_TIM_PERIOD_M      = 0x0000ffff, // Tim/Beacon period(TU) mask
    MAC_SLEEP3_TIM_PERIOD_S      = 0,          // Tim/Beacon period(TU) shift
    MAC_SLEEP3_DTIM_PERIOD_M     = 0xffff0000, // DTIM period(TU) mask
    MAC_SLEEP3_DTIM_PERIOD_S     = 16,         // DTIM period(TU) shift

    MAC_BSS_ID_MASK0            = 0x80e0,           // BSSID mask lower 32 bits
    MAC_BSS_ID_MASK1            = 0x80e4,           // BSSID mask upper 16 bits

    MAC_TPC             = 0x80e8,   // Transmit power control for self gen frames
    MAC_TPC_ACK_M           = 0x0000003f, // ack frames mask
    MAC_TPC_ACK_S           = 0x00,       // ack frames shift
    MAC_TPC_CTS_M           = 0x00003f00, // cts frames mask
    MAC_TPC_CTS_S           = 0x08,       // cts frames shift
    MAC_TPC_CHIRP_M         = 0x003f0000, // chirp frames mask
    MAC_TPC_CHIRP_S         = 0x16,       // chirp frames shift

    MAC_TFCNT           = 0x80ec,   // Profile count, transmit frames
    MAC_RFCNT           = 0x80f0,   // Profile count, receive frames
    MAC_RCCNT           = 0x80f4,   // Profile count, receive clear
    MAC_CCCNT           = 0x80f8,   // Profile count, cycle counter

    MAC_QUIET1          = 0x80fc,                   // Quiet time programming for TGh
    MAC_QUIET1_NEXT_QUIET_S         = 0,            // TSF of next quiet period (TU)
    MAC_QUIET1_NEXT_QUIET_M         = 0x0000ffff,
    MAC_QUIET1_QUIET_ENABLE         = 0x00010000,   // Enable Quiet time operation
    MAC_QUIET1_QUIET_ACK_CTS_ENABLE = 0x00020000,   // Do we ack/cts during quiet period
    MAC_QUIET2          = 0x8100,                   // More Quiet time programming
    MAC_QUIET2_QUIET_PERIOD_S       = 0,            // Periodicity of quiet period (TU)
    MAC_QUIET2_QUIET_PERIOD_M       = 0x0000ffff,
    MAC_QUIET2_QUIET_DURATION_S     = 16,           // Duration of quiet period (TU)
    MAC_QUIET2_QUIET_DURATION_M     = 0xffff0000,

    MAC_TSF_PARM        = 0x8104,           // TSF parameters
    MAC_TSF_INCREMENT_M     = 0x000000ff,
    MAC_TSF_INCREMENT_S     = 0x00,

    MAC_PHY_ERR         = 0x810c,           // Phy errors to be filtered
    MAC_PHY_ERR_DCHIRP      = 0x00000008,   // Bit  3 enables double chirp
    MAC_PHY_ERR_RADAR       = 0x00000020,   // Bit  5 is Radar signal
    MAC_PHY_ERR_OFDM_TIMING = 0x00020000,   // Bit 17 is false detect for OFDM
    MAC_PHY_ERR_CCK_TIMING  = 0x02000000,   // Bit 25 is false detect for CCK

    MAC_PCU_MISC        = 0x8120,        // PCU Miscellaneous Mode
    MAC_PCU_DUAL_CHN_ANT_MODE = 0x00000200,   // Dual Chain Antenna Mode
    MAC_PCU_MISC_DESC_MODE  = 0x00000400,     // Falcon Descriptor Mode
    MAC_PCU_RX_ANT_UPDT     = 0x00000800,     // KC_RX_ANT_UPDATE

    MAC_KC_MASK         = 0x81c4,        // MAC Key Cache Mask for words 0x10, 0x14
                                                // 0 is write allow, 1 is write blocked
    MAC_KC_MASK_TYPE_M          = 0x00000007,   // MAC Key Cache Type Mask
    MAC_KC_MASK_LAST_TX_ANT     = 0x00000008,   // MAC Key Cache Last Tx Ant Mask
    MAC_KC_MASK_ASYNC_MASK_M    = 0x000001f0,   // MAC Key Cache Async Rate Offset Mask
    MAC_KC_MASK_UPDT_BF         = 0x00000200,   // MAC Key Cache Update Bf coef Mask
    MAC_KC_MASK_RX_CHAIN0_ACK   = 0x00000400,   // MAC Key Cache Ack Ant Ch 0 Mask
    MAC_KC_MASK_RX_CHAIN1_ACK   = 0x00000800,   // MAC Key Cache Ack Ant Ch 1 Mask
    MAC_KC_MASK_TX_CHAIN0_SEL   = 0x00001000,   // MAC Key Cache Tx Sel Ant Ch 0 Mask
    MAC_KC_MASK_TX_CHAIN1_SEL   = 0x00002000,   // MAC Key Cache Tx Sel Ant Ch 1 Mask
    MAC_KC_MASK_CHAIN_SEL       = 0x00004000,   // MAC Key Cache Chain Sel Mask
    MAC_KC_MASK_WORD_10         = 0x00010000,   // MAC Key Cache Word 0x10 Mask

    MAC_TXOPX           = 0x81EC,
    MAC_TXOP_0_3        = 0x81F0,
    MAC_TXOP_4_7        = 0x81F4,
    MAC_TXOP_8_11       = 0x81F8,
    MAC_TXOP_12_15      = 0x81FC,
    
    MAC_FRM_TYPE_CAP_TBL  = 0x8500,      // Frame Type Capabilities Table
    MAC_FRM_TYPE_CAP_SIZE   = 64,          // Frame Type Cap. Table Size
    MAC_FTC_BF_RX_UPDT_NORM = 0x00000001,  // BFCOEF_RX_UPDATE_NORMAL
    MAC_FTC_BF_RX_UPDT_SELF = 0x00000002,  // BFCOEF_RX_UPDATE_SELF_GEN
    MAC_FTC_BF_TX_ENB_NORM  = 0x00000004,  // BFCOEF_TX_ENABLE_NORMAL
    MAC_FTC_BF_TX_ENB_SELF  = 0x00000008,  // BFCOEF_TX_ENABLE_SELF_GEN
    MAC_FTC_BF_TX_ENB_GEN   = 0x00000010,  // BFCOEF_TX_ENABLE_GEN
    MAC_FTC_BF_TX_ENB_MCAST = 0x00000020,  // BFCOEF_TX_ENABLE_MCAST

    //  rate duration registers - used for Multi-rate retry.
    MAC_RATE_DURATION_0   = 0x8700,  // 32 registers from 0x8700 to 0x87CC
    MAC_RATE_DURATION_31  = 0x87CC,

    MAC_KEY_CACHE        = 0x8800,  // MAC Key Cache
    MAC_KEY_CACHE_SIZE    = 128,
    MAC_KEY_TYPE_M        = 0x00000007, // MAC Key Type Mask
    MAC_KEY_TYPE_WEP_40   = 0,
    MAC_KEY_TYPE_WEP_104  = 1,
    MAC_KEY_TYPE_WEP_128  = 3,
    MAC_KEY_TYPE_TKIP     = 4,
    MAC_KEY_TYPE_AES      = 5,
    MAC_KEY_TYPE_CCM      = 6,
    MAC_KEY_TYPE_CLEAR    = 7,

    MAC_RESERVE_KEYCACHE_ENTRIES = 4,

// PHY registers
    PHY_BASE                   = 0x9800, // PHY registers base address
    CHN_0_BASE                 = 0x9800, // Chain 0 register base address
    CHN_1_BASE                 = 0xa800, // Chain 1 register base address
    CHN_ALL_BASE               = 0xb800, // Combined Chain's 0 & 1 address
    CHAIN_0                    = 0,      // Chain 0 index into EEMAP
    CHAIN_1                    = 1,      // Chain 1 index into EEMAP

    PHY_TURBO                  = 0x9804, // PHY frame control register
    PHY_FC_TURBO_MODE           = 0x00000001, // Set turbo mode bits
    PHY_FC_TURBO_SHORT          = 0x00000002, // Set short symbols to turbo mode setting

    PHY_TESTCTRL               = 0x9808, // PHY Test Control/Status

    PHY_TIMING3                = 0x9814, // PHY timing control 3
    PHY_TIMING3_DSC_MAN_M       = 0xFFFE0000,
    PHY_TIMING3_DSC_MAN_S       = 17,
    PHY_TIMING3_DSC_EXP_M       = 0x0001E000,
    PHY_TIMING3_DSC_EXP_S       = 13,

    PHY_CHIP_ID                = 0x9818, // PHY chip revision ID
                                
    PHY_ACTIVE                 = 0x981C, // PHY activation register
    PHY_ACTIVE_EN               = 0x00000001, // Activate PHY chips
    PHY_ACTIVE_DIS              = 0x00000000, // Deactivate PHY chips

    PHY_ADC_CTL                = 0x982C,
    PHY_ADC_CTL_OFF_INBUFGAIN_M = 0x00000003,
    PHY_ADC_CTL_OFF_INBUFGAIN_S = 0,
    PHY_ADC_CTL_OFF_PWDDAC      = 0x00002000,
    PHY_ADC_CTL_OFF_PWDBANDGAP  = 0x00004000, // BB Rev 4.2+ only
    PHY_ADC_CTL_OFF_PWDADC      = 0x00008000, // BB Rev 4.2+ only
    PHY_ADC_CTL_ON_INBUFGAIN_M  = 0x00030000,
    PHY_ADC_CTL_ON_INBUFGAIN_S  = 16,

    PHY_PA_CTL                 = 0x9838,
    PHY_PA_CTL_XPAA_ACTIVE_HIGH = 0x00000001,
    PHY_PA_CTL_XPAB_ACTIVE_HIGH = 0x00000002,

    PHY_RXGAIN                 = 0x9848,
    PHY_RXGAIN_TXRX_RF_MAX_M    = 0x007C0000,
    PHY_RXGAIN_TXRX_RF_MAX_S    = 18,

    PHY_DESIRED_SZ             = 0x9850,
    PHY_DESIRED_SZ_TOT_DES_M    = 0x0FF00000,
    PHY_DESIRED_SZ_TOT_DES_S    = 20,

    PHY_FIND_SIG               = 0x9858,
    PHY_FIND_SIG_FIRSTEP_M      = 0x0003F000,
    PHY_FIND_SIG_FIRSTEP_S      = 12,
    PHY_FIND_SIG_FIRPWR_M       = 0x03FC0000,
    PHY_FIND_SIG_FIRPWR_S       = 18,

    PHY_AGC_CTL1               = 0x985C,
    PHY_AGC_CTL1_COARSE_LOW_M   = 0x00007F80,
    PHY_AGC_CTL1_COARSE_LOW_S   = 7,
    PHY_AGC_CTL1_COARSE_HIGH_M  = 0x003F8000,
    PHY_AGC_CTL1_COARSE_HIGH_S  = 15,

    PHY_AGC_CONTROL            = 0x9860, // PHY chip calibration and noise floor setting
    PHY_AGC_CONTROL_CAL         = 0x00000001, // Perform PHY chip internal calibration
    PHY_AGC_CONTROL_NF          = 0x00000002, // Perform PHY chip noise-floor calculation
    PHY_AGC_CONTROL_ENABLE_NF    = 0x00008000,
    PHY_AGC_CONTROL_NO_UPDATE_NF = 0x00020000,

    PHY_CCA                    = 0x9864,
    PHY_CCA_THRESH62_M          = 0x0007F000,
    PHY_CCA_THRESH62_S          = 12,

    PHY_SFCORR_LOW             = 0x986C,
    PHY_SFCORR_LOW_USE_SELF_CORR_LOW = 0x00000001,
    PHY_SFCORR_LOW_M2COUNT_THR_LOW_M = 0x00003F00,
    PHY_SFCORR_LOW_M2COUNT_THR_LOW_S = 8,
    PHY_SFCORR_LOW_M1_THRESH_LOW_M   = 0x001FC000,
    PHY_SFCORR_LOW_M1_THRESH_LOW_S   = 14,
    PHY_SFCORR_LOW_M2_THRESH_LOW_M   = 0x0FE00000,
    PHY_SFCORR_LOW_M2_THRESH_LOW_S   = 21,

    PHY_SFCORR                 = 0x9868,
    PHY_SFCORR_M2COUNT_THR_M    = 0x0000001F,
    PHY_SFCORR_M2COUNT_THR_S    = 0,
    PHY_SFCORR_M1_THRESH_M      = 0x00FE0000,
    PHY_SFCORR_M1_THRESH_S      = 17,
    PHY_SFCORR_M2_THRESH_M      = 0x7F000000,
    PHY_SFCORR_M2_THRESH_S      = 24,

    PHY_SLEEP_CTR_CONTROL      = 0x9870,
    PHY_SLEEP_CTR_LIMIT        = 0x9874,
    PHY_SLEEP_SCAL             = 0x9878,      // ADC/DAC select lines1

    PHY_PLL_CTL                = 0x987c,      // PLL control register
    PHY_PLL_CTL_44              = 0xab,        // 44 MHz for 11b, 11g
    PHY_PLL_CTL_40              = 0xaa,        // 40 MHz for 11a, turbos
#if defined(AR5312)
    PHY_PLL_CTL_44_5112         = 0x14d6,      // 44 MHz for 11b, 11g
    PHY_PLL_CTL_40_5112         = 0x14d4,      // 40 MHz for 11a, turbos
#elif defined(AR5513)
    PHY_PLL_CTL_44_5112         = 0x114eb,        // 44 MHz for 11b, 11g
    PHY_PLL_CTL_40_5112         = 0x114ea,        // 40 MHz for 11a, turbos
#elif PCI_INTERFACE
    PHY_PLL_CTL_44_5112         = 0x114eb,        // 44 MHz for 11b, 11g
    PHY_PLL_CTL_40_5112         = 0x114ea,        // 40 MHz for 11a, turbos
#endif
                                
    PHY_RX_DELAY               = 0x9914, // PHY analog_power_on_time, in 100ns increments
    PHY_RX_DELAY_M              = 0x00003FFF, // Mask for delay from active assertion (wake up)
                                              // to enable_receiver

    PHY_TIMING_CTRL4   = 0x9920, // PHY
    PHY_TIMING_CTRL4_IQCORR_Q_Q_COFF_M     = 0x0000001F, // Mask for kcos_theta-1 for q correction
    PHY_TIMING_CTRL4_IQCORR_Q_Q_COFF_S     = 0         , // Shift for kcos_theta-1 for q correction
    PHY_TIMING_CTRL4_IQCORR_Q_I_COFF_M     = 0x000007E0, // Mask for sin_theta for i correction
    PHY_TIMING_CTRL4_IQCORR_Q_I_COFF_S     = 5         , // Shift for sin_theta for i correction
    PHY_TIMING_CTRL4_IQCORR_ENABLE         = 0x00000800, // enable IQ correction
    PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX_M = 0x0000F000, // Mask for max number of samples (logarithmic)
    PHY_TIMING_CTRL4_IQCAL_LOG_COUNT_MAX_S = 12        , // Shift for max number of samples
    PHY_TIMING_CTRL4_DO_IQCAL              = 0x00010000, // perform IQ calibration

    PHY_TIMING5                = 0x9924,
    PHY_TIMING5_CYCPWR_THR1_M   = 0x000000FE,
    PHY_TIMING5_CYCPWR_THR1_S   = 1,

    PHY_PAPD_PROBE             = 0x9930,
    PHY_PAPD_PROBE_POWERTX_M    = 0x00007E00,
    PHY_PAPD_PROBE_POWERTX_S    = 9,
    PHY_PAPD_PROBE_NEXT_TX      = 0x00008000, // bit 15 as command to take next reading
    PHY_PAPD_PROBE_TYPE_M       = 0x01800000,
    PHY_PAPD_PROBE_TYPE_S       = 23,
    PHY_PAPD_PROBE_GAINF_M      = 0xFE000000,
    PHY_PAPD_PROBE_GAINF_S      = 25,
    PROBE_TYPE_OFDM             = 0,
    PROBE_TYPE_XR               = 1,
    PROBE_TYPE_CCK              = 2,

    PHY_POWER_TX_RATE1         = 0x9934,
    PHY_POWER_TX_RATE2         = 0x9938,
    PHY_POWER_TX_RATE_MAX      = 0x993c,

    PHY_FRAME_CTL              = 0x9944,
    PHY_FRAME_CTL_TX_CLIP_M     = 0x00000038,
    PHY_FRAME_CTL_TX_CLIP_S     = 3,

    PHY_TXPWRADJ                  = 0x994C,      // BB Rev 4.2+ only
    PHY_TXPWRADJ_CCK_GAIN_DELTA_M  = 0x00000FC0,
    PHY_TXPWRADJ_CCK_GAIN_DELTA_S  = 6,
    PHY_TXPWRADJ_CCK_PCDAC_INDEX_M = 0x00FC0000,
    PHY_TXPWRADJ_CCK_PCDAC_INDEX_S = 18,

    PHY_RADAR_0                = 0x9954,      // PHY radar detection settings
    PHY_RADAR_0_EN              = 0x00000001, // Enable radar detection
    PHY_RADAR_0_INBAND_M        = 0x0000003E, // pulse_inband_thresh
    PHY_RADAR_0_INBAND_S        = 1,
    PHY_RADAR_0_PRSSI_M         = 0x00000FC0, // pulse_rssi_thresh
    PHY_RADAR_0_PRSSI_S         = 6,
    PHY_RADAR_0_HEIGHT_M        = 0x0003F000, // pulse_height_thresh
    PHY_RADAR_0_HEIGHT_S        = 12,
    PHY_RADAR_0_RRSSI_M         = 0x00FC0000, // radar_rssi_thresh
    PHY_RADAR_0_RRSSI_S         = 18,
    PHY_RADAR_0_FIRPWR_M        = 0x7F000000, // radar_firpwr_thresh
    PHY_RADAR_0_FIRPWR_S        = 24,

    PHY_CHN0_PHASE_ANT1        = 0x9950,     // Chain 0 Ant 1 Phase Ramp
    PHY_CHN0_PHASE_ANT0        = 0x995c,     // Chain 0 Ant 0 Phase Ramp
    PHY_PHASE_RAMP_INIT0_S      = 7,          // Phase Init0 Shift value
    PHY_PHASE_RAMP_ENB_M        = 0x00000001, // Phase Ramp Enable
    PHY_CHN1_PHASE_ANT1        = 0xa950,     // Chain 1 Ant 1 Phase Ramp
    PHY_CHN1_PHASE_ANT0        = 0xa95c,     // Chain 1 Ant 0 Phase Ramp
    
    ANT_SWITCH_TABLE1_CH0       = 0x9960,
    ANT_SWITCH_TABLE2_CH0       = 0x9964,
    ANT_SWITCH_TABLE1_CH1       = 0xa960,
    ANT_SWITCH_TABLE2_CH1       = 0xa964,
    ANT_SWITCH_TABLE1           = 0xb960,
    ANT_SWITCH_TABLE2           = 0xb964,

    PHY_SIGMA_DELTA            = 0x996C,        // AR5312 Only
    PHY_SIGMA_DELTA_ADC_SEL_M   = 0x00000003,
    PHY_SIGMA_DELTA_ADC_SEL_S   = 0,
    PHY_SIGMA_DELTA_FILT2_M     = 0x000000F8,
    PHY_SIGMA_DELTA_FILT2_S     = 3,
    PHY_SIGMA_DELTA_FILT1_M     = 0x00001F00,
    PHY_SIGMA_DELTA_FILT1_S     = 8,
    PHY_SIGMA_DELTA_ADC_CLIP_M  = 0x01FFE000,
    PHY_SIGMA_DELTA_ADC_CLIP_S  = 13,

    PHY_RESTART               = 0x9970,  // PHY restart
    PHY_RESTART_FAST_DIV_GC_M  = 0x001C0000,    // bb_ant_fast_div_gc_limit mask
    PHY_RESTART_FAST_DIV_GC_S  = 18,            // bb_ant_fast_div_gc_limit shift

    PHY_MULTICHN_ENB           = 0x99a4, // PHY Multichain Enable
    PHY_RX_CHN_MASK            = 0x03,   // PHY RX Chain Mask
    PHY_RX_CHN_SHIFT           = 0,      // PHY RX Chain Shift bits
    PHY_TX_CHN_MASK            = 0x30,   // PHY TX Chain Mask
    PHY_TX_CHN_SHIFT           = 4,      // PHY TX Chain Shift bits
    PHY_CHN_0_ENB              = 0x1,    // PHY Chain 0 Enable
    PHY_CHN_1_ENB              = 0x2,    // PHY Chain 1 Enable
    PHY_CHN_ALL_ENB            = 0x3,    // PHY Chain 0 & 1 Enable

    PHY_MULTICHN_GAIN_CTRL     = 0x99ac, // PHY Multichain Gain Control
    PHY_ANT_FAST_DIV_BIAS_SHFT = 9,      // Ant Fast Diversity Bias Shift
                                         // Bias in 1/2 dB units
    PHY_ANT_FAST_DIV_BIAS_MASK = 0x7e00, // Ant Fast Diversity Bias Mask
    PHY_10_DB_BIAS             = 20,     // 10 dB in 1/2 dB units
    PHY_ENB_CHK_STRONG_ANT     = 0x0100, // Enable check strong antenna
    PHY_QUICKDROP_LOW_SHFT     = 0,      // Quickdrop Low Shift

    PHY_TXBF_CTRL              = 0x99b0, // PHY Multichain Tx Beamform control
    PHY_CHN_0_FORCE            = 0x00000200,  // PHY Chain 0 Force
    PHY_CHN_1_FORCE            = 0x00000600,  // PHY Chain 1 Force
    PHY_TXBF_CTRL_EIRP_MODE_M  = 0x00000800,  // EIRP Limited Mode
    PHY_TXBF_CTRL_EIRP_MODE_S  = 11,
    PHY_TXBF_CTRL_BAND_EIRP_M  = 0x0007E000,
    PHY_TXBF_CTRL_BAND_EIRP_S  = 13,

    PHY_M_SLEEP                = 0x99f0, // PHY Sleep control registers - don't write to 5312
    PHY_REFCLKDLY              = 0x99f4,
    PHY_REFCLKPD               = 0x99f8,

    PHY_IQCAL_RES_PWR_MEAS_I   = 0x9c10, //PHY IQ calibration results - power measurement for I
    PHY_IQCAL_RES_PWR_MEAS_Q   = 0x9c14, //PHY IQ calibration results - power measurement for Q
    PHY_IQCAL_RES_IQ_CORR_MEAS = 0x9c18, //PHY IQ calibration results - IQ correlation measurement
    PHY_CURRENT_RSSI           = 0x9c1c, // rssi of current frame being received

    PHY_PCDAC_TX_POWER         = 0xA180,

    PHY_MODE                   = 0xA200,   // Mode register
    PHY_MODE_XR                 = 0x10,     // bit 4 = 1 for XR
    PHY_MODE_AR5112             = 0x08,     // bit 3 = 1 for AR5112
    PHY_MODE_AR5111             = 0x00,     // bit 3 = 0 for AR5111/AR2111
    PHY_MODE_DYNAMIC            = 0x04,     // bit 2 = 1 for dynamic CCK/OFDM mode
    PHY_MODE_RF2GHZ             = 0x02,     // bit 1 = 1 for 2.4 GHz
    PHY_MODE_RF5GHZ             = 0x00,     // bit 1 = 0 for 5 GHz
    PHY_MODE_CCK                = 0x01,     // bit 0 = 1 for CCK
    PHY_MODE_OFDM               = 0x00,     // bit 0 = 0 for OFDM

    PHY_CCK_TX_CTRL            = 0xA204,
    PHY_CCK_TX_CTRL_JAPAN       = 0x00000010,

    PHY_CCK_DETECT             = 0xA208,
    PHY_CCK_DETECT_WEAK_SIG_THR_CCK_M     = 0x0000003F,
    PHY_CCK_DETECT_WEAK_SIG_THR_CCK_S     = 0,
    PHY_CCK_DETECT_BB_ENABLE_ANT_FAST_DIV = 0x2000,

    PHY_GAIN_2GHZ              = 0xA20C,
    PHY_GAIN_2GHZ_RXTX_MARGIN_M = 0x00FC0000,
    PHY_GAIN_2GHZ_RXTX_MARGIN_S = 18,

    PHY_DAG_CTRLCCK            = 0xA228,
    PHY_DAG_CTRLCCK_EN_RSSI_THR = 0x00000200, // BB Rev 4.2+ only
    PHY_DAG_CTRLCCK_RSSI_THR_M  = 0x0001FC00, // BB Rev 4.2+ only
    PHY_DAG_CTRLCCK_RSSI_THR_S  = 10,         // BB Rev 4.2+ only

    PHY_POWER_TX_RATE3         = 0xA234,
    PHY_POWER_TX_RATE4         = 0xA238,

    PHY_SCRM_SEQ_XR            = 0xA23C,
    PHY_HEADER_DETECT_XR       = 0xA240,
    PHY_CHIRP_DETECTED_XR      = 0xA244,
    PHY_BLUETOOTH              = 0xA254,

    /* PCI thick driver */
    PCI_MCFG                   = 0x1000C,
    ADDRESS_SHIFT              = 0x200000,

    RST_AHB_ARB_CTL            = 0x14008,
    CPU_AHB_MASTER             = 0x001, 
    WMAC_AHB_MASTER            = 0x002, 
    MPEGTS_TS_AHB_MASTER       = 0x004,
    LOCAL_BUS_AHB_MASTER       = 0x008,
    PCI_AHB_MASTER             = 0x010,

    RST_BYTESWAP_CTL           = 0x1400C,
    RST_BYTESWAP_CTL_WMAC      = 0x02,
    RST_BYTESWAP_CTL_PCI       = 0x08,
    
    RST_SREV                   = 0x14014,
    RST_SREV_M                 = 0x000000FF,
    RST_SREV_REVISION_M        = 0x0000000F, // Chip revision level
    RST_SREV_VERSION_S         = 4, 
    RST_SREV_VERSION_M         = 0x000000F0, // Chip version indication

    RST_IF_CTL                 = 0x14018,
    ENABLE_PCI_INTERFACE       = 0x01,
    PCI_CLIENT_INT_ENABLE      = 0x20,

    RST_MIMR                   = 0x14024,
    RST_PLLC_CTL               = 0x14064,

    RST_PLLV_CTL               = 0x14068,
    POWER_DOWN_ENABLE          = 0x80000,

    RST_AMBACLK_CTL            = 0x14070,
    AMBACLK_PLLC_DIV2_CLK      = 0x0,
    AMBACLK_PLLC_DIV3_CLK      = 0x1,
    AMBACLK_PLLC_V_CLK         = 0x2,
    AMBACLK_REF_CLK            = 0x3,

    RST_DSL_SLEEP_CTL          = 0x14080,
    CLIENT_SLEEP_CTL           = 0x200,
    
    RST_OBS_CTL                = 0x140b0,
    RST_OBS_CTL_RX_CLEAR_EN    = 0x200,
    RST_OBS_CTL_LED_0          = 0x400,
    RST_OBS_CTL_LED_1          = 0x800,

    RST_CIMR                   = 0x140b8,
    WMAC_INTERRUPT_MASK        = 0x02,
    WMAC_POLL_INTERRUPT_MASK   = 0x10,
}; /* enum ar5513Registers */

#ifdef _cplusplus
}
#endif

#endif /* _AR5513_REG_H_ */
