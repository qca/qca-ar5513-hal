/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Register definitions for Atheros AR5211/AR5311 chipset
 *
 * Supports the following MAC revisions
 *      2    - Maui 2.0
 *      3    - Spirit   (5311)
 *      4    - Oahu     (5211)
 *
 * A maximum of 10 QCUs and 10 DCUs are supported to provide
 * compatibility across all MAC revisions.
 *
 * Maui2/Spirit specific registers/fields are indicated by MAC_5311.
 * Oahu specific registers/fields are indicated by MAC_5211.
 *
 * $Id: //depot/sw/branches/AV_dev/src/hal/ar5211/ar5211Reg.h#3 $
 */

#ifndef _AR5211_REG_H_
#define _AR5211_REG_H_

#ifdef _cplusplus
extern "C" {
#endif

enum ar5211Registers {
// DMA Control and Interrupt Registers
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
     MAC_5211_CFG_CLK_GATE_DIS = 0x00000400, // Clock gating disable (Oahu only)
#if defined(PCI_INTERFACE)
     MAC_CFG_EEBS          = 0x00000200, // EEPROM busy
     MAC_CFG_PCI_MASTER_REQ_Q_THRESH_M       = 0x00060000, // Mask of PCI core master request queue full threshold
     MAC_CFG_PCI_MASTER_REQ_Q_THRESH_S       = 17        , // Shift for PCI core master request queue full threshold
#endif /* PCI_INTERFACE */

     MAC_IER              = 0x0024, // MAC Interrupt enable register
     MAC_IER_ENABLE        = 0x00000001, // Global interrupt enable
     MAC_IER_DISABLE       = 0x00000000, // Global interrupt disable

     MAC_RTSD0            = 0x0028, // MAC RTS Duration Parameters 0
     MAC_RTSD0_RTS_DURATION_6_M = 0x000000FF,
     MAC_RTSD0_RTS_DURATION_6_S = 0,
     MAC_RTSD0_RTS_DURATION_9_M = 0x0000FF00,
     MAC_RTSD0_RTS_DURATION_9_S = 8,
     MAC_RTSD0_RTS_DURATION_12_M = 0x00FF0000,
     MAC_RTSD0_RTS_DURATION_12_S = 16,
     MAC_RTSD0_RTS_DURATION_18_M = 0xFF000000,
     MAC_RTSD0_RTS_DURATION_18_S = 24,

     MAC_RTSD1            = 0x002c, // MAC RTS Duration Parameters 1
     MAC_RTSD0_RTS_DURATION_24_M = 0x000000FF,
     MAC_RTSD0_RTS_DURATION_24_S = 0,
     MAC_RTSD0_RTS_DURATION_36_M = 0x0000FF00,
     MAC_RTSD0_RTS_DURATION_36_S = 8,
     MAC_RTSD0_RTS_DURATION_48_M = 0x00FF0000,
     MAC_RTSD0_RTS_DURATION_48_S = 16,
     MAC_RTSD0_RTS_DURATION_54_M = 0xFF000000,
     MAC_RTSD0_RTS_DURATION_54_S = 24,

     MAC_DMASIZE_4B        = 0x00000000, // DMA size 4 bytes (TXCFG + RXCFG)
     MAC_DMASIZE_8B        = 0x00000001, // DMA size 8 bytes
     MAC_DMASIZE_16B       = 0x00000002, // DMA size 16 bytes
     MAC_DMASIZE_32B       = 0x00000003, // DMA size 32 bytes
     MAC_DMASIZE_64B       = 0x00000004, // DMA size 64 bytes
     MAC_DMASIZE_128B      = 0x00000005, // DMA size 128 bytes
     MAC_DMASIZE_256B      = 0x00000006, // DMA size 256 bytes
     MAC_DMASIZE_512B      = 0x00000007, // DMA size 512 bytes

     MAC_TXCFG            = 0x0030, // MAC tx DMA size config register
     MAC_5211_TXCFG_EN_11B = 0x00000008, // 802.11b mode enable (Oahu only)
     MAC_FTRIG_M           = 0x000003F0, // Mask for Frame trigger level
     MAC_FTRIG_S           = 4         , // Shift for Frame trigger level
     MAC_FTRIG_IMMED       = 0x00000000, // bytes in PCU TX FIFO before air
     MAC_FTRIG_64B         = 0x00000010, // default
     MAC_FTRIG_128B        = 0x00000020,
     MAC_FTRIG_192B        = 0x00000030,
     MAC_FTRIG_256B        = 0x00000040, // 5 bits total

     MAC_RXCFG            = 0x0034, // MAC rx DMA size config register
     MAC_531X_RXCFG_DEF_RX_ANTENNA = 0x00000008, // Default Receive Antenna
                                                 // Maui2/Spirit/Freedom only
                                                 // reserved on Oahu
     MAC_RXCFG_ZLFDMA      = 0x00000010, // Enable DMA of zero-length frame
     MAC_5211_RXCFG_EN_JUM = 0x00000020, // Enable jumbo rx descriptors
     MAC_5211_RXCFG_WR_JUM = 0x00000040, // Wrap jumbo rx descriptors

     MAC_5211_JUMBO_LAST  = 0x0038, // Jumbo descriptor last address

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
     MAC_TXNPTO_QCU_MASK   = 0x03FFFC00, // Mask indicating the set of QCUs
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

     MAC_531X_QDCLKGATE    = 0x005c, // MAC QCU/DCU clock gating control register
                                     // Maui2/Spirit only.
     MAC_531X_QDCLKGATE_QCU_M    = 0x0000FFFF, // Mask for QCU clock disable
     MAC_531X_QDCLKGATE_DCU_M    = 0x07FF0000, // Mask for DCU clock disable

// Interrupt Status Registers
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
     MAC_ISR_S2_RESV0        = 0xFFF80000, // Reserved

     MAC_ISR_S3             = 0x0090, // MAC Secondary interrupt status register 3
     MAC_ISR_S3_QCU_QCBROVF_M  = 0x000003FF, // Mask for QCBROVF (QCU 0-9)
     MAC_ISR_S3_QCU_QCBRURN_M  = 0x03FF0000, // Mask for QCBRURN (QCU 0-9)

     MAC_ISR_S4             = 0x0094, // MAC Secondary interrupt status register 4
     MAC_ISR_S4_QCU_QTRIG_M  = 0x000003FF, // Mask for QTRIG (QCU 0-9)
     MAC_ISR_S4_RESV0        = 0xFFFFFC00, // Reserved

// Interrupt Mask Registers
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
     MAC_IMR_S2_MCABT        = 0x00010000, // Master cycle abort interrupt
     MAC_IMR_S2_SSERR        = 0x00020000, // SERR interrupt
     MAC_IMR_S2_DPERR        = 0x00040000, // PCI bus parity error
     MAC_IMR_S2_RESV0        = 0xFFF80000, // Reserved

     MAC_IMR_S3             = 0x00b0, // MAC Secondary interrupt mask register 3
     MAC_IMR_S3_QCU_QCBROVF_M  = 0x000003FF, // Mask for QCBROVF (QCU 0-9)
     MAC_IMR_S3_QCU_QCBRURN_M  = 0x03FF0000, // Mask for QCBRURN (QCU 0-9)
     MAC_IMR_S3_QCU_QCBRURN_S  = 16        , // Shift for QCBRURN (QCU 0-9)

     MAC_IMR_S4             = 0x00b4, // MAC Secondary interrupt mask register 4
     MAC_IMR_S4_QCU_QTRIG_M  = 0x000003FF, // Mask for QTRIG (QCU 0-9)
     MAC_IMR_S4_RESV0        = 0xFFFFFC00, // Reserved

// Interrupt status registers (read-and-clear access, secondary shadow copies)
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

// QCU registers
     MAC_NUM_QCU      = 10    , // Only use QCU 0-9 for forward QCU compatibility
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

     MAC_Q_ONESHOTARM_SC       = 0x0940, // MAC OneShotArm set control
     MAC_Q_ONESHOTARM_SC_M    = 0x0000FFFF, // Mask for MAC_Q_ONESHOTARM_SC (QCU 0-15)
     MAC_Q_ONESHOTARM_SC_RESV0  = 0xFFFF0000, // Reserved

     MAC_Q_ONESHOTARM_CC       = 0x0980, // MAC OneShotArm clear control
     MAC_Q_ONESHOTARM_CC_M    = 0x0000FFFF, // Mask for MAC_Q_ONESHOTARM_CC (QCU 0-15)
     MAC_Q_ONESHOTARM_CC_RESV0  = 0xFFFF0000, // Reserved

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

// DCU registers
     MAC_NUM_DCU      = 10    , // Only use 10 DCU's for forward QCU/DCU compatibility
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
     MAC_D_RETRY_LIMIT_FR_SH_S     = 0x0,        // Shift for frame short retry limit
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
     MAC_D_MISC_BKOFF_THRESH_M      = 0x000007FF, // Mask for Backoff threshold setting
     MAC_D_MISC_HCF_POLL_EN         = 0x00000800, // HFC poll enable
     MAC_D_MISC_BKOFF_PERSISTENCE   = 0x00001000, // Backoff persistence factor setting
     MAC_D_MISC_FR_PREFETCH_EN      = 0x00002000, // Frame prefetch enable
     MAC_D_MISC_VIR_COL_HANDLING_M  = 0x0000C000, // Mask for Virtual collision handling policy
     MAC_D_MISC_VIR_COL_HANDLING_NORMAL = 0     , // Normal
     MAC_D_MISC_VIR_COL_HANDLING_MODIFIED = 1   , // Modified
     MAC_D_MISC_VIR_COL_HANDLING_IGNORE = 2     , // Ignore
     MAC_D_MISC_BEACON_USE          = 0x00010000, // Beacon use indication
     MAC_D_MISC_ARB_LOCKOUT_CNTRL_M = 0x00060000, // Mask for DCU arbiter lockout control
     MAC_D_MISC_ARB_LOCKOUT_CNTRL_S = 17        , // Shift for DCU arbiter lockout control
     MAC_D_MISC_ARB_LOCKOUT_CNTRL_NONE = 0      , // No lockout
     MAC_D_MISC_ARB_LOCKOUT_CNTRL_INTRA_FR = 1  , // Intra-frame
     MAC_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL = 2    , // Global
     MAC_D_MISC_ARB_LOCKOUT_IGNORE  = 0x00080000, // DCU arbiter lockout ignore control
     MAC_D_MISC_SEQ_NUM_INCR_DIS    = 0x00100000, // Sequence number increment disable
     MAC_D_MISC_POST_FR_BKOFF_DIS   = 0x00200000, // Post-frame backoff disable
     MAC_D_MISC_VIRT_COLL_POLICY    = 0x00400000, // Virtual coll. handling policy
     MAC_D_MISC_BLOWN_IFS_POLICY    = 0x00800000, // Blown IFS handling policy
     MAC_531X_D_MISC_SEQ_NUM_CONTROL = 0x01000000, // Sequence Number local or global
                                                   // Maui2/Spirit/Freedom only
                                                   // reserved on Oahu
     MAC_D_MISC_RESV0               = 0xFE000000, // Reserved

     // NOTE: MAC_5211/MAC_5311 difference
     // On Maui2/Spirit, the frame sequence number is controlled per DCU.
     // On Oahu, the frame sequence number is global across all DCUs and is controlled
     // using MAC_D0_SEQNUM.
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

     MAC_D_FPCTL                = 0x1230,         // DCU frame prefetch settings
     MAC_D_FPCTL_DCU_M            = 0x0000000F,   // Mask for DCU for which prefetch is enabled
     MAC_D_FPCTL_DCU_S            = 0,            // Shift for DCU for which prefetch is enabled
     MAC_D_FPCTL_PREFETCH_EN      = 0x00000010,   // Enable prefetch for normal (non-burst) operation
     MAC_D_FPCTL_BURST_PREFETCH_M = 0x00007FE0,   // Mask for Burst frame prefetch per DCU
     MAC_D_FPCTL_BURST_PREFETCH_S = 5,            // Shift for Burst frame prefetch per DCU

     MAC_5211_D_TXPSE        = 0x1270, // MAC DCU transmit pause control/status
                                       // Oahu only
     MAC_5211_D_TXPSE_CTRL_M  = 0x000003FF, // Mask of DCUs to pause (DCUs 0-9)
     MAC_5211_D_TXPSE_RESV0   = 0x0000FC00, // Reserved
     MAC_5211_D_TXPSE_STATUS  = 0x00010000, // Transmit pause status
     MAC_5211_D_TXPSE_RESV1   = 0xFFFE0000, // Reserved

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

     MAC_INTPEND          = 0x4008, // Interrupt Pending register
     MAC_INTPEND_TRUE      = 0x00000001, // interrupt pending

     MAC_SFR              = 0x400C, // Sleep force register
     MAC_SFR_SLEEP         = 0x00000001, // force sleep

     MAC_PCICFG           = 0x4010, // PCI configuration register
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
     MAC_PCICFG_LED_MODE_S            = 17,
     MAC_PCICFG_LED_BLINK_THRESHOLD_M = 0x00700000, // Mask for LED blink threshold select
     MAC_PCICFG_LED_SLOW_BLINK_MODE   = 0x00800000, // LED slowest blink rate mode
     MAC_PCICFG_RESV2                 = 0xFF000000, // Reserved

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
#else
     MAC_PCICFG_EEPROM_SIZE_16K = 2,    // EEPROM size 16 Kbit
#endif /* PCI_INTERFACE */
     MAC_SREV_ID_S         = 4,          // Mask to shift Major Rev Info
     MAC_SREV_REVISION_M   = 0x0000000F, // Mask for Chip revision level
     MAC_SREV_FPGA         = 1,
     MAC_SREV_D2PLUS       = 2,
     MAC_SREV_D2PLUS_MS    = 3,          // metal spin
     MAC_SREV_CRETE        = 4,
     MAC_SREV_CRETE_MS     = 5,          // FCS metal spin
     MAC_SREV_CRETE_MS23   = 7,          // 2.3 metal spin (6 skipped)
     MAC_SREV_CRETE_23     = 8,          // 2.3 full tape out
     MAC_SREV_VERSION_M  = 0x000000F0, // Mask for Chip version indication
     MAC_SREV_VERSION_CRETE  = 0,
     MAC_SREV_VERSION_MAUI_1 = 1,
     MAC_SREV_VERSION_MAUI_2 = 2,
     MAC_SREV_VERSION_SPIRIT = 3,
     MAC_SREV_VERSION_OAHU   = 4,
     MAC_SREV_OAHU_ES        = 0,       // Engineering Sample
     MAC_SREV_OAHU_PROD      = 2,       // Production

     RAD5_SREV_MAJOR       = 0x10,  // All current supported ar5211 5 GHz radios are rev 0x10
     RAD5_SREV_PROD        = 0x15,  // Current production level radios
     RAD2_SREV_MAJOR       = 0x20,  // All current supported ar5211 2 GHz radios are rev 0x10

#if defined(PCI_INTERFACE)
// EEPROM Registers in the MAC
     MAC_EEPROM_ADDR        = 0x6000, // EEPROM address register (10 bit)
     MAC_EEPROM_DATA        = 0x6004, // EEPROM data register (16 bit)

     MAC_EEPROM_CMD     = 0x6008, // EEPROM command register
     MAC_EEPROM_CMD_READ     = 0x00000001,
     MAC_EEPROM_CMD_WRITE    = 0x00000002,
     MAC_EEPROM_CMD_RESET    = 0x00000004,

     MAC_EEPROM_STS     = 0x600c, // EEPROM status register
     MAC_EEPROM_STS_READ_ERROR   = 0x00000001,
     MAC_EEPROM_STS_READ_COMPLETE    = 0x00000002,
     MAC_EEPROM_STS_WRITE_ERROR  = 0x00000004,
     MAC_EEPROM_STS_WRITE_COMPLETE = 0x00000008,

     MAC_EEPROM_CFG     = 0x6010, // EEPROM configuration register
     MAC_EEPROM_CFG_SIZE_M   = 0x00000003, // Mask for EEPROM size determination override
     MAC_EEPROM_CFG_SIZE_AUTO   = 0,
     MAC_EEPROM_CFG_SIZE_4KBIT  = 1,
     MAC_EEPROM_CFG_SIZE_8KBIT  = 2,
     MAC_EEPROM_CFG_SIZE_16KBIT = 3,
     MAC_EEPROM_CFG_DIS_WAIT_WRITE_COMPL = 0x00000004, // Disable wait for write completion
     MAC_EEPROM_CFG_CLOCK_M  = 0x00000018, // Mask for EEPROM clock rate control
     MAC_EEPROM_CFG_CLOCK_S  = 3        , // Shift for EEPROM clock rate control
     MAC_EEPROM_CFG_CLOCK_156KHZ  = 0,
     MAC_EEPROM_CFG_CLOCK_312KHZ  = 1,
     MAC_EEPROM_CFG_CLOCK_625KHZ  = 2,
     MAC_EEPROM_CFG_RESV0        = 0x000000E0, // Reserved
     MAC_EEPROM_CFG_PROT_KEY_M = 0x00FFFF00, // Mask for EEPROM protection key
     MAC_EEPROM_CFG_PROT_KEY_S = 8          , // Shift for EEPROM protection key
     MAC_EEPROM_CFG_EN_L         = 0x01000000, // EPRM_EN_L setting
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
     MAC_STA_ID1_SADH_MASK   = 0x0000FFFF, // Mask for upper 16 bits of MAC addr
     MAC_STA_ID1_STA_AP      = 0x00010000, // Device is AP
     MAC_STA_ID1_AD_HOC      = 0x00020000, // Device is ad-hoc
     MAC_STA_ID1_PWR_SAV     = 0x00040000, // Power save reporting in self-generated frames
     MAC_STA_ID1_KSRCHDIS    = 0x00080000, // Key search disable
     MAC_STA_ID1_PCF           = 0x00100000, // Observe PCF
     MAC_STA_ID1_USE_DEFANT    = 0x00200000, // Use default antenna
     MAC_STA_ID1_DEFANT_UPDATE = 0x00400000, // Update default antenna w/ TX antenna
     MAC_STA_ID1_RTS_USE_DEF   = 0x00800000, // Use default antenna to send RTS
     MAC_STA_ID1_ACKCTS_6MB    = 0x01000000, // Use 6Mb/s rate for ACK & CTS
     MAC_STA_ID1_BASE_RATE_11B = 0x02000000, // Use 11b base rate for ACK & CTS

     MAC_BSS_ID0          = 0x8008, // MAC BSSID low 32 bits
     MAC_BSS_ID1          = 0x800C, // MAC BSSID upper 16 bits / AID
     MAC_BSS_ID1_U16_M     = 0x0000FFFF, // Mask for upper 16 bits of BSSID
     MAC_BSS_ID1_AID_M     = 0xFFFF0000, // Mask for association ID
     MAC_BSS_ID1_AID_S     = 16       , // Shift for association ID

     MAC_SLOT_TIME        = 0x8010, // MAC Time-out after a collision
     MAC_SLOT_TIME_MASK    = 0x000007FF, // Slot time mask

     MAC_TIME_OUT         = 0x8014, // MAC ACK & CTS time-out
     MAC_TIME_OUT_ACK_M    = 0x00001FFF, // Mask for ACK time-out
     MAC_TIME_OUT_CTS_M    = 0x1FFF0000, // Mask for CTS time-out
     MAC_TIME_OUT_CTS_S    = 16       , // Shift for CTS time-out

     MAC_RSSI_THR         = 0x8018, // MAC Beacon RSSI warning and missed beacon threshold
     MAC_RSSI_THR_MASK     = 0x000000FF, // Mask for Beacon RSSI warning threshold
     MAC_RSSI_THR_BM_THR_M = 0x0000FF00, // Mask for Missed beacon threshold
     MAC_RSSI_THR_BM_THR_S = 8        , // Shift for Missed beacon threshold

     MAC_USEC             = 0x801c, // MAC transmit latency register
     MAC_USEC_M            = 0x0000007F, // Mask for clock cycles in 1 usec
     MAC_USEC_32_M         = 0x00003F80, // Mask for number of 32MHz clock cycles in 1 usec
     MAC_USEC_32_S         = 7        , // Shift for number of 32MHz clock cycles in 1 usec
     // NOTE: MAC_5211/MAC_5311 difference
     // On Oahu, the TX latency field has increased from 6 bits to 9 bits.
     // The RX latency field is unchanged, but is shifted over 3 bits.
     MAC_531X_USEC_TX_LAT_M     = 0x000FC000, // Mask for tx latency to start of SIGNAL (usec)
     MAC_531X_USEC_TX_LAT_S     = 14       , // Shift for tx latency to start of SIGNAL (usec)
     MAC_531X_USEC_RX_LAT_M     = 0x03F00000, // Mask for rx latency to start of SIGNAL (usec)
     MAC_531X_USEC_RX_LAT_S     = 20       , // Shift for rx latency to start of SIGNAL (usec)

     MAC_5211_USEC_TX_LAT_M     = 0x007FC000, // Mask for tx latency to start of SIGNAL (usec)
     MAC_5211_USEC_TX_LAT_S     = 14       , // Shift for tx latency to start of SIGNAL (usec)
     MAC_5211_USEC_RX_LAT_M     = 0x1F800000, // Mask for rx latency to start of SIGNAL (usec)
     MAC_5211_USEC_RX_LAT_S     = 23       , // Shift for rx latency to start of SIGNAL (usec)


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
     MAC_RX_PHY_ERR        = 0x00000040, // Allow all phy errors
     MAC_RX_PHY_RADAR      = 0x00000080, // Allow only radar phy errors

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
     MAC_531X_DIAG_USE_ECO = 0x00000400, // "super secret" use ECO enable bit
     MAC_DIAG_SCRAM_SEED_M = 0x0001FC00, // Mask for fixed scrambler seed
     MAC_DIAG_SCRAM_SEED_S = 10       , // Shift for fixed scrambler seed
     MAC_DIAG_FRAME_NV0    = 0x00020000, // accept frames of non-zero protocol version
     MAC_DIAG_OBS_PT_SEL_M = 0x000C0000, // Mask for observation point select
     MAC_DIAG_OBS_PT_SEL_S = 18       , // Shift for observation point select

     MAC_TSF_L32          = 0x804c, // MAC local clock lower 32 bits
     MAC_TSF_U32          = 0x8050, // MAC local clock upper 32 bits

     MAC_TST_ADDAC        = 0x8054, // ADDAC test register
     MAC_DEF_ANTENNA      = 0x8058, // default antenna register

     MAC_LAST_TSTP        = 0x8080, // MAC Time stamp of the last beacon received
     MAC_NAV              = 0x8084, // MAC current NAV value
     MAC_RTS_OK           = 0x8088, // MAC RTS exchange success counter
     MAC_RTS_FAIL         = 0x808c, // MAC RTS exchange failure counter
     MAC_ACK_FAIL         = 0x8090, // MAC ACK failure counter
     MAC_FCS_FAIL         = 0x8094, // FCS check failure counter
     MAC_BEACON_CNT       = 0x8098, // Valid beacon counter

    MAC_KEY_CACHE        = 0x8800,  // MAC Key Cache
    MAC_KEY_CACHE_SIZE    = 128,
    MAC_KEY_TYPE_M        = 0x00000007, // MAC Key Type Mask
    MAC_KEY_TYPE_WEP_40   = 0,
    MAC_KEY_TYPE_WEP_104  = 1,
    MAC_KEY_TYPE_WEP_128  = 3,
    MAC_KEY_TYPE_AES      = 5,
    MAC_KEY_TYPE_CLEAR    = 7,

// PHY registers
    PHY_BASE             = 0x9800, // PHY registers base address

    PHY_TURBO            = 0x9804, // PHY frame control register
    PHY_FC_TURBO_MODE     = 0x00000001, // Set turbo mode bits
    PHY_FC_TURBO_SHORT    = 0x00000002, // Set short symbols to turbo mode setting

    PHY_CHIP_ID          = 0x9818, // PHY chip revision ID

    PHY_ACTIVE           = 0x981C, // PHY activation register
    PHY_ACTIVE_EN         = 0x00000001, // Activate PHY chips
    PHY_ACTIVE_DIS        = 0x00000000, // Deactivate PHY chips

    PHY_AGC_CONTROL      = 0x9860, // PHY chip calibration and noise floor setting
    PHY_AGC_CONTROL_CAL   = 0x00000001, // Perform PHY chip internal calibration
    PHY_AGC_CONTROL_NF    = 0x00000002, // Perform PHY chip noise-floor calculation
    PHY_CCA               = 0x9864,
    PHY_CCA_THRESH62_M    = 0x0007F000,
    PHY_CCA_THRESH62_S    = 12,

    PHY_PLL_CTL          = 0x987c,      // PLL control register
    PHY_PLL_CTL_44       = 0x19,        // 44 MHz for 11b channels and FPGA
    PHY_PLL_CTL_40       = 0x18,        // 40 MHz
    PHY_PLL_CTL_20       = 0x13,        // 20 MHz half rate 11a for emulation
    PHY_PLL_CTL_AR5312A  = 0x5,         // 32MHz ref clk * 5 = 160MHz clock
    PHY_PLL_CTL_AR5312B  = 0x4b,        // 32MHz ref clk / 2 * 11 = 176MHz



    PHY_RX_DELAY         = 0x9914, // PHY analog_power_on_time, in 100ns increments
    PHY_RX_DELAY_M       = 0x00003FFF, // Mask for delay from active assertion (wake up)
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

    PHY_PAPD_PROBE             = 0x9930,
    PHY_PAPD_PROBE_POWERTX_M    = 0x00007E00,
    PHY_PAPD_PROBE_POWERTX_S    = 9,
    PHY_PAPD_PROBE_NEXT_TX      = 0x00008000, // bit 15 as command to take next reading
    PHY_PAPD_PROBE_GAINF_M      = 0xFE000000,
    PHY_PAPD_PROBE_GAINF_S      = 25,

    PHY_POWER_TX_RATE1         = 0x9934,
    PHY_POWER_TX_RATE2         = 0x9938,
    PHY_POWER_TX_RATE_MAX      = 0x993c,

    PHY_FRAME_CTL              = 0x9944,
    PHY_FRAME_CTL_TX_CLIP_M     = 0x00000038,
    PHY_FRAME_CTL_TX_CLIP_S     = 3,

    PHY_RADAR_0                = 0x9954, // PHY radar detection settings
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

     ANT_SWITCH_TABLE1          = 0x9960,
     ANT_SWITCH_TABLE2          = 0x9964,

    PHY_IQCAL_RES_PWR_MEAS_I   = 0x9c10, //PHY IQ calibration results - power measurement for I
    PHY_IQCAL_RES_PWR_MEAS_Q   = 0x9c14, //PHY IQ calibration results - power measurement for Q
    PHY_IQCAL_RES_IQ_CORR_MEAS = 0x9c18, //PHY IQ calibration results - IQ correlation measurement
    PHY_CURRENT_RSSI           = 0x9c1c, // rssi of current frame being received

    PHY_5211_MODE              = 0xA200,   // Mode register
    PHY_5211_MODE_OFDM          = 0x0,      // bit 0 = 0 for OFDM
    PHY_5211_MODE_CCK           = 0x1,      // bit 0 = 1 for CCK
    PHY_5211_MODE_RF5GHZ        = 0x0,      // bit 1 = 0 for 5 GHz
    PHY_5211_MODE_RF2GHZ        = 0x2,      // bit 1 = 1 for 2.4 GHz
}; /* enum ar5211Registers */

#ifdef _cplusplus
}
#endif

#endif /* _AR5211_REG_H_ */
