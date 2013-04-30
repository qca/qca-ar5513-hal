/*
 * Copyright © 2000-2002 Atheros Communications, Inc.,  All Rights Reserved.
 */

#ifndef _HAL_DEVID_H_
#define _HAL_DEVID_H_

#ifdef _cplusplus
extern "C" {
#endif

/* AR5210 (for reference) */
#define AR5210_DEFAULT          0x1107          /* No eeprom HW default */
#define AR5210_PROD             0x0007          /* Final device ID */
#define AR5210_AP               0x0207          /* Early AP11s */

/* AR5211 */
#define AR5211_DEFAULT          0x1112          /* No eeprom HW default */
#define AR5311_DEVID            0x0011          /* Final ar5311 devid */
#define AR5211_DEVID            0x0012          /* Final ar5211 devid */
#define AR5211_LEGACY           0xff12          /* Original emulation board */
#define AR5211_FPGA11B          0xf11b          /* 11b emulation board */
#define AR5211_SREV_1_0         0x40
#define AR5211_SREV_1_1         0x42
#define AR5211_SREV_REG         0x4020

/* AR5212 */
#define AR5212_DEFAULT          0x1113          /* No eeprom HW default */
#define AR5212_DEVID            0x0013          /* ar5212 devid */
#define AR5312_DEVID            0x0030          /* Final ar5312 devid */
#define AR5212_SREV_1_0         0x50
#define AR5212_SREV_1_1         0x51
#define AR5212_SREV_1_3         0x53
#define AR5212_SREV_1_4         0x56
#define AR5212_SREV_REG         0x4020

/* AR5212 compatible devid's also attach to 5212 */
#define AR5212_DEVID_0014       0x0014          /* ar5212 forward compatibility id */
#define AR5212_DEVID_0015       0x0015          /* ar5212 forward compatibility id */
#define AR5212_DEVID_0016       0x0016          /* ar5212 forward compatibility id */
#define AR5212_DEVID_0017       0x0017          /* ar5212 forward compatibility id */
#define AR5212_DEVID_0018       0x0018          /* ar5212 forward compatibility id */
#define AR5212_DEVID_0019       0x0019          /* ar5212 forward compatibility id */
#define AR5212_DEVID_001A       0x001A          /* ar2413 id */

/* AR5213 */
#define AR5213_SREV_1_0         0x55
#define AR5213_SREV_REG         0x4020

/* AR5513 */
#define AR5513_DEFAULT          0x1120          /* No eeprom HW default */
#define AR5513_DEVID            0x0020          /* ar5513 devid */
#define AR5513_DEVID_0013       0x0013          /* Temporary devid */
#define AR5513_DEVID_ff18       0xff18          /* Falcon 1.0 */
#if defined(FALCON_EMUL)
#define AR5513_DEVID_fb40        0xfb40          /* Falcon Emulation */
#endif /* FALCON_EMUL */

#define AR5513_SREV_1_0         0x70
#define AR5513_SREV_REG         0x4014

/* SubVendor Specific Ids */
#define SUBVENDOR_ID_NO_G       0x0e11          /* No 11G subVendor ID */
#define SUBSYSTEM_ID_NEW_A      0x7065          /* Update device to new RD */

#ifdef _cplusplus
}
#endif

#endif /* _HAL_DEVID_H */
