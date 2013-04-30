/*
 * $Id: //depot/sw/branches/AV_dev/src/hal/halDesc.h#6 $
 *
 * Copyright © 2000-2003 Atheros Communications, Inc.,  All Rights Reserved.
 *
 * Defines the hardware descriptor data structures.
 *
 * Fields named hwSpecific are chipset dependent. Use the HAL descriptor
 * access functions to access these fields.
 * All other fields are common to all Atheros chipsets.
 */

#ifndef _HAL_DESC_H_
#define _HAL_DESC_H_

#ifdef _cplusplus
extern "C" {
#endif

#if defined(BUILD_AR5513)
/*
**
** IMPORTANT:  HAL_MAX_DESC_WORDS must be CACHE ALIGNED, for example
**             if cache line size is 32 bytes or 8 words, then
**             valid values would be increments of 8 words minus
**             the next and buffer pointers.
*/
#define  HAL_DESC_SIZE  160
#define  HAL_MAX_DESC_WORDS  14    /* 14 + 2 = 16 */
#else
#define  HAL_DESC_SIZE  128
#define  HAL_MAX_DESC_WORDS   6    /* 6 + 2 = 8 */
#endif /* ! BUILD_AR5513 */

typedef struct hwTxControlAccess {
#ifdef BIG_ENDIAN
    A_UINT32    hwSpecific1:20,     /* 31:12 hardware specific bits */
                frameLength:12;     /* 11:0  length of the 802.11 packet */
    A_UINT32    hwSpecific2:19,     /* 31:13 hardware specific  */
                more:1,             /* 12    set if packet continues in next descriptor */
                bufferLength:12;    /* 11:0  size of buffer for this descriptor */
#else
    A_UINT32    frameLength:12,     /* 11:0  length of the 802.11 packet */
                hwSpecific1:20;     /* 31:12 hardware specific */
    A_UINT32    bufferLength:12,    /* 11:0  size of buffer for this descriptor */
                more:1,             /* 12    set if packet continues in next descriptor */
                hwSpecific2:19;     /* 31:13 hardware specific */
#endif
    A_UINT32    pad[HAL_MAX_DESC_WORDS - 2]; /* to make it align to the rest of hw area */
} HW_TX_CONTROL_ACCESS;

/* Hardware Descriptor Packet Types */
typedef enum {
    HAL_DESC_PKT_TYPE_NORMAL = 0,
    HAL_DESC_PKT_TYPE_ATIM,
    HAL_DESC_PKT_TYPE_PSPOLL,
    HAL_DESC_PKT_TYPE_BEACON,
    HAL_DESC_PKT_TYPE_PROBE_RESP,
    HAL_DESC_PKT_TYPE_CHIRP,
    HAL_DESC_PKT_TYPE_GRP_POLL,
#if defined(FALCON_ES1)
    /* ES1 Silicon used Type 7 as Sync Frame type */
    HAL_DESC_PKT_TYPE_SYNC          /* Specific to AR5513 */
#else  /* ! FALCON_ES1 */
    /* ES2 Silicon used Type 8 as Sync Frame type */
    HAL_DESC_PKT_TYPE_DIRECT_POLL,  /* Directed Poll Frame Type from Eagle */
    HAL_DESC_PKT_TYPE_SYNC          /* Specific to AR5513 */
#endif /* ! FALCON_ES1 */
} HAL_DESC_PKT_TYPE;

#ifdef _cplusplus
}
#endif

#endif /* _HAL_DESC_H_ */
