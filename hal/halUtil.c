/*
 *  Copyright (c) 2000-2002 Atheros Communications, Inc., All Rights Reserved
 *
 *  Connects reset Reg Vectors, EEPROM Data, and device Functions to pDev
 *  TODO: check comment for accuracy
 */

#ident "$Id: //depot/sw/branches/AV_dev/src/hal/halUtil.c#1 $"

#include "wlantype.h"

/**************************************************************
 * reverseBits
 *
 * reverse the bits starting at the low bit for a value of
 * bit_count in size
 * TODO: Someone could optimize this pretty easily
 */
A_UINT32
reverseBits(A_UINT32 val, A_UINT32 bitCount)
{
    A_UINT32    retval = 0;
    A_UINT32    bit;
    A_UINT32    i;

    for (i = 0; i < bitCount; i++) {
        bit = (val >> i) & 1;
        retval = (retval << 1) | bit;
    }

    return retval;
}
