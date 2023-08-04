/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _REGISTER_TYPES_H_
#define _REGISTER_TYPES_H_

/******************************************************************************
* Includes
******************************************************************************/

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
/* Bits peripheral register access macros */
#define testReg16Bits(RegName, GetMask)                          (RegName & (uint16_t)(GetMask))
#define clrReg16Bits(RegName, ClrMask)                           (RegName &= (uint16_t)(~(uint16_t)(ClrMask)))
#define setReg16Bits(RegName, SetMask)                           (RegName |= (uint16_t)(SetMask))
#define invertReg16Bits(RegName, InvMask)                        (RegName ^= (uint16_t)(InvMask))

/* Whole peripheral register access macros */
#define setReg16(RegName, val)                                   (RegName = (uint16_t)(val))
#define getReg16(RegName)                                        (RegName)

#endif /* _REGISTER_TYPES_H_ */