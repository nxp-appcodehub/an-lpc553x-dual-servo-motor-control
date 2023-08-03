/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCDRV_ENC_H_
#define MCDRV_ENC_H_

#include "amclib.h"
#include "amclib_fp.h"
#include "fsl_device_registers.h"
#include "mlib.h"
#include "mlib_fp.h"
#include "trigonometric.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _enc_to
{
	frac16_t    f16PosErr; // Position error to the tracking observer
	AMCLIB_TRACK_OBSRV_T_FLT    sTO; // Tracking observer
	frac16_t    f16PosEstim;
	float_t     fltSpeedEstim;
}enc_to_t;

typedef struct _enc_speed
{
	int16_t i16POSDH;
	uint16_t ui16POSDPERH;
	uint16_t ui16LASTEDGEH;

	int16_t  i16PosDiff;    // Position counter differences between 2 consecutive speed calculations
	uint16_t ui16Period;     // Time duration corresponding to the position counter differences
	uint16_t ui16Period_1;   // Last time duration

	int8_t   i8SpeedSign;   // Speed sign in this step
	int8_t   i8SpeedSign_1; // Speed sign in last step
	frac32_t f32Speed;      // Calculated speed
	frac16_t f16SpeedFilt;

	GDFLIB_FILTER_IIR1_T_F32 			sENCSpeedFilter;
	frac32_t f32SpeedCalConst;
	float_t  fltSpeedFrac16ToAngularCoeff;

	float_t fltSpeed;		// float type, electrical angular speed
	int32_t i32Position;
	int32_t i32Position_1;
}enc_speed_t;

typedef struct _enc_block
{
	enc_to_t    sSpeedEstim;    // A module to estimate speed by tracking observer
	enc_speed_t sSpeed;			// A module to calculate speed by ENC HW feature


	/* Encoder attributes */
    uint16_t    ui16Line;       // Encoder lines
	uint16_t    ui16PolePair;   // Motor pole pairs
    frac32_t    f32ReciprocalLines;  // Reciprocal of 4*Lines
    uint32_t    ui32Mod;        // ENC counter MOD value
    ENC_Type    *pENC_base;     // ENC module base address

    /* Rotor position related for FOC algorithm */
    uint32_t    ui32InitCount;      // ENC counter value when the initial position has been identified
    frac32_t    f32PosMechInit;     // The mechanical rotor position corresponding to the initial ENC counter value
    frac32_t    f32PosMechOffset;   // Rotor real mechanical position at the initial position
    uint32_t    ui32CurrentCount;   // Current ENC counter value
    int32_t     i32Q10Cnt2PosGain;      // A gain to convert ENC counter value to scaled position value -1~1. This gain is Q22.10 format
	frac32_t    f32PosMech;         // Rotor real mechanical position
	frac32_t    f32PosElec;         // Rotor real electrical position, Q1.31
	frac16_t    f16PosElec;			// Rotor real electrical position, Q1.15

	/* Rotor position related for position loop control */
    int32_t     i32Q16InitRev;      // Revolution value when initial rotor position has been identified. Q16.16 format, integer part represents revolutions, fractional part represents the part which is "less than 1 revolution"
	int32_t     i32Q16Rev;			// Current revolution value, Q16.16
	int32_t     i32Q16DeltaRev;     // Revolutions between current rotor position and the initial rotor position, Q16.16

	bool_t      bPosAbsoluteFlag; // A flag indicating whether rotor position is an absolute correct value
	uint16_t    ui16Dummy;          // A dummy variable to get captured position counter value
	uint8_t  	ui8MotorNum;

	/* Index signal related */
	uint32_t    ui32IndexCount;    // ENC counter value when index signal occurs
	frac32_t    f32PosMechIndex;   // The mechanical rotor position corresponding to the ENC counter value when index signal occurs
	frac32_t    f32PosElecIndex;
	frac16_t    f16PosElecIndex;
}enc_block_t;

__attribute__((always_inline)) static inline void MCDRV_GetRotorDeltaRev(enc_block_t *this)
{
	this->i32Q16DeltaRev = this->i32Q16Rev - this->i32Q16InitRev;
}

extern bool_t MCDRV_EncSpeedCalInit(enc_block_t *this);
extern bool_t MCDRV_EncSpeedCalUpdate(enc_block_t *this);
extern bool_t MCDRV_EncToSpeedCalInit(enc_block_t *this);
extern bool_t MCDRV_EncToSpeedCalUpdate(enc_block_t *this);
extern bool_t MCDRV_EncInit(enc_block_t *this);
extern bool_t MCDRV_GetRotorInitPos(enc_block_t *this, frac32_t f32PosMechOffset);
extern bool_t MCDRV_GetRotorInitRev(enc_block_t *this);
extern bool_t MCDRV_GetRotorCurrentPos(enc_block_t *this);
extern bool_t MCDRV_GetRotorCurrentRev(enc_block_t *this);
#endif /* MCDRV_ENC_H_ */
