/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _MOTOR_DEF_H_
#define _MOTOR_DEF_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "register_types.h"
#include "fsl_device_registers.h"
#include "motor_structure.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define	M1_IA_ADC_0		        3
#define M1_IB_ADC_0 	        8
#define M1_IB_ADC_1		    8
#define M1_IC_ADC_1 	        3

#define	M2_IA_ADC_0		        3
#define M2_IB_ADC_0 	        2
#define M2_IB_ADC_1		    2
#define M2_IC_ADC_1 	        3

#define MC_FAULT_STARTUP_FAIL 	0   	/* Start-up fail */
#define MC_FAULT_LOAD_OVER      1		/* Overload Flag */
#define MC_FAULT_I_DCBUS_OVER 	2		/* OverCurrent fault flag */
#define MC_FAULT_U_DCBUS_UNDER 	3	/* Undervoltage fault flag */
#define MC_FAULT_U_DCBUS_OVER 	4		/* Overvoltage fault flag */

/* Sets the fault bit defined by faultid in the faults variable */
#define MC_FAULT_SET(faults, faultid)   (faults |= ((MCDEF_FAULT_T)1 << faultid))

/* Clears the fault bit defined by faultid in the faults variable */
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((MCDEF_FAULT_T)1 << faultid))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((MCDEF_FAULT_T)1 << faultid)) >> faultid)

/* Clears all fault bits in the faults variable */
#define MC_FAULT_CLEAR_ALL(faults) (faults = 0)

/* Check if a fault bit is set in the faults variable, 0 = no fault */
#define MC_FAULT_ANY(faults) (faults > 0)

/* Update a fault bit defined by faultid in the faults variable according to the LSB of value */
#define MC_FAULT_UPDATE(faults, faultid, value) {MC_FAULT_CLEAR(faults, faultid); faults |= (((MC_FAULT_T)value & (MC_FAULT_T)1) << faultid);}

#define M1_PWM1_RUN()	(PWM0->MCTRL |= PWM_MCTRL_RUN(0x7U))
#define M2_PWM0_RUN()	(PWM1->MCTRL |= PWM_MCTRL_RUN(0x7U))

#define M1_DISABLE_PWM_OUTPUT() clrReg16Bits(PWM0->OUTEN, PWM_OUTEN_PWMA_EN(0x7U) | PWM_OUTEN_PWMB_EN(0x7U))
#define M1_ENABLE_PWM_OUTPUT()  setReg16Bits(PWM0->OUTEN, PWM_OUTEN_PWMA_EN(0x7U) | PWM_OUTEN_PWMB_EN(0x7U))

#define M2_DISABLE_PWM_OUTPUT() clrReg16Bits(PWM1->OUTEN, PWM_OUTEN_PWMA_EN(7U) | PWM_OUTEN_PWMB_EN(7U))
#define M2_ENABLE_PWM_OUTPUT()  setReg16Bits(PWM1->OUTEN, PWM_OUTEN_PWMA_EN(7U) | PWM_OUTEN_PWMB_EN(7U))

#define M1_OVERCURRENT_FAULT() ((PWM0->FSTS & PWM_FSTS_FFLAG(1)) > 0) 
#define M2_OVERCURRENT_FAULT() ((PWM1->FSTS & PWM_FSTS_FFLAG(1)) > 0) 

#define M1_CLEAR_OVERCURRENT_FAULT()  (PWM0->FSTS |= PWM_FSTS_FFLAG(1))
#define M2_CLEAR_OVERCURRENT_FAULT()  (PWM1->FSTS |= PWM_FSTS_FFLAG(1))

#define DISABLE		0
#define ENABLE		1
/******************************************************************************
* Types
******************************************************************************/
typedef uint16_t MCDEF_FAULT_T;
typedef struct
{
	float_t					fltUDcBusOver;		/* DC bus over voltage level */
	float_t					fltUDcBusUnder;		/* DC bus under voltage level */
	float_t					fltSpeedOver;		/* Over speed level */
	float_t					fltSpeedUnder;		/* Under speed level */
} MCDEF_FAULT_THRESHOLDS_T;


/* PMSM FOC with Encoder with speed PI controller */
typedef struct
{
	MCDEF_FAULT_T				sFaultId;
	MCDEF_FAULT_T				sFaultIdPending;
	MCDEF_FAULT_THRESHOLDS_T	        sFaultThresholds;
	MCSTRUC_FOC_PMSM_T			sFocPMSM;
	MCSTRUC_SPEED_CTRL_T			sSpeedCtrl;
	MCSTRUC_POS_SPEED_ENCODER_T		sSpeedPos;
	MCSTRUC_POS_CTRL_T			SPosCtrl;
    int32_t 				i32PosRelative;  		// the rotor relative position
	uint16_t 				ui16FlagSlowLoop;
	uint16_t 				ui16CounterSlowLoop;
	uint16_t 				ui16DividerSlowLoop;
    frac16_t                f16AdcAuxSample;                      /* Auxiliary ADC sample  */
	uint16_t 			    ui16CounterState;	// defines how many slow loops that the sub-state lasts for
	uint16_t 				ui16TimeFullSpeedFreeWheel;
	uint16_t				ui16TimeCalibration;
	uint16_t				ui16TimeFaultRelease;
	uint16_t				ui16CounterSpdCal;
    uint16_t               ui16FlagAlignFinished;
} MCDEF_FOC_PMSM_ENC_SPEED_PI_T;

typedef struct
{
	int16_t					i16EncCnt;
	int16_t					i16IndexCnt;
} MCDEF_ENCODER_COUNTER_T;

/******************************************************************************
* Global variables
******************************************************************************/
// Variables for FreeMASTER 
volatile static 			int16_t							FmIBase;			// mA
volatile static 			int16_t							FmIObsrvBase;		// mA
volatile static 			int16_t							FmUBase;			// Volts
volatile static 			int16_t							FmBemfBase;			// Volts
volatile static 			int16_t							FmSpdElRadBase; 	// electrical:rad.s-1
volatile static 			int16_t							FmSpdElHzBase;		// electrical:Hz
volatile static 			int16_t   						FmSpdMechRpmBase;	// mechanical:rpm
volatile static 			int16_t							FmPosElBase;		// electrical:deg		
volatile static			int16_t							FmPosMechBase;		// mechanical: rounds
volatile static			int16_t							FmSpdCtrlKpBase;	// Speed controller Parameter: Kp
volatile static			int32_t							FmBBase;			// inertia identification: B
volatile static			int16_t							FmTorqueBase;		// torque : N *M^2
volatile static			int16_t							FmOmegaBase;		// angular speed: Rad/s
volatile static			int16_t							FmFlagRecorder;		// flag for recorder
volatile static			int16_t							FmSineWaveFreqBase;	// sine wave reference frequency: hz
volatile static			int16_t							FmPosCtrlKpBase;	// position controller Parameter: Kp

/******************************************************************************
* Global functions
******************************************************************************/
extern void FM_Variables_Init(void);

/******************************************************************************
* Inline functions
******************************************************************************/
inline void M1_PWM_UPDATE(GMCLIB_3COOR_T_F16 *psDuty)
{
    int16_t i16ModuloHalf = PWM0->SM[0].VAL1 + 1;
	int16_t i16Result;
	
	/* PWM channels 0&1  */ 
	i16Result = MLIB_MulRnd_F16(psDuty->f16A, i16ModuloHalf);

	setReg16(PWM0->SM[0].VAL2, -i16Result);
	setReg16(PWM0->SM[0].VAL3, i16Result);

	/* PWM channels 2&3 */								 
	i16Result = MLIB_MulRnd_F16(psDuty->f16B, i16ModuloHalf);
	
	setReg16(PWM0->SM[1].VAL2, -i16Result);
	setReg16(PWM0->SM[1].VAL3, i16Result);
														 
	/* PWM channels 4&5 */								 
	i16Result = MLIB_MulRnd_F16(psDuty->f16C, i16ModuloHalf);

	setReg16(PWM0->SM[2].VAL2, -i16Result);
	setReg16(PWM0->SM[2].VAL3, i16Result);
														 
	/* load new values to PWM registers */
	setReg16Bits(PWM0->MCTRL, PWM_MCTRL_LDOK(0x7U)); /* LDOK */ 
}

inline void M2_PWM_UPDATE(GMCLIB_3COOR_T_F16 *psDuty)
{
    int16_t i16ModuloHalf = PWM1->SM[0].VAL1 + 1;
	int16_t i16Result;
	
	/* PWM channels 0&1  */ 
	i16Result = MLIB_MulRnd_F16(psDuty->f16A, i16ModuloHalf);

	setReg16(PWM1->SM[0].VAL2, -i16Result);
	setReg16(PWM1->SM[0].VAL3, i16Result);

	/* PWM channels 2&3 */								 
	i16Result = MLIB_MulRnd_F16(psDuty->f16B, i16ModuloHalf);
	
	setReg16(PWM1->SM[1].VAL2, -i16Result);
	setReg16(PWM1->SM[1].VAL3, i16Result);
														 
	/* PWM channels 4&5 */								 
	i16Result = MLIB_MulRnd_F16(psDuty->f16C, i16ModuloHalf);

	setReg16(PWM1->SM[2].VAL2, -i16Result);
	setReg16(PWM1->SM[2].VAL3, i16Result);
														 
	/* load new values to PWM registers */
	setReg16Bits(PWM1->MCTRL, PWM_MCTRL_LDOK(7U)); /* LDOK */ 
}

#endif /* _MOTOR_DEF_H_ */