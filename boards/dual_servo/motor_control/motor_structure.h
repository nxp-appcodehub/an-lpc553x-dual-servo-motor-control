/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _MCSTRUC_H_
#define _MCSTRUC_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "stdbool.h"
#include "mlib.h"
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"
#include "mlib_FP.h"
#include "gflib_FP.h"
#include "gmclib_FP.h"
#include "gdflib_FP.h"
#include "mlib_FP.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
/* Alignment */
#define ALIGN_METHOD					1	// indicate to which alignment method to be use

typedef struct
{
    frac16_t f16A;
    frac16_t f16B;
} MCLIB_2_COOR_SYST_T;

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    uint16_t					ui16Ph0;		/* Phase 0 channel number */
    uint16_t					ui16Ph1;		/* Phase 1 channel number */
} MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T;

typedef struct
{
	frac32_t f32X;						// input value
	frac32_t f32X_1;						// last time input value
	frac32_t f32Y; 						// output value
	frac32_t f32Y_1;						// last time output value
	frac32_t f32B1;						// b1 coefficient of the filter
	frac32_t f32B2;						// b2 coefficient of the filter
	frac32_t f32A2;						// a2 coefficient of the filter
} IIR32_FILTER_T;

typedef struct
{
    frac16_t						f16Position;			/* Position of field at alignment, executed in current loop */
    float_t				    		fltU;					/* D voltage at alignment, f32U += f32UStep */
    float_t				    		fltId;					/* D current at alignment, f32Id += f32IdStep */
    frac16_t						f16Speed;				/* Speed of field at alignment */
    float_t			    			fltUStep;				/* D voltage ramp at alignment */
    float_t				    		fltIdStep;				/* D voltage ramp at alignment */
    float_t						    fltIMax;				/* Max D current at alignment */
    float_t						    fltUMax;				/* Max D voltage at alignment */
    uint16_t						ui16TimeAlignment;		/* Alignment time duration */
} MCSTRUC_ALIGNMENT_T;

typedef struct
{
    GFLIB_CTRL_PI_P_AW_T_FLT            sIdPiParams;		/* Id PI controller parameters */
    GFLIB_CTRL_PI_P_AW_T_FLT            sIqPiParams;		/* Iq PI controller parameters */
    GDFLIB_FILTER_IIR1_T_FLT			sIdZcFilter;		/* D current zero-cancellation filter */
    GDFLIB_FILTER_IIR1_T_FLT			sIqZcFilter;		/* Q current zero-cancellation filter */
    GDFLIB_FILTER_IIR1_T_FLT			sUDcBusFilter;	/* Dc bus voltage filter */
    GMCLIB_3COOR_T_F16                 sIABCFrac;
    GMCLIB_3COOR_T_FLT      			sIABC;			/* Measured 3-phase current */
    GMCLIB_2COOR_ALBE_T_FLT	                sIAlBe;   		/* Alpha/Beta current */
    GMCLIB_2COOR_DQ_T_FLT			sIDQ;     		/* DQ current */
    GMCLIB_2COOR_DQ_T_FLT			sIDQReq;  		/* DQ required current */
    GMCLIB_2COOR_DQ_T_FLT			sIDQReqZc;  		/* DQ required current after zero cancellation */
    GMCLIB_2COOR_DQ_T_FLT			sIDQError;  		/* DQ current error */
    GMCLIB_3COOR_T_F16    			sDutyABC;			/* Applied duty cycles ABC */
    GMCLIB_2COOR_ALBE_T_FLT	    sUAlBeReq;   		/* Required Alpha/Beta voltage */
    GMCLIB_2COOR_ALBE_T_F16     	sUAlBeCompFrac;	 	/* Compensated to DC bus Alpha/Beta voltage */
    GMCLIB_2COOR_DQ_T_FLT			sUDQReq;     		/* Required DQ voltage */
    GMCLIB_2COOR_DQ_T_FLT			sUDQController; 	/* Required DQ voltage */
    GMCLIB_2COOR_SINCOS_T_FLT					sAnglePosEl; 		/* Electrical position sin/cos (at the moment of PWM current reading) */
    MCSTRUC_ALIGNMENT_T				sAlignment;		/* Alignment structure params */
	uint16_t 						ui16SectorSVM;		/* SVM sector */
	float_t							fltDutyCycleLimit;	/* Max. allowable duty cycle in frac */
	frac16_t						f16UDcBus;			/* DC bus voltage */
    float_t                         fltUDcBus;
	float_t						    fltUDcBusFilt;		/* Filtered DC bus voltage */
	bool_t 					        bIdPiSatFlag;		/* Id PI controller saturation flag */
	bool_t 					        bIqPiSatFlag;		/* Iq PI controller saturation flag */
	bool_t							bOpenLoop;			/* Current control loop is open */
	bool_t							bUseMaxBus;			/* Calculate the max. possible DQ current controllers' output limits based on dc bus voltage */
	bool_t							bUseZc;				/* User zero-cancellation filter */
} MCSTRUC_FOC_PMSM_T;

typedef struct
{
	float_t                          fltSpdErr;				// speed error between speed reference and speed feedback
    GFLIB_CTRL_PI_P_AW_T_FLT            sSpdPiParams;		/* Speed PI controller parameters */
	bool_t 					            bSpdPiSatFlag;     /* Speed PI controller saturation flag */
	float_t fltIqRef;				// output: q axis current reference
} MCSTRUC_SPEED_PI_T;

typedef struct
{
	acc32_t 							a32PropGain;			// PropGain of the Position controller
    acc32_t                            a32FwdGain;
	int16_t 							i16SatFlag;				// saturation flag of calculation
	float_t		    					fltUpperLimit;			// upper limit value
	float_t    							fltLowerLimit;			// lower limit value
	acc32_t 							a32ResultPreSat;		// result of calculation before saturation
    float_t                           fltSpeedCtrl;
    float_t                           fltSpeedPreSat;
	float_t 					    	fltResultAftSat;		// result of calculation after saturation
	float_t			    				fltSpeedRef;			// output of Position controller
    float_t                              fltFreqToAngularSpeedCoeff;
    int32_t                             i32Q16PosRefErr;
    int32_t                             i32PosRef;
    int32_t                             i32PosRef_1;
    acc32_t                             a32SpeedFwd;
    float_t                               fltSpeedFwd;
    float_t                               fltMechToElecCoef;
    int32_t                             i32PosLoopFreq;
} MCSTRUC_POS_CONTROLLER_T;

typedef struct
{
	IIR32_FILTER_T		    			sSpeedFilter;		/* Speed filter */				
    MCSTRUC_SPEED_PI_T                 sSpeedPIController;	/* Speed PI controller  */
	GFLIB_RAMP_T_FLT					sSpeedRampParams; 	/* Speed ramp parameters */
	float_t						    	fltSpeed;			/* Raw Speed */
	float_t					    		fltSpeedFilt;		/* Speed filtered */
	float_t				    			fltSpeedError;		/* Speed error */
	float_t			    				fltSpeedRamp;		/* Required speed (ramp output) */
	float_t		    					fltSpeedReq;		/* Required speed (ramp input) */
	float_t	    						fltSpeedCmd;		/* Speed command (entered by user or master layer) */
    float_t                           fltIqController;
	bool					    		bOpenLoop;			/* Speed control loop is open */
} MCSTRUC_SPEED_CTRL_T;

typedef struct
{
	GMCLIB_2COOR_SINCOS_T_FLT			sAnglePosEl; 				/* Electrical position sin/cos (at the moment of PWM current reading) */
    int16_t						    	i16PolePairs;				/* No. of pole-pairs */
    frac32_t                            f32PosMech;         // Rotor real mechanical position
	frac16_t                            f16PosElec;			// Rotor real electrical position, Q1.15

    IIR32_FILTER_T				    	sSpeedFilter;				/* Speed filter */				

    float_t							    fltSpeedFbk;
	float_t							    fltSpeedFilt;				/* Speed filtered */
} MCSTRUC_POS_SPEED_ENCODER_T;

typedef struct _trajectory_filter
{
	int32_t    i32In;
	int64_t    i64Out;

	int64_t    i64Out_1;
	int64_t    i64M;    // Internal memory
	int64_t    i64M_1;  // Memory of last step
    frac32_t   f32W;    // Oscillation frequency in a 2nd order system
}trajectory_filter_t;

/*! @brief mcs position structure */
typedef struct _mcs_trajectory_a1
{
	int32_t i32Q16PosCmd;      // Desired position
	int32_t i32Q16PosRamp;     // Ramping output of position command
	int32_t i32Q16PosFilt;	   // Filter output to achieve s-curve
	GFLIB_RAMP_T_F32 sPosRamp; // Position ramp, Q16.16
	trajectory_filter_t sTrajFilter; // A second order filter to get s-curve position reference
}mcs_trajectory_t;

typedef struct
{
	MCSTRUC_POS_CONTROLLER_T		sPosController;			// position Kp controller
	GFLIB_RAMP_T_F32					PosRampInc;				// ramp function: desired position value --> position value after ramp
	int32_t							i32PosDesired;			// desired position value
	int32_t							i32PosRamp;				// position value after ramp
	int32_t							i32PosSmooth;			// position value after smooth filter
	int32_t 						i32PError;				// position error
	int16_t							i16FlagSineTest;		// flag indicating to do sine wave testing
	frac32_t						f32SineAmplitude;		// Amplitude of position sine wave
	int16_t							i16FlagPosFilter;	/* flag indicating which filter would be used	*/	
    /* Position command filter */
	mcs_trajectory_t sCurveRef; // Position curve command
} MCSTRUC_POS_CTRL_T;

typedef struct
{
	int16_t  i16ControlFreq;   // control loop frequency in Hz     
	acc32_t  a32DesiredFreq;	 // the expecting frequency of angle increasing between 0~360 degrees in per-unit
	frac32_t f32Delta;		 // the increment of angle in per-unit
	frac32_t f32StartAngle;	 // start angle in per-unit
	frac32_t f32CurAngle;		 // current angle in per-unit
    frac32_t f32PosRamp;        //position ramp every slow loop
} ANGLE_GENERATOR_T;
/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern void MCSTRUC_FocPMSMCurrentCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM);
extern void MCSTRUC_AngleFromEncoder(MCSTRUC_POS_SPEED_ENCODER_T *psAngle);

extern void MCSTRUC_AlignmentPMSM(MCSTRUC_FOC_PMSM_T *psFocPMSM);

extern float MCSTRUC_PosControllerCalc(int32_t w32Err, MCSTRUC_POS_CONTROLLER_T *ptr);
extern void MCSTRUC_SpeedControllerCalc(MCSTRUC_SPEED_CTRL_T *psSpeed);
extern void AngleCalculation(ANGLE_GENERATOR_T *ptr);

extern void 	IIR32FilterClear(IIR32_FILTER_T *ptr);
extern frac32_t 	IIR32FilterCalc(IIR32_FILTER_T *ptr, frac32_t f32Input);
extern int32_t trajectoryFilterUpdate(trajectory_filter_t *this);
extern void trajectoryFilterInit(trajectory_filter_t *this);
#endif /* _MCSTRUC_H_ */
