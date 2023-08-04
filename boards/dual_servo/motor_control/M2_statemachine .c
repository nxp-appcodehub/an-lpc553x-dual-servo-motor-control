/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "M2_statemachine.h"
#include "mcdrv_adc_lpc55s36.h"
#include "mc_periph_init.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define		M2_POSITION_LOOP

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
    CALIB               = 0,
    READY               = 1,
    ALIGN               = 2,
    SPIN	             = 3,
    FREEWHEEL	         = 4
} M2_RUN_SUBSTATE_T;         /* Run sub-states */

/******************************************************************************
* Global variables
******************************************************************************/
/* M2 structure */
MCDEF_FOC_PMSM_ENC_SPEED_PI_T		gsM2_Drive;
MCDEF_ENCODER_COUNTER_T                 gsM2_Enc;

ANGLE_GENERATOR_T			gsM2_AngleGen;

bool                 			mbM2_SwitchAppOnOff;
extern mcdrv_adc_t g_sM2AdcSensor;

volatile float g_fltM2DCBvoltageScale;
volatile float g_fltM2voltageScale;
volatile float g_fltM2currentScale;
volatile float g_fltM2speedScale;
/******************************************************************************
* Local variables
******************************************************************************/
static M2_RUN_SUBSTATE_T		msM2_StateRun;
static		MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T	M2_Channels[8] =
{		{M2_IB_ADC_0, M2_IC_ADC_1},\
		{M2_IB_ADC_0, M2_IC_ADC_1},\
		{M2_IA_ADC_0, M2_IC_ADC_1},\
		{M2_IA_ADC_0, M2_IC_ADC_1},\
		{M2_IA_ADC_0, M2_IB_ADC_1},\
		{M2_IA_ADC_0, M2_IB_ADC_1},\
		{M2_IB_ADC_0, M2_IC_ADC_1},\
		{M2_IB_ADC_0, M2_IC_ADC_1}
};
/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void M2_StateFaultFast(void);
static void M2_StateInitFast(void);
static void M2_StateStopFast(void); 
static void M2_StateRunFast(void);

static void M2_StateFaultSlow(void);
static void M2_StateInitSlow(void);
static void M2_StateStopSlow(void);
static void M2_StateRunSlow(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void M2_TransFaultInit(void);
static void M2_TransInitFault(void);
static void M2_TransInitStop(void);
static void M2_TransStopFault(void);
static void M2_TransStopRun(void);
static void M2_TransRunFault(void);
static void M2_TransRunStop(void);

/* State machine functions field (in pmem) */
static const SM_APP_STATE_FCN_T  s_M2_STATE_FAST = {M2_StateFaultFast, M2_StateInitFast, M2_StateStopFast, M2_StateRunFast};
static const SM_APP_STATE_FCN_T  s_M2_STATE_SLOW = {M2_StateFaultSlow, M2_StateInitSlow, M2_StateStopSlow, M2_StateRunSlow};

/* State-transition functions field (in pmem) */
static const SM_APP_TRANS_FCN_T msTRANS = {M2_TransFaultInit, M2_TransInitFault, M2_TransInitStop, M2_TransStopFault, M2_TransStopRun, M2_TransRunFault, M2_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsM2_Ctrl = 
{
	/* gsM2_Ctrl.psState, User state functions  */
	&s_M2_STATE_FAST,
    
    /* gsM2_Ctrl.psState, User state functions  */
    &s_M2_STATE_SLOW,
 	
 	/* gsM2_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsM2_Ctrl.uiCtrl, Default no control command */
  	SM_CTRL_NONE,
  	
  	/* gsM2_Ctrl.eState, Default state after reset */
  	INIT 	
};


/*------------------------------------
 * User sub-state machine functions
 * ----------------------------------*/
static void M2_StateRunCalibFast(void);
static void M2_StateRunReadyFast(void);
static void M2_StateRunAlignFast(void);
static void M2_StateRunSpinFast(void);
static void M2_StateRunFreewheelFast(void);

static void M2_StateRunCalibSlow(void);
static void M2_StateRunReadySlow(void);
static void M2_StateRunAlignSlow(void);
static void M2_StateRunSpinSlow(void);
static void M2_StateRunFreewheelSlow(void);

/*! @brief Application sub-state function field - fast */
static const PFCN_VOID_VOID mM2_STATE_RUN_TABLE_FAST[5] = {M2_StateRunCalibFast, M2_StateRunReadyFast, M2_StateRunAlignFast, 
                                                M2_StateRunSpinFast,  M2_StateRunFreewheelFast};
/*! @brief Application sub-state function field - slow */
static const PFCN_VOID_VOID mM2_STATE_RUN_TABLE_SLOW[5] = {M2_StateRunCalibSlow, M2_StateRunReadySlow, M2_StateRunAlignSlow,
                                                M2_StateRunSpinSlow,  M2_StateRunFreewheelSlow};

/*------------------------------------
 * User sub-state-transition functions
 * ----------------------------------*/
static void M2_TransRunCalibReady(void);
static void M2_TransRunReadyAlign(void);
static void M2_TransRunReadySpin(void);
static void M2_TransRunAlignSpin(void);
static void M2_TransRunAlignReady(void);
static void M2_TransRunSpinFreewheel(void);
static void M2_TransRunFreewheelReady(void);

static void M2_FaultDetection(void);
static void M2_ADCChannelMapping(mcdrv_adc_t *this);
static void M2_ClearFOCVariables(void);
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateFaultFast(void)
{
    /* Disables PWM outputs */
    M2_DISABLE_PWM_OUTPUT();
    
    /* M2 Current */
    M2_MCDRV_ADC_GET(&g_sM2AdcSensor);

    /* DC bus voltage filter */
    gsM2_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.f16UDcBus, g_fltM2DCBvoltageScale);
    gsM2_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM2_Drive.sFocPMSM.fltUDcBus, &gsM2_Drive.sFocPMSM.sUDcBusFilter);
	
    /* get position and speed from quadrature encoder sensor */
	if(g_sM2EncSensor.bPosAbsoluteFlag == TRUE)
	{
        /* Calculate the position for position loop */
        MCDRV_GetRotorCurrentPos(&g_sM2EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM2EncSensor);
        
        /* Get M2 rotor position */
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
    }
    
    M2_Fault();
    
    /* Disable user application switch */
    mbM2_SwitchAppOnOff = false;
}
static void M2_StateFaultSlow(void)
{
    /* Get speed from ENC M/T method in slow loop */
	MCDRV_EncSpeedCalUpdate(&g_sM2EncSensor);
    
    /* speed measure algorithm */
    gsM2_Drive.sSpeedPos.fltSpeedFbk = g_sM2EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM2_Drive.sSpeedCtrl.fltSpeedFilt = gsM2_Drive.sSpeedPos.fltSpeedFbk;
    
    M2_CLEAR_OVERCURRENT_FAULT();

    MC_FAULT_CLEAR(gsM2_Drive.sFaultIdPending, MC_FAULT_STARTUP_FAIL);
    MC_FAULT_CLEAR(gsM2_Drive.sFaultIdPending, MC_FAULT_LOAD_OVER);
    
    if (!MC_FAULT_ANY(gsM2_Drive.sFaultIdPending))
    {
        if (gsM2_Drive.ui16CounterState == 0)
        {
            /* Clear fault command */
            gsM2_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR; 
        }
        else
        {
            gsM2_Drive.ui16CounterState--;
        }
    }
    else
    {
        gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeFaultRelease;
    }
}
/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateInitFast(void)
{
	mbM2_SwitchAppOnOff	= false;

    MC_FAULT_CLEAR_ALL(gsM2_Drive.sFaultId);

    MC_FAULT_CLEAR_ALL(gsM2_Drive.sFaultIdPending);

    gsM2_Drive.sFaultThresholds.fltUDcBusOver 	    = M2_OVERVOLT_LIMIT;
    gsM2_Drive.sFaultThresholds.fltUDcBusUnder 	= M2_UNDERVOLT_LIMIT;
//	gsM2_Drive.sFaultThresholds.f16SpeedOver 	= FRAC16(SPEED_MAX / SPEED_SCALE);
//	gsM2_Drive.sFaultThresholds.f16SpeedUnder 	= FRAC16(SPEED_MIN / SPEED_SCALE);
	
    /* PMSM FOC params */
    gsM2_Drive.sFocPMSM.sIdPiParams.fltPGain 		= M2_D_KP_GAIN;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltIGain 		= M2_D_KI_GAIN;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltIAccK_1 	        = 0;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltUpperLim 	= M2_D_LIMIT;  
    gsM2_Drive.sFocPMSM.sIdPiParams.fltLowerLim 	= -M2_D_LIMIT;
    gsM2_Drive.sFocPMSM.sIdPiParams.bLimFlag 		= 0;

    gsM2_Drive.sFocPMSM.sIqPiParams.fltPGain 		= M2_Q_KP_GAIN;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltIGain 		= M2_Q_KI_GAIN;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltIAccK_1 	        = 0;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltUpperLim 	= M2_Q_LIMIT;  
    gsM2_Drive.sFocPMSM.sIqPiParams.fltLowerLim 	= -M2_Q_LIMIT;    
    gsM2_Drive.sFocPMSM.sIqPiParams.bLimFlag 		= 0;

    gsM2_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB0    = M2_FILTER_UDCBUS_B0 ;
    gsM2_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB1    = M2_FILTER_UDCBUS_B1;
    gsM2_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltA1    = M2_FILTER_UDCBUS_A1;
    GDFLIB_FilterIIR1Init_FLT(&gsM2_Drive.sFocPMSM.sUDcBusFilter);
    
    gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000; // 0.5
    gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
    gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;

    gsM2_Drive.sFocPMSM.sUDQController.fltD = 0; //  the output of D-current controller
    gsM2_Drive.sFocPMSM.sUDQController.fltQ = 0; //  the output of Q-current controller

    gsM2_Drive.sFocPMSM.sAlignment.f16Position 			= 0;// f32Speed will be added to f32Position every current loop cycle
    gsM2_Drive.sFocPMSM.sAlignment.fltU 				= 0;	// initial D voltage length during alignment
    gsM2_Drive.sFocPMSM.sAlignment.fltId				= 0;	// initial D current length during alignment
    gsM2_Drive.sFocPMSM.sAlignment.f16Speed 			= 0;//M2_ALIGN_SPEED;
    gsM2_Drive.sFocPMSM.sAlignment.fltUStep 			= (M2_ALIGN_VOLT_RAMP / M2_CONTROL_FREQ);// step is added every current loop cycle
    gsM2_Drive.sFocPMSM.sAlignment.fltIdStep 			= (M2_ALIGN_CURRENT_RAMP / M2_CONTROL_FREQ);// step is added every current loop cycle
    gsM2_Drive.sFocPMSM.sAlignment.fltIMax 				= (M2_ALIGN_CURRENT);// maximal D current threshold
    gsM2_Drive.sFocPMSM.sAlignment.fltUMax 				= (M2_ALIGN_VOLT_MAX);// maximal D voltage threshold
    gsM2_Drive.sFocPMSM.sAlignment.ui16TimeAlignment 	= (uint16_t)(M2_DURATION_TASK_ALIGN * (M2_CONTROL_FREQ/M2_SPEED_LOOP_CNTR));

    gsM2_Drive.sFocPMSM.ui16SectorSVM = 4;
    gsM2_Drive.sFocPMSM.fltDutyCycleLimit = M2_DUTY_CYCLE_LIMIT;
    gsM2_Drive.sFocPMSM.fltUDcBus = 0;
    gsM2_Drive.sFocPMSM.fltUDcBusFilt = 0;
    gsM2_Drive.sFocPMSM.bIdPiSatFlag = 0;
    gsM2_Drive.sFocPMSM.bIqPiSatFlag = 0;
    gsM2_Drive.sFocPMSM.bOpenLoop = false;
    gsM2_Drive.sFocPMSM.bUseMaxBus = false;
    gsM2_Drive.sFocPMSM.bUseZc = false;
    
    gsM2_Drive.ui16FlagAlignFinished = 0;

    /* Speed params */
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltPGain = M2_SPEED_PI_PROP_GAIN;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIGain = M2_SPEED_PI_INTEG_GAIN;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIAccK_1 = 0;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltUpperLim = M2_SPEED_LOOP_LIMIT;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltLowerLim = -M2_SPEED_LOOP_LIMIT;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.bLimFlag = 0;

    // 32-bit ramp for the cmd speed, used in slow loop
    gsM2_Drive.sSpeedCtrl.sSpeedRampParams.fltRampUp   	= M2_SPEED_RAMP;
    gsM2_Drive.sSpeedCtrl.sSpeedRampParams.fltRampDown 	= M2_SPEED_RAMP;
    gsM2_Drive.sSpeedCtrl.fltSpeed 						= 0;
    gsM2_Drive.sSpeedCtrl.fltSpeedCmd 					= 0;
    gsM2_Drive.sSpeedCtrl.fltSpeedError 			    	= 0;
    gsM2_Drive.sSpeedCtrl.fltSpeedFilt 					= 0;
    gsM2_Drive.sSpeedCtrl.fltSpeedRamp 					= 0;
    gsM2_Drive.sSpeedCtrl.fltSpeedReq 					= 0;
    gsM2_Drive.sSpeedCtrl.bOpenLoop 					= false;
    
    // Position calculation 
    gsM2_Drive.sSpeedPos.i16PolePairs 				= M2_POLE_PAIRS;
    gsM2_Drive.sSpeedPos.fltSpeedFbk 		    	    = 0;
    
    /* position loop */
    gsM2_Drive.SPosCtrl.i16FlagPosFilter				= 0;
    gsM2_Drive.SPosCtrl.i32PosDesired					= 0; 
    gsM2_Drive.SPosCtrl.i32PosRamp						= 0;
    gsM2_Drive.SPosCtrl.PosRampInc.f32RampUp			= M2_POS_RAMP_UP; 
    gsM2_Drive.SPosCtrl.PosRampInc.f32RampDown		= M2_POS_RAMP_DOWN;
    GFLIB_RampInit_F32(0, &gsM2_Drive.SPosCtrl.PosRampInc);
    gsM2_Drive.SPosCtrl.i16FlagSineTest				= 0;
    gsM2_Drive.SPosCtrl.f32SineAmplitude				= M2_POS_SINE_WAVE_AMPLITUDE;
    
    /* position controller */
    gsM2_Drive.SPosCtrl.sPosController.a32PropGain				= M2_POS_CTRL_PROP_GAIN;
    gsM2_Drive.SPosCtrl.sPosController.fltUpperLimit				= 2.0F*PI*M2_POLE_PAIRS*M2_Pos_Ctrl_AW_Limit/60;
    gsM2_Drive.SPosCtrl.sPosController.fltLowerLimit				= -2.0F*PI*M2_POLE_PAIRS*M2_Pos_Ctrl_AW_Limit/60;
    gsM2_Drive.SPosCtrl.sPosController.fltFreqToAngularSpeedCoeff = (float_t)(2.0*PI*M2_POLE_PAIRS);
    gsM2_Drive.SPosCtrl.sPosController.i32PosLoopFreq            = M2_MC_SLOW_CONTROL_LOOP_FREQ;
    gsM2_Drive.SPosCtrl.sPosController.fltMechToElecCoef         = M2_SPEED_MECH_TO_ElEC_COEFF;
    gsM2_Drive.SPosCtrl.sPosController.a32FwdGain                  =M2_POSITION_CTRL_SPEED_FWD_GAIN;
    gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter.f32W = M2_ENC_TRAJECTORY_FILTER_FREQ_FRAC;

    /* Angle Generator initialization for SineWave Test*/
    gsM2_AngleGen.f32StartAngle 		= 0;  		// simulated angle starts from 0 degree
    gsM2_AngleGen.a32DesiredFreq		= M2_SINE_WAVE_FREQ;//
    gsM2_AngleGen.f32PosRamp           = FRAC32(1.0)/M2_MC_SLOW_CONTROL_LOOP_FREQ;
    gsM2_AngleGen.f32CurAngle 			= gsM2_AngleGen.f32StartAngle; // configure current angle
	
    gsM2_Drive.ui16CounterSlowLoop 			= 1;
    gsM2_Drive.ui16DividerSlowLoop 			= M2_SPEED_LOOP_CNTR;
    gsM2_Drive.ui16CounterState 			    = 0;
    gsM2_Drive.ui16TimeFullSpeedFreeWheel 	= (uint16_t)(M2_DURATION_TASK_FREE_WHEEL * (M2_CONTROL_FREQ/M2_SPEED_LOOP_CNTR));
    gsM2_Drive.ui16TimeCalibration 			= (uint16_t)(M2_DURATION_TASK_CALIB * (M2_CONTROL_FREQ/M2_SPEED_LOOP_CNTR));
    gsM2_Drive.ui16TimeFaultRelease 		    = (uint16_t)(M2_DURATION_TASK_FAULT_RELEASE * (M2_CONTROL_FREQ/M2_SPEED_LOOP_CNTR));

    /* Defined scaling for FreeMASTER*/
    g_fltM2DCBvoltageScale                   = M2_U_DCB_MAX;
    g_fltM2voltageScale                      = M2_U_FOC_MAX;
    g_fltM2currentScale                      = M2_I_MAX;
    g_fltM2speedScale                         = M2_N_MAX;
    
    /* Filter init not to enter to fault */
    gsM2_Drive.sFocPMSM.sUDcBusFilter.fltFltBfrX[0] = (M2_OVERVOLT_LIMIT / 2.0F) + (M2_UNDERVOLT_LIMIT / 2.0F); 
    gsM2_Drive.sFocPMSM.sUDcBusFilter.fltFltBfrY[0] = (M2_OVERVOLT_LIMIT / 2.0F) + (M2_UNDERVOLT_LIMIT / 2.0F);  
    
    /* Init sensors/actuators pointers */
    /* For ADC driver */
    g_sM2AdcSensor.pf16UDcBus     = &(gsM2_Drive.sFocPMSM.f16UDcBus);
    g_sM2AdcSensor.psIABC         = &(gsM2_Drive.sFocPMSM.sIABCFrac);
    g_sM2AdcSensor.pui16SVMSector = &(gsM2_Drive.sFocPMSM.ui16SectorSVM);
    g_sM2AdcSensor.pui16AuxChan   = &(gsM2_Drive.f16AdcAuxSample);
    
    /* For ENC driver */
    g_sM2EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32B0 = M2_ENC_SPEED_FILTER_IIR_B0_FRAC;
    g_sM2EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32B1 = M2_ENC_SPEED_FILTER_IIR_B1_FRAC;
    g_sM2EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32A1 = M2_ENC_SPEED_FILTER_IIR_A1_FRAC;
    
    MCDRV_EncSpeedCalInit(&g_sM2EncSensor);
    MCDRV_EncToSpeedCalInit(&g_sM2EncSensor);
    trajectoryFilterInit(&gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter);

    /* INIT_DONE command */
    gsM2_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}
static void M2_StateInitSlow(void)
{
}
/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateStopFast(void)
{
    /* Disable PWM output */
    M2_DISABLE_PWM_OUTPUT();

    /* M2 Current */
    M2_MCDRV_ADC_GET(&g_sM2AdcSensor);
    
    /* DC bus voltage filter */
    gsM2_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.f16UDcBus, g_fltM2DCBvoltageScale);
    gsM2_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM2_Drive.sFocPMSM.fltUDcBus, &gsM2_Drive.sFocPMSM.sUDcBusFilter);

    /* get position and speed from quadrature encoder sensor */
	if(g_sM2EncSensor.bPosAbsoluteFlag == TRUE)
	{
        MCDRV_GetRotorCurrentPos(&g_sM2EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM2EncSensor);
        /* Calculate the position for position loop */
        
        /* Get M2 rotor position */
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
    }
    
    M2_Fault();
}
static void M2_StateStopSlow(void)
{
    /* Actual position */                 
    MCDRV_GetRotorDeltaRev(&g_sM2EncSensor);
    gsM2_Drive.i32PosRelative = g_sM2EncSensor.i32Q16DeltaRev;
    
    /* Get speed from ENC M/T method in slow loop*/
	MCDRV_EncSpeedCalUpdate(&g_sM2EncSensor);
    
    /* speed measure algorithm */
    gsM2_Drive.sSpeedPos.fltSpeedFbk = g_sM2EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM2_Drive.sSpeedCtrl.fltSpeedFilt = gsM2_Drive.sSpeedPos.fltSpeedFbk;
    
    if (mbM2_SwitchAppOnOff)
    {
            /* Start command */
            gsM2_Ctrl.uiCtrl |= SM_CTRL_START;
    }
}
/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunFast(void)
{
    /* M2 Current */
    M2_MCDRV_ADC_GET(&g_sM2AdcSensor);
    
    /* DC bus voltage filter */
    gsM2_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.f16UDcBus, g_fltM2DCBvoltageScale);
    gsM2_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM2_Drive.sFocPMSM.fltUDcBus, &gsM2_Drive.sFocPMSM.sUDcBusFilter);
    
    /* get position and speed from quadrature encoder sensor */
	if(g_sM2EncSensor.bPosAbsoluteFlag == TRUE)
	{
        MCDRV_GetRotorCurrentPos(&g_sM2EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM2EncSensor);
        /* Calculate the position for position loop */
        
        /* Get M2 rotor position */
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
        gsM2_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM2EncSensor.f16PosElec);
    }
    
    M2_Fault();
    
    /* Convert phase currents from fractional measured values to float */
    gsM2_Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.sIABCFrac.f16A, g_fltM2currentScale);
    gsM2_Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.sIABCFrac.f16B, g_fltM2currentScale);
    gsM2_Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(gsM2_Drive.sFocPMSM.sIABCFrac.f16C, g_fltM2currentScale);
    
	/* Run sub-state function */
	mM2_STATE_RUN_TABLE_FAST[msM2_StateRun]();
    
    /* set current sensor for  sampling */
    M2_ADCChannelMapping(&g_sM2AdcSensor);

	if (!mbM2_SwitchAppOnOff)
	{
		/* Stop command */
		gsM2_Ctrl.uiCtrl |= SM_CTRL_STOP;	
	}
}
static void M2_StateRunSlow(void)
{
    /* Get speed from ENC M/T method in slow loop */
	MCDRV_EncSpeedCalUpdate(&g_sM2EncSensor);
    
    /* speed measure algorithm */
    gsM2_Drive.sSpeedPos.fltSpeedFbk = g_sM2EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM2_Drive.sSpeedCtrl.fltSpeedFilt = gsM2_Drive.sSpeedPos.fltSpeedFbk;

	/* Run sub-state function */
    mM2_STATE_RUN_TABLE_SLOW[msM2_StateRun]();
}
/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransFaultInit(void)
{    

}

/***************************************************************************//*!
*
* @brief   INIT to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransInitFault(void)
{
	M2_DISABLE_PWM_OUTPUT();

	gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeFaultRelease;	
}

/***************************************************************************//*!
*
* @brief   INIT to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransInitStop(void)
{

}

/***************************************************************************//*!
*
* @brief   STOP to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransStopFault(void)
{
	/* Disables PWM outputs */
	M2_DISABLE_PWM_OUTPUT();
	
	gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeFaultRelease;
}

/***************************************************************************//*!
*
* @brief   STOP to RUN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransStopRun(void)
{
	gsM2_Drive.SPosCtrl.i32PosDesired = 0;
	
	gsM2_Drive.sFocPMSM.ui16SectorSVM = 4;
  
    gsM2_Drive.ui16FlagAlignFinished = 0;

	/* 50% duty cycle */
	gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
	gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
	gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;
	
	/* PWM update */
	M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
	
	/* Enable PWM output */
	M2_ENABLE_PWM_OUTPUT();

	/* Required time for ADC calibration */
	gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeCalibration;

	/* Init sub-state when transition to RUN */
	msM2_StateRun = CALIB;

	/* Acknowledge that the system can proceed into the RUN state */
	gsM2_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/***************************************************************************//*!
*
* @brief   RUN to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunFault(void)
{
	/* Disables PWM outputs */
	M2_DISABLE_PWM_OUTPUT();

	gsM2_Drive.sFocPMSM.sIABC.fltA = 0;
	gsM2_Drive.sFocPMSM.sIABC.fltB = 0;
	gsM2_Drive.sFocPMSM.sIABC.fltC = 0;
				
	gsM2_Drive.sFocPMSM.sIDQReq.fltD = 0;
	gsM2_Drive.sFocPMSM.sIDQReq.fltQ = 0;
	gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
	gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

	gsM2_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
	gsM2_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
	gsM2_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
	gsM2_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	

	gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeFaultRelease;
}

/***************************************************************************//*!
*
* @brief   RUN to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunStop(void)
{
	gsM2_Drive.sSpeedCtrl.fltSpeedCmd = 0;
	
    switch(msM2_StateRun)
    {
        case (CALIB):
		{
			/* Acknowledge that the system can proceed into the STOP state */
			gsM2_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
		}
        case (READY):
        {
			/* Acknowledge that the system can proceed into the STOP state */
			gsM2_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
        }        
        case (ALIGN):
        {
			/* Disables PWM outputs */
			M2_DISABLE_PWM_OUTPUT();

			gsM2_Drive.sFocPMSM.sIABC.fltA = 0;
			gsM2_Drive.sFocPMSM.sIABC.fltB = 0;
			gsM2_Drive.sFocPMSM.sIABC.fltC = 0;

			gsM2_Drive.sFocPMSM.sIDQReq.fltD = 0;
			gsM2_Drive.sFocPMSM.sIDQReq.fltQ = 0;
			gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
			gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

			gsM2_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
			gsM2_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
			gsM2_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
			gsM2_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	

			/* 50% duty cycle */
			gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
			gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
			gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

			/* PWM update */
			M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);

			/* Acknowledge that the system can proceed into the STOP state */
			gsM2_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
        }

        case (SPIN):
		{
			M2_TransRunSpinFreewheel();
			
			break;
		}

        case (FREEWHEEL):
		{
			if (gsM2_Drive.ui16CounterState == 0)
			{
				/* Disable PWM output */
				M2_DISABLE_PWM_OUTPUT();

				gsM2_Drive.sFocPMSM.sIABC.fltA = 0;
				gsM2_Drive.sFocPMSM.sIABC.fltB = 0;
				gsM2_Drive.sFocPMSM.sIABC.fltC = 0;
							
				gsM2_Drive.sFocPMSM.sIDQReq.fltD = 0;
				gsM2_Drive.sFocPMSM.sIDQReq.fltQ = 0;
				gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
				gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

				gsM2_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
				gsM2_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
				gsM2_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
				gsM2_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	
				
				/* Acknowledge that the system can proceed into the STOP state */
				gsM2_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

				break;
			}
		}
    }
}

/***************************************************************************//*!
*
* @brief   RUN CALIB sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunCalibFast(void)
{	
    MCDRV_Curr3Ph2ShCalib(&g_sM2AdcSensor);
	
    /* change SVM sector in range <1;6> to measure all AD channel mapping combinations */
    if (++gsM2_Drive.sFocPMSM.ui16SectorSVM > 6)
        gsM2_Drive.sFocPMSM.ui16SectorSVM = 1;

    /* 50% duty cycle */
    gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
}
static void M2_StateRunCalibSlow(void)
{
    if (--gsM2_Drive.ui16CounterState == 0)
    {
        /* Offset calculation */
        MCDRV_Curr3Ph2ShCalibSet(&g_sM2AdcSensor);
        /* Transition to the RUN READY sub-state */
        M2_TransRunCalibReady();	
    }
}
/***************************************************************************//*!
*
* @brief   RUN READY sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunReadyFast(void)
{
    gsM2_Drive.sFocPMSM.sIDQReq.fltD = 0;
    gsM2_Drive.sFocPMSM.sIDQReq.fltQ = 0;
    gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
    gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;
    gsM2_Drive.sFocPMSM.sIDQ.fltD = 0;
    gsM2_Drive.sFocPMSM.sIDQ.fltQ = 0;

    gsM2_Drive.sSpeedCtrl.fltSpeedReq = 0;
    gsM2_Drive.sSpeedCtrl.fltSpeed = 0;
    
    gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000; // 0.5
    gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
}
static void M2_StateRunReadySlow(void)
{	
    if(g_sM2EncSensor.bPosAbsoluteFlag == TRUE)
    {
      /* Transition to the RUN SPIN sub-state */
        M2_TransRunReadySpin();
    }
    else
    {
      /* Transition to the RUN ALIGN sub-state */
        M2_TransRunReadyAlign();
    }
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunAlignFast(void)
{
	MCSTRUC_AlignmentPMSM(&gsM2_Drive.sFocPMSM);
    
    /* PWM update */
    M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
}
static void M2_StateRunAlignSlow(void)
{
    if(--gsM2_Drive.ui16CounterState == 0)
    {
        /* Transition to the RUN SPIN sub-state */
        M2_TransRunAlignSpin();
    }
}
/***************************************************************************//*!
*
* @brief   RUN SPIN sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunSpinFast(void)
{
    /* Position for FOC */
    gsM2_Drive.sFocPMSM.sAnglePosEl = gsM2_Drive.sSpeedPos.sAnglePosEl;
    
    /* FOC */
    MCSTRUC_FocPMSMCurrentCtrl(&gsM2_Drive.sFocPMSM);

    /* PWM update */
    M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
}
static void M2_StateRunSpinSlow(void)
{
    /* Actual position */                 
    MCDRV_GetRotorDeltaRev(&g_sM2EncSensor);
    gsM2_Drive.i32PosRelative = g_sM2EncSensor.i32Q16DeltaRev;
#ifdef	M2_POSITION_LOOP
    /* generate sine signal when sine wave testing in position loop */
    if (1 == gsM2_Drive.SPosCtrl.i16FlagSineTest)
    {
        AngleCalculation(&gsM2_AngleGen);
        gsM2_Drive.SPosCtrl.i32PosDesired = MLIB_Mul_F32(gsM2_Drive.SPosCtrl.f32SineAmplitude, MLIB_Conv_F32s(GFLIB_Sin_F16(MLIB_Conv_F16l(gsM2_AngleGen.f32CurAngle))));
    }
    /* choice one kind of filter group types and calculate position error */
    switch(gsM2_Drive.SPosCtrl.i16FlagPosFilter)
    {
    case 0:
    default:
            /* i32PosDesired --> i32PosRamp --> i32PosSmooth --> i32PError */
            gsM2_Drive.SPosCtrl.i32PosRamp = GFLIB_Ramp_F32(gsM2_Drive.SPosCtrl.i32PosDesired, &gsM2_Drive.SPosCtrl.PosRampInc);
            gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter.i32In = gsM2_Drive.SPosCtrl.i32PosRamp;
            gsM2_Drive.SPosCtrl.i32PosSmooth = trajectoryFilterUpdate(&gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter);
            gsM2_Drive.SPosCtrl.i32PError =  gsM2_Drive.SPosCtrl.i32PosSmooth - gsM2_Drive.i32PosRelative;
            gsM2_Drive.SPosCtrl.sPosController.i32PosRef = gsM2_Drive.SPosCtrl.i32PosSmooth;
            break;
            
    case 1:
            /* i32PosDesired --> i32PosSmooth --> i32PError */
            gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter.i32In = gsM2_Drive.SPosCtrl.i32PosDesired;
            gsM2_Drive.SPosCtrl.i32PosSmooth = trajectoryFilterUpdate(&gsM2_Drive.SPosCtrl.sCurveRef.sTrajFilter);
            gsM2_Drive.SPosCtrl.i32PError =  gsM2_Drive.SPosCtrl.i32PosSmooth - gsM2_Drive.i32PosRelative;
            gsM2_Drive.SPosCtrl.sPosController.i32PosRef = gsM2_Drive.SPosCtrl.i32PosSmooth;
            break;
            
    case 2:
            /* i32PosDesired --> i32PosRamp --> i32PError */
            gsM2_Drive.SPosCtrl.i32PosRamp = GFLIB_Ramp_F32(gsM2_Drive.SPosCtrl.i32PosDesired, &gsM2_Drive.SPosCtrl.PosRampInc);
            gsM2_Drive.SPosCtrl.i32PError =  gsM2_Drive.SPosCtrl.i32PosRamp - gsM2_Drive.i32PosRelative;
            gsM2_Drive.SPosCtrl.sPosController.i32PosRef = gsM2_Drive.SPosCtrl.i32PosRamp;
            break;
            
    case 3:
            /* i32PosDesired --> i32PError */
            gsM2_Drive.SPosCtrl.i32PError =  gsM2_Drive.SPosCtrl.i32PosDesired - gsM2_Drive.i32PosRelative;
            gsM2_Drive.SPosCtrl.sPosController.i32PosRef = gsM2_Drive.SPosCtrl.i32PosDesired;
            break;
    }
    
    /* position controller */
    gsM2_Drive.sSpeedCtrl.fltSpeedCmd = MCSTRUC_PosControllerCalc(gsM2_Drive.SPosCtrl.i32PError, &gsM2_Drive.SPosCtrl.sPosController);
#endif

    if (!gsM2_Drive.sFocPMSM.bOpenLoop)
    {
      /* Desired current by the speed PI controller */
      MCSTRUC_SpeedControllerCalc(&gsM2_Drive.sSpeedCtrl);
      gsM2_Drive.sFocPMSM.sIDQReq.fltQ = gsM2_Drive.sSpeedCtrl.fltIqController;
    }
}
/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_StateRunFreewheelFast(void)
{
    /* Disable PWM output */
    M2_DISABLE_PWM_OUTPUT();

    gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
}
static void M2_StateRunFreewheelSlow(void)
{
    if (--gsM2_Drive.ui16CounterState == 0)
    {
        /* If app switch is on */
        if (mbM2_SwitchAppOnOff)
        {
			 gsM2_Drive.sSpeedCtrl.fltSpeed = 0;
            gsM2_Drive.sSpeedCtrl.fltSpeedFilt = 0;
            
            /* If speed command is non-zero */
            if (MLIB_Abs_FLT(gsM2_Drive.sSpeedCtrl.fltSpeedCmd) > 0)
            {
                    /* If the motor has been in the run phase */
                            /* Sub-state RUN READY */
                            M2_TransRunFreewheelReady();
            }
            /* If speed command is zero */
            else
            {
                    /* Sub-state RUN READY */
                    M2_TransRunFreewheelReady();	
            }
        }
    }
}

/***************************************************************************//*!
*
* @brief   RUN CALIB to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunCalibReady(void)
{
	/* Sub-state RUN READY */
	msM2_StateRun = READY;
}

/***************************************************************************//*!
*
* @brief   RUN READY to ALIGN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunReadyAlign(void)
{
	gsM2_Drive.sFocPMSM.sAlignment.f16Speed = MLIB_Abs_F16(gsM2_Drive.sFocPMSM.sAlignment.f16Speed);
	
	if (gsM2_Drive.sSpeedCtrl.fltSpeedCmd < 0)
	{
		gsM2_Drive.sFocPMSM.sAlignment.f16Speed = MLIB_Neg_F16(gsM2_Drive.sFocPMSM.sAlignment.f16Speed);
	}
	
	/* Alignment voltage init point */
	gsM2_Drive.sFocPMSM.sAlignment.fltU = 0;
	
	/* Alignment current init point */
	gsM2_Drive.sFocPMSM.sAlignment.fltId = 0;
	gsM2_Drive.sFocPMSM.sAlignment.fltId = 0;
	
	/* Alignment voltage init position */
	gsM2_Drive.sFocPMSM.sAlignment.f16Position = 0;
	/* Alignment duration set-up */
	gsM2_Drive.ui16CounterState = gsM2_Drive.sFocPMSM.sAlignment.ui16TimeAlignment;
    
    /* Enable PWM output */
    M2_ENABLE_PWM_OUTPUT();
    
	/* Sub-state RUN ALIGN */
	msM2_StateRun = ALIGN;
}

/***************************************************************************//*!
*
* @brief   RUN READY to SPIN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunReadySpin(void)
{
    /* PMSM FOC params */
    gsM2_Drive.sFocPMSM.bIdPiSatFlag 					= 0;
    gsM2_Drive.sFocPMSM.bIqPiSatFlag 					= 0;	
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;

	/* Speed params */
	gsM2_Drive.sSpeedCtrl.fltSpeedRamp 						= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeedReq 				    		= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeedError 			    		= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeed 							= 0;
	
	IIR32FilterClear(&gsM2_Drive.sSpeedCtrl.sSpeedFilter); 
	gsM2_Drive.sSpeedCtrl.fltSpeedFilt 						= 0;
	
	/* Position loop */
    GFLIB_RampInit_F32(0, &gsM2_Drive.SPosCtrl.PosRampInc);
	gsM2_Drive.SPosCtrl.i32PosRamp						= 0;
	gsM2_Drive.SPosCtrl.i32PosSmooth 					= 0;
	gsM2_Drive.SPosCtrl.i32PError						= 0;

    /* Clear all FOC variables */
    M2_ClearFOCVariables();

	/* PWM update */
	M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
	
	/* limit the PI controller of Id and Iq */
	gsM2_Drive.sFocPMSM.bUseMaxBus = true;
	
	gsM2_Drive.ui16CounterSlowLoop = 1;
	
	/* Sub-state RUN SPIN */
	msM2_StateRun = SPIN;
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN to SPIN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunAlignSpin(void)
{
    /* initialize encoder driver */
	MCDRV_GetRotorInitPos(&g_sM2EncSensor, MLIB_Conv_F32s(gsM2_Drive.sFocPMSM.sAlignment.f16Position));

    /* Ensure the feedback position is reset to zero whenever state machine goes to spin */
    MCDRV_GetRotorInitRev(&g_sM2EncSensor);
    
	/* PMSM FOC params */
    gsM2_Drive.sFocPMSM.sIdPiParams.fltPGain 		= M2_D_KP_GAIN;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltIGain 		= M2_D_KI_GAIN;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltIAccK_1 	        = 0;
    gsM2_Drive.sFocPMSM.sIdPiParams.fltUpperLim 	= M2_D_LIMIT;  
    gsM2_Drive.sFocPMSM.sIdPiParams.fltLowerLim 	= -M2_D_LIMIT;
    gsM2_Drive.sFocPMSM.sIdPiParams.bLimFlag 		= 0;

    gsM2_Drive.sFocPMSM.sIqPiParams.fltPGain 		= M2_Q_KP_GAIN;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltIGain 		= M2_Q_KI_GAIN;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltIAccK_1 	        = 0;
    gsM2_Drive.sFocPMSM.sIqPiParams.fltUpperLim 	= M2_Q_LIMIT;  
    gsM2_Drive.sFocPMSM.sIqPiParams.fltLowerLim 	= -M2_Q_LIMIT;    
    gsM2_Drive.sFocPMSM.sIqPiParams.bLimFlag 		= 0;
  
    gsM2_Drive.sFocPMSM.bIdPiSatFlag 					= 0;
    gsM2_Drive.sFocPMSM.bIqPiSatFlag 					= 0;	

	/* Speed params */
	gsM2_Drive.sSpeedCtrl.fltSpeedRamp 						= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeedReq 						= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeedError 					= 0;
	gsM2_Drive.sSpeedCtrl.fltSpeed 							= 0;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;
	
	IIR32FilterClear(&gsM2_Drive.sSpeedCtrl.sSpeedFilter); 
	gsM2_Drive.sSpeedCtrl.fltSpeedFilt 						= 0;

    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltPGain = M2_SPEED_PI_PROP_GAIN;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIGain = M2_SPEED_PI_INTEG_GAIN;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIAccK_1 = 0;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltUpperLim = M2_SPEED_LOOP_LIMIT;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltLowerLim = -M2_SPEED_LOOP_LIMIT;
    gsM2_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.bLimFlag = 0;
	
	/* Position loop */
    GFLIB_RampInit_F32(0, &gsM2_Drive.SPosCtrl.PosRampInc);
	gsM2_Drive.SPosCtrl.i32PosRamp						= 0;
	gsM2_Drive.SPosCtrl.i32PosSmooth 					= 0;
	gsM2_Drive.SPosCtrl.i32PError						= 0;

    /* Clear all FOC variables */
    M2_ClearFOCVariables();

	/* PWM update */
	M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
	
	/* limit the PI controller of Id and Iq */
	gsM2_Drive.sFocPMSM.bUseMaxBus = true;
	
	gsM2_Drive.ui16CounterSlowLoop = 1;
	
	/* Sub-state RUN SPIN */
	msM2_StateRun = SPIN;
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunAlignReady(void)
{
    /* Clear all FOC variables */
	M2_ClearFOCVariables();

	/* PWM update */
	M2_PWM_UPDATE(&gsM2_Drive.sFocPMSM.sDutyABC);
	
	/* Sub-state RUN READY */
	msM2_StateRun = READY;
}

/***************************************************************************//*!
*
* @brief   RUN SPIN to FREEWHEEL transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunSpinFreewheel(void)
{
	/* Generates a time gap before the alignment to assure the rotor is not rotating */
    gsM2_Drive.ui16CounterState = gsM2_Drive.ui16TimeFullSpeedFreeWheel;	
	
    /* Clear all FOC variables */
    M2_ClearFOCVariables();
    
	/* Sub-state RUN FREEWHEEL */
	msM2_StateRun = FREEWHEEL;	
}

/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_TransRunFreewheelReady(void)
{
      /* 50% duty cycle */
    gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

	/* Sub-state RUN READY */
	msM2_StateRun = READY;
}

/***************************************************************************//*!
*
* @brief   Clear FOc variables in global variable
*
* @param   void No input parameter
*
* @return  none
*
******************************************************************************/
static void M2_ClearFOCVariables(void)
{
    gsM2_Drive.sFocPMSM.sIDQReq.fltD = 0;
	gsM2_Drive.sFocPMSM.sIDQReq.fltQ = 0;

	gsM2_Drive.sFocPMSM.sUDQReq.fltD = 0;
	gsM2_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

	gsM2_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
	gsM2_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
	gsM2_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
	gsM2_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	
    
    /* 50% duty cycle */
	gsM2_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
	gsM2_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
	gsM2_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;
}
/***************************************************************************//*!
*
* @brief   Fault Detection function
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void M2_FaultDetection(void)
{
    //-----------------------------
    // Actual Faults
    //-----------------------------

    // Fault:   DC-bus over-voltage
	if (gsM2_Drive.sFocPMSM.fltUDcBusFilt >= gsM2_Drive.sFaultThresholds.fltUDcBusOver)
	{
		MC_FAULT_SET(gsM2_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_OVER);
	}
	else
	{
		MC_FAULT_CLEAR(gsM2_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_OVER);
	}

    // Fault:   DC-bus over-current
    if (M2_OVERCURRENT_FAULT())
    {
//		MC_FAULT_SET(gsM2_Drive.sFaultIdPending, MC_FAULT_I_DCBUS_OVER);    	
    }
    else
    {
		MC_FAULT_CLEAR(gsM2_Drive.sFaultIdPending, MC_FAULT_I_DCBUS_OVER);
    }
    
    // Fault:   DC-bus under-voltage
    if (gsM2_Drive.sFocPMSM.fltUDcBusFilt <= gsM2_Drive.sFaultThresholds.fltUDcBusUnder)
    {
		MC_FAULT_SET(gsM2_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_UNDER);
    }
    else
    {
		MC_FAULT_CLEAR(gsM2_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_UNDER);
    }	
    
    gsM2_Drive.sFaultId = gsM2_Drive.sFaultIdPending;     
}
/***************************************************************************//*!
*
* @brief   Set the app switch function
*
* @param   void
*
* @return  none
*
******************************************************************************/
void M2_SetAppSwitch(bool bValue)
{
	mbM2_SwitchAppOnOff = bValue;
}

/***************************************************************************//*!
*
* @brief   Read the app switch function
*
* @param   void
*
* @return  none
*
******************************************************************************/
bool M2_GetAppSwitch(void)
{
	return (mbM2_SwitchAppOnOff);
}

/***************************************************************************//*!
*
* @brief   Returns if in run state
*
* @param   void
*
* @return  none
*
******************************************************************************/
bool M2_IsRunning(void)
{
	return (gsM2_Ctrl.eState == RUN);
}

/***************************************************************************//*!
*
* @brief   Set the speed command function
*
* @param   void
*
* @return  none
*
******************************************************************************/
void M2_SetSpeed(float fltSpeedCmd)
{
	if (mbM2_SwitchAppOnOff)
	{
		if (fltSpeedCmd < gsM2_Drive.sFaultThresholds.fltSpeedUnder)
		{
			gsM2_Drive.sSpeedCtrl.fltSpeedCmd = 0;
		}
		else if (fltSpeedCmd > gsM2_Drive.sFaultThresholds.fltSpeedOver)
		{
			gsM2_Drive.sSpeedCtrl.fltSpeedCmd = 0;
		}
		else
		{
			gsM2_Drive.sSpeedCtrl.fltSpeedCmd = fltSpeedCmd;	
		}
	}
	else
	{
		gsM2_Drive.sSpeedCtrl.fltSpeedCmd = 0;
	}
}

/***************************************************************************//*!
*
* @brief   Fault function
*
* @param   void
*
* @return  none
*
******************************************************************************/
void M2_Fault(void)
{
	M2_FaultDetection();

	if (MC_FAULT_ANY(gsM2_Drive.sFaultId))
	{
		/* Fault state */
		gsM2_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}
}
/***************************************************************************//*!
*
* @brief   Update ADC sample channel when SVM sector is updated.
*
* @param   this   Pointer to the current object
*
* @return  none
*
******************************************************************************/
static void M2_ADCChannelMapping(mcdrv_adc_t *this)
{
    //set A side channel number
    this->pToAdcBase->CMD[0].CMDL &= ~ADC_CMDL_ADCH_MASK;
    this->pToAdcBase->CMD[0].CMDL |= ADC_CMDL_ADCH(M2_Channels[*this->pui16SVMSector].ui16Ph0);
    //set B side channel number
    this->pToAdcBase->CMD[0].CMDL &= ~ADC_CMDL_ALTB_ADCH_MASK;
    this->pToAdcBase->CMD[0].CMDL |= ADC_CMDL_ALTB_ADCH(M2_Channels[*this->pui16SVMSector].ui16Ph1);
}
/******************************************************************************
* Inline functions
******************************************************************************/
