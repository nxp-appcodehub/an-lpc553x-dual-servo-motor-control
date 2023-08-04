/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "M1_statemachine.h"
#include "mcdrv_adc_lpc55s36.h"
#include "mc_periph_init.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define		M1_POSITION_LOOP

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
    CALIB               = 0,
    READY               = 1,
    ALIGN               = 2,
    SPIN	             = 3,
    FREEWHEEL	         = 4
} M1_RUN_SUBSTATE_T;         /* Run sub-states */

/******************************************************************************
* Global variables
******************************************************************************/
/* M1 structure */
MCDEF_FOC_PMSM_ENC_SPEED_PI_T		gsM1_Drive;
MCDEF_ENCODER_COUNTER_T                 gsM1_Enc;

ANGLE_GENERATOR_T			gsM1_AngleGen;

bool                 			mbM1_SwitchAppOnOff;
extern mcdrv_adc_t g_sM1AdcSensor;

volatile float g_fltM1DCBvoltageScale;
volatile float g_fltM1voltageScale;
volatile float g_fltM1currentScale;
volatile float g_fltM1speedScale;
/******************************************************************************
* Local variables
******************************************************************************/
static M1_RUN_SUBSTATE_T		msM1_StateRun;
static		MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T	M1_Channels[8] =
{		{M1_IB_ADC_0, M1_IC_ADC_1},\
		{M1_IB_ADC_0, M1_IC_ADC_1},\
		{M1_IA_ADC_0, M1_IC_ADC_1},\
		{M1_IA_ADC_0, M1_IC_ADC_1},\
		{M1_IA_ADC_0, M1_IB_ADC_1},\
		{M1_IA_ADC_0, M1_IB_ADC_1},\
		{M1_IB_ADC_0, M1_IC_ADC_1},\
		{M1_IB_ADC_0, M1_IC_ADC_1}
};
/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void M1_StateFaultFast(void);
static void M1_StateInitFast(void);
static void M1_StateStopFast(void); 
static void M1_StateRunFast(void);

static void M1_StateFaultSlow(void);
static void M1_StateInitSlow(void);
static void M1_StateStopSlow(void);
static void M1_StateRunSlow(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void M1_TransFaultInit(void);
static void M1_TransInitFault(void);
static void M1_TransInitStop(void);
static void M1_TransStopFault(void);
static void M1_TransStopRun(void);
static void M1_TransRunFault(void);
static void M1_TransRunStop(void);

/* State machine functions field (in pmem) */
static const SM_APP_STATE_FCN_T  s_M1_STATE_FAST = {M1_StateFaultFast, M1_StateInitFast, M1_StateStopFast, M1_StateRunFast};
static const SM_APP_STATE_FCN_T  s_M1_STATE_SLOW = {M1_StateFaultSlow, M1_StateInitSlow, M1_StateStopSlow, M1_StateRunSlow};

/* State-transition functions field (in pmem) */
static const SM_APP_TRANS_FCN_T msTRANS = {M1_TransFaultInit, M1_TransInitFault, M1_TransInitStop, M1_TransStopFault, M1_TransStopRun, M1_TransRunFault, M1_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsM1_Ctrl = 
{
	/* gsM1_Ctrl.psState, User state functions  */
	&s_M1_STATE_FAST,
    
    /* gsM1_Ctrl.psState, User state functions  */
    &s_M1_STATE_SLOW,
 	
 	/* gsM1_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsM1_Ctrl.uiCtrl, Default no control command */
  	SM_CTRL_NONE,
  	
  	/* gsM1_Ctrl.eState, Default state after reset */
  	INIT 	
};


/*------------------------------------
 * User sub-state machine functions
 * ----------------------------------*/
static void M1_StateRunCalibFast(void);
static void M1_StateRunReadyFast(void);
static void M1_StateRunAlignFast(void);
static void M1_StateRunSpinFast(void);
static void M1_StateRunFreewheelFast(void);

static void M1_StateRunCalibSlow(void);
static void M1_StateRunReadySlow(void);
static void M1_StateRunAlignSlow(void);
static void M1_StateRunSpinSlow(void);
static void M1_StateRunFreewheelSlow(void);

/*! @brief Application sub-state function field - fast */
static const PFCN_VOID_VOID mM1_STATE_RUN_TABLE_FAST[5] = {M1_StateRunCalibFast, M1_StateRunReadyFast, M1_StateRunAlignFast, 
                                                M1_StateRunSpinFast,  M1_StateRunFreewheelFast};
/*! @brief Application sub-state function field - slow */
static const PFCN_VOID_VOID mM1_STATE_RUN_TABLE_SLOW[5] = {M1_StateRunCalibSlow, M1_StateRunReadySlow, M1_StateRunAlignSlow,
                                                M1_StateRunSpinSlow,  M1_StateRunFreewheelSlow};

/*------------------------------------
 * User sub-state-transition functions
 * ----------------------------------*/
static void M1_TransRunCalibReady(void);
static void M1_TransRunReadyAlign(void);
static void M1_TransRunReadySpin(void);
static void M1_TransRunAlignSpin(void);
static void M1_TransRunAlignReady(void);
static void M1_TransRunSpinFreewheel(void);
static void M1_TransRunFreewheelReady(void);

static void M1_FaultDetection(void);
static void M1_ADCChannelMapping(mcdrv_adc_t *this);
static void M1_ClearFOCVariables(void);
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
static void M1_StateFaultFast(void)
{
    /* Disables PWM outputs */
    M1_DISABLE_PWM_OUTPUT();
    
    /* M1 Current */
    M1_MCDRV_ADC_GET(&g_sM1AdcSensor);

    /* DC bus voltage filter */
    gsM1_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
    gsM1_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM1_Drive.sFocPMSM.fltUDcBus, &gsM1_Drive.sFocPMSM.sUDcBusFilter);
	
    /* get position and speed from quadrature encoder sensor */
	if(g_sM1EncSensor.bPosAbsoluteFlag == TRUE)
	{
        /* Calculate the position for position loop */
        MCDRV_GetRotorCurrentPos(&g_sM1EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM1EncSensor);
        
        /* Get M1 rotor position */
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
    }
    
    M1_Fault();
    
    /* Disable user application switch */
    mbM1_SwitchAppOnOff = false;
}
static void M1_StateFaultSlow(void)
{
    /* Get speed from ENC M/T method in slow loop */
	MCDRV_EncSpeedCalUpdate(&g_sM1EncSensor);
    
    /* speed measure algorithm */
    gsM1_Drive.sSpeedPos.fltSpeedFbk = g_sM1EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM1_Drive.sSpeedCtrl.fltSpeedFilt = gsM1_Drive.sSpeedPos.fltSpeedFbk;
    
    M1_CLEAR_OVERCURRENT_FAULT();

    MC_FAULT_CLEAR(gsM1_Drive.sFaultIdPending, MC_FAULT_STARTUP_FAIL);
    MC_FAULT_CLEAR(gsM1_Drive.sFaultIdPending, MC_FAULT_LOAD_OVER);
    
    if (!MC_FAULT_ANY(gsM1_Drive.sFaultIdPending))
    {
        if (gsM1_Drive.ui16CounterState == 0)
        {
            /* Clear fault command */
            gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR; 
        }
        else
        {
            gsM1_Drive.ui16CounterState--;
        }
    }
    else
    {
        gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeFaultRelease;
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
static void M1_StateInitFast(void)
{
	mbM1_SwitchAppOnOff	= false;

    MC_FAULT_CLEAR_ALL(gsM1_Drive.sFaultId);

    MC_FAULT_CLEAR_ALL(gsM1_Drive.sFaultIdPending);

    gsM1_Drive.sFaultThresholds.fltUDcBusOver 	    = M1_OVERVOLT_LIMIT;
    gsM1_Drive.sFaultThresholds.fltUDcBusUnder 	= M1_UNDERVOLT_LIMIT;
//	gsM1_Drive.sFaultThresholds.f16SpeedOver 	= FRAC16(SPEED_MAX / SPEED_SCALE);
//	gsM1_Drive.sFaultThresholds.f16SpeedUnder 	= FRAC16(SPEED_MIN / SPEED_SCALE);
	
    /* PMSM FOC params */
    gsM1_Drive.sFocPMSM.sIdPiParams.fltPGain 		= M1_D_KP_GAIN;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltIGain 		= M1_D_KI_GAIN;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltIAccK_1 	        = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltUpperLim 	= M1_D_LIMIT;  
    gsM1_Drive.sFocPMSM.sIdPiParams.fltLowerLim 	= -M1_D_LIMIT;
    gsM1_Drive.sFocPMSM.sIdPiParams.bLimFlag 		= 0;

    gsM1_Drive.sFocPMSM.sIqPiParams.fltPGain 		= M1_Q_KP_GAIN;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltIGain 		= M1_Q_KI_GAIN;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltIAccK_1 	        = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltUpperLim 	= M1_Q_LIMIT;  
    gsM1_Drive.sFocPMSM.sIqPiParams.fltLowerLim 	= -M1_Q_LIMIT;    
    gsM1_Drive.sFocPMSM.sIqPiParams.bLimFlag 		= 0;

    gsM1_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB0    = M1_FILTER_UDCBUS_B0 ;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB1    = M1_FILTER_UDCBUS_B1;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltA1    = M1_FILTER_UDCBUS_A1;
    GDFLIB_FilterIIR1Init_FLT(&gsM1_Drive.sFocPMSM.sUDcBusFilter);
    
    gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000; // 0.5
    gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
    gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;

    gsM1_Drive.sFocPMSM.sUDQController.fltD = 0; //  the output of D-current controller
    gsM1_Drive.sFocPMSM.sUDQController.fltQ = 0; //  the output of Q-current controller

    gsM1_Drive.sFocPMSM.sAlignment.f16Position 			= 0;// f32Speed will be added to f32Position every current loop cycle
    gsM1_Drive.sFocPMSM.sAlignment.fltU 				= 0;	// initial D voltage length during alignment
    gsM1_Drive.sFocPMSM.sAlignment.fltId				= 0;	// initial D current length during alignment
    gsM1_Drive.sFocPMSM.sAlignment.f16Speed 			= 0;//M1_ALIGN_SPEED;
    gsM1_Drive.sFocPMSM.sAlignment.fltUStep 			= (M1_ALIGN_VOLT_RAMP / M1_CONTROL_FREQ);// step is added every current loop cycle
    gsM1_Drive.sFocPMSM.sAlignment.fltIdStep 			= (M1_ALIGN_CURRENT_RAMP / M1_CONTROL_FREQ);// step is added every current loop cycle
    gsM1_Drive.sFocPMSM.sAlignment.fltIMax 				= (M1_ALIGN_CURRENT);// maximal D current threshold
    gsM1_Drive.sFocPMSM.sAlignment.fltUMax 				= (M1_ALIGN_VOLT_MAX);// maximal D voltage threshold
    gsM1_Drive.sFocPMSM.sAlignment.ui16TimeAlignment 	= (uint16_t)(M1_DURATION_TASK_ALIGN * (M1_CONTROL_FREQ/M1_SPEED_LOOP_CNTR));

    gsM1_Drive.sFocPMSM.ui16SectorSVM = 4;
    gsM1_Drive.sFocPMSM.fltDutyCycleLimit = M1_DUTY_CYCLE_LIMIT;
    gsM1_Drive.sFocPMSM.fltUDcBus = 0;
    gsM1_Drive.sFocPMSM.fltUDcBusFilt = 0;
    gsM1_Drive.sFocPMSM.bIdPiSatFlag = 0;
    gsM1_Drive.sFocPMSM.bIqPiSatFlag = 0;
    gsM1_Drive.sFocPMSM.bOpenLoop = false;
    gsM1_Drive.sFocPMSM.bUseMaxBus = false;
    gsM1_Drive.sFocPMSM.bUseZc = false;
    
    gsM1_Drive.ui16FlagAlignFinished = 0;

    /* Speed params */
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltPGain = M1_SPEED_PI_PROP_GAIN;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIAccK_1 = 0;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltUpperLim = M1_SPEED_LOOP_LIMIT;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltLowerLim = -M1_SPEED_LOOP_LIMIT;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.bLimFlag = 0;

    // 32-bit ramp for the cmd speed, used in slow loop
    gsM1_Drive.sSpeedCtrl.sSpeedRampParams.fltRampUp   	= M1_SPEED_RAMP;
    gsM1_Drive.sSpeedCtrl.sSpeedRampParams.fltRampDown 	= M1_SPEED_RAMP;
    gsM1_Drive.sSpeedCtrl.fltSpeed 						= 0;
    gsM1_Drive.sSpeedCtrl.fltSpeedCmd 					= 0;
    gsM1_Drive.sSpeedCtrl.fltSpeedError 			    	= 0;
    gsM1_Drive.sSpeedCtrl.fltSpeedFilt 					= 0;
    gsM1_Drive.sSpeedCtrl.fltSpeedRamp 					= 0;
    gsM1_Drive.sSpeedCtrl.fltSpeedReq 					= 0;
    gsM1_Drive.sSpeedCtrl.bOpenLoop 					= false;
    
    // Position calculation 
    gsM1_Drive.sSpeedPos.i16PolePairs 				= M1_POLE_PAIRS;
    gsM1_Drive.sSpeedPos.fltSpeedFbk 		    	    = 0;
    
    /* position loop */
    gsM1_Drive.SPosCtrl.i16FlagPosFilter				= 0;
    gsM1_Drive.SPosCtrl.i32PosDesired					= 0; 
    gsM1_Drive.SPosCtrl.i32PosRamp						= 0;
    gsM1_Drive.SPosCtrl.PosRampInc.f32RampUp			= M1_POS_RAMP_UP; 
    gsM1_Drive.SPosCtrl.PosRampInc.f32RampDown		= M1_POS_RAMP_DOWN;
    GFLIB_RampInit_F32(0, &gsM1_Drive.SPosCtrl.PosRampInc);
    gsM1_Drive.SPosCtrl.i16FlagSineTest				= 0;
    gsM1_Drive.SPosCtrl.f32SineAmplitude				= M1_POS_SINE_WAVE_AMPLITUDE;
    
    /* position controller */
    gsM1_Drive.SPosCtrl.sPosController.a32PropGain				= M1_POS_CTRL_PROP_GAIN;
    gsM1_Drive.SPosCtrl.sPosController.fltUpperLimit				= 2.0F*PI*M1_POLE_PAIRS*M1_Pos_Ctrl_AW_Limit/60;
    gsM1_Drive.SPosCtrl.sPosController.fltLowerLimit				= -2.0F*PI*M1_POLE_PAIRS*M1_Pos_Ctrl_AW_Limit/60;
    gsM1_Drive.SPosCtrl.sPosController.fltFreqToAngularSpeedCoeff = (float_t)(2.0*PI*M1_POLE_PAIRS);
    gsM1_Drive.SPosCtrl.sPosController.i32PosLoopFreq            = M1_MC_SLOW_CONTROL_LOOP_FREQ;
    gsM1_Drive.SPosCtrl.sPosController.fltMechToElecCoef         = M1_SPEED_MECH_TO_ElEC_COEFF;
    gsM1_Drive.SPosCtrl.sPosController.a32FwdGain                  =M1_POSITION_CTRL_SPEED_FWD_GAIN;
    gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter.f32W = M1_ENC_TRAJECTORY_FILTER_FREQ_FRAC;

    /* Angle Generator initialization for SineWave Test*/
    gsM1_AngleGen.f32StartAngle 		= 0;  		// simulated angle starts from 0 degree
    gsM1_AngleGen.a32DesiredFreq		= M1_SINE_WAVE_FREQ;//
    gsM1_AngleGen.f32PosRamp           = FRAC32(1.0)/M1_MC_SLOW_CONTROL_LOOP_FREQ;
    gsM1_AngleGen.f32CurAngle 			= gsM1_AngleGen.f32StartAngle; // configure current angle
	
    gsM1_Drive.ui16CounterSlowLoop 			= 1;
    gsM1_Drive.ui16DividerSlowLoop 			= M1_SPEED_LOOP_CNTR;
    gsM1_Drive.ui16CounterState 			    = 0;
    gsM1_Drive.ui16TimeFullSpeedFreeWheel 	= (uint16_t)(M1_DURATION_TASK_FREE_WHEEL * (M1_CONTROL_FREQ/M1_SPEED_LOOP_CNTR));
    gsM1_Drive.ui16TimeCalibration 			= (uint16_t)(M1_DURATION_TASK_CALIB * (M1_CONTROL_FREQ/M1_SPEED_LOOP_CNTR));
    gsM1_Drive.ui16TimeFaultRelease 		    = (uint16_t)(M1_DURATION_TASK_FAULT_RELEASE * (M1_CONTROL_FREQ/M1_SPEED_LOOP_CNTR));

    /* Defined scaling for FreeMASTER*/
    g_fltM1DCBvoltageScale                   = M1_U_DCB_MAX;
    g_fltM1voltageScale                      = M1_U_FOC_MAX;
    g_fltM1currentScale                      = M1_I_MAX;
    g_fltM1speedScale                         = M1_N_MAX;
    
    /* Filter init not to enter to fault */
    gsM1_Drive.sFocPMSM.sUDcBusFilter.fltFltBfrX[0] = (M1_OVERVOLT_LIMIT / 2.0F) + (M1_UNDERVOLT_LIMIT / 2.0F); 
    gsM1_Drive.sFocPMSM.sUDcBusFilter.fltFltBfrY[0] = (M1_OVERVOLT_LIMIT / 2.0F) + (M1_UNDERVOLT_LIMIT / 2.0F);  
    
    /* Init sensors/actuators pointers */
    /* For ADC driver */
    g_sM1AdcSensor.pf16UDcBus     = &(gsM1_Drive.sFocPMSM.f16UDcBus);
    g_sM1AdcSensor.psIABC         = &(gsM1_Drive.sFocPMSM.sIABCFrac);
    g_sM1AdcSensor.pui16SVMSector = &(gsM1_Drive.sFocPMSM.ui16SectorSVM);
    g_sM1AdcSensor.pui16AuxChan   = &(gsM1_Drive.f16AdcAuxSample);
    
    /* For ENC driver */
    g_sM1EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32B0 = M1_ENC_SPEED_FILTER_IIR_B0_FRAC;
    g_sM1EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32B1 = M1_ENC_SPEED_FILTER_IIR_B1_FRAC;
    g_sM1EncSensor.sSpeed.sENCSpeedFilter.sFltCoeff.f32A1 = M1_ENC_SPEED_FILTER_IIR_A1_FRAC;
    
    MCDRV_EncSpeedCalInit(&g_sM1EncSensor);
    MCDRV_EncToSpeedCalInit(&g_sM1EncSensor);
    trajectoryFilterInit(&gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter);

    /* INIT_DONE command */
    gsM1_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}
static void M1_StateInitSlow(void)
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
static void M1_StateStopFast(void)
{
    /* Disable PWM output */
    M1_DISABLE_PWM_OUTPUT();

    /* M1 Current */
    M1_MCDRV_ADC_GET(&g_sM1AdcSensor);
    
    /* DC bus voltage filter */
    gsM1_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
    gsM1_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM1_Drive.sFocPMSM.fltUDcBus, &gsM1_Drive.sFocPMSM.sUDcBusFilter);

    /* get position and speed from quadrature encoder sensor */
	if(g_sM1EncSensor.bPosAbsoluteFlag == TRUE)
	{
        MCDRV_GetRotorCurrentPos(&g_sM1EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM1EncSensor);
        /* Calculate the position for position loop */
        
        /* Get M1 rotor position */
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
    }
    
    M1_Fault();
}
static void M1_StateStopSlow(void)
{
    /* Actual position */                 
    MCDRV_GetRotorDeltaRev(&g_sM1EncSensor);
    gsM1_Drive.i32PosRelative = g_sM1EncSensor.i32Q16DeltaRev;
    
    /* Get speed from ENC M/T method in slow loop*/
	MCDRV_EncSpeedCalUpdate(&g_sM1EncSensor);
    
    /* speed measure algorithm */
    gsM1_Drive.sSpeedPos.fltSpeedFbk = g_sM1EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM1_Drive.sSpeedCtrl.fltSpeedFilt = gsM1_Drive.sSpeedPos.fltSpeedFbk;
    
    if (mbM1_SwitchAppOnOff)
    {
            /* Start command */
            gsM1_Ctrl.uiCtrl |= SM_CTRL_START;
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
static void M1_StateRunFast(void)
{
    /* M1 Current */
    M1_MCDRV_ADC_GET(&g_sM1AdcSensor);
    
    /* DC bus voltage filter */
    gsM1_Drive.sFocPMSM.fltUDcBus = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
    gsM1_Drive.sFocPMSM.fltUDcBusFilt = GDFLIB_FilterIIR1_FLT(gsM1_Drive.sFocPMSM.fltUDcBus, &gsM1_Drive.sFocPMSM.sUDcBusFilter);
    
    /* get position and speed from quadrature encoder sensor */
	if(g_sM1EncSensor.bPosAbsoluteFlag == TRUE)
	{
        MCDRV_GetRotorCurrentPos(&g_sM1EncSensor);
        MCDRV_GetRotorCurrentRev(&g_sM1EncSensor);
        /* Calculate the position for position loop */
        
        /* Get M1 rotor position */
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
        gsM1_Drive.sSpeedPos.sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)g_sM1EncSensor.f16PosElec);
    }
    
    M1_Fault();
    
    /* Convert phase currents from fractional measured values to float */
    gsM1_Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.sIABCFrac.f16A, g_fltM1currentScale);
    gsM1_Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.sIABCFrac.f16B, g_fltM1currentScale);
    gsM1_Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(gsM1_Drive.sFocPMSM.sIABCFrac.f16C, g_fltM1currentScale);
    
	/* Run sub-state function */
	mM1_STATE_RUN_TABLE_FAST[msM1_StateRun]();
    
    /* set current sensor for  sampling */
    M1_ADCChannelMapping(&g_sM1AdcSensor);

	if (!mbM1_SwitchAppOnOff)
	{
		/* Stop command */
		gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP;	
	}
}
static void M1_StateRunSlow(void)
{
    /* Get speed from ENC M/T method in slow loop */
	MCDRV_EncSpeedCalUpdate(&g_sM1EncSensor);
    
    /* speed measure algorithm */
    gsM1_Drive.sSpeedPos.fltSpeedFbk = g_sM1EncSensor.sSpeed.fltSpeed;
    
    /* Speed filter */
    gsM1_Drive.sSpeedCtrl.fltSpeedFilt = gsM1_Drive.sSpeedPos.fltSpeedFbk;

	/* Run sub-state function */
    mM1_STATE_RUN_TABLE_SLOW[msM1_StateRun]();
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
static void M1_TransFaultInit(void)
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
static void M1_TransInitFault(void)
{
	M1_DISABLE_PWM_OUTPUT();

	gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeFaultRelease;	
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
static void M1_TransInitStop(void)
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
static void M1_TransStopFault(void)
{
	/* Disables PWM outputs */
	M1_DISABLE_PWM_OUTPUT();
	
	gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeFaultRelease;
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
static void M1_TransStopRun(void)
{
	gsM1_Drive.SPosCtrl.i32PosDesired = 0;
	
	gsM1_Drive.sFocPMSM.ui16SectorSVM = 4;
  
    gsM1_Drive.ui16FlagAlignFinished = 0;

	/* 50% duty cycle */
	gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
	gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
	gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;
	
	/* PWM update */
	M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
	
	/* Enable PWM output */
	M1_ENABLE_PWM_OUTPUT();

	/* Required time for ADC calibration */
	gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeCalibration;

	/* Init sub-state when transition to RUN */
	msM1_StateRun = CALIB;

	/* Acknowledge that the system can proceed into the RUN state */
	gsM1_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
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
static void M1_TransRunFault(void)
{
	/* Disables PWM outputs */
	M1_DISABLE_PWM_OUTPUT();

	gsM1_Drive.sFocPMSM.sIABC.fltA = 0;
	gsM1_Drive.sFocPMSM.sIABC.fltB = 0;
	gsM1_Drive.sFocPMSM.sIABC.fltC = 0;
				
	gsM1_Drive.sFocPMSM.sIDQReq.fltD = 0;
	gsM1_Drive.sFocPMSM.sIDQReq.fltQ = 0;
	gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
	gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

	gsM1_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
	gsM1_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
	gsM1_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
	gsM1_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	

	gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeFaultRelease;
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
static void M1_TransRunStop(void)
{
	gsM1_Drive.sSpeedCtrl.fltSpeedCmd = 0;
	
    switch(msM1_StateRun)
    {
        case (CALIB):
		{
			/* Acknowledge that the system can proceed into the STOP state */
			gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
		}
        case (READY):
        {
			/* Acknowledge that the system can proceed into the STOP state */
			gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
        }        
        case (ALIGN):
        {
			/* Disables PWM outputs */
			M1_DISABLE_PWM_OUTPUT();

			gsM1_Drive.sFocPMSM.sIABC.fltA = 0;
			gsM1_Drive.sFocPMSM.sIABC.fltB = 0;
			gsM1_Drive.sFocPMSM.sIABC.fltC = 0;

			gsM1_Drive.sFocPMSM.sIDQReq.fltD = 0;
			gsM1_Drive.sFocPMSM.sIDQReq.fltQ = 0;
			gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
			gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

			gsM1_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
			gsM1_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
			gsM1_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
			gsM1_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	

			/* 50% duty cycle */
			gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
			gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
			gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

			/* PWM update */
			M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);

			/* Acknowledge that the system can proceed into the STOP state */
			gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

			break;
        }

        case (SPIN):
		{
			M1_TransRunSpinFreewheel();
			
			break;
		}

        case (FREEWHEEL):
		{
			if (gsM1_Drive.ui16CounterState == 0)
			{
				/* Disable PWM output */
				M1_DISABLE_PWM_OUTPUT();

				gsM1_Drive.sFocPMSM.sIABC.fltA = 0;
				gsM1_Drive.sFocPMSM.sIABC.fltB = 0;
				gsM1_Drive.sFocPMSM.sIABC.fltC = 0;
							
				gsM1_Drive.sFocPMSM.sIDQReq.fltD = 0;
				gsM1_Drive.sFocPMSM.sIDQReq.fltQ = 0;
				gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
				gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

				gsM1_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
				gsM1_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
				gsM1_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
				gsM1_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	
				
				/* Acknowledge that the system can proceed into the STOP state */
				gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

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
static void M1_StateRunCalibFast(void)
{	
    MCDRV_Curr3Ph2ShCalib(&g_sM1AdcSensor);
	
    /* change SVM sector in range <1;6> to measure all AD channel mapping combinations */
    if (++gsM1_Drive.sFocPMSM.ui16SectorSVM > 6)
        gsM1_Drive.sFocPMSM.ui16SectorSVM = 1;

    /* 50% duty cycle */
    gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
}
static void M1_StateRunCalibSlow(void)
{
    if (--gsM1_Drive.ui16CounterState == 0)
    {
        /* Offset calculation */
        MCDRV_Curr3Ph2ShCalibSet(&g_sM1AdcSensor);
        /* Transition to the RUN READY sub-state */
        M1_TransRunCalibReady();	
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
static void M1_StateRunReadyFast(void)
{
    gsM1_Drive.sFocPMSM.sIDQReq.fltD = 0;
    gsM1_Drive.sFocPMSM.sIDQReq.fltQ = 0;
    gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
    gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;
    gsM1_Drive.sFocPMSM.sIDQ.fltD = 0;
    gsM1_Drive.sFocPMSM.sIDQ.fltQ = 0;

    gsM1_Drive.sSpeedCtrl.fltSpeedReq = 0;
    gsM1_Drive.sSpeedCtrl.fltSpeed = 0;
    
    gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000; // 0.5
    gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
}
static void M1_StateRunReadySlow(void)
{	
    if(g_sM1EncSensor.bPosAbsoluteFlag == TRUE)
    {
      /* Transition to the RUN SPIN sub-state */
        M1_TransRunReadySpin();
    }
    else
    {
      /* Transition to the RUN ALIGN sub-state */
        M1_TransRunReadyAlign();
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
static void M1_StateRunAlignFast(void)
{
	MCSTRUC_AlignmentPMSM(&gsM1_Drive.sFocPMSM);
    
    /* PWM update */
    M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
}
static void M1_StateRunAlignSlow(void)
{
    if(--gsM1_Drive.ui16CounterState == 0)
    {
        /* Transition to the RUN SPIN sub-state */
        M1_TransRunAlignSpin();
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
static void M1_StateRunSpinFast(void)
{
    /* Position for FOC */
    gsM1_Drive.sFocPMSM.sAnglePosEl = gsM1_Drive.sSpeedPos.sAnglePosEl;
    
    /* FOC */
    MCSTRUC_FocPMSMCurrentCtrl(&gsM1_Drive.sFocPMSM);

    /* PWM update */
    M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
}
static void M1_StateRunSpinSlow(void)
{
    /* Actual position */                 
    MCDRV_GetRotorDeltaRev(&g_sM1EncSensor);
    gsM1_Drive.i32PosRelative = g_sM1EncSensor.i32Q16DeltaRev;
#ifdef	M1_POSITION_LOOP
    /* generate sine signal when sine wave testing in position loop */
    if (1 == gsM1_Drive.SPosCtrl.i16FlagSineTest)
    {
        AngleCalculation(&gsM1_AngleGen);
        gsM1_Drive.SPosCtrl.i32PosDesired = MLIB_Mul_F32(gsM1_Drive.SPosCtrl.f32SineAmplitude, MLIB_Conv_F32s(GFLIB_Sin_F16(MLIB_Conv_F16l(gsM1_AngleGen.f32CurAngle))));
    }
    /* choice one kind of filter group types and calculate position error */
    switch(gsM1_Drive.SPosCtrl.i16FlagPosFilter)
    {
    case 0:
    default:
            /* i32PosDesired --> i32PosRamp --> i32PosSmooth --> i32PError */
            gsM1_Drive.SPosCtrl.i32PosRamp = GFLIB_Ramp_F32(gsM1_Drive.SPosCtrl.i32PosDesired, &gsM1_Drive.SPosCtrl.PosRampInc);
            gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter.i32In = gsM1_Drive.SPosCtrl.i32PosRamp;
            gsM1_Drive.SPosCtrl.i32PosSmooth = trajectoryFilterUpdate(&gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter);
            gsM1_Drive.SPosCtrl.i32PError =  gsM1_Drive.SPosCtrl.i32PosSmooth - gsM1_Drive.i32PosRelative;
            gsM1_Drive.SPosCtrl.sPosController.i32PosRef = gsM1_Drive.SPosCtrl.i32PosSmooth;
            break;
            
    case 1:
            /* i32PosDesired --> i32PosSmooth --> i32PError */
            gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter.i32In = gsM1_Drive.SPosCtrl.i32PosDesired;
            gsM1_Drive.SPosCtrl.i32PosSmooth = trajectoryFilterUpdate(&gsM1_Drive.SPosCtrl.sCurveRef.sTrajFilter);
            gsM1_Drive.SPosCtrl.i32PError =  gsM1_Drive.SPosCtrl.i32PosSmooth - gsM1_Drive.i32PosRelative;
            gsM1_Drive.SPosCtrl.sPosController.i32PosRef = gsM1_Drive.SPosCtrl.i32PosSmooth;
            break;
            
    case 2:
            /* i32PosDesired --> i32PosRamp --> i32PError */
            gsM1_Drive.SPosCtrl.i32PosRamp = GFLIB_Ramp_F32(gsM1_Drive.SPosCtrl.i32PosDesired, &gsM1_Drive.SPosCtrl.PosRampInc);
            gsM1_Drive.SPosCtrl.i32PError =  gsM1_Drive.SPosCtrl.i32PosRamp - gsM1_Drive.i32PosRelative;
            gsM1_Drive.SPosCtrl.sPosController.i32PosRef = gsM1_Drive.SPosCtrl.i32PosRamp;
            break;
            
    case 3:
            /* i32PosDesired --> i32PError */
            gsM1_Drive.SPosCtrl.i32PError =  gsM1_Drive.SPosCtrl.i32PosDesired - gsM1_Drive.i32PosRelative;
            gsM1_Drive.SPosCtrl.sPosController.i32PosRef = gsM1_Drive.SPosCtrl.i32PosDesired;
            break;
    }
    
    /* position controller */
    gsM1_Drive.sSpeedCtrl.fltSpeedCmd = MCSTRUC_PosControllerCalc(gsM1_Drive.SPosCtrl.i32PError, &gsM1_Drive.SPosCtrl.sPosController);
#endif

    if (!gsM1_Drive.sFocPMSM.bOpenLoop)
    {
      /* Desired current by the speed PI controller */
      MCSTRUC_SpeedControllerCalc(&gsM1_Drive.sSpeedCtrl);
      gsM1_Drive.sFocPMSM.sIDQReq.fltQ = gsM1_Drive.sSpeedCtrl.fltIqController;
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
static void M1_StateRunFreewheelFast(void)
{
    /* Disable PWM output */
    M1_DISABLE_PWM_OUTPUT();

    gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

    /* PWM update */
    M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
}
static void M1_StateRunFreewheelSlow(void)
{
    if (--gsM1_Drive.ui16CounterState == 0)
    {
        /* If app switch is on */
        if (mbM1_SwitchAppOnOff)
        {
			 gsM1_Drive.sSpeedCtrl.fltSpeed = 0;
            gsM1_Drive.sSpeedCtrl.fltSpeedFilt = 0;
            
            /* If speed command is non-zero */
            if (MLIB_Abs_FLT(gsM1_Drive.sSpeedCtrl.fltSpeedCmd) > 0)
            {
                    /* If the motor has been in the run phase */
                            /* Sub-state RUN READY */
                            M1_TransRunFreewheelReady();
            }
            /* If speed command is zero */
            else
            {
                    /* Sub-state RUN READY */
                    M1_TransRunFreewheelReady();	
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
static void M1_TransRunCalibReady(void)
{
	/* Sub-state RUN READY */
	msM1_StateRun = READY;
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
static void M1_TransRunReadyAlign(void)
{
	gsM1_Drive.sFocPMSM.sAlignment.f16Speed = MLIB_Abs_F16(gsM1_Drive.sFocPMSM.sAlignment.f16Speed);
	
	if (gsM1_Drive.sSpeedCtrl.fltSpeedCmd < 0)
	{
		gsM1_Drive.sFocPMSM.sAlignment.f16Speed = MLIB_Neg_F16(gsM1_Drive.sFocPMSM.sAlignment.f16Speed);
	}
	
	/* Alignment voltage init point */
	gsM1_Drive.sFocPMSM.sAlignment.fltU = 0;
	
	/* Alignment current init point */
	gsM1_Drive.sFocPMSM.sAlignment.fltId = 0;
	gsM1_Drive.sFocPMSM.sAlignment.fltId = 0;
	
	/* Alignment voltage init position */
	gsM1_Drive.sFocPMSM.sAlignment.f16Position = 0;
	/* Alignment duration set-up */
	gsM1_Drive.ui16CounterState = gsM1_Drive.sFocPMSM.sAlignment.ui16TimeAlignment;
    
    /* Enable PWM output */
    M1_ENABLE_PWM_OUTPUT();
    
	/* Sub-state RUN ALIGN */
	msM1_StateRun = ALIGN;
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
static void M1_TransRunReadySpin(void)
{
    /* PMSM FOC params */
    gsM1_Drive.sFocPMSM.bIdPiSatFlag 					= 0;
    gsM1_Drive.sFocPMSM.bIqPiSatFlag 					= 0;	
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;

	/* Speed params */
	gsM1_Drive.sSpeedCtrl.fltSpeedRamp 						= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeedReq 				    		= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeedError 			    		= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeed 							= 0;
	
	IIR32FilterClear(&gsM1_Drive.sSpeedCtrl.sSpeedFilter); 
	gsM1_Drive.sSpeedCtrl.fltSpeedFilt 						= 0;
	
	/* Position loop */
    GFLIB_RampInit_F32(0, &gsM1_Drive.SPosCtrl.PosRampInc);
	gsM1_Drive.SPosCtrl.i32PosRamp						= 0;
	gsM1_Drive.SPosCtrl.i32PosSmooth 					= 0;
	gsM1_Drive.SPosCtrl.i32PError						= 0;

    /* Clear all FOC variables */
    M1_ClearFOCVariables();

	/* PWM update */
	M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
	
	/* limit the PI controller of Id and Iq */
	gsM1_Drive.sFocPMSM.bUseMaxBus = true;
	
	gsM1_Drive.ui16CounterSlowLoop = 1;
	
	/* Sub-state RUN SPIN */
	msM1_StateRun = SPIN;
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
static void M1_TransRunAlignSpin(void)
{
    /* initialize encoder driver */
	MCDRV_GetRotorInitPos(&g_sM1EncSensor, MLIB_Conv_F32s(gsM1_Drive.sFocPMSM.sAlignment.f16Position));

    /* Ensure the feedback position is reset to zero whenever state machine goes to spin */
    MCDRV_GetRotorInitRev(&g_sM1EncSensor);
    
	/* PMSM FOC params */
    gsM1_Drive.sFocPMSM.sIdPiParams.fltPGain 		= M1_D_KP_GAIN;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltIGain 		= M1_D_KI_GAIN;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltIAccK_1 	        = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.fltUpperLim 	= M1_D_LIMIT;  
    gsM1_Drive.sFocPMSM.sIdPiParams.fltLowerLim 	= -M1_D_LIMIT;
    gsM1_Drive.sFocPMSM.sIdPiParams.bLimFlag 		= 0;

    gsM1_Drive.sFocPMSM.sIqPiParams.fltPGain 		= M1_Q_KP_GAIN;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltIGain 		= M1_Q_KI_GAIN;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltIAccK_1 	        = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.fltUpperLim 	= M1_Q_LIMIT;  
    gsM1_Drive.sFocPMSM.sIqPiParams.fltLowerLim 	= -M1_Q_LIMIT;    
    gsM1_Drive.sFocPMSM.sIqPiParams.bLimFlag 		= 0;
  
    gsM1_Drive.sFocPMSM.bIdPiSatFlag 					= 0;
    gsM1_Drive.sFocPMSM.bIqPiSatFlag 					= 0;	

	/* Speed params */
	gsM1_Drive.sSpeedCtrl.fltSpeedRamp 						= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeedReq 						= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeedError 					= 0;
	gsM1_Drive.sSpeedCtrl.fltSpeed 							= 0;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.bSpdPiSatFlag = 0;
	
	IIR32FilterClear(&gsM1_Drive.sSpeedCtrl.sSpeedFilter); 
	gsM1_Drive.sSpeedCtrl.fltSpeedFilt 						= 0;

    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltPGain = M1_SPEED_PI_PROP_GAIN;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltIAccK_1 = 0;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltUpperLim = M1_SPEED_LOOP_LIMIT;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.fltLowerLim = -M1_SPEED_LOOP_LIMIT;
    gsM1_Drive.sSpeedCtrl.sSpeedPIController.sSpdPiParams.bLimFlag = 0;
	
	/* Position loop */
    GFLIB_RampInit_F32(0, &gsM1_Drive.SPosCtrl.PosRampInc);
	gsM1_Drive.SPosCtrl.i32PosRamp						= 0;
	gsM1_Drive.SPosCtrl.i32PosSmooth 					= 0;
	gsM1_Drive.SPosCtrl.i32PError						= 0;

    /* Clear all FOC variables */
    M1_ClearFOCVariables();

	/* PWM update */
	M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
	
	/* limit the PI controller of Id and Iq */
	gsM1_Drive.sFocPMSM.bUseMaxBus = true;
	
	gsM1_Drive.ui16CounterSlowLoop = 1;
	
	/* Sub-state RUN SPIN */
	msM1_StateRun = SPIN;
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
static void M1_TransRunAlignReady(void)
{
    /* Clear all FOC variables */
	M1_ClearFOCVariables();

	/* PWM update */
	M1_PWM_UPDATE(&gsM1_Drive.sFocPMSM.sDutyABC);
	
	/* Sub-state RUN READY */
	msM1_StateRun = READY;
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
static void M1_TransRunSpinFreewheel(void)
{
	/* Generates a time gap before the alignment to assure the rotor is not rotating */
    gsM1_Drive.ui16CounterState = gsM1_Drive.ui16TimeFullSpeedFreeWheel;	
	
    /* Clear all FOC variables */
    M1_ClearFOCVariables();
    
	/* Sub-state RUN FREEWHEEL */
	msM1_StateRun = FREEWHEEL;	
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
static void M1_TransRunFreewheelReady(void)
{
      /* 50% duty cycle */
    gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
    gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;

	/* Sub-state RUN READY */
	msM1_StateRun = READY;
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
static void M1_ClearFOCVariables(void)
{
    gsM1_Drive.sFocPMSM.sIDQReq.fltD = 0;
	gsM1_Drive.sFocPMSM.sIDQReq.fltQ = 0;

	gsM1_Drive.sFocPMSM.sUDQReq.fltD = 0;
	gsM1_Drive.sFocPMSM.sUDQReq.fltQ = 0;	

	gsM1_Drive.sFocPMSM.sIAlBe.fltAlpha = 0;
	gsM1_Drive.sFocPMSM.sIAlBe.fltBeta = 0;
	gsM1_Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0;
	gsM1_Drive.sFocPMSM.sUAlBeReq.fltBeta = 0;	
    
    /* 50% duty cycle */
	gsM1_Drive.sFocPMSM.sDutyABC.f16A = 0x4000;
	gsM1_Drive.sFocPMSM.sDutyABC.f16B = 0x4000;
	gsM1_Drive.sFocPMSM.sDutyABC.f16C = 0x4000;
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
static void M1_FaultDetection(void)
{
    //-----------------------------
    // Actual Faults
    //-----------------------------

    // Fault:   DC-bus over-voltage
	if (gsM1_Drive.sFocPMSM.fltUDcBusFilt >= gsM1_Drive.sFaultThresholds.fltUDcBusOver)
	{
		MC_FAULT_SET(gsM1_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_OVER);
	}
	else
	{
		MC_FAULT_CLEAR(gsM1_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_OVER);
	}

    // Fault:   DC-bus over-current
    if (M1_OVERCURRENT_FAULT())
    {
//		MC_FAULT_SET(gsM1_Drive.sFaultIdPending, MC_FAULT_I_DCBUS_OVER);    	
    }
    else
    {
		MC_FAULT_CLEAR(gsM1_Drive.sFaultIdPending, MC_FAULT_I_DCBUS_OVER);
    }
    
    // Fault:   DC-bus under-voltage
    if (gsM1_Drive.sFocPMSM.fltUDcBusFilt <= gsM1_Drive.sFaultThresholds.fltUDcBusUnder)
    {
		MC_FAULT_SET(gsM1_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_UNDER);
    }
    else
    {
		MC_FAULT_CLEAR(gsM1_Drive.sFaultIdPending, MC_FAULT_U_DCBUS_UNDER);
    }	
    
    gsM1_Drive.sFaultId = gsM1_Drive.sFaultIdPending;     
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
void M1_SetAppSwitch(bool bValue)
{
	mbM1_SwitchAppOnOff = bValue;
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
bool M1_GetAppSwitch(void)
{
	return (mbM1_SwitchAppOnOff);
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
bool M1_IsRunning(void)
{
	return (gsM1_Ctrl.eState == RUN);
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
void M1_SetSpeed(float fltSpeedCmd)
{
	if (mbM1_SwitchAppOnOff)
	{
		if (fltSpeedCmd < gsM1_Drive.sFaultThresholds.fltSpeedUnder)
		{
			gsM1_Drive.sSpeedCtrl.fltSpeedCmd = 0;
		}
		else if (fltSpeedCmd > gsM1_Drive.sFaultThresholds.fltSpeedOver)
		{
			gsM1_Drive.sSpeedCtrl.fltSpeedCmd = 0;
		}
		else
		{
			gsM1_Drive.sSpeedCtrl.fltSpeedCmd = fltSpeedCmd;	
		}
	}
	else
	{
		gsM1_Drive.sSpeedCtrl.fltSpeedCmd = 0;
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
void M1_Fault(void)
{
	M1_FaultDetection();

	if (MC_FAULT_ANY(gsM1_Drive.sFaultId))
	{
		/* Fault state */
		gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
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
static void M1_ADCChannelMapping(mcdrv_adc_t *this)
{
    //set A side channel number
    this->pToAdcBase->CMD[0].CMDL &= ~ADC_CMDL_ADCH_MASK;
    this->pToAdcBase->CMD[0].CMDL |= ADC_CMDL_ADCH(M1_Channels[*this->pui16SVMSector].ui16Ph0);
    //set B side channel number
    this->pToAdcBase->CMD[0].CMDL &= ~ADC_CMDL_ALTB_ADCH_MASK;
    this->pToAdcBase->CMD[0].CMDL |= ADC_CMDL_ALTB_ADCH(M1_Channels[*this->pui16SVMSector].ui16Ph1);
}
/******************************************************************************
* Inline functions
******************************************************************************/
