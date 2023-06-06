/******************************************************************************
* 
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved                       
*
******************************************************************************* 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file      motor_structure.c
*
*******************************************************************************
*
* Motor control structure.
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "motor_structure.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief   PMSM field oriented control
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN:->sIABC - input ABC phases currents
*			IN:->sAnglePosEl - angle where the currents were measured
*			IN:->sAnglePosElUpdate - angle where the next PWM reload
*			IN:->f16UDcBusFilt - DC bus voltage
*			IN:->bUseMaxBus - true to calculate max. possible output DQ voltage
*					limits in dependence on the dc bus voltage. False to keep the
*					output DQ voltage limits intact.
*			IN:->f16DutyCycleLimit - determines the max. value of duty cycle
*			IN/OUT->sIdPiParams - D current controller structure
*			IN/OUT->sIqPiParams - Q current controller structure
*			IN/OUT->i16IdPiSatFlag - D current controller saturation flag
*			IN/OUT->i16IqPiSatFlag - Q current controller saturation flag
*			OUT->sDutyABC - ABC duty cycles
*			OUT->uw16SectorSVM - Next step SVM sector
*
* @return  N/A
*
******************************************************************************/
void MCSTRUC_FocPMSMCurrentCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM)
{
    /* 3-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_Clark_FLT(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);
    
    /* 2-phase to 2-phase transformation to rotary ref. frame */
    GMCLIB_Park_FLT(&psFocPMSM->sIAlBe, &psFocPMSM->sAnglePosEl, &psFocPMSM->sIDQ);

    if (psFocPMSM->bUseZc)
    {
            /* Zero cancellation filters for the D, Q current components */
            psFocPMSM->sIDQReqZc.fltD = GDFLIB_FilterIIR1_FLT(psFocPMSM->sIDQReq.fltD, &psFocPMSM->sIdZcFilter);
            psFocPMSM->sIDQReqZc.fltQ = GDFLIB_FilterIIR1_FLT(psFocPMSM->sIDQReq.fltQ, &psFocPMSM->sIqZcFilter);

            /* D current error calculation */
            psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReqZc.fltD, psFocPMSM->sIDQ.fltD);
    
            /* Q current error calculation */
            psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReqZc.fltQ, psFocPMSM->sIDQ.fltQ);

    }
    else
    {
            /* D current error calculation */
            psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltD, psFocPMSM->sIDQ.fltD);

            /* Q current error calculation */
            psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltQ, psFocPMSM->sIDQ.fltQ);
    }
	
    /*** D - controller ***/
		
    if (psFocPMSM->bUseMaxBus)
    {
        psFocPMSM->sIdPiParams.fltLowerLim = MLIB_MulNeg_FLT(psFocPMSM->fltDutyCycleLimit, psFocPMSM->fltUDcBusFilt);
        psFocPMSM->sIdPiParams.fltUpperLim = MLIB_Neg_FLT(psFocPMSM->sIdPiParams.fltLowerLim);
    }

    /* D current PI controller */
    psFocPMSM->sUDQController.fltD = GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltD, &psFocPMSM->bIdPiSatFlag, &psFocPMSM->sIdPiParams);

    /* D current controller saturation flag */
    psFocPMSM->bIdPiSatFlag = psFocPMSM->sIdPiParams.bLimFlag;


    /*** Q - controller ***/
    if (psFocPMSM->bUseMaxBus)
    {
            psFocPMSM->sIqPiParams.fltUpperLim = GFLIB_Sqrt_FLT(MLIB_Sub_FLT( \
            MLIB_Mul_FLT(psFocPMSM->sIdPiParams.fltUpperLim, psFocPMSM->sIdPiParams.fltUpperLim), \
                    MLIB_Mul_FLT(psFocPMSM->sUDQController.fltD, psFocPMSM->sUDQController.fltD)));
    
    psFocPMSM->sIqPiParams.fltLowerLim = MLIB_Neg_FLT(psFocPMSM->sIqPiParams.fltUpperLim);
    }

    /* Q current PI controller */
    psFocPMSM->sUDQController.fltQ = GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltQ, &psFocPMSM->bIqPiSatFlag, &psFocPMSM->sIqPiParams);

    /* Q current controller saturation flag */
    psFocPMSM->bIqPiSatFlag = psFocPMSM->sIqPiParams.bLimFlag;

    if (!psFocPMSM->bOpenLoop)
    {
        /* D, Q voltage application */
        psFocPMSM->sUDQReq.fltD = psFocPMSM->sUDQController.fltD;
        psFocPMSM->sUDQReq.fltQ = psFocPMSM->sUDQController.fltQ;	
    }

    /* 2-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_ParkInv_FLT(&psFocPMSM->sUDQReq, &psFocPMSM->sAnglePosEl, &psFocPMSM->sUAlBeReq);

    /* Begin - voltage control */
    GMCLIB_ElimDcBusRipFOC_F16ff(psFocPMSM->fltUDcBusFilt, &psFocPMSM->sUAlBeReq, &psFocPMSM->sUAlBeCompFrac);
    
    psFocPMSM->ui16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
    /* End - voltage control */	
}
/***************************************************************************//*!
*
* @brief   PMSM Alignment with rotation
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN/OUT:->sAlignment.f32Position - position of the field
*			IN:->sAlignment.f32Speed - speed of the field
*			IN:->sIDQ.f16D - measured D current
*			IN:->sAlignment.f16UMax - max D voltage at alignment
*			IN:->sAlignment.f16IMax - max D current at alignment
*			IN/OUT:->sAlignment.f32U - alignment D voltage which is ramped
*			IN:->sAlignment.f32UStep - voltage step to ramp the voltage
*			OUT:->psFocPMSM->sDutyABC - duty cycles ABC
*
*
* @return  N/A
*
******************************************************************************/
void MCSTRUC_AlignmentPMSM(MCSTRUC_FOC_PMSM_T *psFocPMSM)
{
	GMCLIB_2COOR_SINCOS_T_FLT sAngle;

    sAngle.fltSin = GFLIB_Sin_FLTa((acc32_t)psFocPMSM->sAlignment.f16Position);
    sAngle.fltCos = GFLIB_Cos_FLTa((acc32_t)psFocPMSM->sAlignment.f16Position);

  	/* 3-phase to 2-phase transformation to stationary ref. frame */
	GMCLIB_Clark_FLT(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);

	/* 2-phase to 2-phase transformation to rotary ref. frame */
	GMCLIB_Park_FLT(&psFocPMSM->sIAlBe, &sAngle, &psFocPMSM->sIDQ);

#if ALIGN_METHOD == 0
	/* Ramp D voltage until it reaches f16MaxVoltage or current reaches f16MaxCurrent */
	if ((psFocPMSM->sUDQReq.fltD < psFocPMSM->sAlignment.fltUMax) && (psFocPMSM->sIDQ.fltD < psFocPMSM->sAlignment.fltIMax))
	{
		/* 32 frac calculation because the step is smaller than 16-bit LSB */
		psFocPMSM->sAlignment.fltU += psFocPMSM->sAlignment.fltUStep;
		
		/* Copies the desired voltage to the system */
		psFocPMSM->sUDQReq.fltD = psFocPMSM->sAlignment.fltU;		
	}
	psFocPMSM->sUDQReq.fltQ = 0;
#else
	if (psFocPMSM->sAlignment.fltId < psFocPMSM->sAlignment.fltIMax)
	{
		/* Align current ramp */
		psFocPMSM->sAlignment.fltId += psFocPMSM->sAlignment.fltIdStep;
	}
	psFocPMSM->sIDQReq.fltD = psFocPMSM->sAlignment.fltId;
	psFocPMSM->sIDQReq.fltQ = 0;
	
	/* D current error calculation */
    psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltD, psFocPMSM->sIDQ.fltD);

	/* Q current error calculation */
    psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltQ, psFocPMSM->sIDQ.fltQ);
	
    /* D current PI controller */
    psFocPMSM->sUDQController.fltD = GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltD, &psFocPMSM->bIdPiSatFlag, &psFocPMSM->sIdPiParams);

    /* D current controller saturation flag */
    psFocPMSM->bIdPiSatFlag = psFocPMSM->sIdPiParams.bLimFlag;

    /* Q current PI controller */
    psFocPMSM->sUDQController.fltQ = GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltQ, &psFocPMSM->bIqPiSatFlag, &psFocPMSM->sIqPiParams);

    /* Q current controller saturation flag */
    psFocPMSM->bIqPiSatFlag = psFocPMSM->sIqPiParams.bLimFlag;

    /* D, Q voltage application */
    psFocPMSM->sUDQReq.fltD = psFocPMSM->sUDQController.fltD;
    psFocPMSM->sUDQReq.fltQ = psFocPMSM->sUDQController.fltQ;	
#endif
	
   	/* 2-phase to 2-phase transformation to stationary ref. frame */
 	GMCLIB_ParkInv_FLT(&psFocPMSM->sUDQReq, &sAngle, &psFocPMSM->sUAlBeReq);

	/* Begin - voltage control */ 	
 	GMCLIB_ElimDcBusRipFOC_F16ff(psFocPMSM->fltUDcBusFilt, &psFocPMSM->sUAlBeReq, &psFocPMSM->sUAlBeCompFrac);
 	
	psFocPMSM->ui16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
	/* End - voltage control */

	psFocPMSM->sAlignment.f16Position += psFocPMSM->sAlignment.f16Speed;
}

float MCSTRUC_PosControllerCalc(int32_t i32Err, MCSTRUC_POS_CONTROLLER_T *ptr)
{
    int32_t i32Tmp;
    int64_t i64Tmp;

    ptr->a32ResultPreSat = MLIB_MulSat_A32(i32Err, ptr->a32PropGain)>>1;
    //Q16.16*Q17.15=Q33.31, Mul_A32SatA_32() include '>>15',so we need to >>1 to get Q17.15
    ptr->fltSpeedCtrl = MLIB_ConvSc_FLTaf(ptr->a32ResultPreSat, ptr->fltMechToElecCoef);
    
    // Speed reference feed forward
    ptr->i32Q16PosRefErr = ptr->i32PosRef - ptr->i32PosRef_1; // Get revolution deviation
    i64Tmp = (int64_t)ptr->i32Q16PosRefErr * ptr->i32PosLoopFreq;// Q16.16 * Q32.0 = Q48.16, get the derivative of position reference against time: i64Tmp represents mechanical frequency here
    i32Tmp = i64Tmp>>1; // Q48.16 -> Q17.15, [round/s]
        
    ptr->a32SpeedFwd = MLIB_MulSat_A32(i32Tmp, ptr->a32FwdGain);//mechanical speedReq, [r/s]
    ptr->fltSpeedFwd = MLIB_ConvSc_FLTaf(ptr->a32SpeedFwd, ptr->fltMechToElecCoef);//electric speed,rad/s
    
    ptr->a32ResultPreSat = ptr->a32ResultPreSat + ptr->a32SpeedFwd;
    ptr->fltSpeedPreSat = MLIB_ConvSc_FLTaf(ptr->a32ResultPreSat, ptr->fltFreqToAngularSpeedCoeff);
	if(ptr->fltSpeedPreSat > ptr->fltUpperLimit)
	{
		ptr->fltResultAftSat = ptr->fltUpperLimit;
		ptr->i16SatFlag = 1;
	}
	else if(ptr->fltSpeedPreSat < ptr->fltLowerLimit)
	{
		ptr->fltResultAftSat = ptr->fltLowerLimit;
		ptr->i16SatFlag = -1;
	}
	else
	{
		ptr->fltResultAftSat = ptr->fltSpeedPreSat;
		ptr->i16SatFlag = 0;
	}

    ptr->fltSpeedRef = ptr->fltResultAftSat;
    
    // Store historical data
	ptr->i32PosRef_1 = ptr->i32PosRef;
    
	return(ptr->fltSpeedRef);
}
void MCSTRUC_SpeedControllerCalc(MCSTRUC_SPEED_CTRL_T *psSpeed)
{
    /* Speed ramp generation */
    psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);

    /* Speed error calculation */
    psSpeed->fltSpeedError = MLIB_Sub_FLT(psSpeed->fltSpeedRamp, psSpeed->fltSpeedFilt);
    
    /* Desired current by the speed PI controller */
    psSpeed->fltIqController =
        GFLIB_CtrlPIpAW_FLT(psSpeed->fltSpeedError, &psSpeed->sSpeedPIController.bSpdPiSatFlag, &psSpeed->sSpeedPIController.sSpdPiParams);
}
void AngleCalculation(ANGLE_GENERATOR_T *ptr)
{
	ptr->f32Delta = MLIB_MulSat_A32(ptr->a32DesiredFreq, ptr->f32PosRamp) << 1; // Q17.15 * Q1.31 = Q1.31
	ptr->f32CurAngle += ptr->f32Delta;
}

void IIR32FilterClear(IIR32_FILTER_T *ptr)
{
	ptr->f32X 		= 0;
	ptr->f32X_1 	= 0;
	ptr->f32Y 		= 0;
	ptr->f32Y_1		= 0;
}

frac32_t IIR32FilterCalc(IIR32_FILTER_T *ptr, frac32_t f32Input)
{
	ptr->f32X = f32Input;
	
	/* A2 is negative */
	ptr->f32Y = MLIB_Sub_F32(MLIB_Add_F32(MLIB_Mul_F32(ptr->f32B1, ptr->f32X), \
			MLIB_Mul_F32(ptr->f32B2, ptr->f32X_1)),\
					MLIB_Mul_F32(ptr->f32A2, ptr->f32Y_1));
	ptr->f32Y = ptr->f32Y << 1;
	ptr->f32Y_1 = ptr->f32Y;
	ptr->f32X_1 = ptr->f32X;
	return(ptr->f32Y);
}

void trajectoryFilterInit(trajectory_filter_t *this)
{
	this->i32In = 0;
	this->i64Out = 0;
	this->i64Out_1 = 0;
	this->i64M = 0;
	this->i64M_1 = 0;
}

int32_t trajectoryFilterUpdate(trajectory_filter_t *this)
{
	int32_t i32Tmp;

	i32Tmp = this->i32In - 2*(int32_t)(this->i64M_1>>32) - (int32_t)(this->i64Out_1>>32);
	this->i64M = this->i64M_1 + (((int64_t)i32Tmp * this->f32W)<<1);

	i32Tmp = (int32_t)(this->i64M>>32);
	this->i64Out = this->i64Out_1 + (((int64_t)i32Tmp * this->f32W)<<1);

	this->i64M_1 = this->i64M;
	this->i64Out_1 = this->i64Out;

	return (int32_t)(this->i64Out>>32);
}
/******************************************************************************
* Inline functions
******************************************************************************/