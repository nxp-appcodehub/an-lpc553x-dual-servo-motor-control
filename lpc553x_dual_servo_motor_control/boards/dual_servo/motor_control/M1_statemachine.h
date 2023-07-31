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
* @file      M1_statemachine.h
*
* @author    nxf64158
* 
* @version   
* 
* @date      Mar-8-2022
* 
* @brief     M1 state machine
*
*******************************************************************************
*
* M1 state machine.
*
******************************************************************************/
#ifndef _M1_STATEMACHINE_H_
#define _M1_STATEMACHINE_H_

/******************************************************************************
* Includes
******************************************************************************/
/* library headers */
#include "mlib.h"
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"

/* application constants */
#include "M1_Params.h"
#include "state_machine.h"
#include "motor_def.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define M1_DURATION_TASK_ALIGN              1		/* Duration of alignment [s] */
#define M1_DURATION_TASK_CALIB              1.0		/* Duration of current calibration [s] */
#define M1_DURATION_TASK_INTER_RUN		    2.0		/* Duration between start-ups [s] */
#define M1_DURATION_TASK_FREE_WHEEL		  	1.0		/* Duration of free wheeling [s] */
#define M1_DURATION_TASK_FAULT_RELEASE		2.0		/* Duration after fault clear [s] */

#define M1_ALIGN_CURRENT                  	1.0		/* Alignment current [A] */
#define M1_ALIGN_VOLT                       5.0  	/* Alignment voltage (in case the current is not controlled) [V] */
#define M1_ALIGN_VOLT_MAX                   3.0  	/* Max. voltage for alignment [V] */
#define M1_ALIGN_VOLT_RAMP				  	5.0  	/* Alignment voltage ramp [V/s]*/
#define M1_ALIGN_CURRENT_RAMP				2.0  	/* Alignment current ramp [V/s]*/
#define M1_ALIGN_SPEED				  	  	150.0  	/* Alignment speed [RPM] */
#define M1_ALIGN_SPEED_SCALE		  	  	(0.5 * M1_CONTROL_FREQ * 60 / M1_POLE_PAIRS)  /* Speed scale at alignment */

/*=============================================================================================================================
 
 	 	 	 	 	 	 	 CNT * FREQ_TIMER_H*POLE_PAIRS
    speed in fractional = ----------------------------------, where CNT is the captured value of QEP's counter and Tcnt
	 	 	                 Tcnt * 4*ENC_LINES*FREQ_BASE
	 	 	                 	 	 	 	 	 	 	 	 is the captured value of Qtimer's counter
	 	 	                 	 	 	 	 	 	 	 	 
													2*FREQ_TIMER_H*POLE_PAIRS
	The constant is chosen as SPEED_ENC_CONST_H = -----------------------------, it's twice as its original value due to the fact that  
	 	 	                 	 	 	 	 	 	 4*ENC_LINES*FREQ_BASE	 	 	
	 	 	                 	 	 	 	 	 	                             division algorithm will right shift the result one bit
			 CNT * SPEED_ENC_CONST_H
	speed = -------------------------
	               Tcnt
	               
	Fractional style of this constant is chosen per its value, in this case SPEED_ENC_CONST_H = 6.677.
	Because CNT is multiplied with 0.51212 in the algorithm, so SPEED_ENC_CONST_H is actually 6.677/0.51212 = 13.038, and Q5.11 is used
=================================================================================================================================*/
//#define M1_SPEED_ENC_CONST_H 				(Word16)(2048*((2*M1_FREQ_TIMER_H*M1_POLE_PAIRS)/(4.0*M1_ENC_LINES*M1_ELEC_FREQ*M1_ENC_SCALE_FRAC)))  // Q5.11

#define M1_SPD_CTRL_AW_LOW_SPD				400		// low speed (RPM)
#define M1_SPD_CTRL_AW_HIGH_SPD				1000	// high speed (RPM)
#define M1_SPD_CTRL_AW_LOW_SPD_KP			FRAC16(6.0 /M1_SPD_CTRL_AW_KP_BASE)	// proportion coefficient when low speed
#define M1_SPD_CTRL_AW_HIGH_SPD_KP			FRAC16(2.0 /M1_SPD_CTRL_AW_KP_BASE)	// proportion coefficient when high speed
#define M1_SPD_CTRL_AW_LOW_SPD_KI			FRAC16(0.03)	// integral coefficient when low speed
#define M1_SPD_CTRL_AW_HIGH_SPD_KI			FRAC16(0.01)	// integral coefficient when high speed

/*============================================================================================
 * Position loop
 ============================================================================================*/
#define M1_SINE_WAVE_FREQ       			ACC32(1)
#define	M1_POS_SINE_WAVE_AMPLITUDE			ACC32(0.5)//for test

/* parameter linear of position controller */
#define M1_POS_PROP_GAIN_LOW				2000	// proportion coefficient when low position error
#define M1_POS_PROP_GAIN_HIGH				6000	// proportion coefficient when high position error
#define M1_POS_ERR_LOW						0.002	// low position error (Rounds)
#define M1_POS_ERR_HIGH						0.02	// high position error (Rounds)
#define M1_POS_ERR_AMPLIFICATION			1000	// the magnification of position error


/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern SM_APP_CTRL_T						gsM1_Ctrl;
extern MCDEF_FOC_PMSM_ENC_SPEED_PI_T		gsM1_Drive;
extern bool                 				mbM1_SwitchAppOnOff;
extern MCDEF_ENCODER_COUNTER_T                  gsM1_Enc;

extern volatile float g_fltM1DCBvoltageScale;
extern volatile float g_fltM1voltageScale;
extern volatile float g_fltM1currentScale;
extern volatile float g_fltM1speedScale;
/******************************************************************************
* Global functions
******************************************************************************/

extern void M1_SetAppSwitch(bool bValue);
extern bool M1_GetAppSwitch(void);
extern bool M1_IsRunning(void);
extern void M1_SetSpeed(float fltSpeedCmd);
extern float M1_GetSpeed(void);
extern void M1_Fault(void);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* STATEMACHINE */
