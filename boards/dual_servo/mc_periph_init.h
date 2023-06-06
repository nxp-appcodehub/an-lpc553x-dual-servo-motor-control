/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This is a generic configuration file of the motor control driver. You need to edit the file.
 * Remove this warning statement after this file is edited manually or
 * re-generate this file using MC_PMSM Config Tool component.
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "mcdrv_eflexpwm_lpc.h"
#include "mcdrv_adc_lpc55s36.h"
#include "mcdrv_enc_lpc55s36.h"
#include "M1_Params.h"
#include "M2_Params.h"
/******************************************************************************
 * Timing
 ******************************************************************************/
/* MCU core clock */
#define MCU_CLOCK_FREQ          (150000000U) /* 150 MHz */
/* Fast loop interrupt generation timer*/
#define M1_PWM_TIMER
/* PWM generation timer */
#define M1_PWM_TIMER_FTM0
/* PWM modulo = PWM_input_clock / M1_CONTROL_FREQ */
#define M1_PWM_MODULO           (MCU_CLOCK_FREQ / M1_CONTROL_FREQ)
/* Output PWM deadtime value in nanoseconds */
#define M1_PWM_DEADTIME (500)       
/* PWM vs. Fast control loop ratio */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)
/* Fast loop frequency in Hz */
#define M1_FAST_LOOP_FREQ       (M1_CONTROL_FREQ / M1_FOC_FREQ_VS_PWM_FREQ)
/* Slow loop interrupt generation timer*/
#define M1_SLOW_LOOP_TIMER      (CTIMER0)
/* Slow control loop frequency */
#define M1_SLOW_LOOP_FREQ       (M1_MC_SLOW_CONTROL_LOOP_FREQ)
/* Fast loop period */
#define M1_FAST_LOOP_TS         ((float_t)1.0 / (float_t)(M1_FAST_LOOP_FREQ))
/* Slow loop period */
#define M1_SLOW_LOOP_TS         ((float_t)1.0 / (float_t)(M1_SLOW_LOOP_FREQ))

/* Fast loop interrupt generation timer*/
#define M2_PWM_TIMER
/* PWM generation timer */
#define M2_PWM_TIMER_FTM0
/* PWM modulo = PWM_input_clock / M2_CONTROL_FREQ */
#define M2_PWM_MODULO           (MCU_CLOCK_FREQ / M2_CONTROL_FREQ)
/* Output PWM deadtime value in nanoseconds */
#define M2_PWM_DEADTIME (500)       
/* PWM vs. Fast control loop ratio */
#define M2_FOC_FREQ_VS_PWM_FREQ (1U)
/* Fast loop frequency in Hz */
#define M2_FAST_LOOP_FREQ       (M2_CONTROL_FREQ / M2_FOC_FREQ_VS_PWM_FREQ)
/* Slow loop interrupt generation timer*/
#define M2_SLOW_LOOP_TIMER      (CTIMER1)
/* Slow control loop frequency */
#define M2_SLOW_LOOP_FREQ       (M2_MC_SLOW_CONTROL_LOOP_FREQ)
/* Fast loop period */
#define M2_FAST_LOOP_TS         ((float_t)1.0 / (float_t)(M2_FAST_LOOP_FREQ))
/* Slow loop period */
#define M2_SLOW_LOOP_TS         ((float_t)1.0 / (float_t)(M2_SLOW_LOOP_FREQ))
 /******************************************************************************
  * Output control
  ******************************************************************************/
/* DC bus braking resistor control */
#define M1_BRAKE_SET()          
#define M1_BRAKE_CLEAR()        

/******************************************************************************
 * DAC level conversion
 ******************************************************************************/
#define FLT_DCB_CURRENT_SCALE			(4096.0f / (5.0f * 3.3f))

#define FLT_DCB_MAX_CURENT_TO_DAC_COUNT( fltMaxDcbCurrent ) \
	( fltMaxDcbCurrent <= 0.0f ? 2048U : ((uint16_t)(fltMaxDcbCurrent * FLT_DCB_CURRENT_SCALE) + 2048U) )

/******************************************************************************
 * MC driver macro definition and check - do not change this part
 ******************************************************************************/
/******************************************************************************
 * Define motor ADC control functions
 ******************************************************************************/
#define M1_MCDRV_ADC_GET(par)	(MCDRV_CurrAndVoltDcBusGet(par))
//    MCDRV_Curr3Ph2ShGet(par); \
//    MCDRV_VoltDcBusGet(par);  \
//    MCDRV_AuxValGet(par);
#define M1_MCDRV_CURR_3PH_CHAN_ASSIGN(par) (MCDRV_Curr3Ph2ShChanAssign(par))
#define M1_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M1_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M1_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

#define M2_MCDRV_ADC_GET(par)	(MCDRV_CurrAndVoltDcBusGet(par))
/******************************************************************************
 * Define motor 3-ph PWM control functions
 ******************************************************************************/
#define M1_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet(par))
#define M1_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn(par))
#define M1_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis(par))
#define M1_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))
#define M1_MCDRV_P2M3PH_FLT_TRY_CLR(par) (MCDRV_eFlexPwm3PhFltTryClr(par))

#define M1_MCDRV_QD_CLEAR(par) (MCDRV_QdEncClear(par))
/******************************************************************************
 * Global typedefs
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions
 ******************************************************************************/
extern mcdrv_eflexpwm_t g_sM1Pwm3ph, g_sM2Pwm3ph;
extern mcdrv_adc_t g_sM1AdcSensor, g_sM2AdcSensor;
extern enc_block_t g_sM1EncSensor, g_sM2EncSensor;
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void MCDRV_Init_M1(void);

#ifdef __cplusplus
}
#endif
#endif /* _MC_PERIPH_INIT_H_  */
