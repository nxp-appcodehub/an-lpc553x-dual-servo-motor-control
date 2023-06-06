/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_eflexpwm_lpc.h"
#include "mc_periph_init.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t s_statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function updates FTM value register
 *
 * @param this   Pointer to the current object
 * TODO - Deadtime compensation?
 * @return none
 */
void MCDRV_eFlexPwm3PhSet(mcdrv_eflexpwm_t *this)
{
	PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    GMCLIB_3COOR_T_F16 sUABCtemp;

    int16_t w16ModuloHalf;
    int16_t w16PwmValPhA;
	int16_t w16PwmValPhB;
	int16_t w16PwmValPhC;

	/* Clear error flag if no overvoltage is present */


    /* pointer to duty cycle structure */
    sUABCtemp = *this->psUABC;

    /* Get PWM_modulo/2 from PWM register, always correct regardless of PWM runtime setting, variable used for update of duty cycles of all 3 phases */
    w16ModuloHalf = (PWM0->SM[0].VAL1+1);

    /* Phase A - duty cycle calculation */
    w16PwmValPhA = (w16ModuloHalf * sUABCtemp.f16A) >> 15;
    pCurrentPwm->SM[0].VAL2 = (uint16_t)(-w16PwmValPhA); // rising edge value register update
    pCurrentPwm->SM[0].VAL3 = (uint16_t)w16PwmValPhA;  // falling edge value register update, no need to calculate it

    /* Phase B - duty cycle calculation */
    w16PwmValPhB = (w16ModuloHalf * sUABCtemp.f16B) >> 15;
    pCurrentPwm->SM[1].VAL2 = (uint16_t)(-w16PwmValPhB); // rising edge value register update
    pCurrentPwm->SM[1].VAL3 = (uint16_t)w16PwmValPhB; // falling edge value register update, no need to calculate it

    /* Phase C - duty cycle calculation */
    w16PwmValPhC = (w16ModuloHalf * sUABCtemp.f16C) >> 15;
    pCurrentPwm->SM[2].VAL2 = (uint16_t)(-w16PwmValPhC); // rising edge value register update
    pCurrentPwm->SM[2].VAL3 = (uint16_t)w16PwmValPhC; // falling edge value register update, no need to calculate it

    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(15);

//    uint32_t periodMatchReg0 = 0;
//    uint32_t pulseMatchReg0 = 0, pulseMatchReg1 = 0, pulseMatchReg2 = 0, pulseMatchReg3 = 0, pulseMatchReg4 = 0,
//             pulseMatchReg5 = 0;
//    uint32_t pulsePeriod0 = 0, pulsePeriod1 = 0, pulsePeriod2 = 0, pulsePeriod3 = 0, pulsePeriod4 = 0, pulsePeriod5 = 0;
//    uint32_t period;
//
//    /* Retrieve the match register number for the PWM period */
//    periodMatchReg0 = SCT0->EV[this->eventNumberOutput0].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    period          = SCT0->MATCH[periodMatchReg0];
//
//    // A TOP   P18-5
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg0 = SCT0->EV[this->eventNumberOutput0 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod0 = ((period * sUABCtemp.f16A) / 32768) + this->ui16DeadTimePWM;
//    // A BOTTOM    P18-20
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg1 = SCT0->EV[this->eventNumberOutput1 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod1 = ((period * sUABCtemp.f16A) / 32768) - this->ui16DeadTimePWM;
//    // B TOP     P18-6
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg2 = SCT0->EV[this->eventNumberOutput2 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod2 = ((period * sUABCtemp.f16B) / 32768) + this->ui16DeadTimePWM;
//    // B BOTTOM      P18-3
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg3 = SCT0->EV[this->eventNumberOutput3 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod3 = ((period * sUABCtemp.f16B) / 32768) - this->ui16DeadTimePWM;
//    // C TOP      P18-19
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg4 = SCT0->EV[this->eventNumberOutput4 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod4 = ((period * sUABCtemp.f16C) / 32768) + this->ui16DeadTimePWM;
//    // C BOTTOM       P17-13
//    /* Retrieve the match register number for the PWM pulse period */
//    pulseMatchReg5 = SCT0->EV[this->eventNumberOutput5 + 1].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;
//    /* Calculate pulse width match value */
//    pulsePeriod5 = ((period * sUABCtemp.f16C) / 32768) - this->ui16DeadTimePWM;
//
//    /* Stop the counter before updating match register */
//    SCT0->CTRL_ACCESS16BIT.CTRLL |= (SCT_CTRLL_HALT_L_MASK);
//
//    /* Update dutycycle */
//    SCT0->MATCHREL[pulseMatchReg0] = SCT_MATCHREL_RELOADn_L(pulsePeriod0);
//    SCT0->MATCHREL[pulseMatchReg1] = SCT_MATCHREL_RELOADn_L(pulsePeriod1);
//    SCT0->MATCHREL[pulseMatchReg2] = SCT_MATCHREL_RELOADn_L(pulsePeriod2);
//    SCT0->MATCHREL[pulseMatchReg3] = SCT_MATCHREL_RELOADn_L(pulsePeriod3);
//    SCT0->MATCHREL[pulseMatchReg4] = SCT_MATCHREL_RELOADn_L(pulsePeriod4);
//    SCT0->MATCHREL[pulseMatchReg5] = SCT_MATCHREL_RELOADn_L(pulsePeriod5);
//
//    /* Restart the counter */
//    SCT0->CTRL_ACCESS16BIT.CTRLL &= ~((uint16_t)SCT_CTRLL_HALT_L_MASK);
}

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutEn(mcdrv_eflexpwm_t *this)
{
	PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

	pCurrentPwm->SM[0].VAL2 = 0U;
	pCurrentPwm->SM[1].VAL2 = 0U;
	pCurrentPwm->SM[2].VAL2 = 0U;
	pCurrentPwm->SM[3].VAL2 = 0U;

	pCurrentPwm->SM[0].VAL3 = 0U;
	pCurrentPwm->SM[1].VAL3 = 0U;
	pCurrentPwm->SM[2].VAL3 = 0U;
	pCurrentPwm->SM[3].VAL3 = 0U;

    /* PWM0 module 0 trigger on VAL4 enabled for ADC synchronization */
	pCurrentPwm->SM[0].VAL4 = (uint16_t)(-(M1_PWM_MODULO/2));

    /* Clear fault flag (we're in safe mode so the PWM won't run if there's an error condition */
	pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(1U);

	/* Start PWMs (set load OK flags and run) */
	pCurrentPwm->MCTRL |= PWM_MCTRL_CLDOK(15);
	pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(15);
	pCurrentPwm->MCTRL |= PWM_MCTRL_RUN(15);

	/* Enable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
	pCurrentPwm->OUTEN |= PWM_OUTEN_PWMA_EN(0xF);
	pCurrentPwm->OUTEN |= PWM_OUTEN_PWMB_EN(0xF);

	/* Enable & setup interrupt from PWMA */
	if( PWM0 == (pCurrentPwm) )
	{
		NVIC_SetPriority(FLEXPWM0_COMPARE0_IRQn, 0U);
		NVIC_EnableIRQ(FLEXPWM0_COMPARE0_IRQn);
	}
	else
	{
		NVIC_SetPriority(FLEXPWM1_COMPARE0_IRQn, 0U);
		NVIC_EnableIRQ(FLEXPWM1_COMPARE0_IRQn);
	}

//
//    /* Stop the counter before updating match register */
//    SCT0->CTRL_ACCESS16BIT.CTRLL |= (SCT_CTRLL_HALT_L_MASK);
//
//    SCT0->OUT[0].SET |= this->ui32OutSet0;
//    SCT0->OUT[1].SET |= this->ui32OutSet1;
//    SCT0->OUT[2].SET |= this->ui32OutSet2;
//    SCT0->OUT[3].SET |= this->ui32OutSet3;
//    SCT0->OUT[4].SET |= this->ui32OutSet4;
//    SCT0->OUT[5].SET |= this->ui32OutSet5;
//
//    SCT0->OUT[0].CLR |= this->ui32OutClr0;
//    SCT0->OUT[1].CLR |= this->ui32OutClr1;
//    SCT0->OUT[2].CLR |= this->ui32OutClr2;
//    SCT0->OUT[3].CLR |= this->ui32OutClr3;
//    SCT0->OUT[4].CLR |= this->ui32OutClr4;
//    SCT0->OUT[5].CLR |= this->ui32OutClr5;
//
//    /* Restart the counter */
//    SCT0->CTRL_ACCESS16BIT.CTRLL &= ~((uint16_t)SCT_CTRLL_HALT_L_MASK);

}

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutDis(mcdrv_eflexpwm_t *this)
{
	PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

	/* Disable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
	pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMA_EN(0xF));
	pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMB_EN(0xF));

//    /* Stop the counter before updating match register */
//    SCT0->CTRL_ACCESS16BIT.CTRLL |= (SCT_CTRLL_HALT_L_MASK);
//
//    SCT0->OUT[0].SET = 0U;
//    SCT0->OUT[1].SET = 0U;
//    SCT0->OUT[2].SET = 0U;
//    SCT0->OUT[3].SET = 0U;
//    SCT0->OUT[4].SET = 0U;
//    SCT0->OUT[5].SET = 0U;
//
//    SCT0->OUT[0].CLR = 0U;
//    SCT0->OUT[1].CLR = 0U;
//    SCT0->OUT[2].CLR = 0U;
//    SCT0->OUT[3].CLR = 0U;
//    SCT0->OUT[4].CLR = 0U;
//    SCT0->OUT[5].CLR = 0U;
//
//    SCT0->OUTPUT = 0U; // Clear all SCTimer output
//
//    /* Restart the counter */
//    SCT0->CTRL_ACCESS16BIT.CTRLL &= ~((uint16_t)SCT_CTRLL_HALT_L_MASK);

}

/*!
 * @brief Function initialite PWM outputs structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutInit(mcdrv_eflexpwm_t *this)
{
//
//    this->ui32OutSet0 = SCT0->OUT[0].SET;
//    this->ui32OutSet1 = SCT0->OUT[1].SET;
//    this->ui32OutSet2 = SCT0->OUT[2].SET;
//    this->ui32OutSet3 = SCT0->OUT[3].SET;
//    this->ui32OutSet4 = SCT0->OUT[4].SET;
//    this->ui32OutSet5 = SCT0->OUT[5].SET;
//
//    this->ui32OutClr0 = SCT0->OUT[0].CLR;
//    this->ui32OutClr1 = SCT0->OUT[1].CLR;
//    this->ui32OutClr2 = SCT0->OUT[2].CLR;
//    this->ui32OutClr3 = SCT0->OUT[3].CLR;
//    this->ui32OutClr4 = SCT0->OUT[4].CLR;
//    this->ui32OutClr5 = SCT0->OUT[5].CLR;
}

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_eFlexPwm3PhFltGet(mcdrv_eflexpwm_t *this)
{
    /* read over-current flags */
    s_statusPass = (((this->pui32PwmBaseAddress->FSTS & PWM_FSTS_FFPIN_MASK) >> 8) &
                    (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));

    /* clear faults flag */
    this->pui32PwmBaseAddress->FSTS = ((this->pui32PwmBaseAddress->FSTS & ~(uint16_t)(PWM_FSTS_FFLAG_MASK)) |
                                       (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));

    return ((s_statusPass > 0));
}

void MCDRV_eFlexPwm3PhFltTryClr(mcdrv_eflexpwm_t *this)
{
	PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

	/* We can clear the FFLAGs only if the respective FFPIN (raw fault input) isn't set. */
	const uint8_t u8FfpinNoErrorMask =
			(uint8_t)(~(((pCurrentPwm->FSTS) & PWM_FSTS_FFLAG_MASK) >> PWM_FSTS_FFLAG_SHIFT));

	pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(u8FfpinNoErrorMask);
}
