/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "peripherals.h"
#include "fsl_common.h"

#include "fsl_dac.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_power.h"
#include "fsl_ctimer.h"
#include "mcdrv_enc_lpc55s36.h"
#include "M1_Params.h"
#include "M2_Params.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedef
 ******************************************************************************/
typedef struct
{
    LPDAC_Type * const PtoDac;
    const uint32_t InitDacCount;
    const clock_attach_id_t ClockAttachId;
    const clock_div_name_t ClockDivName;
    const pd_bit_t PdVectorBit;
} t_DacInitData;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void InitADC0(void);
static void InitADC1(void);
static void InitHsCmp0(void);
static void InitHsCmp1(void);
static void InitDac( const t_DacInitData * const pInitData );
static void M1_InitSlowLoop(void);
static void M2_InitSlowLoop(void);
static void InitPWM0(void);
static void InitPWM1(void);
static void InitENC0(void);
static void InitENC1(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static const t_DacInitData Dac0InitData = { DAC0, FLT_DCB_MAX_CURENT_TO_DAC_COUNT(5.0F), kMAIN_CLK_to_DAC0, kCLOCK_DivDac0Clk, kPDRUNCFG_PD_DAC0 };
static const t_DacInitData Dac1InitData = { DAC1, FLT_DCB_MAX_CURENT_TO_DAC_COUNT(5.0F), kMAIN_CLK_to_DAC1, kCLOCK_DivDac1Clk, kPDRUNCFG_PD_DAC1 };
/* configuration structure for 3-phase PWM mc driver */
mcdrv_eflexpwm_t g_sM1Pwm3ph, g_sM2Pwm3ph;

/* structure for current and voltage measurement*/
mcdrv_adc_t g_sM1AdcSensor, g_sM2AdcSensor;
enc_block_t g_sM1EncSensor, g_sM2EncSensor;
/*******************************************************************************
 * Local functions
 ******************************************************************************/
/*!
 * @brief   void InitPWM(void)
 *           - Initialization of the eFlexPWM0 peripheral for motor M1
 *           - 3-phase center-aligned PWM
 *
 * @param   void
 *
 * @return  none
 */
static void InitPWM0(void)
{
    PWM_Type *PWMBase = (PWM_Type *)PWM0;

    /*eFlexPWM0 init*/
    SYSCON->PWM0SUBCTL = (SYSCON_PWM0SUBCTL_CLK0_EN_MASK | SYSCON_PWM0SUBCTL_CLK1_EN_MASK | SYSCON_PWM0SUBCTL_CLK2_EN_MASK | SYSCON_PWM0SUBCTL_CLK3_EN_MASK); //Enable Sub-module0 clock
    CLOCK_EnableClock(kCLOCK_Pwm0);
    
    /* value register initial values, duty cycle 50% */
    PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
    PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
    PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
    PWMBase->SM[3].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));   

    PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
    PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
    PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));    
    PWMBase->SM[3].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));      

    // TODO - TEST, init to 0 otherwise (see above).
    PWMBase->SM[0].VAL2 = (uint16_t)(-(M1_PWM_MODULO/4));
    PWMBase->SM[1].VAL2 = (uint16_t)(-(M1_PWM_MODULO/4));
    PWMBase->SM[2].VAL2 = (uint16_t)(-(M1_PWM_MODULO/4));
    PWMBase->SM[3].VAL2 = (uint16_t)(-(M1_PWM_MODULO/4));

    PWMBase->SM[0].VAL3 = (uint16_t)((M1_PWM_MODULO/4));
    PWMBase->SM[1].VAL3 = (uint16_t)((M1_PWM_MODULO/4));
    PWMBase->SM[2].VAL3 = (uint16_t)((M1_PWM_MODULO/4));
    PWMBase->SM[3].VAL3 = (uint16_t)((M1_PWM_MODULO/4));
    
    PWMBase->SM[0].VAL5 = (uint16_t)((M1_PWM_MODULO/3));

    /* PWM0 module 0 trigger on VAL4 enabled for ADC synchronization */
    PWMBase->SM[1].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(M1_PWM_MODULO / 2))));

    PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1);
    PWMBase->SM[0].INTEN |= PWM_INTEN_CMPIE(1U << 5);

    /* set deadtime (number of Fast Peripheral Clocks)
       DTCNT0,1 = T_dead * f_fpc = 1.5us * 72MHz = 108 */
    /* DTCNTx = 95 if the clock is 95977472 Hz and deadtime = 1 us */    
    PWMBase->SM[0].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[1].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[2].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[3].DTCNT0 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[0].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[1].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[2].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[3].DTCNT1 = ((M1_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    
     /* Half cycle reload */
    PWMBase->SM[0].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[1].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[2].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[3].CTRL = PWM_CTRL_HALF(1);

    /* Fault0 (HSCMP0_OUT) trigger */
    PWMBase->SM[0].DISMAP[0] = 0xF111U;
    PWMBase->SM[1].DISMAP[0] = 0xF111U;
    PWMBase->SM[2].DISMAP[0] = 0xF111U;
    PWMBase->SM[3].DISMAP[0] = 0xF111U;
    
    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);

    /* PWM fault filter - 3 Fast periph. clocks sample rate, 5 agreeing
       samples to activate */
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2);

    /* All interrupts disabled, safe manual fault clearing, inversed logic (trigger level = high) */
    PWMBase->FCTRL &= ~(PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK); /* clear FCTRL register prior further settings */
    PWMBase->FCTRL |= PWM_FCTRL_FIE(0U); /* FAULT 0 & FAULT 1 - Interrupt disable */
    PWMBase->FCTRL |= PWM_FCTRL_FLVL(0x1U);
    PWMBase->FCTRL |= PWM_FCTRL_FAUTO(0U);
    PWMBase->FCTRL |= PWM_FCTRL_FSAFE(0xFU);

    /* Clear all fault flags */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);
    
    /* Start PWMs (set load OK flags and run - we need to trigger the ADC) */
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);

    /* eFlexPWM base address */
    g_sM1Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;
    
    /* Enable & setup interrupt from PWMA */
    NVIC_SetPriority(FLEXPWM0_COMPARE0_IRQn, 0U);
    NVIC_EnableIRQ(FLEXPWM0_COMPARE0_IRQn);
}

static void InitPWM1(void)
{
    PWM_Type *PWMBase = (PWM_Type *)PWM1;

    /*eFlexPWM1 init*/
    SYSCON->PWM1SUBCTL = (SYSCON_PWM1SUBCTL_CLK0_EN_MASK | SYSCON_PWM1SUBCTL_CLK1_EN_MASK | SYSCON_PWM1SUBCTL_CLK2_EN_MASK | SYSCON_PWM1SUBCTL_CLK3_EN_MASK); //Enable Sub-module0 clock
    CLOCK_EnableClock(kCLOCK_Pwm1);
    
    /* value register initial values, duty cycle 50% */
    PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(M2_PWM_MODULO / 2)));
    PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(M2_PWM_MODULO / 2)));
    PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(M2_PWM_MODULO / 2)));
    PWMBase->SM[3].INIT = PWM_INIT_INIT((uint16_t)(-(M2_PWM_MODULO / 2)));   

    PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((M2_PWM_MODULO / 2) - 1));
    PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((M2_PWM_MODULO / 2) - 1));
    PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((M2_PWM_MODULO / 2) - 1));    
    PWMBase->SM[3].VAL1 = PWM_VAL1_VAL1((uint16_t)((M2_PWM_MODULO / 2) - 1));      

    // TODO - TEST, init to 0 otherwise (see above).
    PWMBase->SM[0].VAL2 = (uint16_t)(-(M2_PWM_MODULO/4));
    PWMBase->SM[1].VAL2 = (uint16_t)(-(M2_PWM_MODULO/4));
    PWMBase->SM[2].VAL2 = (uint16_t)(-(M2_PWM_MODULO/4));
    PWMBase->SM[3].VAL2 = (uint16_t)(-(M2_PWM_MODULO/4));

    PWMBase->SM[0].VAL3 = (uint16_t)((M2_PWM_MODULO/4));
    PWMBase->SM[1].VAL3 = (uint16_t)((M2_PWM_MODULO/4));
    PWMBase->SM[2].VAL3 = (uint16_t)((M2_PWM_MODULO/4));
    PWMBase->SM[3].VAL3 = (uint16_t)((M2_PWM_MODULO/4));

    /* PWM1 module 0 trigger on VAL4 enabled for ADC synchronization */
    PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(M2_PWM_MODULO / 2))));;

    /* set deadtime (number of Fast Peripheral Clocks)
       DTCNT0,1 = T_dead * f_fpc = 1.5us * 72MHz = 108 */
    /* DTCNTx = 95 if the clock is 95977472 Hz and deadtime = 1 us */    
    PWMBase->SM[0].DTCNT0 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[1].DTCNT0 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[2].DTCNT0 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[3].DTCNT0 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[0].DTCNT1 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[1].DTCNT1 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[2].DTCNT1 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    PWMBase->SM[3].DTCNT1 = ((M2_PWM_DEADTIME * (MCU_CLOCK_FREQ / 1000000U)) / 1000U);
    
     /* Half cycle reload */
    PWMBase->SM[0].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[1].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[2].CTRL = PWM_CTRL_HALF(1);
    PWMBase->SM[3].CTRL = PWM_CTRL_HALF(1);

    /* Fault0 (HSCMP0_OUT) trigger */
    PWMBase->SM[0].DISMAP[0] = 0xF111U;
    PWMBase->SM[1].DISMAP[0] = 0xF111U;
    PWMBase->SM[2].DISMAP[0] = 0xF111U;
    PWMBase->SM[3].DISMAP[0] = 0xF111U;
    
    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);

    /* PWM fault filter - 3 Fast periph. clocks sample rate, 5 agreeing
       samples to activate */
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2);

    /* All interrupts disabled, safe manual fault clearing, inversed logic (trigger level = high) */

    PWMBase->FCTRL &= ~(PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK); /* clear FCTRL register prior further settings */
    PWMBase->FCTRL |= PWM_FCTRL_FIE(0U); /* FAULT 0 & FAULT 1 - Interrupt disable */
    PWMBase->FCTRL |= PWM_FCTRL_FLVL(0x1U);
    PWMBase->FCTRL |= PWM_FCTRL_FAUTO(0U);
    PWMBase->FCTRL |= PWM_FCTRL_FSAFE(0xFU);

    /* Clear all fault flags */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);
    
    /* Start PWMs (set load OK flags and run - we need to trigger the ADC) */
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);

    /* eFlexPWM base address */
    g_sM2Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;
}

/*!
 * @brief   void M1_InitSlowLoop(void)
 *           - Initialization of the CTIMER0 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
static void M1_InitSlowLoop(void)
{
    ctimer_config_t cTimerConfig;
    ctimer_match_config_t cTimerMatchConfig;
    uint32_t ui32CTimerFreq;

    /* Use 96 MHz clock for some of the Ctimer0 */
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);
    /* Get defaut configuration */
    CTIMER_GetDefaultConfig(&cTimerConfig);
    /* Init timer */
    CTIMER_Init(CTIMER0, &cTimerConfig);

    /* Get CTimer0 frequency for correct set Match register value */
    ui32CTimerFreq = CLOCK_GetFreq(kCLOCK_FroHf);
    cTimerMatchConfig.enableCounterReset = true;
    cTimerMatchConfig.enableCounterStop  = false;
    cTimerMatchConfig.matchValue         = (uint32_t)(ui32CTimerFreq / M1_SLOW_LOOP_FREQ);
    cTimerMatchConfig.outControl         = kCTIMER_Output_NoAction;
    cTimerMatchConfig.outPinInitState    = false;
    cTimerMatchConfig.enableInterrupt    = true;
    
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &cTimerMatchConfig);
    CTIMER_StartTimer(CTIMER0);
    
    NVIC_SetPriority(CTIMER0_IRQn, 2);
}
/*!
 * @brief   void M2_InitSlowLoop(void)
 *           - Initialization of the CTIMER0 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
static void M2_InitSlowLoop(void)
{
    ctimer_config_t cTimerConfig;
    ctimer_match_config_t cTimerMatchConfig;
    uint32_t ui32CTimerFreq;
    
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 0U, true);            /*!< Reset CTIMER0CLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 1U, false);           /*!< Set CTIMER0CLKDIV divider to value 1 */

    /* Use 96 MHz clock for some of the Ctimr1 */
    CLOCK_AttachClk(kFRO_HF_to_CTIMER1);
    /* Get defaut configuration */
    CTIMER_GetDefaultConfig(&cTimerConfig);
    /* Init timer */
    CTIMER_Init(CTIMER1, &cTimerConfig);
    
    /* Get CTimer1 frequency for correct set Match register value */
    ui32CTimerFreq = CLOCK_GetFreq(kCLOCK_FroHf);

    cTimerMatchConfig.enableCounterReset = true;
    cTimerMatchConfig.enableCounterStop  = false;
    cTimerMatchConfig.matchValue         = (uint32_t)(ui32CTimerFreq / M2_SLOW_LOOP_FREQ);
    cTimerMatchConfig.outControl         = kCTIMER_Output_NoAction;
    cTimerMatchConfig.outPinInitState    = false;
    cTimerMatchConfig.enableInterrupt    = true;
    
    CTIMER_SetupMatch(CTIMER1, kCTIMER_Match_0, &cTimerMatchConfig);
    CTIMER_StartTimer(CTIMER1);
    
    NVIC_SetPriority(CTIMER1_IRQn, 2);
}
/*!
 * @brief   void InitADC0(void)
 *           - Initialization of the ADC0 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
static void InitADC0(void)
{
    lpadc_config_t lpadcConfig;

    /* Disable VREF power down */
    POWER_DisablePD(kPDRUNCFG_PD_VREF);

    /* Init the lpadcConfig */
    LPADC_GetDefaultConfig(&lpadcConfig);
    lpadcConfig.enableAnalogPreliminary = true;
    lpadcConfig.powerLevelMode = kLPADC_PowerLevelAlt4;
    lpadcConfig.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3;
    lpadcConfig.conversionAverageMode = kLPADC_ConversionAverage128;
    
    /* Init ADC */
    lpadc_conv_trigger_config_t lpadcTriggerConfig;
    lpadc_conv_command_config_t lpadcCommandConfig;

    LPADC_Init(ADC0, &lpadcConfig);

    LPADC_DoOffsetCalibration(ADC0);
    LPADC_DoAutoCalibration(ADC0);

    /* Init commands */
    /* Set conversion CMD1 configuration. */
    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = 3U;                                      /* Set ADC channel ADC0IN3A (CUR_A) */
    lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;/* Dual single end sample for CUR_B */
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    lpadcCommandConfig.chainedNextCommandNumber = 2U;                           /* Next execuited CMD will be CMD2 */
    LPADC_SetConvCommandConfig( ADC0, 1U, &lpadcCommandConfig );                /* Configure the CMD 1 */   
    ADC0->CMD[0].CMDL |= ADC_CMDL_ALTBEN(1);

    /* Set conversion CMD2 configuration. */
    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = 1U;                                      /* Set ADC channel ADC0IN1A (VLT_DCB) */
    lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    lpadcCommandConfig.chainedNextCommandNumber = 0U;
    LPADC_SetConvCommandConfig( ADC0, 2U, &lpadcCommandConfig );                /* Configure the CMD 4 */
    
    /* Init triggers (use trigger 0). */
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
    lpadcTriggerConfig.targetCommandId = 1U;
    lpadcTriggerConfig.enableHardwareTrigger = true; 
    lpadcTriggerConfig.channelAFIFOSelect = 0U; //Channels A store to FIFO0
    lpadcTriggerConfig.channelBFIFOSelect = 1U; //Channels B store to FIFO1
    LPADC_SetConvTriggerConfig(ADC0, 0U, &lpadcTriggerConfig);
    
    /* ADC base address */
    g_sM1AdcSensor.pToAdcBase = (ADC_Type *)ADC0;
    
    /* Enable TCOMP interrupt. */
    LPADC_EnableInterrupts(ADC0, ADC_IE_TCOMP_IE(0xFU));
    NVIC_SetPriority(ADC0_IRQn, 1);
    NVIC_EnableIRQ(ADC0_IRQn);
}
/*!
 * @brief   void InitADC1(void)
 *           - Initialization of the ADC1 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
static void InitADC1(void)
{
    lpadc_config_t lpadcConfig;

    /* Disable VREF power down */
    POWER_DisablePD(kPDRUNCFG_PD_VREF);

    /* Init the lpadcConfig */
    LPADC_GetDefaultConfig(&lpadcConfig);
    lpadcConfig.enableAnalogPreliminary = true;
    lpadcConfig.powerLevelMode = kLPADC_PowerLevelAlt4;
    lpadcConfig.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3;
    lpadcConfig.conversionAverageMode = kLPADC_ConversionAverage128;
    
    /* Init ADC */
    lpadc_conv_trigger_config_t lpadcTriggerConfig;
    lpadc_conv_command_config_t lpadcCommandConfig;

    LPADC_Init(ADC1, &lpadcConfig);

    LPADC_DoOffsetCalibration(ADC1);
    LPADC_DoAutoCalibration(ADC1);

    /* Init commands */
    /* Set conversion CMD1 configuration. */
    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = 3U;                                      /* Set ADC channel ADC1IN3A (CUR_A) */
    lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;/* Dual single end sample for CUR_B */
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    lpadcCommandConfig.chainedNextCommandNumber = 2U;                           /* Next execuited CMD will be CMD2 */
    LPADC_SetConvCommandConfig( ADC1, 1U, &lpadcCommandConfig );                /* Configure the CMD 1 */
    ADC1->CMD[0].CMDL |= ADC_CMDL_ALTBEN(1);
      
    /* Set conversion CMD2 configuration. */
    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);
    lpadcCommandConfig.channelNumber = 1U;                                      /* Set ADC channel ADC1IN1A (VLT_DCB) */
    lpadcCommandConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    lpadcCommandConfig.chainedNextCommandNumber = 0U;
    LPADC_SetConvCommandConfig( ADC1, 2U, &lpadcCommandConfig );                /* Configure the CMD 4 */
    
    /* Init triggers (use trigger 0). */
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
    lpadcTriggerConfig.targetCommandId = 1U;
    lpadcTriggerConfig.enableHardwareTrigger = true; 
    lpadcTriggerConfig.channelAFIFOSelect = 0U; //Channels A store to FIFO0
    lpadcTriggerConfig.channelBFIFOSelect = 1U; //Channels B store to FIFO1
    LPADC_SetConvTriggerConfig(ADC1, 0U, &lpadcTriggerConfig);
    
    /* ADC base address */
    g_sM2AdcSensor.pToAdcBase = (ADC_Type *)ADC1;
    
    /* Enable TCOMP interrupt. */
    LPADC_EnableInterrupts(ADC1, ADC_IE_TCOMP_IE(0xFU));
    NVIC_SetPriority(ADC1_IRQn, 1);
    NVIC_EnableIRQ(ADC1_IRQn);
}
/*!
@brief   void InitCMP(void)
          - Initialization of the comparator 0 module for dc-bus over current
            detection to generate eFlexPWM0 fault

@param   void

@return  none
*/
static void InitHsCmp0(void)
{
    POWER_DisablePD(kPDRUNCFG_PD_HSCMP0);
    CLOCK_EnableClock(kCLOCK_Hscmp0);

    /* Input minus = External DAC, input plus = analog mux in3, high power/high speed mode */
    HSCMP0->CCR2 |= (uint32_t)(HSCMP_CCR2_MSEL(5U) | HSCMP_CCR2_PSEL(3U) | HSCMP_CCR2_CMP_HPMD_MASK);
 
    /* HSCMP enable */
    HSCMP0->CCR0 |= HSCMP_CCR0_CMP_EN_MASK;
}
/*!
@brief   void InitCMP(void)
          - Initialization of the comparator 1 module for dc-bus over current
            detection to generate eFlexPWM1 fault

@param   void

@return  none
*/
static void InitHsCmp1(void)
{
    POWER_DisablePD(kPDRUNCFG_PD_HSCMP1);
    CLOCK_EnableClock(kCLOCK_Hscmp1);

    /* Input minus = External DAC, input plus = analog mux in3, high power/high speed mode */
    HSCMP1->CCR2 |= (uint32_t)(HSCMP_CCR2_MSEL(5U) | HSCMP_CCR2_PSEL(3U) | HSCMP_CCR2_CMP_HPMD_MASK);
 
    /* HSCMP enable */
    HSCMP1->CCR0 |= HSCMP_CCR0_CMP_EN_MASK;
}

/*!
@brief   void InitDac(void)
          - Initialization of the DAC module for comparator voltage reference

@param   pInitData   Structure for initialization DAC peripheral   

@return  none
*/

static void InitDac( const t_DacInitData * const pInitData )
{
    dac_config_t dacConfig;

    CLOCK_AttachClk(pInitData->ClockAttachId);
    CLOCK_SetClkDiv(pInitData->ClockDivName, 11UL, true);   /* 150 / (11+1) = 12.5 MHz */
    PMC->PDRUNCFGCLR1 = pInitData->PdVectorBit;

    DAC_GetDefaultConfig(&dacConfig);
    dacConfig.referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
    DAC_Init(pInitData->PtoDac, &dacConfig);

    DAC_Enable(pInitData->PtoDac, true);

    DAC_SetData(pInitData->PtoDac, pInitData->InitDacCount);
}

/*!
@brief   void InitENC0(void)
          - Initialization of the ENC 0 module for rotor position detection

@param   void

@return  none
*/
static void InitENC0(void)
{
    CLOCK_EnableClock(kCLOCK_Enc0);

    ENC0->LMOD = 4*M1_ENCODER_LINES-1;
    ENC0->UMOD = 0;
    ENC0->LPOS = 0;
    ENC0->UPOS = 0;
    ENC0->FILT = ENC_FILT_FILT_CNT(2)|ENC_FILT_FILT_PER(1);
    ENC0->CTRL2 |= ENC_CTRL2_MOD_MASK|ENC_CTRL2_REVMOD_MASK; // Enable modulo counting, and REV is controlled by modulo counting
    ENC0->CTRL3 = ENC_CTRL3_PMEN_MASK|ENC_CTRL3_PRSC(M1_ENC_TIMER_PRESCALER);

    g_sM1EncSensor.pENC_base = ENC0;
    g_sM1EncSensor.ui8MotorNum = MOTOR_1;
    MCDRV_EncInit(&g_sM1EncSensor);
    MCDRV_EncSpeedCalInit(&g_sM1EncSensor);
    MCDRV_EncToSpeedCalInit(&g_sM1EncSensor);
}
/*!
@brief   void InitENC1(void)
          - Initialization of the ENC 1 module for rotor position detection

@param   void

@return  none
*/
static void InitENC1(void)
{
    CLOCK_EnableClock(kCLOCK_Enc1);

    ENC1->LMOD = 4*M2_ENCODER_LINES-1;
    ENC1->UMOD = 0;
    ENC1->LPOS = 0;
    ENC1->UPOS = 0;
    ENC1->FILT = ENC_FILT_FILT_CNT(2)|ENC_FILT_FILT_PER(1);
    ENC1->CTRL2 |= ENC_CTRL2_MOD_MASK|ENC_CTRL2_REVMOD_MASK; // Enable modulo counting, and REV is controlled by modulo counting
    ENC1->CTRL3 = ENC_CTRL3_PMEN_MASK|ENC_CTRL3_PRSC(M2_ENC_TIMER_PRESCALER);

    g_sM2EncSensor.pENC_base = ENC1;
    g_sM2EncSensor.ui8MotorNum = MOTOR_2;
    MCDRV_EncInit(&g_sM2EncSensor);
    MCDRV_EncSpeedCalInit(&g_sM2EncSensor);
    MCDRV_EncToSpeedCalInit(&g_sM2EncSensor);
}
/*******************************************************************************
 * Public functions
 ******************************************************************************/
/*!
 * @brief   void MCDRV_Init_M1(void)
 *           - Motor control driver main initialization
 *           - Calls initialization functions of peripherals required for motor
 *             control functionality
 *
 * @param   void
 *
 * @return  none
 */
void MCDRV_Init_M1(void)
{
    /* Init ADC */
    InitADC0();
    InitADC1();
    
    /* Init slow loop counter*/
    M1_InitSlowLoop();
    M2_InitSlowLoop();

    InitDac(&Dac0InitData);	/* DAC0 is now used as a HSCMP0 reference source. */
    InitDac(&Dac1InitData);	/* DAC1 is now used as a HSCMP1 reference source. */
    InitHsCmp0();
    InitHsCmp1();

    /* 6-channel PWM peripheral init */
    InitPWM0();
    InitPWM1();
    
    InitENC0();
    InitENC1();
}
