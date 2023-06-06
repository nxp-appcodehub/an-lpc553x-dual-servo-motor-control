/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "freemaster.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "fsl_gpio.h"
#include "fsl_usart.h"
#include "fsl_device_registers.h"
#include "fsl_ctimer.h"
#include "fsl_debug_console.h"
#include "freemaster_serial_miniusart.h"
#include "clock_config.h"
#include "board.h"
#include "M1_Params.h"
#include "M2_Params.h"
#include "M1_statemachine.h"
#include "M2_statemachine.h"
#include "mcdrv_enc_lpc55s36.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Three instruction added after interrupt flag clearing as required */
#define END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val

static void BOARD_Init(void);
static void BOARD_InitSysTick(void);
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define HTML_CONTROL            1  
   
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void M1_Html_Input(void);
void M2_Html_Input(void);
void User_Control(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* CPU load measurement using Systick */
uint32_t g_ui32NumberOfCycles    = 0U;
uint32_t g_ui32MaxNumberOfCycles = 0U;   

volatile        uint32_t              cnt;
volatile        uint8_t               Led_value;

volatile        bool                   g_AdcConversionDoneFlag;
volatile        uint32_t               g_AdcConversionValue;
volatile        uint32_t               g_AdcInterruptCounter;

// X-Y Axis
static			int16_t	    	i16AxisFlag = 0;
static			frac32_t	f32AxisAngle;
static			frac32_t	f32AxisSpeed;
static			frac16_t	f16AxisR;
static			frac16_t	f16AxisAngle;

// Html
static			int16_t		i16HtmlSwitch = 0;
static			int16_t		i16HtmlM1Flag = 99;
static			int16_t		i16HtmlM2Flag = 99;
static			int32_t	    	i32HtmlPosMax;
static			int32_t	    	i32HtmlPosMin;	

static			int16_t			i16UserMode = 0;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   Application main function processing peripheral function calling and
 *          infinite loop
 *
 * @param   void
 *
 * @return  none
 */
int main(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Init board hardware. */
    BOARD_Init();

    SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN(1);
    
    /* Initialize peripheral motor control driver for motor M1 */
    MCDRV_Init_M1();

    /* Initialize RTCESL PQ */
    RTCESL_PQ_Init();

    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();

    /* Turn off application */
    M1_SetAppSwitch(FALSE);
    M2_SetAppSwitch(FALSE);

    /* FreeMsater Variables Initial */
    FM_Variables_Init();  
	
    /* Enable interrupts  */
    EnableGlobalIRQ(ui32PrimaskReg);
    
    /* X-Y Axis */
    i16AxisFlag 		= 0;
    f32AxisAngle		= 0;
    f32AxisSpeed		= FRAC32(0.3L /M1_MC_SLOW_CONTROL_LOOP_FREQ); 
    f16AxisR			= FRAC16(90.0 /M1_POS_BASE); 
    /* Html */
    i16HtmlSwitch		= 0;
    i16HtmlM1Flag		= 99;
    i16HtmlM2Flag 	        = 99;
    i32HtmlPosMax		= ACC32(90.0L /M1_POS_BASE);
    i32HtmlPosMin		= ACC32(-90.0L /M1_POS_BASE);
    
    /* User Control */
    i16UserMode		        = 0;
    
    /* Infinite loop */
    while (1)
    {
        /* FreeMASTER Polling function */
        FMSTR_Poll();
    }
}

/*!
 *@brief      Initialization of the Clocks and Pins
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_Init(void)
{
    /* Initialize clock configuration */
    BOARD_InitBootClocks();
    /* Init pins set in pin_mux file */
    BOARD_InitBootPins();
    /* Init peripherals set in peripherals file */
    BOARD_InitBootPeripherals();
}

/*!
 * @brief   Slow loop interrupt handler (1ms period)
 *           - motor M1 slow application machine function
 *
 * @param   void
 *
 * @return  none
 */
void CTIMER0_IRQHandler(void)
{
    /* X-Y Axis */
    if(1 == i16AxisFlag)
    {
        f32AxisAngle += f32AxisSpeed;
        f16AxisAngle = MLIB_Conv_F16l(f32AxisAngle);
        gsM1_Drive.SPosCtrl.i32PosDesired = MLIB_Mul_F32ss(f16AxisR, GFLIB_Sin_F16(f16AxisAngle)) >> 16;
        gsM2_Drive.SPosCtrl.i32PosDesired = MLIB_Mul_F32ss(f16AxisR, GFLIB_Cos_F16(f16AxisAngle)) >> 16;
    }        
        
#if	HTML_CONTROL == 1
	M1_Html_Input();
#endif
    
    /* M1 Slow StateMachine call */
    SM_StateMachineSlow(&gsM1_Ctrl);
    
    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    /* Clear the match interrupt flag. */
    CTIMER0->IR |= CTIMER_IR_MR0INT(1U);

    /* Add empty instructions for correct interrupt flag clearing */
    END_OF_ISR;
}
/*!
 * @brief   Slow loop interrupt handler (1ms period)
 *           - motor M2 slow application machine function
 *
 * @param   void
 *
 * @return  none
 */
void CTIMER1_IRQHandler(void)
{
#if	HTML_CONTROL == 1
	M2_Html_Input();
#endif
    
    /* M2 Slow StateMachine call */
    SM_StateMachineSlow(&gsM2_Ctrl);

    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    /* Clear the match interrupt flag. */
    CTIMER1->IR |= CTIMER_IR_MR0INT(1U);

    /* Add empty instructions for correct interrupt flag clearing */
    END_OF_ISR;
}

void FLEXPWM0_COMPARE0_IRQHandler(void)
{
	/* Clear the VAL5 compare flag */
    PWM0->SM[0].STS |= PWM_STS_CMPF(1U << 5);
    PWM0->SM[0].INTEN &= ~PWM_INTEN_CMPIE(1U << 5);

    /* clear and enable PWM0_SM0_VAL4 trigger */
    PWM0->SM[1].STS = PWM_STS_CMPF(1U << 4);
    PWM0->SM[1].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1U << 4);
    
    /* enable PWM1_SM0_VAL4 trigger */
    PWM1->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1U << 4);
    
    /* Add empty instructions for correct interrupt flag clearing */
    END_OF_ISR;
}

/*!
 * @brief   ADC ISR called with 100us period processes
 *           - motor M1 fast application machine function
 *           - demo mode if enabled
 *
 * @param   void
 *
 * @return  none
 */
void ADC0_IRQHandler(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Start CPU tick number couting */
    SYSTICK_START_COUNT();
    LED_GREEN_ON();
    
    /* StateMachine call */
    SM_StateMachineFast(&gsM1_Ctrl);

    /* html */
#if	HTML_CONTROL == 1
    User_Control();
#endif
    
    /* Stop CPU tick number couting and store actual and maximum ticks */
    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles);
    g_ui32MaxNumberOfCycles =
        g_ui32NumberOfCycles > g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;

    /* Enable interrupts  */
    EnableGlobalIRQ(ui32PrimaskReg);
    LED_GREEN_OFF();
    
    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    /* Clear the TCOMP INT flag */
    ADC0->STAT |= (uint32_t)(1U << 9);

    /* Add empty instructions for correct interrupt flag clearing */
    END_OF_ISR;
}

/*!
 * @brief   ADC ISR called with 100us period processes
 *           - motor M1 fast application machine function
 *           - demo mode if enabled
 *
 * @param   void
 *
 * @return  none
 */
void ADC1_IRQHandler(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    LED_GREEN_ON();
    /* StateMachine call */
    SM_StateMachineFast(&gsM2_Ctrl);

    /* Enable interrupts  */
    EnableGlobalIRQ(ui32PrimaskReg);
    LED_GREEN_OFF();
    
    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    /* Clear the TCOMP INT flag */
    ADC1->STAT |= (uint32_t)(1U << 9);

    /* Add empty instructions for correct interrupt flag clearing */
    END_OF_ISR;
}

void User_Control(void)
{
    if(i16UserMode != 2)
    {
        if(0 == i16HtmlSwitch)
        {
            i16HtmlSwitch = 2;
            mbM1_SwitchAppOnOff = 0;
            mbM2_SwitchAppOnOff = 0;
            i16UserMode = 0;
        }
        else if(1 == i16HtmlSwitch)
        {
            i16HtmlSwitch = 3;
            mbM1_SwitchAppOnOff = 1;
            mbM2_SwitchAppOnOff = 1;
            i16UserMode = 1;
        }
        
        if(mbM1_SwitchAppOnOff && mbM2_SwitchAppOnOff)
        {
        }
        else
        {
            mbM1_SwitchAppOnOff = 0;
            mbM2_SwitchAppOnOff = 0;
            i16AxisFlag = 0;
            
            gsM1_Drive.SPosCtrl.i32PosDesired = 0;
            gsM2_Drive.SPosCtrl.i32PosDesired = 0;
            f16AxisAngle = 0;
        }
    }
}

void M1_Html_Input(void)
{
    int32_t i32Temp;

    i32Temp = gsM1_Drive.SPosCtrl.i32PosDesired;

    if(99 == i16HtmlM1Flag)
    {

    }
    else
    {
        switch(i16HtmlM1Flag)
        {
        case 0:
        default:
                i32Temp = 0;
                break;
                
        case 1:
                i32Temp += ACC32(0.1L/M1_POS_BASE);
                break;
                
        case -1:
                i32Temp += ACC32(-0.1L/M1_POS_BASE);
                break;
                
        case 2:
                i32Temp += ACC32(1.0L/M1_POS_BASE);
                break;
                
        case -2:
                i32Temp += ACC32(-1.0L/M1_POS_BASE);
                break;
                
        case 3:
                i32Temp += ACC32(10.0L/M1_POS_BASE);
                break;
                
        case -3:
                i32Temp += ACC32(-10.0L/M1_POS_BASE);
                break;
                
        case 4:
                i32Temp = i32HtmlPosMax;
                break;
                
        case -4:
                i32Temp = i32HtmlPosMin;
                break;
        }
        
        if(i32Temp > i32HtmlPosMax)
        {
                i32Temp = i32HtmlPosMax;
        }
        else if(i32Temp < i32HtmlPosMin)
        {
                i32Temp = i32HtmlPosMin;
        }
        gsM1_Drive.SPosCtrl.i32PosDesired = i32Temp;
        
        i16HtmlM1Flag = 99;
    }
}
void M2_Html_Input(void)
{
    acc32_t i32Temp;

    i32Temp = gsM2_Drive.SPosCtrl.i32PosDesired;

    if(99 == i16HtmlM2Flag)
    {

    }
    else
    {
        switch(i16HtmlM2Flag)
        {
        case 10:
        default:
                i32Temp = 0;
                break;
                
        case 11:
                i32Temp += ACC32(0.1L/M2_POS_BASE);
                break;
                
        case -11:
                i32Temp += ACC32(-0.1L/M2_POS_BASE);
                break;
                
        case 12:
                i32Temp += ACC32(1.0L/M2_POS_BASE);
                break;
                
        case -12:
                i32Temp += ACC32(-1.0L/M2_POS_BASE);
                break;
                
        case 13:
                i32Temp += ACC32(10.0L/M2_POS_BASE);
                break;
                
        case -13:
                i32Temp += ACC32(-10.0L/M2_POS_BASE);
                break;
                
        case 14:
                i32Temp = i32HtmlPosMax;
                break;
                
        case -14:
                i32Temp = i32HtmlPosMin;
                break;
        }
        
        if(i32Temp > i32HtmlPosMax)
        {
                i32Temp = i32HtmlPosMax;
        }
        else if(i32Temp < i32HtmlPosMin)
        {
                i32Temp = i32HtmlPosMin;
        }
        gsM2_Drive.SPosCtrl.i32PosDesired = i32Temp;
        
        i16HtmlM2Flag = 99;
    }
}
void FM_Variables_Init(void)
{
    /* Variables for FreeMASTER */
    FmFlagRecorder		= 0;
    FmIBase 			= M1_FM_I_SCALE;			// mA	
    FmSpdMechRpmBase 	        = M1_FM_NMAX;	// rpm-mechanical	
    FmPosElBase			= M1_FM_POS_SCALE;			// deg
    FmPosMechBase		= M1_POS_BASE;
}
/*!
 *@brief      SysTick initialization for CPU cycle measurement
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}