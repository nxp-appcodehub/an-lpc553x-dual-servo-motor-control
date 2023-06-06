/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef M2_PARAMS_H_					
#define M2_PARAMS_H_					
					
/* Motor basic parameters */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	M2_STATOR_R		    0.58	     /* [ohm] */	
#define	M2_LD_INDUCTANCE	0.00032	    /* [Henry] */	
#define	M2_LQ_INDUCTANCE	0.00032	    /* [Henry] */	
#define	M2_POLE_PAIRS		4	         /* [pairs] */	
					
/* Based value */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	M2_U_DCB_MAX	    	(60.8)	                  /* [V] */	
#define	M2_U_FOC_MAX		    (M2_U_DCB_MAX/1.732F)	  /* [V] */
#define M2_U_DCB_OVERVOLTAGE   (30)                     /* [V] */	
#define M2_U_DCB_UNDERVOLTAGE   (18)                     /* [V] */	
#define	M2_I_MAX		        (8.25)	                  /* [A] */	
#define	M2_E_MAX		        (12)	                      /* [V] */	
#define	M2_N_MAX		        (3200)	                  /* [RPM] */	
#define	M2_OVERVOLT_LIMIT	M2_U_DCB_OVERVOLTAGE	
#define	M2_UNDERVOLT_LIMIT	M2_U_DCB_UNDERVOLTAGE
#define	M2_DUTY_CYCLE_LIMIT		0.9	              /* [ ] */	
					
/* Time scaling */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define 	M2_MC_PWM_CLK_FREQ		150000000	              /* [Hz], PWM clock frequency */	
#define 	M2_CONTROL_FREQ		    16000	                  /* [Hz], PWM frequency */	
#define 	M2_MC_SLOW_CONTROL_LOOP_FREQ		2000    	  /* [Hz], slow loop control frequency */
#define 	M2_SPEED_LOOP_CNTR		(M2_CONTROL_FREQ/M2_MC_SLOW_CONTROL_LOOP_FREQ)		
					
/*Speed  Loop */		
#define M2_Spd_Ramp                  8000                /* [RPM/s], mechanical speed accelerating rate during closed-loop */
#define M2_SPEED_LOOP_LIMIT        (2.0)                    /* [A], current output limitation */
#define M2_SPEED_PI_PROP_GAIN       (0.015)               /* proportional gain */
#define M2_SPEED_PI_INTEG_GAIN      (0.0007)              /* integral gain */
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	M2_SPEED_RAMP           	(M2_Spd_Ramp/M2_MC_SLOW_CONTROL_LOOP_FREQ)

/* Positon Loop */		
#define M2_Pos_Speed_Limit_Up           12000                /* [RPM/s], position ramping rate for a position command */
#define M2_Pos_Speed_Limit_Down         12000                /* [RPM/s], position ramping rate for a position command */
#define M2_Pos_Ctrl_AW_Limit            1500.0                /* [RPM], mechanical speed - position controller output upper limit */
#define	M2_POS_CTRL_PROP_GAIN	         ACC32(26.95)                /* proportional gain */
#define M2_POSITION_CTRL_SPEED_FWD_GAIN  ACC32(1)
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	M2_POS_BASE		            180	
#define	M2_POS_RAMP_UP	            FRAC32(M2_Pos_Speed_Limit_Up/60.0/M2_MC_SLOW_CONTROL_LOOP_FREQ)
#define	M2_POS_RAMP_DOWN	        FRAC32(M2_Pos_Speed_Limit_Down/60.0/M2_MC_SLOW_CONTROL_LOOP_FREQ)
#define M2_SPEED_MECH_TO_ElEC_COEFF (float)(2*PI*M2_POLE_PAIRS)

/* Freemaster Variables */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	M2_FM_I_SCALE		8250	 /* [mA] */	
#define	M2_FM_NMAX	    	3000	 /* [RPM] */	
#define	M2_FM_POS_SCALE		180	     /* */		

/*ACR parameter*/		
#define M2_CLOOP_ATT			(0.85F) 	/* Attenuation */
#define M2_CLOOP_FREQ			(1000.0F)	/* [Hz], Current loop bandwidth frequency */
#define M2_CLOOP_LIMIT         (0.9F)     /* Voltage output limitation, based on real time available maximum phase voltage amplitude */
//------------------------------------------------------------------------------------------------------------------------------------------					
/*Damping Coefficient_D&Q_ACR=	0.85	*/			
/*bandWidth=	1000	[Hz]*/		
#define	M2_D_KP_GAIN	    (2.0*M2_CLOOP_ATT*2*PI*M2_CLOOP_FREQ*M2_LD_INDUCTANCE - M2_STATOR_R)
#define	M2_D_KI_GAIN	    (2.0*PI*M2_CLOOP_FREQ*2*PI*M2_CLOOP_FREQ*M2_LD_INDUCTANCE/M2_CONTROL_FREQ)
#define	M2_D_LIMIT	    	M2_CLOOP_LIMIT*M2_U_FOC_MAX
				
#define	M2_Q_KP_GAIN	    (2.0*M2_CLOOP_ATT*2*PI*M2_CLOOP_FREQ*M2_LQ_INDUCTANCE - M2_STATOR_R)
#define	M2_Q_KI_GAIN	    (2.0*PI*M2_CLOOP_FREQ*2*PI*M2_CLOOP_FREQ*M2_LQ_INDUCTANCE/M2_CONTROL_FREQ)
#define	M2_Q_LIMIT		    M2_CLOOP_LIMIT	*M2_U_FOC_MAX

		
//M2 ENC
#define MOTOR_2                            2           /* Motor number */
#define M2_ENCODER_LINES                   1000        /* Motor encoder line numbers */
#define M2_ENC_TIMER_PRESCALER				6 			 /* Prescaler for the timer within ENC, the prescaling value is 2^Mx_ENC_TIMER_PRESCALER */
#define M2_ENC_CLOCK	 					M2_MC_PWM_CLK_FREQ	 /* [Hz], ENC module clock, which is the bus clock of the system */
#define M2_ENC_SPEED_FILTER_CUTOFF_FREQ 	100.0 		 /* [Hz], cutoff frequency of IIR1 low pass filter for calculated raw speed out of ENC HW feature */
#define M2_ENC_TO_ATT						(0.85F)		 /* attenuation for tracking observer, which is to estimate rotor speed from real ENC position */
#define M2_ENC_TO_FREQ						(300.0)		 /* [Hz], oscillating frequency for tracking observer */
#define M2_ENC_TRAJECTORY_FILTER_FREQ		10.0 		 /* [Hz], cutoff frequency of a 2nd order IIR filter to get a smoothed position reference */
//------------------------------------------------------------------------------------------------------------------------------------------			
#define M2_ENC_TIMER_FREQUENCY  			(M2_ENC_CLOCK/EXPON(2,M2_ENC_TIMER_PRESCALER))//(M2_ENC_TIMER_PRESCALER)) 			/* [Hz], the clock frequency for the timer within ENC */
#define M2_SPEED_CAL_CONST  				((60.0*M2_ENC_TIMER_FREQUENCY/(4*M2_ENCODER_LINES*M2_N_MAX)) * 0x8000000)	/* A constant to calculate mechanical speed out of ENC HW feature, Q5.27 */
#define M2_ENC_TO_KP_GAIN					(2.0F*M2_ENC_TO_ATT*2*PI*M2_ENC_TO_FREQ)
#define M2_ENC_TO_KI_GAIN					(2*PI*M2_ENC_TO_FREQ * 2*PI*M2_ENC_TO_FREQ/1000)
#define M2_ENC_TO_THETA_GAIN				(1.0F/(PI*1000))

#define M2_ENC_SPEED_FILTER_IIR_B0				   	   WARP(M2_ENC_SPEED_FILTER_CUTOFF_FREQ, M2_MC_SLOW_CONTROL_LOOP_FREQ)/(WARP(M2_ENC_SPEED_FILTER_CUTOFF_FREQ, M2_MC_SLOW_CONTROL_LOOP_FREQ) + 2.0F)
#define M2_ENC_SPEED_FILTER_IIR_B1				       WARP(M2_ENC_SPEED_FILTER_CUTOFF_FREQ, M2_MC_SLOW_CONTROL_LOOP_FREQ)/(WARP(M2_ENC_SPEED_FILTER_CUTOFF_FREQ, M2_MC_SLOW_CONTROL_LOOP_FREQ) + 2.0F)
#define M2_ENC_SPEED_FILTER_IIR_A1				       (1.0F - M2_ENC_SPEED_FILTER_IIR_B0 - M2_ENC_SPEED_FILTER_IIR_B1)
#define M2_ENC_SPEED_FILTER_IIR_B0_FRAC				   FRAC32(M2_ENC_SPEED_FILTER_IIR_B0/2)
#define M2_ENC_SPEED_FILTER_IIR_B1_FRAC				   FRAC32(M2_ENC_SPEED_FILTER_IIR_B1/2)
#define M2_ENC_SPEED_FILTER_IIR_A1_FRAC				   FRAC32(M2_ENC_SPEED_FILTER_IIR_A1/2)
#define M2_SPEED_FRAC_TO_ANGULAR_COEFF                (float_t)(2*PI*M2_N_MAX*M2_POLE_PAIRS/60.0)
#define M2_ENC_TRAJECTORY_FILTER_FREQ_FRAC              FRAC32(2*PI*M2_ENC_TRAJECTORY_FILTER_FREQ/M2_SLOW_LOOP_FREQ)

/* Filters */	
#define M2_UDCBUS_FILTER_CUTOFF_FREQ     100     /* [Hz] */
//------------------------------------------------------------------------------------------------------------------------------------------					
// Udc bus IIR, cutoff freq = 	100	[Hz]	Ts =	0.0000625	[s]
#define	M2_FILTER_UDCBUS_B0		WARP(M2_UDCBUS_FILTER_CUTOFF_FREQ, M2_CONTROL_FREQ)/(WARP(M2_UDCBUS_FILTER_CUTOFF_FREQ, M2_CONTROL_FREQ) + 2.0F)		
#define	M2_FILTER_UDCBUS_B1		WARP(M2_UDCBUS_FILTER_CUTOFF_FREQ, M2_CONTROL_FREQ)/(WARP(M2_UDCBUS_FILTER_CUTOFF_FREQ, M2_CONTROL_FREQ) + 2.0F)	
#define	M2_FILTER_UDCBUS_A1		(1.0F - M2_FILTER_UDCBUS_B0 - M2_FILTER_UDCBUS_B1)			
					
#endif /* M2_PARAMS_H_ */					
