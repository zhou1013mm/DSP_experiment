/*********************************************************************************

Copyright(c) 2014-2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
 * @file      adc_dac_playback.h
 * @brief     Example to demonstrate the submission of DMA driven audio buffers
 *            to both the ADC and DAC.
 * @version:  $Revision: 31528 $
 * @date:     $Date: 2017-11-22 02:18:08 -0500 (Wed, 22 Nov 2017) $
 *
 * @details
 *            This is the primary include file for ADC / DAC play-back example that shows
 *            how to to submit DMA driven audio buffers to both the ADC and DAC .
 *
 */

#ifndef __ADC_DAC_PLAYBACK_H__
#define __ADC_DAC_PLAYBACK_H__

/*=============  I N C L U D E S   =============*/

/*==============  D E F I N E S  ===============*/

#ifndef MACROS_DEFINED_ON_COMMAND_LINE
#define PLAYBACK

/* TDM mode will only work in PLAYBACK mode */
#ifdef PLAYBACK
//#define TDM_MODE
//#define TDM4
//#define TDM8
#endif
#endif

/* Number of callbacks that occur before the program exits */
#define CALLBACK_COUNT  		(20000u)

/* Macro to specify delay count for ADC/DAC reset */
#define DELAY_COUNT             (100000u)

/*
 * ADC settings
 */
/* ADAU1979 SPORT config parameters */
#define LR_B_CLK_MASTER_1979    (true)
#define BCLK_RISING_1979 	    (true)
#ifdef TDM_MODE
#define LRCLK_HI_LO_1979 	    (false)
#else
#define LRCLK_HI_LO_1979 	    (true)
#endif
/*
 * DAC settings
 */
/* DAC Master clock frequency */
#define ADAU1962A_MCLK_IN       (24576000u)
/* DAC sample rate */
#define SAMPLE_RATE   			(48000u)

/* ADAU1962A SPORT config parameters */
#define LR_B_CLK_MASTER_1962    (true)
#define BCLK_RISING_1962 	    (true)

#ifdef TDM_MODE
#define LRCLK_HI_LO_1962        (false)
#else
#define LRCLK_HI_LO_1962        (true)
#endif
/* Sine wave parameters */
#define REFERENCE_FREQ 				(3000u)
#define SAMPLES_PER_PERIOD 			(SAMPLE_RATE) / (REFERENCE_FREQ)
#define AMPLITUDE					((float)3388607)
#define PI							((float)3.141592765309)
#define SAMPLE_SIZE 				(4u)

#define NUM_CHANNELS				(4u)

/* Macro to set buffer size */
#define AUDIO_BUFFER_SIZE 	        (SAMPLES_PER_PERIOD * NUM_CHANNELS * SAMPLE_SIZE * 4u)


/* correlation parameters */
#define CORRELATION_SIZE            (SAMPLES_PER_PERIOD) * (2u)
#define CORRELATION_NORM_PEAK		(0x19000000u)
#define CORRELATION_NORM_TOLERANCE	(0x02000000u)
#define CORRELATION_SCALING_FACTOR1 (0x8u)
#define CORRELATION_SCALING_FACTOR2 (0x0u)

/* SPU Peripheral ID */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
#define	SPORT_4A_SPU_PID		    (23u)
#define	SPORT_4B_SPU_PID		    (24u)
#define	SPORT_4A_DMA10_SPU_PID		(74u)
#define	SPORT_4B_DMA11_SPU_PID		(75u)
#elif defined(__ADSPSC573_FAMILY__)
#define	SPORT_2A_SPU_PID		    (15u)
#define	SPORT_2B_SPU_PID		    (16u)
#define	SPORT_2A_DMA4_SPU_PID		(52u)
#define	SPORT_2B_DMA5_SPU_PID		(53u)
#else
#error "processor not supported"
#endif

#define PIN_MASK (0x0000007Fu)
#define PIN_LEN  (7u)

#define DATA_MASK (0x0000003Fu)
#define DATA_LEN  (6u)

#define CLK_MASK (0x0000001Fu)
#define CLK_LEN  (5u)

#define FS_MASK (0x0000001Fu)
#define FS_LEN  (5u)

#define PE_MASK (0x0000003Fu)
#define PE_LEN  (6u)

/* IF (Debug info enabled) */
#if defined(ENABLE_DEBUG_INFO)
#define DBG_MSG                     printf
#else
#define DBG_MSG(...)
#endif

#endif /* __ADC_DAC_PLAYBACK_H__ */
