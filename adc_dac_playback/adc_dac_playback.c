/*********************************************************************************

Copyright(c) 2014-2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
 * @file      adc_dac_playback.c
 * @brief     This example demonstrates the submission of DMA driven audio buffers
 *            to both the ADC and DAC.
 * @version:  $Revision: 31665 $
 * @date:     $Date: 2017-12-12 02:26:29 -0500 (Tue, 12 Dec 2017) $
 *
 * @details
 *            This is the primary source file for ADC / DAC play-back example that shows
 *            how to submit DMA driven audio buffers to both the ADC and DAC .
 *
 */

/*=============  I N C L U D E S   =============*/

#include <sys/platform.h>
/* SPU Manager includes */
#include <services/spu/adi_spu.h>
#include <services/gpio/adi_gpio.h>
#include <drivers/adc/adau1979/adi_adau1979.h>
#include <drivers/dac/adau1962a/adi_adau1962a.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "adc_dac_playback.h"
#include "adi_initialize.h"

#include <SRU.h>

/*==============  D E F I N E S  ===============*/

/*=============  D A T A  =============*/

/* Twi  */
uint32_t TwiMemory[ADI_TWI_MEMORY_SIZE];
/* Gpio */
uint32_t GpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE];
uint32_t gpioMaxCallbacks;

/* ADAU1979 ADC */
static ADI_ADAU1979_HANDLE phAdau1979;
uint32_t Adau1979Memory[ADI_ADAU1979_MEMORY_SIZE];
/* ADAU1979 Sport */
uint32_t Adau1979SportMemory[ADI_SPORT_DMA_MEMORY_SIZE];

/* ADAU1962A DAC DATA */
static ADI_ADAU1962A_HANDLE phAdau1962a;
uint32_t Adau1962aMemory[ADI_ADAU1962A_MEMORY_SIZE];
/* ADAU1962A Sport */
uint32_t Adau1962aSportMemory[ADI_SPORT_DMA_MEMORY_SIZE];

/* Counter to keep track of number of ADC buffers processed */
volatile uint32_t AdcCount = 0u;
/* Counter to keep track of number of DAC buffers processed */
volatile uint32_t DacCount = 0u;

volatile uint32_t AdcCount100 = 0u;

/* DAC buffer pointer */
volatile void *pGetADC = NULL;
/* ADC buffer pointer */
volatile void *pGetDAC = NULL;

void *pADC;

/* Flag to register callback error */
volatile bool bEventError = false;

/* Adc linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align 4
int8_t AdcBuf[AUDIO_BUFFER_SIZE * 2];

/* Dac linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align 4
int8_t DacBuf[AUDIO_BUFFER_SIZE * 2];


int32_t CorrTable[CORRELATION_SIZE];

#if defined(__ADSPBF707_FAMILY__) || defined(__ADSP215xx__)
/* Memory required for the SPU operation */
static uint8_t  SpuMemory[ADI_SPU_MEMORY_SIZE];

/* SPU handle */
static ADI_SPU_HANDLE hSpu;
#endif

/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/
/* Initialize GPIO and reset peripherals */
uint32_t    GpioInit(void);
/* Initializes ADC */
uint32_t    Adau1979Init(void);
/* Initializes DAC */
uint32_t    Adau1962aInit(void);
/* Submit buffers to ADC */
uint32_t    Adau1979SubmitBuffers(void);
/* Submit buffers to DAC */
uint32_t    Adau1962aSubmitBuffers(void);
/* Process audio buffers */
uint32_t 	ProcessBuffers(void);
/* Generate sine wave test tone */
uint32_t 	Generate_SineWave(uint32_t Frequency, uint32_t SampleRate, void * pBuffer, uint32_t samples, uint32_t numchannels);
/* Perform a correlation on a specified buffer */
uint32_t 	Correlation(int32_t * pBuffer, uint32_t samples, uint32_t stride, int32_t * pCoorelationTable);
/* Check a specified correlation table for a reference frequency */
uint32_t	Check_Correlation(uint32_t ref_freq, uint32_t samples, int32_t * pCoorelationTable);
/*=============  C A L L B A C K    F U N C T I O N    P R O T O T Y P E S =============*/

/* ADC callback */
void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg);
/* DAC callback */
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg);

/*=============  E X T E R N A L    F U N C T I O N    P R O T O T Y P E S =============*/

/* Configures soft switches */
extern void ConfigSoftSwitches(void);

/*=============  C O D E  =============*/

/*
 * Main function
 */
int main()
{
	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */
	uint32_t Result = 0u;
	uint32_t i;

	/* configure System Event Controller SEC and Signal Routing Unit SRU */
	adi_initComponents();

	/* Software Switch Configuration for the EZ-Board */
	ConfigSoftSwitches();

	/* Initialize GPIO for ADC/DAC reset control */
	if(Result == 0u)
	{
		Result = GpioInit();
	}

/* ADSP-BF707 or ADSP-SC589 Processor family */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)

    /* Initialize SPU Service */
    if(adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to initialize SPU service\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
        return (ADI_SPU_FAILURE);
    }

#elif defined(__ADSPSC573_FAMILY__)

    /* Initialize SPU Service */
    if(adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to initialize SPU service\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2A_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2A\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2B_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2B\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2A DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2A_DMA4_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2A DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2B DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2B_DMA5_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2B DMA\n");
        return (ADI_SPU_FAILURE);
    }

#else
#error "processor not supported"
#endif

/* Generate sine wave test tone for ping and pong buffers */
#ifndef PLAYBACK
	/* generate sine wave */
	if(Result == 0u)
	{
		for( i = 0u; i < NUM_CHANNELS ; i++ )
		{
			Result = Generate_SineWave(REFERENCE_FREQ, SAMPLE_RATE, &DacBuf[(AUDIO_BUFFER_SIZE * 0u) + (4u*i)], SAMPLES_PER_PERIOD * 4u, NUM_CHANNELS);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
			Result = Generate_SineWave(REFERENCE_FREQ, SAMPLE_RATE, &DacBuf[(AUDIO_BUFFER_SIZE * 1u) + (4u*i)], SAMPLES_PER_PERIOD * 4u, NUM_CHANNELS);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
		}
	}
#endif

	/* Initialize ADAU1979 */
	if(Result == 0u)
	{
		Result = Adau1979Init();
	}

	/* Initialize ADAU1962A */
	if(Result == 0u)
	{
		Result = Adau1962aInit();
	}

	/* Submit ADC buffers */
	if(Result == 0u)
	{
		Result = Adau1979SubmitBuffers();
	}

	/* Submit DAC buffers */
	if(Result == 0u)
	{
		Result = Adau1962aSubmitBuffers();
	}

	/* Enable data flow for only the ADC and DAC */

	if((uint32_t)adi_adau1962a_Enable(phAdau1962a, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_Enable(phAdau1979, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	if(Result == 0u)
	{
	    /* Audio loopback */
		while(1)
		{
		    /* Process audio buffers */
			Result = ProcessBuffers();

			/* IF (Error) */
			if(Result != 0u)
			{
			    /* exit loopback */
				break;
			}

			/* check if an error has been detected in callback */
            if(bEventError)
			{
			    /* there has been an error returned in the callback */
				Result =1u;
				break;
            }

			/* Callback count limit reached */
			if (0)
			{

				/* Disable ADC data flow */
				if((uint32_t)adi_adau1979_Enable(phAdau1979, false) != 0u)
				{
					/* return error */
					Result = 1u;
				}

				/* Disable DAC data flow */
				adi_adau1962a_Enable(phAdau1962a, false);

				/* Close ADC device */
				adi_adau1979_Close(phAdau1979);

				/* Close DAC device */
				adi_adau1962a_Close(phAdau1962a);
				/* while loop break */
				break;
			}
		}
    }
#ifndef PLAYBACK
	if(Result == 0u)
	{
		for( i = 0u; i < NUM_CHANNELS ; i++ )
		{
			/* perform correlation of input buffer */
			Result = Correlation((int32_t *)&AdcBuf[(AUDIO_BUFFER_SIZE * 0u) + (NUM_CHANNELS*i)], CORRELATION_SIZE, 4u, &CorrTable[0]);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
			Result = Check_Correlation(REFERENCE_FREQ, CORRELATION_SIZE, &CorrTable[0]);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
		}
	}
#endif

	if (Result == 0u)
	{
		printf("All done\n");
	}
	else
	{
		printf("Error\n");
	}

	return 0;
}

/*
 * Initializes GPIO service
 * A GPIO line is used to control reset of the ADC and DAC devices
 */
uint32_t GpioInit(void)
{
	uint32_t Result = 0u;
	/* Loop variable */
	volatile uint32_t i;

	if((uint32_t)adi_gpio_Init((void*)GpioMemory, ADI_GPIO_CALLBACK_MEM_SIZE, &gpioMaxCallbacks) != 0u)
	{
		/* return error */
		return 1u;
	}

#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}
	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}
	
#elif defined(__ADSPSC573_FAMILY__)
	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}
	
	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7) != 0u)
	{
		/* return error */
		return 1u;
	}
	
	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7) != 0u)
	{
		/* return error */
		return 1u;
	}
	
#else
#error "processor not supported"
#endif

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	return Result;
}

/*
 * Opens and initializes ADAU1979 ADC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979Init(void)
{
	uint32_t Result = 0u;
	/* Instance to submit SPORT configuration */
	ADI_ADAU1979_SPORT_CONFIG   SportConfig;
	/* Instance to submit TWI configuration */
    ADI_ADAU1979_TWI_CONFIG     TwiConfig;

	/* open ADAU1979 instance */
	if((uint32_t)adi_adau1979_Open(0u,
#ifdef TDM_MODE
            					   ADI_ADAU1979_SERIAL_MODE_TDM4,
#else
            					   ADI_ADAU1979_SERIAL_MODE_STEREO,
#endif
            					   &Adau1979Memory,
            					   ADI_ADAU1979_MEMORY_SIZE,
            					   &phAdau1979) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_Open failed\n");
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum 	= 0u;
	TwiConfig.TwiDevMemSize	= ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem 	= &TwiMemory;
	TwiConfig.eTwiAddr 		= ADI_ADAU1979_TWI_ADDR_11;

	if((uint32_t)adi_adau1979_ConfigTwi(phAdau1979, &TwiConfig) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigTwi failed\n");
		/* return error */
		return 1u;
	}

	/* SPORT parameters required to open/configure TWI */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)	
	SportConfig.SportDevNum 	= 4u;
#elif defined(__ADSPSC573_FAMILY__)
    SportConfig.SportDevNum 	= 2u;
#else	
#error "processor not defined"
#endif
	SportConfig.SportDevMemSize	= ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem 	= &Adau1979SportMemory;
	SportConfig.eSportChnl	    = ADI_ADAU1979_SPORT_A;
	SportConfig.eSportPri	    = ADI_ADAU1979_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec	    = ADI_ADAU1979_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec	    = ADI_ADAU1979_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.bLsbFirst		= true;

	if((uint32_t)adi_adau1979_ConfigSport(phAdau1979, &SportConfig) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigSport failed\n");
		/* return error */
		return 1u;
	}

	/* ADC is a master source of SPORT clk and FS, MCLK 25.576 MHz and PLL used MCLK */
	if((uint32_t)adi_adau1979_ConfigPllClk(phAdau1979,
										   LR_B_CLK_MASTER_1979,
			                               ADI_ADAU1979_MCLK_IN_FREQ_24576000HZ,
			                               ADI_ADAU1979_PLL_SOURCE_MCLK) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigPllClk failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_ConfigSerialClk(phAdau1979,
            								  BCLK_RISING_1979,
            								  LRCLK_HI_LO_1979) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigSerialClk failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_SetSampleRate(phAdau1979, ADI_ADAU1979_SAMPLE_RATE_48000HZ) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_SetSampleRate failed\n");
		/* return error */
		return 1u;
	}
	if((uint32_t)adi_adau1979_SetWordWidth(phAdau1979, ADI_ADAU1979_WORD_WIDTH_24) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_SetWordWidth failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_RegisterCallback(phAdau1979, AdcCallback, NULL) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_RegisterCallback failed\n");
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Opens and initializes ADAU1962A DAC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1962aInit(void)
{
    ADI_ADAU1962A_RESULT        eResult;
    ADI_ADAU1962A_TWI_CONFIG    TwiConfig;
    ADI_ADAU1962A_SPORT_CONFIG  SportConfig;

	/* Open ADAU1962A device instance */
	if((eResult = adi_adau1962a_Open(0u,
#ifdef TDM_MODE
            						 ADI_ADAU1962A_SERIAL_MODE_TDM4,
#else
             					     ADI_ADAU1962A_SERIAL_MODE_STEREO,
#endif
            					     &Adau1962aMemory,
            					     ADI_ADAU1962A_MEMORY_SIZE,
            					     &phAdau1962a)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to open ADAU1962A device instance, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum 	= 0u;
	TwiConfig.eTwiAddr 		= ADI_ADAU1962A_TWI_ADDR_04;
	TwiConfig.TwiDevMemSize	= ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem 	= &TwiMemory;

    /* Configure TWI */
	if ((eResult = adi_adau1962a_ConfigTwi (phAdau1962a, &TwiConfig)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure TWI, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* SPORT parameters required to open/configure SPORT */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)	
	SportConfig.SportDevNum 	= 4u;
#elif defined(__ADSPSC573_FAMILY__)
    SportConfig.SportDevNum 	= 2u;
#else	
#error "processor not defined"
#endif
	SportConfig.eSportChnl	    = ADI_ADAU1962A_SPORT_B;
	SportConfig.eSportPri	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.SportDevMemSize	= ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem 	= &Adau1962aSportMemory;

    /* Configure SPORT */
	if ((eResult = adi_adau1962a_ConfigSport (phAdau1962a, &SportConfig)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure SPORT, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* DAC Master Power-up */
	if ((eResult = adi_adau1962a_ConfigDacPwr (phAdau1962a,
											   ADI_ADAU1962A_CHNL_DAC_MSTR,
											   ADI_ADAU1962A_DAC_PWR_LOW,
											   true)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure DAC power, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

    /*
     * Configure PLL clock - DAC is clock master and drives SPORT clk and FS
     * MCLK 24.576 MHz and PLL uses MCLK
     */
	if ((eResult = adi_adau1962a_ConfigPllClk (phAdau1962a,
	                                           ADAU1962A_MCLK_IN,
	                                           ADI_ADAU1962A_MCLK_SEL_PLL,
	                                           ADI_ADAU1962A_PLL_IN_MCLKI_XTALI)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure PLL clock, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

    /*
     * Configure serial data clock
     * DAC as clock master, External BCLK, Latch on raising edge
     * LRCLK at 50% duty cycle, MSB first, Left channel at LRCLK low
     */
	if ((eResult = adi_adau1962a_ConfigSerialClk (phAdau1962a,
	                                              LR_B_CLK_MASTER_1962,
	                                              false,
	                                              BCLK_RISING_1962,
/* pulse mode - true */
#ifdef TDM_MODE
	                                              true,
#else
	                                              false,
#endif
	                                              false,
	                                              LRCLK_HI_LO_1962)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure serial data clock, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Power-up PLL */
	if ((eResult = adi_adau1962a_ConfigBlockPwr (phAdau1962a,
	                                             false,
	                                             true,
	                                             true)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to Power-up PLL, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Configure Sample rate */
	if ((eResult = adi_adau1962a_SetSampleRate (phAdau1962a, SAMPLE_RATE)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure Sample rate, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Configure Word width */
	if ((eResult = adi_adau1962a_SetWordWidth (phAdau1962a,
	                                           ADI_ADAU1962A_WORD_WIDTH_24)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure word width, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Register callback */
	if ((eResult = adi_adau1962a_RegisterCallback (phAdau1962a,
												   DacCallback,
	                                               NULL)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to register callback, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	return 0u;
}

/*
 * Submits ping-pong buffers to ADC and enables ADC data flow.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979SubmitBuffers(void)
{
	uint32_t Result = 0u;

	/* submit ping buffer */
	if((uint32_t)adi_adau1979_SubmitBuffer(phAdau1979, &AdcBuf[AUDIO_BUFFER_SIZE * 0u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if((uint32_t)adi_adau1979_SubmitBuffer(phAdau1979, &AdcBuf[AUDIO_BUFFER_SIZE * 1u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Submits ping-pong buffers to DAC and enables ADC data flow.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1962aSubmitBuffers(void)
{
	uint32_t Result = 0u;

	/* submit ping buffer */
	if((uint32_t)adi_adau1962a_SubmitBuffer(phAdau1962a, &DacBuf[AUDIO_BUFFER_SIZE * 0u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if((uint32_t)adi_adau1962a_SubmitBuffer(phAdau1962a, &DacBuf[AUDIO_BUFFER_SIZE * 1u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Processes audio buffers that are processed by ADC and DAC driver.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t ProcessBuffers(void)
{
    ADI_ADAU1979_RESULT     eResult1;
	ADI_ADAU1962A_RESULT    eResult2;

	uint32_t i =0u;
	int *pSrc;
	int *pDst;
	void *pDAC;

	if(pGetADC != NULL)
	{
		pADC = (void *)pGetADC;
		/* re-submit the ADC buffer */
		eResult1 = adi_adau1979_SubmitBuffer(phAdau1979, (void *) pADC, AUDIO_BUFFER_SIZE);
		if(eResult1)
		{
			return 1u;
		}
	    pGetADC = NULL;
	}

	if (pGetDAC != NULL)
	{
		pDAC = (void *)pGetDAC;
		/* re-submit the DAC buffer */
	    eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC, AUDIO_BUFFER_SIZE);

		if(eResult2)
		{
			return 1u;
		}

	    pGetDAC = NULL;

	}

	/* copy ADC to DAC buffer */
	if ((pDAC != NULL) && (pADC != NULL))
	{
		pSrc =  (int *)pADC;
		pDst =  (int *)pDAC;

#ifdef PLAYBACK

// ========================================== my own code E1 ============================================
#define fir_level 32
#define iir_level 8

float sum(float* coef, float* buffer, int level, int* data_buffer_cnt){
    float s = 0;
    for(int i=0; i< level; i++){
        s += coef[i]*buffer[(*data_buffer_cnt + i) & (AUDIO_BUFFER_SIZE - 1)]; // &(level -1);
    }
    return s;
}

int length(float* array){
    // printf("Length of array: %d\n", (int)(sizeof(array)/sizeof(array[0])));
    // printf("%d", (int)(sizeof(array)));
    // printf("sizeof(array[0]): %d\n", (int)(sizeof(array[0])));
    return (int)(sizeof(array)/sizeof(array[0]));
}

float* FIR_4C(float * fir_coef , float * data_buffer, int * data_buffer_cnt)
{
    float* y_n  = (float*)malloc(AUDIO_BUFFER_SIZE*sizeof(float));
    // data_buffer[*data_buffer_cnt] = x_n; 
	
	// channels
    for(int c=0; c<4; c++){

		// `count` is for processing index , data_buffer_cnt is for sampling index
		int count = *data_buffer_cnt;
		for(int i=0; i<AUDIO_BUFFER_SIZE/sizeof(int); i+=4){
			y_n[c+i] = sum(fir_coef, data_buffer, fir_level, &count);
			// Circular buffer index update
		}
	}
	
	*data_buffer_cnt = (*data_buffer_cnt + 1 + AUDIO_BUFFER_SIZE) & (AUDIO_BUFFER_SIZE - 1); 
	return y_n;
}


// ps:
//      1. 三项求和不用sum函数不然太绕了，一种调试方式是把自己看着都绕的地方试图简化，比如声明一些辅助变量
//      2. 注意循环队列的方向（时间前后）
float* IIR_4C(float * iir_coef, int iir_coef_length, float * data_buffer, int * data_buffer_cnt, float gain)
{
    float* buf = (float*)malloc(AUDIO_BUFFER_SIZE*sizeof(float));
    float* next = (float*)malloc(AUDIO_BUFFER_SIZE*sizeof(float));
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float in, out;
    
    // data_buffer[*data_buffer_cnt] = x_n;
    for(int i=0; i<AUDIO_BUFFER_SIZE; i++){
        buf[i] = data_buffer[i];
    }

	
	// 4 sos concatenated
	for(int i=0; i<iir_level/2 ; i++){
		b0 = iir_coef[i*6+0]; b1 = iir_coef[i*6+1]; b2 = iir_coef[i*6+2];
		a1 = iir_coef[i*6+4]; a2 = iir_coef[i*6+5];
		x1 = 0, x2 = 0;
		y1 = 0, y2 = 0;
		
		for(int k=0; k<AUDIO_BUFFER_SIZE; k++){
			next[k] = 0;
		}
		
		// channels
		for(int c=0; c<4; c++){
			for(int j=0; j<AUDIO_BUFFER_SIZE/sizeof(int); j+=4){

				in = buf[c+j];
				out = b0*in + b1*x1 + b2*x2 - a1*y1 - a2*y2;
				
				// update history
				x2 = x1; x1 = in;
				y2 = y1; y1 = out;
				
				next[c+j] = out;
			}
		}

		// update the next buf with this sos's result (named `next`) 
		for(int k=0; k<AUDIO_BUFFER_SIZE; k++){
			buf[k] = next[k];
		}
	}
    // apply gain
    for(int i=0; i<AUDIO_BUFFER_SIZE; i++){
        buf[i] *= gain; 
    }
    free(next);
	*data_buffer_cnt = (*data_buffer_cnt +1 + AUDIO_BUFFER_SIZE) & (AUDIO_BUFFER_SIZE - 1);

	return buf;
}
// ================================================================================================

#ifndef TDM_MODE
		// for(i=0; i<AUDIO_BUFFER_SIZE/sizeof(int); i+=4)
		// {
		// 	// 4 channels
		// 	*(pDst+i+0) = *(pSrc+i+0);
		// 	*(pDst+i+1) = *(pSrc+i+1);
		// 	*(pDst+i+2) = *(pSrc+i+2);
		// 	*(pDst+i+3) = *(pSrc+i+3);

		// }
		float fir_coef[] = {   -0.0006,   -0.0018,   -0.0026,   -0.0016,    0.0024,    0.0083,    0.0118,    0.0068,   -0.0092,   -0.0297,   -0.0398,   -0.0223,    0.0310,    0.1111,    0.1925,    0.2437,    0.2437,    0.1925,    0.1111,     0.0310,   -0.0223,   -0.0398,   -0.0297,   -0.0092,    0.0068,    0.0118,    0.0083,    0.0024,   -0.0016,   -0.0026,   -0.0018,   -0.0006};
		float sos[] = {
			1.0000,    2.0003,    0.9999,    1.0000,   -0.8351,    0.1810,
			1.0000,    2.0305,    1.0309,    1.0000,   -0.8906,    0.2595,
			1.0000,    1.9997,    1.0001,    1.0000,   -1.0153,    0.4359,
			1.0000,    1.9695,    0.9700,    1.0000,   -1.2428,    0.7575
		};
		float g = 1.0791e-04;
		int data_buffer_cnt = 0;
	    int sos_length = (int)(sizeof(sos)/sizeof(sos[0]));
		pDst = FIR_4C(fir_coef, pSrc, &data_buffer_cnt);
		//pDst = IIR_4C(sos, sos_length, pSrc, &data_buffer_cnt, g);
	    pDAC = NULL;
	    pADC = NULL;
#endif
#ifdef TDM4

		for(uint32_t i=0; i<(AUDIO_BUFFER_SIZE);i+=16)
		{
			pSrc =  (int8_t *)(int8_t *)pADC +i;

			for(uint32_t j=0; j<8; j++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t K=8; K<16; K++)
				{
					*pDst++ = *pSrc++;
				}

		}
#endif
#ifdef TDM8

		for(uint32_t i=0; i<(AUDIO_BUFFER_SIZE);i+=32)
		{
			pSrc =  (int8_t *)(int8_t *)pADC +i;

			for(uint32_t j=0; j<8; j++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t K=8; K<16; K++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t x=16; x<24; x++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;
			for(uint32_t y=24; y<32; y++)
				{
					*pDst++ = *pSrc++;
				}

		}
#endif

#endif
}
	return (0u);
}

/*
 * Generate a sine wave and fill a buffer.
 *
 * Parameters
 *  Frequency	- Sine wave frequency in Hz
 *  SampleRate 	- Sample rate of ADC/DAC in Hz
 *  pBuffer		- Pointer to the buffer where the sine wave will be placed
 *  samples		- Number of samples generated
 *  numchannels - Number of audio channels
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Generate_SineWave(uint32_t Frequency, uint32_t SampleRate, void * pBuffer, uint32_t samples, uint32_t numchannels)
{

	uint32_t i;
	uint32_t mod;
	int32_t *p;
	p = pBuffer;

	/* sine wave frequency must be a multiple of the sample rate,
	 * otherwise waveform will not be continuous
	 */
	mod = SampleRate % Frequency;
	if(mod != 0)
	{
		return (1u);
	}

	/* number of samples in the sine wave must to a multiple of the number of
	 * samples in 1 period, otherwise waveform will not be continuous
	 */
	mod = samples % (SAMPLES_PER_PERIOD);
	if(mod != 0)
	{
		return (1u);
	}

	/* create out sine wave */
	for( i = 0u; i < samples ; i++ )
	{
#ifdef TDM_MODE
		*(p + numchannels * i) = ((int32_t)(AMPLITUDE * sin((2.0 * PI * Frequency * (((float)(i)) / ((float)SampleRate))))) << 8);
#else
		*(p + numchannels * i) = (int32_t)(AMPLITUDE * sin((2.0 * PI * Frequency * (((float)(i)) / ((float)SampleRate)))));
#endif
	}

	return (0u);
}

/*
 * Generates a correlation table from a specified audio buffer.
 *
 * Parameters
 *  pBuffer				- Pointer to the buffer of the waveform to be correlated
 *  samples				- Number of samples to correlate
 *  stride				- The stride width that separates each sample
 *  pCoorelationTable	- Pointer to the generated correlation table
 *
 * Returns
 *  0 if success, other values for error
 *
 */

uint32_t Correlation(int32_t * pBuffer, uint32_t samples, uint32_t stride, int32_t * pCoorelationTable)
{
	uint32_t i, j;
	uint32_t mod;
	int32_t *p;
	int32_t *p1;
	int32_t *pCorr;
	int32_t Sum = 0u;
	int32_t Total = 0u;

	/* number of samples to correlate must be a multiple
	 * of the number of samples in 1 period
	 */
	mod = samples % (SAMPLES_PER_PERIOD);
	if(mod != 0)
	{
		return (1u);
	}

	/* number of sample to correlate must at least twice
	 * the number of samples in 1 period so the
	 * sliding window correlation can be performed
	 */
	if(samples < (SAMPLES_PER_PERIOD) * 2u)
	{
		return (1u);
	}

	pCorr = pCoorelationTable;
	/* reset correlation table */
	for( i = 0u; i < samples ; i++ )
	{
		*pCorr++ = 0u;
	}

	pCorr = pCoorelationTable;

	for( i = 0u; i < samples ; i++ )
	{
		/* pointer to reference correlation starting point */
		p     = (int32_t *)(pBuffer);
		/* pointer to correlation sliding window starting point,
		 * x samples ahead of reference correlation
		 * */
		p1    = (int32_t *)(pBuffer + (i * stride));
		for( j = 0u; j < samples; j++ )
		{
			/* scale then multiply to avoid overflow */
			Sum = (*(p+ (j * stride)) >> CORRELATION_SCALING_FACTOR1) * (*(p1+(j * stride)) >> CORRELATION_SCALING_FACTOR1);
			/* accumulate */
			Total += (Sum >> CORRELATION_SCALING_FACTOR2);
		}
		/* store multiplication/accumulation in the correlation table */
		*pCorr++ = Total;
		Total = 0u;
		Sum = 0u;
	}
	return (0u);

}

/*
 * Analyze a correlation table to determine if its contents represent a specified sine wave frequency
 *
 * Parameters
 *  ref_freq			- reference frequency in Hz
 *  samples				- Number of samples to correlate
 *  pCoorelationTable	- Pointer to the generated correlation table
 *
 * Returns
 *  0 if success, other values for error
 *
 */

/* Analyzes a correlation table to determine if its contents represents the specified sine wave frequency  */
uint32_t Check_Correlation(uint32_t ref_freq, uint32_t samples, int32_t * pCoorelationTable)
{
	uint32_t result = 0u;
	uint32_t i;
	uint32_t i_peak;
	int32_t *pCorr;

	int32_t largest_corr;
	int32_t next_largest_corr;

	pCorr = pCoorelationTable;

	/* The maximum correlation is when sine wave is in phase represented
	 * by the first correlated sample. The next maximum occurs one correlation sample
	 * later.
	 */
	largest_corr      = *pCorr;
	next_largest_corr = *(pCorr +1);

	if(next_largest_corr >= largest_corr)
	{
		/* there has been an error */
		return 1u;
	}

	/* Search for 2nd correlation maximum.
	 * This should occur when the  sine wave is in phase again
	 * ie. 1 period of samples later
	 */
	next_largest_corr = *(pCorr + 1u);
	for( i = 1u; i < samples - 2u ; i++ )
	{
		if(*(pCorr + i) > next_largest_corr)
		{
			next_largest_corr = *(pCorr + i);
			i_peak = i;
		}
	}
	if(i_peak != SAMPLE_RATE/ref_freq)
	{
		/* the 2nd correlation maximum should be 16 samples away from 1st
		 * correlation maximum, assuming a sample rate of 48kHz and input of 3kHz
		 */
		result = 1u;
	}
#if 0
	/* check the maximum and minimum allowable tolerance */
	if((next_largest_corr > (CORRELATION_NORM_PEAK + CORRELATION_NORM_TOLERANCE)) ||
	   (next_largest_corr < (CORRELATION_NORM_PEAK - CORRELATION_NORM_TOLERANCE)))
	{
		result = 1u;
	}
#endif

	/* check the maximum and minimum allowable tolerance */
	if(next_largest_corr < 0x100000u)
	{
		result = 1u;
	}

	return (result);
}


/*
 * ADC Callback.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
    switch(nEvent)
    {
        case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        	/* Update callback count */
        	AdcCount++;
        	/* store pointer to the processed buffer that caused the callback */
        	pGetADC = pArg;
        	break;
        default:
        	bEventError = true;
            break;
    }
}

/*
 * DAC Callback.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
    switch(nEvent)
    {
        case ADI_SPORT_EVENT_TX_BUFFER_PROCESSED:
        	/* Update callback count */
        	DacCount++;
        	/* store pointer to the processed buffer that caused the callback */
        	pGetDAC = pArg;
        	break;
        default:
        	bEventError = true;
            break;
    }
}

/*****/
