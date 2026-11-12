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
#include <limits.h>
#include <stdint.h>
#include "adc_dac_playback.h"
#include "adi_initialize.h"

#include <SRU.h>

// my include

#include "fir_iir.h"
#include "lpf_down_then_up.h"
#include "my_fft.h"                 /* E3: 32 点基-2 FFT / IFFT */

/* ------------------- E3: 与 ADI 库函数 rfft32 的并行对比 -------------------
 * trans.h（rfftN/rfft32）和 cycle_count.h 只有 SHARC 核才提供（ADSP-21000
 * 系列，含 SC5xx 的 SHARC+ 核）；Blackfin 上没有这两个头文件，所以用
 * __ADSP21000__ 把整段隔开，Blackfin 编译时这段完全不参与。
 *
 * 注意：rfft32 在 libdsp.dlb 里，链接时工程要加上这个库，否则会报
 *       undefined symbol rfft32。
 * ------------------------------------------------------------------------ */
#if defined(FFT_32P) && defined(__ADSP21000__)
#define MY_FFT_USE_RFFTN        1
#ifndef DO_CYCLE_COUNTS
#define DO_CYCLE_COUNTS         1   /* 打开 cycle_count.h 里的 START/STOP 宏 */
#endif
#include <trans.h>                  /* rfft32 */
#include <cycle_count.h>            /* START_CYCLE_COUNT / STOP_CYCLE_COUNT */
#endif


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

/* ------------------------ E3: 32 点 FFT 的观测变量 -----------------------
 * 只为在 CCES 的 Expressions / Watch 窗口里观察（或调试时导出）而存在，
 * 不参与音频回环的处理链路，DATA 段里的数据流完全不受影响。
 * ---------------------------------------------------------------------- */
#ifdef FFT_32P
volatile my_complex gFftSpec[MY_FFT_N];         /* 最近一帧的复数谱 X[k]          */
volatile float      gFftMag[MY_FFT_N / 2];      /* |X[k]|/N，k = 0..15，线性幅度  */
volatile uint32_t   gFftFrameCount  = 0u;       /* 已完成的 32 点帧数             */
volatile uint32_t   gFftPeakBin     = 0u;       /* 幅度最大谱线的序号 k           */
volatile float      gFftPeakFreqHz  = 0.0f;     /* 峰值谱线对应的频率 / Hz        */
volatile float      gFftBinHz       = 0.0f;     /* 每根谱线间隔 / Hz              */
#endif

#ifdef MY_FFT_USE_RFFTN
/* rfft32 的结果与两边的耗时（同样只给 watch / 导出用） */
volatile float      gRfftRe[MY_FFT_N];          /* rfft32 的实部输出             */
volatile float      gRfftIm[MY_FFT_N];          /* rfft32 的虚部输出             */
volatile float      time_used[2];               /* [0] my_fft，[1] rfft32，      */
                                                /* 单位：周期/次（累计平均）      */
#endif

#ifdef CAPTURE_FOR_REPORT
#if (CAPTURE_AUTOSTART != 0u)
volatile uint32_t gCaptureStartRequest = 1u;
#else
volatile uint32_t gCaptureStartRequest = 0u;
#endif
volatile uint32_t gCaptureActive = 0u;
volatile uint32_t gCaptureDone = 0u;
volatile uint32_t gCaptureFramesCaptured = 0u;
volatile uint32_t gCaptureInputFramesSeen = 0u;
volatile uint32_t gCaptureDoneNotified = 0u;
int16_t gRawCapture[CAPTURE_TOTAL_SAMPLES];
int16_t gProcessedCapture[CAPTURE_TOTAL_SAMPLES];
#endif

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
/* self-added: Generate composite sine wave test tone for ping and pong buffers */
uint32_t 	Generate_CompositeSine(void *pBuffer, uint32_t samples, uint32_t numchannels);
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

static void Fir4C_NoAlloc(const float *firCoef, const float *dataBuffer, float *outputBuffer)
{
	uint32_t c;
	uint32_t n;
	uint32_t k;
	uint32_t seqLength = (AUDIO_BUFFER_SIZE / (sizeof(float) * NUM_CHANNELS));

	for(c = 0u; c < NUM_CHANNELS; c++)
	{
		for(n = 0u; n < seqLength; n++)
		{
			float acc = 0.0f;
			for(k = 0u; k < 32u; k++)
			{
				if(n >= k)
				{
					acc += firCoef[k] * dataBuffer[((n - k) * NUM_CHANNELS) + c];
				}
			}
			outputBuffer[(n * NUM_CHANNELS) + c] = acc;
		}
	}
}

#ifdef CAPTURE_FOR_REPORT
static void DumpCapturedDataToConsole(void)
{
	uint32_t i;

	printf("CAPTURE_CSV_BEGIN,index,raw,processed\r\n");
	for(i = 0u; i < CAPTURE_TOTAL_SAMPLES; i++)
	{
		printf("%lu,%d,%d\r\n",
				(unsigned long)i,
				(int)gRawCapture[i],
				(int)gProcessedCapture[i]);
	}
	printf("CAPTURE_CSV_END\r\n");
	fflush(stdout);
}
#endif

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
	setvbuf(stdout, NULL, _IONBF, 0);

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

// /* ===== 生成合成正弦波填充 DacBuf（ping + pong） ===== */
// if(Result == 0u)
// {
// 	uint32_t samplesPerBuffer = AUDIO_BUFFER_SIZE / (SAMPLE_SIZE * NUM_CHANNELS);
// 	Generate_CompositeSine(&DacBuf[AUDIO_BUFFER_SIZE * 0u], samplesPerBuffer, NUM_CHANNELS);
// 	Generate_CompositeSine(&DacBuf[AUDIO_BUFFER_SIZE * 1u], samplesPerBuffer, NUM_CHANNELS);
// }

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
#ifdef CAPTURE_FOR_REPORT
#if (CAPTURE_AUTOSTART != 0u)
		gCaptureStartRequest = 1u;
#else
		gCaptureStartRequest = 0u;
#endif
		gCaptureActive = 0u;
		gCaptureDone = 0u;
		gCaptureFramesCaptured = 0u;
		gCaptureInputFramesSeen = 0u;
		gCaptureDoneNotified = 0u;
#endif

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

#ifdef CAPTURE_FOR_REPORT
			if((gCaptureDone != 0u) && (gCaptureDoneNotified == 0u))
			{
				gCaptureDoneNotified = 1u;
			}
#endif

			/* Capture complete: stop streaming and exit loop for easy data export. */
#ifdef CAPTURE_FOR_REPORT
			if ((gCaptureDone != 0u) && (gCaptureFramesCaptured >= CAPTURE_TOTAL_FRAMES))
#else
			if (0)
#endif
			{
#if defined(CAPTURE_FOR_REPORT) && (CAPTURE_DUMP_TO_CONSOLE != 0u)
				DumpCapturedDataToConsole();
#endif
				printf("Capture complete. You can now export gRawCapture and gProcessedCapture.\r\n");

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



// 数组声明
static float dec_hist[DEC_FIR_LEN - 1];        /* 142 */
static float int_hist[INT_FIR_LEN];            /* 143 */
static int   int_hist_cnt = 0;

static float work_dec[DEC_FIR_LEN - 1 + 64 + 8];   /* 142 + 64 + 8 = 214 */
static float work_int[INT_FIR_LEN + 64 + 8];       /* 143 + 64 + 8 = 215 */

static float ch_in_work[64 + 4];
static float buf8_work[16];
static float buf8_filt_work[16];
static float ch_out48_work[128];
static float pF_buf_work[256];

/* E2 每拍实际产出的 8 kHz 域样点数（约 11 个），交给 E3 的 32 点分帧 FFT 用 */
static int   g_n8_last = 0;

uint32_t ProcessBuffers(void)
{
	ADI_ADAU1979_RESULT     eResult1;
    ADI_ADAU1962A_RESULT    eResult2;
    uint32_t i;
    int *pSrc;
    int *pDst;
	
    /* 跨调用保留 pending 指针，让 ADC 和 DAC 的一拍延迟配对 */
    static void *pendingADC = NULL;
    static void *pendingDAC = NULL;
	
    if (pGetADC != NULL)
    {
		pendingADC = (void *)pGetADC;
        pGetADC = NULL;
        eResult1 = adi_adau1979_SubmitBuffer(phAdau1979, pendingADC, AUDIO_BUFFER_SIZE);
        if (eResult1) return 1u;
    }
	
    if (pGetDAC != NULL)
    {
		pendingDAC = (void *)pGetDAC;
        pGetDAC = NULL;
        eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, pendingDAC, AUDIO_BUFFER_SIZE);
        if (eResult2) return 1u;
    }
	
    if ((pendingADC != NULL) && (pendingDAC != NULL))
    {
		pSrc = (int *)pendingADC;
        pDst = (int *)pendingDAC;
		
#ifdef PLAYBACK
#ifndef TDM_MODE

// ================================= E1 =================================
        {
			float fir_coef[] = { -0.0014,   -0.0013,   -0.0011,   -0.0006,    0.0009,    0.0036,    0.0082,    0.0148,    0.0234,    0.0337,    0.0452,    0.0569,    0.0680,    0.0774,    0.0843,    0.0879,    0.0879,    0.0843,    0.0774,
								  0.0680,    0.0569,    0.0452,    0.0337,    0.0234,    0.0148,    0.0082,    0.0036,    0.0009,   -0.0006,   -0.0011,   -0.0013,   -0.0014};
			float sos[] = {1.0000,    2.0003,    0.9999,    1.0000,   -1.5407,    0.5951,
						   1.0000,    2.0305,    1.0309,    1.0000,   -1.5897,    0.6458,
						   1.0000,    1.9997,    1.0001,    1.0000,   -1.6890,    0.7486,
						   1.0000,    1.9695,    0.9700,    1.0000,   -1.8390,    0.9039};
			float g = 4.6020e-08;
            uint32_t sampleCount = AUDIO_BUFFER_SIZE / sizeof(int32_t);
            int32_t *pSrc32 = (int32_t *)pSrc;
            int32_t *pDst32 = (int32_t *)pDst;
			float inputBuffer[AUDIO_BUFFER_SIZE / sizeof(float)];
			float *pFiltered = NULL;

			for (i = 0; i < sampleCount; i++)
			{
				inputBuffer[i] = (float)pSrc32[i];
			}

#ifdef FIR_TEST
			pFiltered = FIR_4C(fir_coef, inputBuffer, NULL);
#elif defined(IIR_TEST)
			pFiltered = IIR_4C(sos,
							(int)(sizeof(sos) / sizeof(sos[0])),
							inputBuffer,
							NULL,
							g);
#endif

// ================================= E2 =================================

#ifdef PIPELINE_48k_8k_48k
    {
        int n_frames = (int)sampleCount / NUM_CHANNELS;
        int i, k, n8, n48, j, c;

        for (j = 0; j < n_frames; j++)
            ch_in_work[j] = inputBuffer[j * NUM_CHANNELS];

        /* ---------- 降采样 + 历史 ---------- */
        {
            int H = DEC_FIR_LEN - 1;
            int total = H + n_frames;
            for (i = 0; i < H; i++) work_dec[i] = dec_hist[i];
            for (i = 0; i < n_frames; i++) work_dec[H + i] = ch_in_work[i];
            n8 = 0;
            for (i = H; i < total; i += DEC_FIR_Q) {
                float acc = 0.0f;
                for (k = 0; k < DEC_FIR_LEN; k++) {
                    int idx = i - k;
                    if (idx >= 0) acc += dec_fir_coeff[k] * work_dec[idx];
                }
                buf8_work[n8++] = acc;
            }
            for (i = 0; i < H; i++)
                dec_hist[i] = work_dec[total - H + i];
        }

        /* ---------- IIR ---------- */
        iir_lpf_8k(buf8_work, buf8_filt_work, n8);
        g_n8_last = n8;             /* 记录本拍 8k 域样点数，供 E3 的 FFT 分帧使用 */

        /* ---------- 上采样 + 历史 ---------- */
        {
            int branch = (INT_FIR_LEN + INT_FIR_P - 1) / INT_FIR_P;
            int total  = int_hist_cnt + n8;
            for (i = 0; i < int_hist_cnt; i++) work_int[i] = int_hist[i];
            for (i = 0; i < n8; i++)            work_int[int_hist_cnt + i] = buf8_filt_work[i];

            n48 = n8 * INT_FIR_P;
            for (i = 0; i < n48; i++) {
                int phase = i % INT_FIR_P;
                int q     = i / INT_FIR_P;
                int base  = int_hist_cnt + q;
                float acc = 0.0f;
                for (k = 0; k < branch; k++) {
                    int hidx = phase + k * INT_FIR_P;
                    if (hidx >= INT_FIR_LEN) break;
                    int xidx = base - k;
                    if (xidx >= 0 && xidx < total)
                        acc += int_fir_coeff[hidx] * work_int[xidx];
                }
                ch_out48_work[i] = acc;
            }

            int keep = branch;
            if (keep > total) keep = total;
            for (i = 0; i < keep; i++)
                int_hist[i] = work_int[total - keep + i];
            int_hist_cnt = keep;
        }

        for (j = 0; j < n_frames; j++) {
            float v = (j < n48) ? ch_out48_work[j] : 0.0f;
            for (c = 0; c < NUM_CHANNELS; c++)
                pF_buf_work[j * NUM_CHANNELS + c] = v;
        }
        pFiltered = pF_buf_work;
    }
#endif
// ================================= E3 =================================

#ifdef FFT_32P
		/* ---------- 32 点实时 FFT（分帧处理） ----------
		 * 每凑满 32 个样点就做一次 my_fft()，谱线放到 gFftSpec / gFftMag，
		 * 峰值频率放到 gFftPeakFreqHz，CCES 里直接 watch 这几个变量即可。
		 *
		 * SHARC 上（MY_FFT_USE_RFFTN）还会用同一帧数据再调一次 ADI 库函数
		 * rfft32()，把结果放到 gRfftRe/gRfftIm、耗时放到 time_used[]，
		 * 用来和自写的 my_fft 比结果、比速度。
		 *
		 * 数据来源（与原有流程保持一致，只“看”数据、不改音频链路）：
		 *   - 若 E2 的 48k->8k->48k 流水线打开：分析 8 kHz 域的样点
		 *     buf8_filt_work[0 .. g_n8_last-1]（每拍约 11 点，约 3 拍凑一帧），
		 *     用来验证 2 kHz 低通和 8 kHz 采样率是否正常，谱线间隔 250 Hz；
		 *   - 否则：分析 48 kHz 域的第 0 通道样点（每拍 64 点、即 2 个
		 *     32 点帧，4 个通道是交织存放的，所以步进取 NUM_CHANNELS），
		 *     谱线间隔 1.5 kHz。
		 *
		 * 注意：这一块不修改 pFiltered，所以原有的 FIR/IIR/重采样流程
		 * 和播放回环完全不受影响。 */
		{
			static float      fft_frame[MY_FFT_N];   /* 分帧缓存，跨调用保持 */
			static int        fft_fill  = 0;         /* 缓存里已攒了多少点   */
#ifdef MY_FFT_USE_RFFTN
			static float      lib_re[MY_FFT_N];      /* rfft32 实部输出      */
			static float      lib_im[MY_FFT_N];      /* rfft32 虚部输出      */
			static float      acc_my  = 0.0f;        /* my_fft 累计周期数    */
			static float      acc_lib = 0.0f;        /* rfft32 累计周期数    */
			cycle_t           cyc_start;             /* EMUCLK 计时用        */
			cycle_t           cyc_my;
			cycle_t           cyc_lib;
#endif

			const float *psrc;                       /* 本拍的样点来源       */
			float   fs;                              /* 样点来源所在的采样率 */
			int     nsrc;                            /* 本拍可用的样点数     */
			int     stride;                          /* 取同一通道时的样点间隔 */
			int     js;
			int     k;
			my_complex spec[MY_FFT_N];               /* 本帧的复数谱 X[k]    */
			float   mag[MY_FFT_N / 2];
			float   peak;
			uint32_t peak_bin;

#ifdef PIPELINE_48k_8k_48k
			psrc   = buf8_filt_work;                 /* 8 kHz 域（IIR 之后），单通道 */
			nsrc   = g_n8_last;                      /* 本拍 8k 域样点数     */
			stride = 1;
			fs     = (float)SAMPLE_RATE / (float)DEC_FIR_Q;  /* 48k/6 = 8 kHz */
#else
			psrc   = (pFiltered != NULL) ? pFiltered : inputBuffer;
			nsrc   = (int)(sampleCount / NUM_CHANNELS);   /* 每通道的样点数  */
			stride = (int)NUM_CHANNELS;                   /* 4 通道交织，取第 0 通道 */
			fs     = (float)SAMPLE_RATE;             /* 48 kHz              */
#endif
			gFftBinHz = fs / (float)MY_FFT_N;

			for (js = 0; js < nsrc; js++)
			{
				fft_frame[fft_fill++] = psrc[js * stride];
				if (fft_fill < MY_FFT_N)
				{
					continue;                        /* 还没攒满一帧 */
				}
				fft_fill = 0;

				/* ---- 一帧 32 点 FFT：X[k] = sum x[n] * W32^(n*k) ---- */
#ifdef MY_FFT_USE_RFFTN
				/* 同一帧数据，两边各算一遍，各自计时 ---------------------- */
				START_CYCLE_COUNT(cyc_start);
				my_fft(fft_frame, spec);
				STOP_CYCLE_COUNT(cyc_my, cyc_start);

				/* 库函数只保证前 N/2+1 个点有效，先清零，剩下的点就是 0 */
				for (k = 0; k < MY_FFT_N; k++)
				{
					lib_re[k] = 0.0f;
					lib_im[k] = 0.0f;
				}
				START_CYCLE_COUNT(cyc_start);
				(void)rfft32(fft_frame, lib_re, lib_im);
				STOP_CYCLE_COUNT(cyc_lib, cyc_start);

				acc_my  += (float)cyc_my;            /* 累计周期，后面统一除帧数 */
				acc_lib += (float)cyc_lib;
#else
				my_fft(fft_frame, spec);              /* 实序列 -> 复数谱   */
#endif
				my_fft_mag(spec, mag);                /* |X[k]|, k = 0..15  */

				/* 峰值谱线：从 k = 1 开始找，跳过直流（ADC 有直流偏置） */
				peak     = mag[1];
				peak_bin = 1u;
				for (k = 0; k < MY_FFT_N / 2; k++)
				{
					gFftMag[k] = mag[k] * (1.0f / (float)MY_FFT_N);
					gFftSpec[k] = spec[k];
					if ((k >= 1) && (mag[k] > peak))
					{
						peak     = mag[k];
						peak_bin = (uint32_t)k;
					}
				}
				for (k = MY_FFT_N / 2; k < MY_FFT_N; k++)
				{
					gFftSpec[k] = spec[k];            /* 后半个谱也存下来 */
				}

				gFftPeakBin    = peak_bin;
				gFftPeakFreqHz = gFftBinHz * (float)peak_bin;
				gFftFrameCount++;                     /* 累计完成的帧数 */

#ifdef MY_FFT_USE_RFFTN
				/* 两边的平均耗时（周期/次）与库函数结果 */
				time_used[0] = acc_my  / (float)gFftFrameCount;
				time_used[1] = acc_lib / (float)gFftFrameCount;

				for (k = 0; k < MY_FFT_N; k++)
				{
					gRfftRe[k] = lib_re[k];
					gRfftIm[k] = lib_im[k];
				}
#endif
			}
		}

		/* 本拍没有任何滤波器接在回环上时（E1/E2 都没打开），把输入直通，
		 * 既保证 DAC 还有输出，也让上面的频谱有意义。 */
		if (pFiltered == NULL)
		{
			pFiltered = inputBuffer;
		}

		/* 导出数据做对比（CCES 里选中变量 -> Export）：
		 *   gFftSpec.dat     32 个复数（re,im 交错），my_fft 的结果
		 *   gRfftRe/Im.dat   32 个 float，rfft32 的结果
		 *   gFftMag.dat      16 个 float，|X[k]|/N
		 *   time_used.dat    2 个 float，my_fft / rfft32 的平均周期数
		 *   gFftBinHz.dat    1 个 float，谱线间隔（8k 域 250 Hz / 48k 域 1.5 kHz）
		 * 然后用 MATLAB 脚本 compare_fft_trans.m 读这些 .dat 做对比。 */
#endif

// ======================================================================

			if (pFiltered == NULL)
			{
				return 1u;
			}

			for (i = 0; i < sampleCount; i++)
			{
				if      (pFiltered[i] >  (float)INT32_MAX) pDst32[i] = INT32_MAX;
				else if (pFiltered[i] <  (float)INT32_MIN) pDst32[i] = INT32_MIN;
				else                                       pDst32[i] = (int32_t)pFiltered[i];
			}

			//free(pFiltered);
        }

#ifdef CAPTURE_FOR_REPORT
        /* Warmup */
        if ((gCaptureStartRequest == 0u) && (gCaptureActive == 0u) && (gCaptureDone == 0u))
        {
            uint32_t framesPerBuffer = (AUDIO_BUFFER_SIZE / SAMPLE_SIZE) / NUM_CHANNELS;
            uint32_t warmupCallbacks = (SAMPLE_RATE * CAPTURE_WARMUP_SECONDS) / framesPerBuffer;
            if (AdcCount >= warmupCallbacks) gCaptureStartRequest = 1u;
        }
        if (gCaptureStartRequest != 0u)
        {
            gCaptureStartRequest = 0u;
            gCaptureInputFramesSeen = 0u;
            gCaptureFramesCaptured  = 0u;
            gCaptureDone            = 0u;
            gCaptureDoneNotified    = 0u;
            gCaptureActive          = 1u;
        }

        if (gCaptureActive != 0u)
        {
            uint32_t frameCount = AUDIO_BUFFER_SIZE / (sizeof(int32_t) * NUM_CHANNELS);
            int32_t *pSrc32 = (int32_t *)pSrc;
            int32_t *pDst32 = (int32_t *)pDst;
            uint32_t frame;
            for (frame = 0u; frame < frameCount; frame++)
            {
                uint32_t srcBase, sampleBase, ch;
                if (gCaptureFramesCaptured >= CAPTURE_TOTAL_FRAMES)
                {
                    gCaptureActive = 0u;
                    gCaptureDone = 1u;
                    break;
                }
                srcBase = frame * NUM_CHANNELS;
                if ((gCaptureInputFramesSeen % CAPTURE_DECIMATION) == 0u)
                {
                    sampleBase = gCaptureFramesCaptured * CAPTURE_CHANNELS;
                    for (ch = 0u; ch < CAPTURE_CHANNELS; ch++)
                    {
                        gRawCapture[sampleBase + ch]       = (int16_t)(pSrc32[srcBase + ch] >> CAPTURE_Q_FORMAT_SHIFT);
                        gProcessedCapture[sampleBase + ch] = (int16_t)(pDst32[srcBase + ch] >> CAPTURE_Q_FORMAT_SHIFT);
                    }
                    gCaptureFramesCaptured++;
                }
                gCaptureInputFramesSeen++;
            }
        }
#endif
#endif
#endif

        /* 用掉后清空，等下一轮配对 */
        pendingADC = NULL;
        pendingDAC = NULL;
    }

    return 0u;
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
 * 生成 1000 + 2000 + 6000 Hz 等比例线性叠加的合成正弦波
 *
 * Parameters
 *  pBuffer     - 目标缓冲区
 *  samples     - 每个通道的采样点数
 *  numchannels - 通道数
 *
 * Returns
 *  0 if success
 */
uint32_t Generate_CompositeSine(void *pBuffer, uint32_t samples, uint32_t numchannels)
{
    uint32_t i, ch;
    int32_t *p = (int32_t *)pBuffer;

    for(i = 0u; i < samples; i++)
    {
        float t = (float)i / (float)SAMPLE_RATE;
        float v = SYNTH_AMP1 * sinf(2.0f * PI * (float)SYNTH_FREQ1 * t)
                + SYNTH_AMP2 * sinf(2.0f * PI * (float)SYNTH_FREQ2 * t)
                + SYNTH_AMP3 * sinf(2.0f * PI * (float)SYNTH_FREQ3 * t);

        if(v >  (float)INT32_MAX) v =  (float)INT32_MAX;
        if(v <  (float)INT32_MIN) v =  (float)INT32_MIN;

        for(ch = 0u; ch < numchannels; ch++)
        {
#ifdef TDM_MODE
            p[i * numchannels + ch] = ((int32_t)v) << 8;
#else
            p[i * numchannels + ch] = (int32_t)v;
#endif
        }
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
