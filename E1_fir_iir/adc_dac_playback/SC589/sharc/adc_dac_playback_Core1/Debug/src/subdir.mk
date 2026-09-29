################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Analog\ Devices/ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2/ADSP-SC5xx_EZ-KIT/Examples/drivers/adc/adau1979/adc_dac_playback/SC589/SoftConfig_SC589.c \
C:/Analog\ Devices/ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2/ADSP-SC5xx_EZ-KIT/Examples/drivers/adc/adau1979/adc_dac_playback/adc_dac_playback.c 

SRC_OBJS += \
./src/SoftConfig_SC589.doj \
./src/adc_dac_playback.doj 

C_DEPS += \
./src/SoftConfig_SC589.d \
./src/adc_dac_playback.d 


# Each subdirectory must supply rules for building sources it contributes
src/SoftConfig_SC589.doj: C:/Analog\ Devices/ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2/ADSP-SC5xx_EZ-KIT/Examples/drivers/adc/adau1979/adc_dac_playback/SC589/SoftConfig_SC589.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore SHARC C/C++ Compiler'
	cc21k -c -file-attr ProjectName="ADC_DAC_Playback_SC589_SHARC_Core1" -proc ADSP-SC589 -flags-compiler --no_wrap_diagnostics -si-revision any -g -DCORE1 -D_DEBUG -DADI_DEBUG @includes-05ecec1be144181c5b6cc87190875abb.txt -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -char-size-8 -swc -absolute-path-dependencies -gnu-style-dependencies -MD -Mo "src/SoftConfig_SC589.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/adc_dac_playback.doj: C:/Analog\ Devices/ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2/ADSP-SC5xx_EZ-KIT/Examples/drivers/adc/adau1979/adc_dac_playback/adc_dac_playback.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore SHARC C/C++ Compiler'
	cc21k -c -file-attr ProjectName="ADC_DAC_Playback_SC589_SHARC_Core1" -proc ADSP-SC589 -flags-compiler --no_wrap_diagnostics -si-revision any -g -DCORE1 -D_DEBUG -DADI_DEBUG @includes-05ecec1be144181c5b6cc87190875abb.txt -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -char-size-8 -swc -absolute-path-dependencies -gnu-style-dependencies -MD -Mo "src/adc_dac_playback.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


