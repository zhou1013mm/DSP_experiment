################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/empty_main.c 

SRC_OBJS += \
./src/empty_main.doj 

C_DEPS += \
./src/empty_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/empty_main.doj: ../src/empty_main.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore SHARC C/C++ Compiler'
	cc21k -c -file-attr ProjectName="ADC_DAC_Playback_SC589_SHARC_Core2" -proc ADSP-SC589 -flags-compiler --no_wrap_diagnostics -si-revision any -g -DCORE2 -D_DEBUG -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -char-size-8 -swc -absolute-path-dependencies -gnu-style-dependencies -MD -Mo "src/empty_main.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


