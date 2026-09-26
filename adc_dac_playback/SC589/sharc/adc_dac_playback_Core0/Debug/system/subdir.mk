################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/adi_initialize.c 

SRC_OBJS += \
./system/adi_initialize.o 

C_DEPS += \
./system/adi_initialize.d 


# Each subdirectory must supply rules for building sources it contributes
system/adi_initialize.o: ../system/adi_initialize.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore ARM Bare Metal C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG @includes-e64be1689249daad5bbc9712ff709209.txt -Wall -c -mproc=ADSP-SC589 -msi-revision=any -MMD -MP -MF"system/adi_initialize.d" -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


