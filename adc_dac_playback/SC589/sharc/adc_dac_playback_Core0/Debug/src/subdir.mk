################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/initialize_cores.c 

SRC_OBJS += \
./src/initialize_cores.o 

C_DEPS += \
./src/initialize_cores.d 


# Each subdirectory must supply rules for building sources it contributes
src/initialize_cores.o: ../src/initialize_cores.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore ARM Bare Metal C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG @includes-e64be1689249daad5bbc9712ff709209.txt -Wall -c -mproc=ADSP-SC589 -msi-revision=any -MMD -MP -MF"src/initialize_cores.d" -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


