################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ff.c \
../src/main.c \
../src/mmc.c \
../src/platform.c 

LD_SRCS += \
../src/lscript.ld 

OBJS += \
./src/ff.o \
./src/main.o \
./src/mmc.o \
./src/platform.o 

C_DEPS += \
./src/ff.d \
./src/main.d \
./src/mmc.d \
./src/platform.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM gcc compiler'
	arm-xilinx-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -I../../standalone_bsp_0/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


