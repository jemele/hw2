################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/testperiph.c \
../src/xdevcfg_selftest_example.c \
../src/xdmaps_example_w_intr.c \
../src/xgpio_tapp_example.c \
../src/xiicps_selftest_example.c \
../src/xscugic_tapp_example.c \
../src/xscutimer_intr_example.c \
../src/xscutimer_polled_example.c \
../src/xscuwdt_intr_example.c \
../src/xwdtps_selftest_example.c 

LD_SRCS += \
../src/lscript.ld 

OBJS += \
./src/testperiph.o \
./src/xdevcfg_selftest_example.o \
./src/xdmaps_example_w_intr.o \
./src/xgpio_tapp_example.o \
./src/xiicps_selftest_example.o \
./src/xscugic_tapp_example.o \
./src/xscutimer_intr_example.o \
./src/xscutimer_polled_example.o \
./src/xscuwdt_intr_example.o \
./src/xwdtps_selftest_example.o 

C_DEPS += \
./src/testperiph.d \
./src/xdevcfg_selftest_example.d \
./src/xdmaps_example_w_intr.d \
./src/xgpio_tapp_example.d \
./src/xiicps_selftest_example.d \
./src/xscugic_tapp_example.d \
./src/xscutimer_intr_example.d \
./src/xscutimer_polled_example.d \
./src/xscuwdt_intr_example.d \
./src/xwdtps_selftest_example.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM gcc compiler'
	arm-xilinx-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -I../../standalone_bsp_0/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


