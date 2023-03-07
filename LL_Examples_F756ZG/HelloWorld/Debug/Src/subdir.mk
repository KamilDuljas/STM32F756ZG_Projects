################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f7xx.c 

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F756ZGTx -DSTM32F7 -DSTM32F756xx -c -I../Inc -I"C:/sources/STM32F756ZG_Projects/LL_Examples_F756ZG/HelloWorld/Drivers/CMSIS/Inc" -I"C:/sources/STM32F756ZG_Projects/LL_Examples_F756ZG/HelloWorld/Drivers/CMSIS/Device/Inc" -I"C:/sources/STM32F756ZG_Projects/LL_Examples_F756ZG/HelloWorld/Drivers/CMSIS/Device/STM32F7xx_LL_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f7xx.d ./Src/system_stm32f7xx.o ./Src/system_stm32f7xx.su

.PHONY: clean-Src

