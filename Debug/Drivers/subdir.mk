################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/system_stm32f1xx.c 

OBJS += \
./Drivers/system_stm32f1xx.o 

C_DEPS += \
./Drivers/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/system_stm32f1xx.o: ../Drivers/system_stm32f1xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/system_stm32f1xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

