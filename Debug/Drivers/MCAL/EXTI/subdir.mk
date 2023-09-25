################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCAL/EXTI/stm32f103c8_exti_driver.c 

OBJS += \
./Drivers/MCAL/EXTI/stm32f103c8_exti_driver.o 

C_DEPS += \
./Drivers/MCAL/EXTI/stm32f103c8_exti_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCAL/EXTI/stm32f103c8_exti_driver.o: ../Drivers/MCAL/EXTI/stm32f103c8_exti_driver.c
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"C:/Users/abdullah/Desktop/My_Drivers/STM32F103C8/STM32F103C8/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MCAL/EXTI/stm32f103c8_exti_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

