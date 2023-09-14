################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.c 

OBJS += \
./Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.o 

C_DEPS += \
./Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.o: ../Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.c
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I"C:/Users/abdullah/Desktop/My_Drivers/STM32F103x6/STM32F103x6_Drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MCAL/GPIO/STM32F103x6_GPIO_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

