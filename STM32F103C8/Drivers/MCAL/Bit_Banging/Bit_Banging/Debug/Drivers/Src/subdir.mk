################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f1xx_hal_gpio_driver.c \
../Drivers/Src/stm32f1xx_hal_spi.c 

OBJS += \
./Drivers/Src/stm32f1xx_hal_gpio_driver.o \
./Drivers/Src/stm32f1xx_hal_spi.o 

C_DEPS += \
./Drivers/Src/stm32f1xx_hal_gpio_driver.d \
./Drivers/Src/stm32f1xx_hal_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f1xx_hal_gpio_driver.o: ../Drivers/Src/stm32f1xx_hal_gpio_driver.c
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"C:/Users/abdul/Desktop/Bit_Banging/Bit_Banging/Drivers/Includes" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f1xx_hal_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f1xx_hal_spi.o: ../Drivers/Src/stm32f1xx_hal_spi.c
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"C:/Users/abdul/Desktop/Bit_Banging/Bit_Banging/Drivers/Includes" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f1xx_hal_spi.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

