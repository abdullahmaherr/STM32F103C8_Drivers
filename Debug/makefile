################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/MCAL/RCC/subdir.mk
-include Drivers/MCAL/NVIC/subdir.mk
-include Drivers/MCAL/GPIO/subdir.mk
-include Drivers/MCAL/EXTI/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F103x6_Drivers.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F103x6_Drivers.list \

OBJCOPY_BIN += \
STM32F103x6_Drivers.bin \


# All Target
all: STM32F103x6_Drivers.elf secondary-outputs

# Tool invocations
STM32F103x6_Drivers.elf: $(OBJS) $(USER_OBJS) C:\Users\abdullah\Desktop\My_Drivers\STM32F103x6\STM32F103x6_Drivers\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -gdwarf-2 -o "STM32F103x6_Drivers.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\abdullah\Desktop\My_Drivers\STM32F103x6\STM32F103x6_Drivers\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F103x6_Drivers.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F103x6_Drivers.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F103x6_Drivers.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F103x6_Drivers.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F103x6_Drivers.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
