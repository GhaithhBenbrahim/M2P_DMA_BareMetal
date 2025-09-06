################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := M2P_DMA_BareMetal
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
M2P_DMA_BareMetal.elf \

MAP_FILES += \
M2P_DMA_BareMetal.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
M2P_DMA_BareMetal.list \


# All Target
all: main-build

# Main-build Target
main-build: M2P_DMA_BareMetal.elf secondary-outputs

# Tool invocations
M2P_DMA_BareMetal.elf M2P_DMA_BareMetal.map: $(OBJS) $(USER_OBJS) C:\Users\benbr\Documents\STM32\DMA-workspace\M2P_DMA_BareMetal\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "M2P_DMA_BareMetal.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\benbr\Documents\STM32\DMA-workspace\M2P_DMA_BareMetal\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="M2P_DMA_BareMetal.map" -Wl,--gc-sections -static -L"C:\Users\benbr\Documents\STM32\DMA-workspace\M2P_DMA_BareMetal\Drivers\CMSIS\Include" -L"C:\Users\benbr\Documents\STM32\DMA-workspace\M2P_DMA_BareMetal\Drivers\CMSIS\Device\ST\STM32F4xx\Include" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

M2P_DMA_BareMetal.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "M2P_DMA_BareMetal.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) M2P_DMA_BareMetal.elf M2P_DMA_BareMetal.list M2P_DMA_BareMetal.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
