################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/advanced_control/example_detector_distance_low_power_off.c 

OBJS += \
./Src/examples/advanced_control/example_detector_distance_low_power_off.o 

C_DEPS += \
./Src/examples/advanced_control/example_detector_distance_low_power_off.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/advanced_control/%.o Src/examples/advanced_control/%.su Src/examples/advanced_control/%.cyclo: ../Src/examples/advanced_control/%.c Src/examples/advanced_control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-advanced_control

clean-Src-2f-examples-2f-advanced_control:
	-$(RM) ./Src/examples/advanced_control/example_detector_distance_low_power_off.cyclo ./Src/examples/advanced_control/example_detector_distance_low_power_off.d ./Src/examples/advanced_control/example_detector_distance_low_power_off.o ./Src/examples/advanced_control/example_detector_distance_low_power_off.su

.PHONY: clean-Src-2f-examples-2f-advanced_control

