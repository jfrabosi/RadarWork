################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/getting_started/example_detector_distance.c 

OBJS += \
./Src/examples/getting_started/example_detector_distance.o 

C_DEPS += \
./Src/examples/getting_started/example_detector_distance.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/getting_started/%.o Src/examples/getting_started/%.su Src/examples/getting_started/%.cyclo: ../Src/examples/getting_started/%.c Src/examples/getting_started/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I"C:/Users/frabb/Documents/RadarWork/xm125/Inc" -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-getting_started

clean-Src-2f-examples-2f-getting_started:
	-$(RM) ./Src/examples/getting_started/example_detector_distance.cyclo ./Src/examples/getting_started/example_detector_distance.d ./Src/examples/getting_started/example_detector_distance.o ./Src/examples/getting_started/example_detector_distance.su

.PHONY: clean-Src-2f-examples-2f-getting_started

