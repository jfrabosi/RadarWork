################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/helper/acc_control_helper.c \
../Src/examples/helper/acc_processing_helpers.c 

OBJS += \
./Src/examples/helper/acc_control_helper.o \
./Src/examples/helper/acc_processing_helpers.o 

C_DEPS += \
./Src/examples/helper/acc_control_helper.d \
./Src/examples/helper/acc_processing_helpers.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/helper/%.o Src/examples/helper/%.su Src/examples/helper/%.cyclo: ../Src/examples/helper/%.c Src/examples/helper/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-helper

clean-Src-2f-examples-2f-helper:
	-$(RM) ./Src/examples/helper/acc_control_helper.cyclo ./Src/examples/helper/acc_control_helper.d ./Src/examples/helper/acc_control_helper.o ./Src/examples/helper/acc_control_helper.su ./Src/examples/helper/acc_processing_helpers.cyclo ./Src/examples/helper/acc_processing_helpers.d ./Src/examples/helper/acc_processing_helpers.o ./Src/examples/helper/acc_processing_helpers.su

.PHONY: clean-Src-2f-examples-2f-helper

