################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/examples/JJH/jjh_v2.c 

OBJS += \
./Src/examples/JJH/jjh_v2.o 

C_DEPS += \
./Src/examples/JJH/jjh_v2.d 


# Each subdirectory must supply rules for building sources it contributes
Src/examples/JJH/%.o Src/examples/JJH/%.su Src/examples/JJH/%.cyclo: ../Src/examples/JJH/%.c Src/examples/JJH/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-examples-2f-JJH

clean-Src-2f-examples-2f-JJH:
	-$(RM) ./Src/examples/JJH/jjh_v2.cyclo ./Src/examples/JJH/jjh_v2.d ./Src/examples/JJH/jjh_v2.o ./Src/examples/JJH/jjh_v2.su

.PHONY: clean-Src-2f-examples-2f-JJH

