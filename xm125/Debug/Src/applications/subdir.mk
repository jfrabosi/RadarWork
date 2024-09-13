################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/applications/acc_exploration_server_stm32.c 

OBJS += \
./Src/applications/acc_exploration_server_stm32.o 

C_DEPS += \
./Src/applications/acc_exploration_server_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/applications/%.o Src/applications/%.su Src/applications/%.cyclo: ../Src/applications/%.c Src/applications/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/frabb/Documents/RadarWork/xm125/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-applications

clean-Src-2f-applications:
	-$(RM) ./Src/applications/acc_exploration_server_stm32.cyclo ./Src/applications/acc_exploration_server_stm32.d ./Src/applications/acc_exploration_server_stm32.o ./Src/applications/acc_exploration_server_stm32.su

.PHONY: clean-Src-2f-applications

