################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/applications/i2c/acc_reg_protocol.c \
../Src/applications/i2c/distance_reg_protocol.c \
../Src/applications/i2c/distance_reg_protocol_access.c \
../Src/applications/i2c/i2c_application_system_stm32.c \
../Src/applications/i2c/i2c_distance_detector.c \
../Src/applications/i2c/i2c_presence_detector.c \
../Src/applications/i2c/i2c_ref_app_breathing.c \
../Src/applications/i2c/presence_reg_protocol.c \
../Src/applications/i2c/presence_reg_protocol_access.c \
../Src/applications/i2c/ref_app_breathing_reg_protocol.c \
../Src/applications/i2c/ref_app_breathing_reg_protocol_access.c 

OBJS += \
./Src/applications/i2c/acc_reg_protocol.o \
./Src/applications/i2c/distance_reg_protocol.o \
./Src/applications/i2c/distance_reg_protocol_access.o \
./Src/applications/i2c/i2c_application_system_stm32.o \
./Src/applications/i2c/i2c_distance_detector.o \
./Src/applications/i2c/i2c_presence_detector.o \
./Src/applications/i2c/i2c_ref_app_breathing.o \
./Src/applications/i2c/presence_reg_protocol.o \
./Src/applications/i2c/presence_reg_protocol_access.o \
./Src/applications/i2c/ref_app_breathing_reg_protocol.o \
./Src/applications/i2c/ref_app_breathing_reg_protocol_access.o 

C_DEPS += \
./Src/applications/i2c/acc_reg_protocol.d \
./Src/applications/i2c/distance_reg_protocol.d \
./Src/applications/i2c/distance_reg_protocol_access.d \
./Src/applications/i2c/i2c_application_system_stm32.d \
./Src/applications/i2c/i2c_distance_detector.d \
./Src/applications/i2c/i2c_presence_detector.d \
./Src/applications/i2c/i2c_ref_app_breathing.d \
./Src/applications/i2c/presence_reg_protocol.d \
./Src/applications/i2c/presence_reg_protocol_access.d \
./Src/applications/i2c/ref_app_breathing_reg_protocol.d \
./Src/applications/i2c/ref_app_breathing_reg_protocol_access.d 


# Each subdirectory must supply rules for building sources it contributes
Src/applications/i2c/%.o Src/applications/i2c/%.su Src/applications/i2c/%.cyclo: ../Src/applications/i2c/%.c Src/applications/i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/frabb/Documents/RadarWork/xm125/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-applications-2f-i2c

clean-Src-2f-applications-2f-i2c:
	-$(RM) ./Src/applications/i2c/acc_reg_protocol.cyclo ./Src/applications/i2c/acc_reg_protocol.d ./Src/applications/i2c/acc_reg_protocol.o ./Src/applications/i2c/acc_reg_protocol.su ./Src/applications/i2c/distance_reg_protocol.cyclo ./Src/applications/i2c/distance_reg_protocol.d ./Src/applications/i2c/distance_reg_protocol.o ./Src/applications/i2c/distance_reg_protocol.su ./Src/applications/i2c/distance_reg_protocol_access.cyclo ./Src/applications/i2c/distance_reg_protocol_access.d ./Src/applications/i2c/distance_reg_protocol_access.o ./Src/applications/i2c/distance_reg_protocol_access.su ./Src/applications/i2c/i2c_application_system_stm32.cyclo ./Src/applications/i2c/i2c_application_system_stm32.d ./Src/applications/i2c/i2c_application_system_stm32.o ./Src/applications/i2c/i2c_application_system_stm32.su ./Src/applications/i2c/i2c_distance_detector.cyclo ./Src/applications/i2c/i2c_distance_detector.d ./Src/applications/i2c/i2c_distance_detector.o ./Src/applications/i2c/i2c_distance_detector.su ./Src/applications/i2c/i2c_presence_detector.cyclo ./Src/applications/i2c/i2c_presence_detector.d ./Src/applications/i2c/i2c_presence_detector.o ./Src/applications/i2c/i2c_presence_detector.su ./Src/applications/i2c/i2c_ref_app_breathing.cyclo ./Src/applications/i2c/i2c_ref_app_breathing.d ./Src/applications/i2c/i2c_ref_app_breathing.o ./Src/applications/i2c/i2c_ref_app_breathing.su ./Src/applications/i2c/presence_reg_protocol.cyclo ./Src/applications/i2c/presence_reg_protocol.d ./Src/applications/i2c/presence_reg_protocol.o ./Src/applications/i2c/presence_reg_protocol.su ./Src/applications/i2c/presence_reg_protocol_access.cyclo ./Src/applications/i2c/presence_reg_protocol_access.d ./Src/applications/i2c/presence_reg_protocol_access.o ./Src/applications/i2c/presence_reg_protocol_access.su ./Src/applications/i2c/ref_app_breathing_reg_protocol.cyclo ./Src/applications/i2c/ref_app_breathing_reg_protocol.d ./Src/applications/i2c/ref_app_breathing_reg_protocol.o ./Src/applications/i2c/ref_app_breathing_reg_protocol.su ./Src/applications/i2c/ref_app_breathing_reg_protocol_access.cyclo ./Src/applications/i2c/ref_app_breathing_reg_protocol_access.d ./Src/applications/i2c/ref_app_breathing_reg_protocol_access.o ./Src/applications/i2c/ref_app_breathing_reg_protocol_access.su

.PHONY: clean-Src-2f-applications-2f-i2c

