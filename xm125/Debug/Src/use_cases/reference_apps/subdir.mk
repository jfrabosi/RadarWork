################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/use_cases/reference_apps/ref_app_breathing.c \
../Src/use_cases/reference_apps/ref_app_breathing_main.c \
../Src/use_cases/reference_apps/ref_app_parking.c \
../Src/use_cases/reference_apps/ref_app_parking_main.c \
../Src/use_cases/reference_apps/ref_app_smart_presence.c \
../Src/use_cases/reference_apps/ref_app_tank_level.c \
../Src/use_cases/reference_apps/ref_app_touchless_button.c 

OBJS += \
./Src/use_cases/reference_apps/ref_app_breathing.o \
./Src/use_cases/reference_apps/ref_app_breathing_main.o \
./Src/use_cases/reference_apps/ref_app_parking.o \
./Src/use_cases/reference_apps/ref_app_parking_main.o \
./Src/use_cases/reference_apps/ref_app_smart_presence.o \
./Src/use_cases/reference_apps/ref_app_tank_level.o \
./Src/use_cases/reference_apps/ref_app_touchless_button.o 

C_DEPS += \
./Src/use_cases/reference_apps/ref_app_breathing.d \
./Src/use_cases/reference_apps/ref_app_breathing_main.d \
./Src/use_cases/reference_apps/ref_app_parking.d \
./Src/use_cases/reference_apps/ref_app_parking_main.d \
./Src/use_cases/reference_apps/ref_app_smart_presence.d \
./Src/use_cases/reference_apps/ref_app_tank_level.d \
./Src/use_cases/reference_apps/ref_app_touchless_button.d 


# Each subdirectory must supply rules for building sources it contributes
Src/use_cases/reference_apps/%.o Src/use_cases/reference_apps/%.su Src/use_cases/reference_apps/%.cyclo: ../Src/use_cases/reference_apps/%.c Src/use_cases/reference_apps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I"C:/Users/frabb/Documents/RadarWork/xm125/Inc" -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-use_cases-2f-reference_apps

clean-Src-2f-use_cases-2f-reference_apps:
	-$(RM) ./Src/use_cases/reference_apps/ref_app_breathing.cyclo ./Src/use_cases/reference_apps/ref_app_breathing.d ./Src/use_cases/reference_apps/ref_app_breathing.o ./Src/use_cases/reference_apps/ref_app_breathing.su ./Src/use_cases/reference_apps/ref_app_breathing_main.cyclo ./Src/use_cases/reference_apps/ref_app_breathing_main.d ./Src/use_cases/reference_apps/ref_app_breathing_main.o ./Src/use_cases/reference_apps/ref_app_breathing_main.su ./Src/use_cases/reference_apps/ref_app_parking.cyclo ./Src/use_cases/reference_apps/ref_app_parking.d ./Src/use_cases/reference_apps/ref_app_parking.o ./Src/use_cases/reference_apps/ref_app_parking.su ./Src/use_cases/reference_apps/ref_app_parking_main.cyclo ./Src/use_cases/reference_apps/ref_app_parking_main.d ./Src/use_cases/reference_apps/ref_app_parking_main.o ./Src/use_cases/reference_apps/ref_app_parking_main.su ./Src/use_cases/reference_apps/ref_app_smart_presence.cyclo ./Src/use_cases/reference_apps/ref_app_smart_presence.d ./Src/use_cases/reference_apps/ref_app_smart_presence.o ./Src/use_cases/reference_apps/ref_app_smart_presence.su ./Src/use_cases/reference_apps/ref_app_tank_level.cyclo ./Src/use_cases/reference_apps/ref_app_tank_level.d ./Src/use_cases/reference_apps/ref_app_tank_level.o ./Src/use_cases/reference_apps/ref_app_tank_level.su ./Src/use_cases/reference_apps/ref_app_touchless_button.cyclo ./Src/use_cases/reference_apps/ref_app_touchless_button.d ./Src/use_cases/reference_apps/ref_app_touchless_button.o ./Src/use_cases/reference_apps/ref_app_touchless_button.su

.PHONY: clean-Src-2f-use_cases-2f-reference_apps

