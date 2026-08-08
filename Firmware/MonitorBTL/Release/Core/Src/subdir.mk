################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/A_SmokeSensor.cpp \
../Core/Src/HX711.cpp \
../Core/Src/Output.cpp \
../Core/Src/Screen.cpp \
../Core/Src/Servo.cpp \
../Core/Src/SmokeSensor.cpp \
../Core/Src/main.cpp \
../Core/Src/parameters.cpp 

C_SRCS += \
../Core/Src/cJSON.c \
../Core/Src/debug_utils.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/cJSON.d \
./Core/Src/debug_utils.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/A_SmokeSensor.o \
./Core/Src/HX711.o \
./Core/Src/Output.o \
./Core/Src/Screen.o \
./Core/Src/Servo.o \
./Core/Src/SmokeSensor.o \
./Core/Src/cJSON.o \
./Core/Src/debug_utils.o \
./Core/Src/main.o \
./Core/Src/parameters.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

CPP_DEPS += \
./Core/Src/A_SmokeSensor.d \
./Core/Src/HX711.d \
./Core/Src/Output.d \
./Core/Src/Screen.d \
./Core/Src/Servo.d \
./Core/Src/SmokeSensor.d \
./Core/Src/main.d \
./Core/Src/parameters.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Fabri/Desktop/debugWork2/debugWork/monitor/Core/Src/bme680" -I"C:/Users/Fabri/Desktop/debugWork2/debugWork/monitor/Core/Inc/bme680" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Fabri/Desktop/debugWork2/debugWork/monitor/Core/Src/bme680" -I"C:/Users/Fabri/Desktop/debugWork2/debugWork/monitor/Core/Inc/bme680" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/A_SmokeSensor.cyclo ./Core/Src/A_SmokeSensor.d ./Core/Src/A_SmokeSensor.o ./Core/Src/A_SmokeSensor.su ./Core/Src/HX711.cyclo ./Core/Src/HX711.d ./Core/Src/HX711.o ./Core/Src/HX711.su ./Core/Src/Output.cyclo ./Core/Src/Output.d ./Core/Src/Output.o ./Core/Src/Output.su ./Core/Src/Screen.cyclo ./Core/Src/Screen.d ./Core/Src/Screen.o ./Core/Src/Screen.su ./Core/Src/Servo.cyclo ./Core/Src/Servo.d ./Core/Src/Servo.o ./Core/Src/Servo.su ./Core/Src/SmokeSensor.cyclo ./Core/Src/SmokeSensor.d ./Core/Src/SmokeSensor.o ./Core/Src/SmokeSensor.su ./Core/Src/cJSON.cyclo ./Core/Src/cJSON.d ./Core/Src/cJSON.o ./Core/Src/cJSON.su ./Core/Src/debug_utils.cyclo ./Core/Src/debug_utils.d ./Core/Src/debug_utils.o ./Core/Src/debug_utils.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/parameters.cyclo ./Core/Src/parameters.d ./Core/Src/parameters.o ./Core/Src/parameters.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

