################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bme680/bme68x.c \
../Core/Src/bme680/bme68x_necessary_functions.c 

C_DEPS += \
./Core/Src/bme680/bme68x.d \
./Core/Src/bme680/bme68x_necessary_functions.d 

OBJS += \
./Core/Src/bme680/bme68x.o \
./Core/Src/bme680/bme68x_necessary_functions.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/bme680/%.o Core/Src/bme680/%.su Core/Src/bme680/%.cyclo: ../Core/Src/bme680/%.c Core/Src/bme680/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fabri/Desktop/Clura Firmware smoke fixed/monitor/Core/Src/bme680" -I"C:/Users/fabri/Desktop/Clura Firmware smoke fixed/monitor/Core/Inc/bme680" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-bme680

clean-Core-2f-Src-2f-bme680:
	-$(RM) ./Core/Src/bme680/bme68x.cyclo ./Core/Src/bme680/bme68x.d ./Core/Src/bme680/bme68x.o ./Core/Src/bme680/bme68x.su ./Core/Src/bme680/bme68x_necessary_functions.cyclo ./Core/Src/bme680/bme68x_necessary_functions.d ./Core/Src/bme680/bme68x_necessary_functions.o ./Core/Src/bme680/bme68x_necessary_functions.su

.PHONY: clean-Core-2f-Src-2f-bme680

