################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BME280/bme280.c 

OBJS += \
./Core/Src/BME280/bme280.o 

C_DEPS += \
./Core/Src/BME280/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BME280/%.o Core/Src/BME280/%.su Core/Src/BME280/%.cyclo: ../Core/Src/BME280/%.c Core/Src/BME280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BME280

clean-Core-2f-Src-2f-BME280:
	-$(RM) ./Core/Src/BME280/bme280.cyclo ./Core/Src/BME280/bme280.d ./Core/Src/BME280/bme280.o ./Core/Src/BME280/bme280.su

.PHONY: clean-Core-2f-Src-2f-BME280

