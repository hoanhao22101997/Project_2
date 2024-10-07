################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.c 

OBJS += \
./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.o 

C_DEPS += \
./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Sensor/SharpGP2Y10/Src/%.o Peripheral/Sensor/SharpGP2Y10/Src/%.su Peripheral/Sensor/SharpGP2Y10/Src/%.cyclo: ../Peripheral/Sensor/SharpGP2Y10/Src/%.c Peripheral/Sensor/SharpGP2Y10/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Analog/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Gpio/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Timer/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Sensor/SharpGP2Y10/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Sensor/DHT/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripheral-2f-Sensor-2f-SharpGP2Y10-2f-Src

clean-Peripheral-2f-Sensor-2f-SharpGP2Y10-2f-Src:
	-$(RM) ./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.cyclo ./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.d ./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.o ./Peripheral/Sensor/SharpGP2Y10/Src/SharpGP2Y10.su

.PHONY: clean-Peripheral-2f-Sensor-2f-SharpGP2Y10-2f-Src

