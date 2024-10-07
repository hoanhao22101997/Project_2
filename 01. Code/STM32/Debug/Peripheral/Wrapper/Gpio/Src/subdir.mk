################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Wrapper/Gpio/Src/gpio.c 

OBJS += \
./Peripheral/Wrapper/Gpio/Src/gpio.o 

C_DEPS += \
./Peripheral/Wrapper/Gpio/Src/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Wrapper/Gpio/Src/%.o Peripheral/Wrapper/Gpio/Src/%.su Peripheral/Wrapper/Gpio/Src/%.cyclo: ../Peripheral/Wrapper/Gpio/Src/%.c Peripheral/Wrapper/Gpio/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Analog/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Gpio/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Wrapper/Timer/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Sensor/SharpGP2Y10/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/Peripheral/Sensor/DHT/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripheral-2f-Wrapper-2f-Gpio-2f-Src

clean-Peripheral-2f-Wrapper-2f-Gpio-2f-Src:
	-$(RM) ./Peripheral/Wrapper/Gpio/Src/gpio.cyclo ./Peripheral/Wrapper/Gpio/Src/gpio.d ./Peripheral/Wrapper/Gpio/Src/gpio.o ./Peripheral/Wrapper/Gpio/Src/gpio.su

.PHONY: clean-Peripheral-2f-Wrapper-2f-Gpio-2f-Src

