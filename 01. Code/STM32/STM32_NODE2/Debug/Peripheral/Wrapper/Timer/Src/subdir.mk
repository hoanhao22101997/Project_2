################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Wrapper/Timer/Src/timer.c 

OBJS += \
./Peripheral/Wrapper/Timer/Src/timer.o 

C_DEPS += \
./Peripheral/Wrapper/Timer/Src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Wrapper/Timer/Src/%.o Peripheral/Wrapper/Timer/Src/%.su Peripheral/Wrapper/Timer/Src/%.cyclo: ../Peripheral/Wrapper/Timer/Src/%.c Peripheral/Wrapper/Timer/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Monitor/Oled/Inc" -I../Core/Inc -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Analog/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Gpio/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Timer/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Sensor/SharpGP2Y10/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Sensor/DHT/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripheral-2f-Wrapper-2f-Timer-2f-Src

clean-Peripheral-2f-Wrapper-2f-Timer-2f-Src:
	-$(RM) ./Peripheral/Wrapper/Timer/Src/timer.cyclo ./Peripheral/Wrapper/Timer/Src/timer.d ./Peripheral/Wrapper/Timer/Src/timer.o ./Peripheral/Wrapper/Timer/Src/timer.su

.PHONY: clean-Peripheral-2f-Wrapper-2f-Timer-2f-Src

