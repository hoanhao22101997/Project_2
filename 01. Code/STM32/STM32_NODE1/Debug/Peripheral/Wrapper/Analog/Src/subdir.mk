################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Wrapper/Analog/Src/analog.c 

OBJS += \
./Peripheral/Wrapper/Analog/Src/analog.o 

C_DEPS += \
./Peripheral/Wrapper/Analog/Src/analog.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Wrapper/Analog/Src/%.o Peripheral/Wrapper/Analog/Src/%.su Peripheral/Wrapper/Analog/Src/%.cyclo: ../Peripheral/Wrapper/Analog/Src/%.c Peripheral/Wrapper/Analog/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Monitor/Oled/Inc" -I../Core/Inc -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Wrapper/Analog/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Wrapper/Gpio/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Wrapper/Timer/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Sensor/SharpGP2Y10/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE1/Peripheral/Sensor/DHT/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripheral-2f-Wrapper-2f-Analog-2f-Src

clean-Peripheral-2f-Wrapper-2f-Analog-2f-Src:
	-$(RM) ./Peripheral/Wrapper/Analog/Src/analog.cyclo ./Peripheral/Wrapper/Analog/Src/analog.d ./Peripheral/Wrapper/Analog/Src/analog.o ./Peripheral/Wrapper/Analog/Src/analog.su

.PHONY: clean-Peripheral-2f-Wrapper-2f-Analog-2f-Src

