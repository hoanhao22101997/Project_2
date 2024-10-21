################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Monitor/Oled/Src/fonts.c \
../Peripheral/Monitor/Oled/Src/ssd1306.c 

OBJS += \
./Peripheral/Monitor/Oled/Src/fonts.o \
./Peripheral/Monitor/Oled/Src/ssd1306.o 

C_DEPS += \
./Peripheral/Monitor/Oled/Src/fonts.d \
./Peripheral/Monitor/Oled/Src/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Monitor/Oled/Src/%.o Peripheral/Monitor/Oled/Src/%.su Peripheral/Monitor/Oled/Src/%.cyclo: ../Peripheral/Monitor/Oled/Src/%.c Peripheral/Monitor/Oled/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Monitor/Oled/Inc" -I../Core/Inc -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Analog/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Gpio/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Wrapper/Timer/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Sensor/SharpGP2Y10/Inc" -I"C:/Users/MSI/Desktop/DAMH2/Project_2/01. Code/STM32/STM32_NODE2/Peripheral/Sensor/DHT/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripheral-2f-Monitor-2f-Oled-2f-Src

clean-Peripheral-2f-Monitor-2f-Oled-2f-Src:
	-$(RM) ./Peripheral/Monitor/Oled/Src/fonts.cyclo ./Peripheral/Monitor/Oled/Src/fonts.d ./Peripheral/Monitor/Oled/Src/fonts.o ./Peripheral/Monitor/Oled/Src/fonts.su ./Peripheral/Monitor/Oled/Src/ssd1306.cyclo ./Peripheral/Monitor/Oled/Src/ssd1306.d ./Peripheral/Monitor/Oled/Src/ssd1306.o ./Peripheral/Monitor/Oled/Src/ssd1306.su

.PHONY: clean-Peripheral-2f-Monitor-2f-Oled-2f-Src

