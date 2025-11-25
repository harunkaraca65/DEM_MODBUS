################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ModbusLib/Src/modbusSlave.c \
../ModbusLib/Src/modbus_crc.c 

OBJS += \
./ModbusLib/Src/modbusSlave.o \
./ModbusLib/Src/modbus_crc.o 

C_DEPS += \
./ModbusLib/Src/modbusSlave.d \
./ModbusLib/Src/modbus_crc.d 


# Each subdirectory must supply rules for building sources it contributes
ModbusLib/Src/%.o ModbusLib/Src/%.su ModbusLib/Src/%.cyclo: ../ModbusLib/Src/%.c ModbusLib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I"C:/ST/STProjects/DEMSAY/DEM_MODBUS_STM_SLAVE/lis2dw12" -I../TransformationApps/Inc -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../ModbusLib/Inc -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ModbusLib-2f-Src

clean-ModbusLib-2f-Src:
	-$(RM) ./ModbusLib/Src/modbusSlave.cyclo ./ModbusLib/Src/modbusSlave.d ./ModbusLib/Src/modbusSlave.o ./ModbusLib/Src/modbusSlave.su ./ModbusLib/Src/modbus_crc.cyclo ./ModbusLib/Src/modbus_crc.d ./ModbusLib/Src/modbus_crc.o ./ModbusLib/Src/modbus_crc.su

.PHONY: clean-ModbusLib-2f-Src

