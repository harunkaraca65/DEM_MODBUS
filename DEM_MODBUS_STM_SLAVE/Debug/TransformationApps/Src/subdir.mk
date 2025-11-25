################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TransformationApps/Src/inputs.c \
../TransformationApps/Src/outpus.c 

OBJS += \
./TransformationApps/Src/inputs.o \
./TransformationApps/Src/outpus.o 

C_DEPS += \
./TransformationApps/Src/inputs.d \
./TransformationApps/Src/outpus.d 


# Each subdirectory must supply rules for building sources it contributes
TransformationApps/Src/%.o TransformationApps/Src/%.su TransformationApps/Src/%.cyclo: ../TransformationApps/Src/%.c TransformationApps/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I"C:/ST/STProjects/DEMSAY/DEM_MODBUS_STM_SLAVE/lis2dw12" -I../TransformationApps/Inc -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../ModbusLib/Inc -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TransformationApps-2f-Src

clean-TransformationApps-2f-Src:
	-$(RM) ./TransformationApps/Src/inputs.cyclo ./TransformationApps/Src/inputs.d ./TransformationApps/Src/inputs.o ./TransformationApps/Src/inputs.su ./TransformationApps/Src/outpus.cyclo ./TransformationApps/Src/outpus.d ./TransformationApps/Src/outpus.o ./TransformationApps/Src/outpus.su

.PHONY: clean-TransformationApps-2f-Src

