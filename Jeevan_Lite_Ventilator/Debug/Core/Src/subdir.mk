################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/APRV.c \
../Core/Src/Alert.c \
../Core/Src/BI-Pap.c \
../Core/Src/Back_UP_PC_CMV.c \
../Core/Src/Back_UP_VC_CMV.c \
../Core/Src/Calibration.c \
../Core/Src/Cpap.c \
../Core/Src/Flow_Sensors_Data.c \
../Core/Src/HFNC.c \
../Core/Src/Nebuliser.c \
../Core/Src/Oxygen_Blending.c \
../Core/Src/Pc_SIMV.c \
../Core/Src/Pc_cmv.c \
../Core/Src/Pressure_Sensors_Data.c \
../Core/Src/Psv.c \
../Core/Src/Service.c \
../Core/Src/Uart.c \
../Core/Src/Vc_SIMV.c \
../Core/Src/Vc_cmv.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/APRV.o \
./Core/Src/Alert.o \
./Core/Src/BI-Pap.o \
./Core/Src/Back_UP_PC_CMV.o \
./Core/Src/Back_UP_VC_CMV.o \
./Core/Src/Calibration.o \
./Core/Src/Cpap.o \
./Core/Src/Flow_Sensors_Data.o \
./Core/Src/HFNC.o \
./Core/Src/Nebuliser.o \
./Core/Src/Oxygen_Blending.o \
./Core/Src/Pc_SIMV.o \
./Core/Src/Pc_cmv.o \
./Core/Src/Pressure_Sensors_Data.o \
./Core/Src/Psv.o \
./Core/Src/Service.o \
./Core/Src/Uart.o \
./Core/Src/Vc_SIMV.o \
./Core/Src/Vc_cmv.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/APRV.d \
./Core/Src/Alert.d \
./Core/Src/BI-Pap.d \
./Core/Src/Back_UP_PC_CMV.d \
./Core/Src/Back_UP_VC_CMV.d \
./Core/Src/Calibration.d \
./Core/Src/Cpap.d \
./Core/Src/Flow_Sensors_Data.d \
./Core/Src/HFNC.d \
./Core/Src/Nebuliser.d \
./Core/Src/Oxygen_Blending.d \
./Core/Src/Pc_SIMV.d \
./Core/Src/Pc_cmv.d \
./Core/Src/Pressure_Sensors_Data.d \
./Core/Src/Psv.d \
./Core/Src/Service.d \
./Core/Src/Uart.d \
./Core/Src/Vc_SIMV.d \
./Core/Src/Vc_cmv.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/APRV.d ./Core/Src/APRV.o ./Core/Src/Alert.d ./Core/Src/Alert.o ./Core/Src/BI-Pap.d ./Core/Src/BI-Pap.o ./Core/Src/Back_UP_PC_CMV.d ./Core/Src/Back_UP_PC_CMV.o ./Core/Src/Back_UP_VC_CMV.d ./Core/Src/Back_UP_VC_CMV.o ./Core/Src/Calibration.d ./Core/Src/Calibration.o ./Core/Src/Cpap.d ./Core/Src/Cpap.o ./Core/Src/Flow_Sensors_Data.d ./Core/Src/Flow_Sensors_Data.o ./Core/Src/HFNC.d ./Core/Src/HFNC.o ./Core/Src/Nebuliser.d ./Core/Src/Nebuliser.o ./Core/Src/Oxygen_Blending.d ./Core/Src/Oxygen_Blending.o ./Core/Src/Pc_SIMV.d ./Core/Src/Pc_SIMV.o ./Core/Src/Pc_cmv.d ./Core/Src/Pc_cmv.o ./Core/Src/Pressure_Sensors_Data.d ./Core/Src/Pressure_Sensors_Data.o ./Core/Src/Psv.d ./Core/Src/Psv.o ./Core/Src/Service.d ./Core/Src/Service.o ./Core/Src/Uart.d ./Core/Src/Uart.o ./Core/Src/Vc_SIMV.d ./Core/Src/Vc_SIMV.o ./Core/Src/Vc_cmv.d ./Core/Src/Vc_cmv.o ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o

.PHONY: clean-Core-2f-Src

