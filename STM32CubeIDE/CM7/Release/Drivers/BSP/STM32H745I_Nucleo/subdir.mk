################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jibon/STM32CubeIDE/workspace_1.13.1/ADC_DualModeInterleaved/Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.o: C:/Users/jibon/STM32CubeIDE/workspace_1.13.1/ADC_DualModeInterleaved/Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.c Drivers/BSP/STM32H745I_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DSTM32H745xx -DUSE_HAL_DRIVER -DCORE_CM7 -DUSE_STM32H7XX_NUCLEO_144_MB1363 -c -I../../../CM7/Inc -I../../../Common/Inc -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../../Drivers/BSP/STM32H7xx_Nucleo -I../../../Utilities/Fonts -I../../../Utilities/CPU -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H745I_Nucleo

clean-Drivers-2f-BSP-2f-STM32H745I_Nucleo:
	-$(RM) ./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.cyclo ./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.d ./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.o ./Drivers/BSP/STM32H745I_Nucleo/stm32h7xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32H745I_Nucleo

