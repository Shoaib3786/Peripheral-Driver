################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f411xx_gpio_drivers.c 

OBJS += \
./drivers/Src/stm32f411xx_gpio_drivers.o 

C_DEPS += \
./drivers/Src/stm32f411xx_gpio_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"/Users/shoaib/Documents/MAIN/Codes/Embedded Resources/Embedded C/My_Workspace/target/stm32f4xx_drivers/drivers/Inc" -I../Inc -I"/Users/shoaib/Documents/MAIN/Codes/Embedded Resources/Embedded C/My_Workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f411xx_gpio_drivers.cyclo ./drivers/Src/stm32f411xx_gpio_drivers.d ./drivers/Src/stm32f411xx_gpio_drivers.o ./drivers/Src/stm32f411xx_gpio_drivers.su

.PHONY: clean-drivers-2f-Src

