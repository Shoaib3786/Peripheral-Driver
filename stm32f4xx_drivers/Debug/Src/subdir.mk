################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001_ToggleLED_PushPull.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/001_ToggleLED_PushPull.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/001_ToggleLED_PushPull.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"/Users/shoaib/Documents/MAIN/Codes/Embedded Resources/Embedded C/My_Workspace/target/stm32f4xx_drivers/drivers/Inc" -I../Inc -I"/Users/shoaib/Documents/MAIN/Codes/Embedded Resources/Embedded C/My_Workspace/target/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/001_ToggleLED_PushPull.cyclo ./Src/001_ToggleLED_PushPull.d ./Src/001_ToggleLED_PushPull.o ./Src/001_ToggleLED_PushPull.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

