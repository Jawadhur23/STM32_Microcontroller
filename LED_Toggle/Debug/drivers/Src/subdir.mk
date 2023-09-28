################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f4xx_driver.c 

OBJS += \
./drivers/Src/stm32f4xx_driver.o 

C_DEPS += \
./drivers/Src/stm32f4xx_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/Software/STM32/stm32f103c8t6_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f4xx_driver.cyclo ./drivers/Src/stm32f4xx_driver.d ./drivers/Src/stm32f4xx_driver.o ./drivers/Src/stm32f4xx_driver.su

.PHONY: clean-drivers-2f-Src

