################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Source/FreeRTOS_Source/list.c \
../Source/FreeRTOS_Source/queue.c \
../Source/FreeRTOS_Source/tasks.c \
../Source/FreeRTOS_Source/timers.c 

OBJS += \
./Source/FreeRTOS_Source/list.o \
./Source/FreeRTOS_Source/queue.o \
./Source/FreeRTOS_Source/tasks.o \
./Source/FreeRTOS_Source/timers.o 

C_DEPS += \
./Source/FreeRTOS_Source/list.d \
./Source/FreeRTOS_Source/queue.d \
./Source/FreeRTOS_Source/tasks.d \
./Source/FreeRTOS_Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Source/FreeRTOS_Source/%.o: ../Source/FreeRTOS_Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_CMSIS=CMSISv2p00_LPC11Uxx -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\RTOSDemo\Source\Common_Demo_Tasks\include" -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\Lib_MCU\inc" -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\RTOSDemo\Source" -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\RTOSDemo\Source\FreeRTOS_Source\include" -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\RTOSDemo\Source\FreeRTOS_Source\portable\GCC\ARM_CM0" -I"C:\Users\Marcelo Salgado\Documents\LPCXpresso_8.2.2_650\workspace\CMSISv2p00_LPC11Uxx\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -Wextra -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


