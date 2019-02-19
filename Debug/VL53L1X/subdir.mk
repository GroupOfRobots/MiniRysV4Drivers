################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../VL53L1X/VL53L1X.cpp 

OBJS += \
./VL53L1X/VL53L1X.o 

CPP_DEPS += \
./VL53L1X/VL53L1X.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L1X/%.o: ../VL53L1X/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/kamil/bcm2835-1.50/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


