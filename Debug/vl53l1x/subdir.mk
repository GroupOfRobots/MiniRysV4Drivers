################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../vl53l1x/VL53L1X.cpp 

OBJS += \
./vl53l1x/VL53L1X.o 

CPP_DEPS += \
./vl53l1x/VL53L1X.d 


# Each subdirectory must supply rules for building sources it contributes
vl53l1x/%.o: ../vl53l1x/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I../__GXX_EXPERIMENTAL_CXX0X__ -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


