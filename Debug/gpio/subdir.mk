################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../gpio/gpio.cpp 

OBJS += \
./gpio/gpio.o 

CPP_DEPS += \
./gpio/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
gpio/%.o: ../gpio/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
#	arm-linux-gnueabihf-g++ -std=c++0x -I../__GXX_EXPERIMENTAL_CXX0X__ 
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


