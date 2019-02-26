################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../lsm6ds3/LSM6DS3.cpp 

OBJS += \
./lsm6ds3/LSM6DS3.o 

CPP_DEPS += \
./lsm6ds3/LSM6DS3.d 


# Each subdirectory must supply rules for building sources it contributes
lsm6ds3/%.o: ../lsm6ds3/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


