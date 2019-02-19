################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../l6470/src/autodriver.cpp \
../l6470/src/l6470.cpp \
../l6470/src/l6470commands.cpp \
../l6470/src/l6470config.cpp \
../l6470/src/l6470dump.cpp \
../l6470/src/l6470support.cpp 

OBJS += \
./l6470/src/autodriver.o \
./l6470/src/l6470.o \
./l6470/src/l6470commands.o \
./l6470/src/l6470config.o \
./l6470/src/l6470dump.o \
./l6470/src/l6470support.o 

CPP_DEPS += \
./l6470/src/autodriver.d \
./l6470/src/l6470.d \
./l6470/src/l6470commands.d \
./l6470/src/l6470config.d \
./l6470/src/l6470dump.d \
./l6470/src/l6470support.d 


# Each subdirectory must supply rules for building sources it contributes
l6470/src/%.o: ../l6470/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I/home/kamil/bcm2835-1.50/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


