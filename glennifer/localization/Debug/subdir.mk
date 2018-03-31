################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../Edge.cc \
../FloatImage.cc \
../GLine2D.cc \
../GLineSegment2D.cc \
../Gaussian.cc \
../GrayModel.cc \
../Homography33.cc \
../MathUtil.cc \
../Quad.cc \
../Segment.cc \
../TagDetection.cc \
../TagDetector.cc \
../TagFamily.cc \
../UnionFindSimple.cc 

CPP_SRCS += \
../Serial.cpp \
../apriltags_demo.cpp \
../imu.cpp 

CC_DEPS += \
./Edge.d \
./FloatImage.d \
./GLine2D.d \
./GLineSegment2D.d \
./Gaussian.d \
./GrayModel.d \
./Homography33.d \
./MathUtil.d \
./Quad.d \
./Segment.d \
./TagDetection.d \
./TagDetector.d \
./TagFamily.d \
./UnionFindSimple.d 

OBJS += \
./Edge.o \
./FloatImage.o \
./GLine2D.o \
./GLineSegment2D.o \
./Gaussian.o \
./GrayModel.o \
./Homography33.o \
./MathUtil.o \
./Quad.o \
./Segment.o \
./Serial.o \
./TagDetection.o \
./TagDetector.o \
./TagFamily.o \
./UnionFindSimple.o \
./apriltags_demo.o \
./imu.o 

CPP_DEPS += \
./Serial.d \
./apriltags_demo.d \
./imu.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/usr/include/opencv -I/usr/include/eigen3 -I/usr/local/lib -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/usr/include/opencv -I/usr/include/eigen3 -I/usr/local/lib -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


