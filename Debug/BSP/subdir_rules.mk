################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/MSP-SDK/CCS_project/optoelectronicrace_26" -I"D:/MSP-SDK/CCS_project/optoelectronicrace_26/Debug" -I"D:/MSP-SDK/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/MSP-SDK/mspm0_sdk_2_10_00_04/source" -I"D:/MSP-SDK/CCS_project/optoelectronicrace_26/BSP" -gdwarf-3 -Wall -MMD -MP -MF"BSP/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


