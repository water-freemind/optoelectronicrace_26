################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./main.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./BSP/Uart.o" \
"./BSP/delay.o" \
"./Drive/Grayscale_Sensor/ADC.o" \
"./Drive/Grayscale_Sensor/No_Mcu_Ganv_Grayscale_Sensor.o" \
"./Drive/Knob/Knob_drv.o" \
"./Drive/Motor/Motor.o" \
"./Drive/OLED_Drv/OLED.o" \
"./Menu/Easy_Menu.o" \
"./Menu/Easy_Menu_Core.o" \
"./Menu/Easy_Menu_Item.o" \
"./Menu/Easy_Menu_Page.o" \
"./Menu/Easy_Menu_User.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include BSP/subdir_vars.mk
-include Drive/Grayscale_Sensor/subdir_vars.mk
-include Drive/Knob/subdir_vars.mk
-include Drive/Motor/subdir_vars.mk
-include Drive/OLED_Drv/subdir_vars.mk
-include Menu/subdir_vars.mk
-include subdir_rules.mk
-include BSP/subdir_rules.mk
-include Drive/Grayscale_Sensor/subdir_rules.mk
-include Drive/Knob/subdir_rules.mk
-include Drive/Motor/subdir_rules.mk
-include Drive/OLED_Drv/subdir_rules.mk
-include Menu/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
optoelectronicrace_26.out 

EXE_OUTPUTS__QUOTED += \
"optoelectronicrace_26.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "optoelectronicrace_26.out"

# Tool invocations
optoelectronicrace_26.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"optoelectronicrace_26.map" -Wl,-i"D:/MSP-SDK/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/MSP-SDK/CCS_project/optoelectronicrace_26" -Wl,-i"D:/MSP-SDK/CCS_project/optoelectronicrace_26/Debug/syscfg" -Wl,-i"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="optoelectronicrace_26_linkInfo.xml" -Wl,--rom_model -o "optoelectronicrace_26.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "BSP\Uart.o" "BSP\delay.o" "Drive\Grayscale_Sensor\ADC.o" "Drive\Grayscale_Sensor\No_Mcu_Ganv_Grayscale_Sensor.o" "Drive\Knob\Knob_drv.o" "Drive\Motor\Motor.o" "Drive\OLED_Drv\OLED.o" "Menu\Easy_Menu.o" "Menu\Easy_Menu_Core.o" "Menu\Easy_Menu_Item.o" "Menu\Easy_Menu_Page.o" "Menu\Easy_Menu_User.o" 
	-$(RM) "main.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "BSP\Uart.d" "BSP\delay.d" "Drive\Grayscale_Sensor\ADC.d" "Drive\Grayscale_Sensor\No_Mcu_Ganv_Grayscale_Sensor.d" "Drive\Knob\Knob_drv.d" "Drive\Motor\Motor.d" "Drive\OLED_Drv\OLED.d" "Menu\Easy_Menu.d" "Menu\Easy_Menu_Core.d" "Menu\Easy_Menu_Item.d" "Menu\Easy_Menu_Page.d" "Menu\Easy_Menu_User.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

