################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1880293935:
	@$(MAKE) --no-print-directory -Onone -f TOOLS/subdir_rules.mk build-1880293935-inproc

build-1880293935-inproc: ../TOOLS/app_ble.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/ccs910/xdctools_3_55_02_22_core/xs" --xdcpath="C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source;C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/kernel/tirtos/packages;C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640R2F -r release -c "C:/ti/ccs910/ccs/tools/compiler/ti-cgt-arm_18.12.7.LTS" --compileOptions "-mv7M4 --code_state=16 -me -O2 --opt_for_speed=0 --include_path=\"C:/Projects/Wristband/wristband_app\" --include_path=\"C:/Projects/Wristband/wristband_app/Application\" --include_path=\"C:/Projects/Wristband/wristband_app/Startup\" --include_path=\"C:/Projects/Wristband/wristband_app/PROFILES\" --include_path=\"C:/Projects/Wristband/wristband_app/Include\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/examples/rtos/CC2640R2_LAUNCHXL/ble5stack/project_zero/src/extra\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/controller/cc26xx/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/rom\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/common/cc26xx\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/icall/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/target\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/hal/src/target/_common\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/hal/src/target/_common/cc26xx\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/hal/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/heapmgr\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/icall/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/osal/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/services/src/saddr\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack/services/src/sdata\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/devices/cc26x0r2\" --include_path=\"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-arm_18.12.7.LTS/include\" --define=DeviceFamily_CC26X0R2 --define=uartlog_FILE=\"\"\"\" --define=UARTLOG_ENABLE -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1880293935 ../TOOLS/app_ble.cfg
configPkg/compiler.opt: build-1880293935
configPkg/: build-1880293935


