# The TARGET variable determines what target system the application is 
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling.

CONFIG ?= Debug

TARGET = C22-ComECAT-IfmBLDC-XScope

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to 
# the project name

APP_NAME =

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
#
#    XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
#
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to 
# xcc for the final link (mapping) stage.

XCC_FLAGS = -O3 -g -report -Wno-timing -lxscope

# The USED_MODULES variable lists other module used by the application. 

USED_MODULES = module_adc sc_somanet-base/module_powermgmt  module_commutation module_hall
USED_MODULES += module_pwm_symmetrical module_pwm_common module_blocks module_ctrl_loops
USED_MODULES += sc_somanet-base/module_nodeconfig 
USED_MODULES += module_qei


#USE_XTCP_MAC_CUSTOM_FILTER = 1

XCC_FLAGS_Debug = $(XCC_FLAGS)
XCC_FLAGS_Release = $(XCC_FLAGS)

#AUTO_ECLIPSE = 0

#=============================================================================
# The following part of the Makefile includes the common build infrastructure
# for compiling XMOS applications. You should not need to edit below here.

XMOS_MAKE_PATH ?= ../..
#XMOS_MAKE_PATH = c:/Program\ Files\ (x86)/XMOS/DevelopmentTools/11.11.0/build
include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common

#=============================================================================
# Additional targets

doc:
	doxygen

showdevices:
	xrun -l

sim: all
	xsim --vcd-tracing "-core stdcore[3] -ports" $(BIN_DIR)/$(APP_NAME).xe > trace.vcd

run: all showdevices
	xrun --io $(BIN_DIR)/$(APP_NAME).xe

rundbg: all showdevices
	xgdb $(BIN_DIR)/$(APP_NAME).xe -ex connect -ex run

runxscope: all showdevices
	xrun --io --xscope-file xscope.xmt $(BIN_DIR)/$(APP_NAME).xe

flash: all showdevices
	xflash --verbose $(BIN_DIR)/$(APP_NAME).xe

flash_erase: all showdevices
	xflash --verbose --target $(TARGET) --erase-all
