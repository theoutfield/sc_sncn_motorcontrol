# This variable should contain a space separated list of all
# the directories containing buildable applications (usually
# prefixed with the app_ prefix)
#
# If the variable is set to "all" then all directories that start with app_
# are built.
BUILD_SUBDIRS = test_adc_ext_potentiometer test_auto-calib test_auto-calib-qei test_ethercat-motorctrl-mode test_position-ctrl test_torque-ctrl test_velocity-ctrl test_watchdog
XMOS_MAKE_PATH ?= ..
include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.toplevel
