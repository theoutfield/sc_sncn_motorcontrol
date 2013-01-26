#include <xs1.h>

#define XCORE_CTRL0_CLOCK_MASK 0x30
#define XCORE_CTRL0_ENABLE_AEC 0x30

void enableAEC(unsigned standbyClockDivider) {
  unsigned xcore_ctrl0_data;

  // Set standby divider
  write_pswitch_reg(get_core_id(),
		    XS1_PSWITCH_PLL_CLK_DIVIDER_NUM,
		    (standbyClockDivider - 1));

  // Modify the clock control bits
  xcore_ctrl0_data = getps(XS1_PS_XCORE_CTRL0);
  xcore_ctrl0_data &= 0xffffffff - XCORE_CTRL0_CLOCK_MASK;
  xcore_ctrl0_data += XCORE_CTRL0_ENABLE_AEC;
  setps(XS1_PS_XCORE_CTRL0, xcore_ctrl0_data);
}
