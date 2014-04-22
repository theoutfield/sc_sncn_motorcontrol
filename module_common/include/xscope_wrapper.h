#pragma once
#ifdef USE_XSCOPE
  #include <xscope.h>
#else
  /* replace xscope functions with NOPs */

  // variadic macro
  #define xscope_register(...) while(0)
  #define xscope_probe_data(A,B) while(0)
  #define xscope_cpu_int(A1,B1) while(0)
  #define xscope_int(A2,B2) while(0)
#endif
