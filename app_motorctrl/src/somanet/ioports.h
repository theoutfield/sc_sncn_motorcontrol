#pragma once

#include "filename_macro.h"
#include "../somanet.h"

#ifndef SOMANET_CORE
#define SOMANET_CORE null
#endif

#ifndef SOMANET_COM
#define SOMANET_COM null
#endif

#ifndef SOMANET_IFM
#define SOMANET_IFM null
#endif

#define CORE_PREF core_
#define COM_PREF com_
#define IFM_PREF ifm_
#define SUFFIX_XC _xc.inc
#define SUFFIX_H _h.inc

#ifdef INCLUDE_FROM_IOPORTS_XC
#include STRFY( CONCAT3( CORE_PREF, SOMANET_CORE, SUFFIX_XC) )
#include STRFY( CONCAT3( COM_PREF, SOMANET_COM, SUFFIX_XC) )
#include STRFY( CONCAT3( IFM_PREF, SOMANET_IFM, SUFFIX_XC) )
#else
// import "extern" port declarations
#include STRFY( CONCAT3( CORE_PREF, SOMANET_CORE, SUFFIX_H) )
#include STRFY( CONCAT3( COM_PREF, SOMANET_COM, SUFFIX_H) )
#include STRFY( CONCAT3( IFM_PREF, SOMANET_IFM, SUFFIX_H) )
#endif

#undef CORE_PREF
#undef COM_PREF
#undef IFM_PREF
#undef SUFFIX_XC
#undef SUFFIX_H
