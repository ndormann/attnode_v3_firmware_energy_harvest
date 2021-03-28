/*
DebugUtils.h - Simple debugging utilities.
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include "config.h"

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__); Serial.flush();
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__); Serial.flush();
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

#endif
