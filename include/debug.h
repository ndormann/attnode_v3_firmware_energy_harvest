/*
DebugUtils.h - Simple debugging utilities.
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__); Serial.flush();
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__); Serial.flush();
  #define DEBUG_PRINTARR(...) for (uint8_t i=0; i<(sizeof(__VA_ARGS__)/sizeof(__VA_ARGS__[0])); i++) { Serial.print(__VA_ARGS__[i], HEX); Serial.print(" ");}; Serial.flush();
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTARR(...)
#endif

#endif