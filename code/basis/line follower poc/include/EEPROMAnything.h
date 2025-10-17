#ifndef EEPROMANYTHING_H
#define EEPROMANYTHING_H

#include <EEPROM.h>

// Call this once in setup() with the size you need
inline void EEPROM_initialize(size_t size = 512) {
  EEPROM.begin(size);
}

// Write any struct or variable to EEPROM
template <class T> int EEPROM_writeAnything(int ee, const T& value) {
  EEPROM.put(ee, value);
  EEPROM.commit(); // Required on ESP32
  return sizeof(value);
}

// Read any struct or variable from EEPROM
template <class T> int EEPROM_readAnything(int ee, T& value) {
  EEPROM.get(ee, value);
  return sizeof(value);
}

#endif