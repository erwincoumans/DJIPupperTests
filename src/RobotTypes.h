#pragma once
#ifndef USE_SIM
#include <Arduino.h>
#include "Streaming.h"
#endif

#include <array>



typedef std::array<float, 12> ActuatorPositionVector;
typedef std::array<float, 12> ActuatorVelocityVector;
typedef std::array<float, 12> ActuatorCurrentVector;
typedef std::array<bool, 12> ActuatorActivations;
#ifndef USE_SIM
template <class T, unsigned int SIZE>
Print &operator<<(Print &stream, const std::array<T, SIZE> &vec) {
  for (auto e : vec) {
    stream << e << " ";
  }
  return stream;
}
#endif