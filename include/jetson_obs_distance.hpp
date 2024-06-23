#include <cstdio>
#include <chrono>
#include <iostream>
#include <unistd.h>

enum class SensorState {

  Uninitialized,
  CommsInit,
  CheckAlive,
  Init,
  SetRes,
  SetFreq,
  RangingActive,
  DataReady,
  Failure,
  InitComplete

};
