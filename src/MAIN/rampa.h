#ifndef PROFILER_H // include guard
#define PROFILER_H

#include <Arduino.h>
#define LINEAR  0
#define ANGULAR 1

double rampa(double set_speed, double acc, int side);
void debugPrint(double set_speed, int side);


#endif