#ifndef IR_h
#define IR_h
using namespace mbed;

float IRDistance(char muxAddress, char sensorAddress, mbed::I2C *i2c);

#endif