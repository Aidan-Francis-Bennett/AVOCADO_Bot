#include <mbed.h>
#include "IR.h"
using namespace mbed;
using namespace rtos;
// Declares where the SCL and SDA are on the chip so read and write operations can be done
extern I2C i2c;

// This function takes an 8 bit multiplexer address, an 8 bit address for the bus of the multiplexer into which the desired sensor is plugged, and a pointer to where in memory the i2c object is stored.
float IRDistance(char muxAddress, char sensorAddress, I2C *i2c) {
  // Any I2C methods require deferencing the address for the i2c object which has been passed into this function.
  // The correct bus on the multiplexer for the given sensor address has a 1 written to it to select it.
  i2c->write(muxAddress, &sensorAddress, 1);

  // Once the bus is selected, one byte is written to ask for data, then it is read into the IR_array in two seperate 4 bit parts: distance high and distance low. 
  char IR_array[2];
  IR_array[0] = 0x5E;
  IR_array[1] = 0x00;
  i2c->write(0x80, IR_array, 1);
  ThisThread::sleep_for(100);
  i2c->read(0x80, IR_array, 2);

  char distance_high = IR_array[0];
  char distance_low = IR_array[1];

  // Distance high and distance low are used in a bit shift operation to return a final value in cm: use the equation given in the datasheet.
  float distance_cm = ((float)distance_high*16.0+(float)distance_low)/16/4.0;

// The return logic accounts for the saturated sensor value of 63.98, which is when the sensor is too close to a wall, by returning 2.45cm instead, which is approx the smallest value the sensor can measure.
  if (distance_cm > 63.97){
    return 2.45;
  }
  else {
    return distance_cm;   
  }
}