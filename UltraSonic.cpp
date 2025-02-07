#include <mbed.h>
#include "UltraSonic.h"
using namespace mbed;

UltraSonic::UltraSonic(PinName signalPin) 
  : _signalPin(signalPin)
  {
    _signalPin.input();

    // This setup code calculates the actual software polling delay to take into acount when calculating distance with the echo pulse timer.
    _Echo.reset();
    _Echo.start();

    // The minimum software polling delay to read echo pin
    // This while loop is empty, and the pin will never equal 2, so is skipped but just adds to the polling time the time it takes to read an ultraSonic pin in input mode for the correction time
    while (_signalPin == 2) {}; 
    _Echo.stop();

    // Sets the software polling delay time as a private variable for the correction required when calculating the distance later.
    _Correction = _Echo.read_us();
}

int UltraSonic::Distance() {
   // This sequence sets the first ultrasonic pin digital in/out to output mode, and sends a 10us pulse to the trigger of the sensor
  _signalPin.output();
  _signalPin = 0;
  wait_us(2);
  _signalPin = 1; 
  _Echo.reset();
  wait_us(10.0);
  _signalPin = 0;

  // Then the pin is set to input mode and waits for an echo signal rising edge to start the timer
  _signalPin.input();
  while (_signalPin==0){};
  _Echo.start();

  // Now it waits for the falling edge echo signal to stop the timer
  while (_signalPin == 1){};
  _Echo.stop();

  // Now it returns the distance as an integer accounting for the software polling delay calculated in the setup code.
  return (_Echo.read_us() - _Correction)/58.0;
}


int UltraSonic::getCorrection()
{
  return _Correction;
}
